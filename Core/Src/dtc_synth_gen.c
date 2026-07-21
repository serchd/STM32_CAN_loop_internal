/* USER CODE BEGIN Header */
/**
 * dtc_synth_gen.c -- ver dtc_synth_gen.h para el resumen de que se porta y que no.
 */
/* USER CODE END Header */
#include "dtc_synth_gen.h"
#include "cmsis_os.h"
#include "can.h"
#include "bcm_sim.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
extern void UART_Print(const char *msg);
extern CAN_HandleTypeDef hcan1;

/* Mismo layout que TaskHealth_t en freertos.c -- se referencia por extern
 * igual que ya hacia main.c, para no exponer un header nuevo solo por esto. */
typedef struct {
    volatile uint32_t led_alive_tick;
    volatile uint32_t can_tx_alive_tick;
    volatile uint32_t can_rx_alive_tick;
    volatile uint32_t vehicle_tx_alive_tick;
} TaskHealth_t;
extern TaskHealth_t g_task_health;

/* =============================================================================
 * PRNG: xorshift32 + Box-Muller. Determinista por semilla (NO bit-exacto a
 * numpy -- ver nota en el .h). Suficiente para reproducibilidad en banco.
 * ========================================================================== */
typedef struct { uint32_t state; } Rng_t;

static void rng_seed(Rng_t *r, uint32_t seed)
{
    r->state = seed ? seed : 0xA5A5A5A5u;
}

static uint32_t rng_next_u32(Rng_t *r)
{
    uint32_t x = r->state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    r->state = x;
    return x;
}

/* uniforme (0,1) */
static float rng_uniform01(Rng_t *r)
{
    return (float)(rng_next_u32(r) >> 8) / (float)(1u << 24);
}

static float rng_uniform(Rng_t *r, float lo, float hi)
{
    return lo + (hi - lo) * rng_uniform01(r);
}

/* ~N(0,1), Box-Muller */
static float rng_gauss(Rng_t *r)
{
    float u1 = rng_uniform01(r) + 1e-7f;
    float u2 = rng_uniform01(r);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159265f * u2);
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* =============================================================================
 * Escenarios -- MISMO orden que TIPOS_ESCENARIO en generador_multidtc.py.
 * El indice se transmite tal cual en RunMeta.ScenarioId (ver VAL_ 240 en la DBC).
 * ========================================================================== */
typedef enum {
    ESC_NORMAL = 0,
    ESC_BATERIA_PROGRESIVA, ESC_BATERIA_SUBITA,
    ESC_TEMPERATURA_PROGRESIVA, ESC_TEMPERATURA_RAPIDA,
    ESC_ACEITE_PROGRESIVA, ESC_ACEITE_SUBITA,
    ESC_CKP_INTERMITENTE, ESC_CKP_PERDIDA_TOTAL,
    ESC_LLANTA_LENTA, ESC_LLANTA_ABRUPTA,
    ESC_COMBUSTIBLE_NORMAL,
    ESC_ACELERADOR_PROGRESIVA, ESC_ACELERADOR_SUBITA,
    ESC_COUNT
} Escenario_t;

#define T_MAX_S   600u
#define N_TICKS   (T_MAX_S + 1u)     /* 601, igual que t = np.arange(0, t_max+1) */
#define SEG       (N_TICKS / 6u)     /* 100, igual que seg = n // 6 */

/* Estado que persiste tick a tick durante UNA corrida (equivalente a lo que
 * en Python son variables locales dentro de generar_corrida()). */
typedef struct {
    Rng_t    rng;
    Escenario_t escenario;
    uint32_t seed;
    uint8_t  run_id;

    /* bateria */
    float    t0_bateria;
    float    pend_bateria;

    /* acelerador / mariposa (gemelo digital) */
    float    tps_prev;          /* tps[i-1] */
    float    tps_esperado_prev; /* pred[i-1] del gemelo digital */
    float    t0_acelerador;
    uint8_t  stuck_activo;
    float    stuck_value;

    /* combustible: nivel inicial elegido una vez por corrida */
    float    fuel_nivel_inicial;

    /* dinamica de vehiculo (solo para acompanar con RPM/velocidad realistas) */
    float    v_prev;
    float    rpm_prev;

    /* freno: contador de confirmacion del brake-override (capa NO-ML) */
    uint8_t  brake_override_contador;
} RunState_t;

static void run_state_init(RunState_t *s, Escenario_t escenario, uint32_t seed, uint8_t run_id)
{
    memset(s, 0, sizeof(*s));
    rng_seed(&s->rng, seed);
    s->escenario = escenario;
    s->seed = seed;
    s->run_id = run_id;
    s->rpm_prev = 800.0f;

    /* Ventanas de aparicion de falla, elegidas UNA vez al inicio de la
     * corrida -- igual que rng.uniform(...) fuera del loop en Python. */
    s->t0_bateria    = rng_uniform(&s->rng, 200.0f, 350.0f);
    s->pend_bateria  = rng_uniform(&s->rng, 0.006f, 0.016f);
    s->t0_acelerador = rng_uniform(&s->rng, (float)N_TICKS / 3.0f, (float)N_TICKS / 3.0f + 40.0f);
    s->fuel_nivel_inicial = rng_uniform(&s->rng, 8.0f, 100.0f);
}

/* =============================================================================
 * Perfil de pedal del conductor (APP), streaming: dado t (0..600), devuelve
 * app(t). Puerto de _base_perfil_pedal() -- misma forma por tramos, evaluada
 * en un solo instante en vez de vectorizada.
 * ========================================================================== */
static float perfil_pedal_tick(Rng_t *rng, uint32_t t)
{
    float app;

    if (t < SEG) {
        app = 55.0f * (float)t / (float)(SEG - 1);
    } else if (t < 2u * SEG) {
        float i = (float)(t - SEG);
        float x = 4.0f * i / (float)(SEG - 1);
        app = 55.0f + 8.0f * sinf(x);
    } else if (t < 3u * SEG) {
        float i = (float)(t - 2u * SEG);
        app = 55.0f - 55.0f * i / (float)(SEG - 1);
    } else if (t < 4u * SEG) {
        app = rng_uniform(rng, 0.0f, 3.0f);
    } else if (t < 5u * SEG) {
        float i = (float)(t - 4u * SEG);
        app = 75.0f * i / (float)(SEG - 1);
    } else {
        float i = (float)(t - 5u * SEG);
        float ultimo_len = (float)(N_TICKS - 5u * SEG - 1u); /* 100 */
        app = 75.0f - 60.0f * i / ultimo_len;
    }

    app += rng_gauss(rng) * 1.0f;
    return clampf(app, 0.0f, 100.0f);
}

/* Freno normal: rampa 0->0.5 justo antes de "acercarse al alto" (puerto de
 * _base_freno_perfil). Sin ruido, igual que el original. */
static float freno_normal_tick(uint32_t t)
{
    uint32_t ini = (3u * SEG > 15u) ? (3u * SEG - 15u) : 0u;
    if (t >= ini && t < 3u * SEG) {
        float i = (float)(t - ini);
        float denom = (float)(3u * SEG - ini - 1u);
        return 0.5f * i / denom;
    }
    return 0.0f;
}

/* =============================================================================
 * Subsistema de acelerador -- gemelo digital + TPS real (sano o con falla).
 * Puerto de _mariposa_esperada_gemelo_digital() + _falla_acelerador().
 * Ambas eran YA recursivas en Python -- se portan practicamente 1:1.
 * ========================================================================== */
static void acelerador_tick(RunState_t *s, uint32_t t, float app, float brake,
                             float *out_tps_real, float *out_tps_esperado)
{
    const float tau_nom = 1.2f;
    float tau = tau_nom;

    /* --- gemelo digital: SIEMPRE corre igual, sano, independiente de la falla --- */
    float objetivo = app * 0.9f;
    float tps_esperado = s->tps_esperado_prev + (objetivo - s->tps_esperado_prev) / tau_nom;

    float tps_real;

    if (s->escenario == ESC_ACELERADOR_SUBITA && (float)t >= s->t0_acelerador) {
        /* mariposa atorada en un valor fijo (tapete / mecanismo trabado) */
        if (!s->stuck_activo) {
            s->stuck_activo = 1;
            s->stuck_value = s->tps_prev;
        }
        tps_real = s->stuck_value + rng_gauss(&s->rng) * 0.3f;
    } else {
        uint8_t en_liberacion =
            (t >= 2u * SEG && t < 3u * SEG) || (t >= 5u * SEG);

        if (s->escenario == ESC_ACELERADOR_PROGRESIVA && en_liberacion) {
            float progreso = (float)t / (float)N_TICKS;
            tau = tau_nom + 35.0f * progreso * progreso;
        }

        tps_real = s->tps_prev + (objetivo - s->tps_prev) / tau + rng_gauss(&s->rng) * 0.2f;
    }

    tps_real = clampf(tps_real, 0.0f, 100.0f);

    s->tps_prev = tps_real;
    s->tps_esperado_prev = tps_esperado;

    /* --- brake-override: capa de seguridad determinista, NO-ML (puerto de
     * _flag_brake_override, t_confirm=1 -- se dispara en el mismo tick) --- */
    uint8_t conflicto = (brake > 0.3f) && (tps_real > 15.0f);
    s->brake_override_contador = conflicto ? (uint8_t)(s->brake_override_contador + 1) : 0;

    *out_tps_real = tps_real;
    *out_tps_esperado = tps_esperado;
}

/* Dinamica longitudinal simplificada -- puerto directo de _dinamica_vehiculo,
 * ya era recursiva. Solo para tener RPM/velocidad realistas de fondo. */
static void dinamica_vehiculo_tick(RunState_t *s, float tps, float brake,
                                    float *out_v, float *out_rpm)
{
    const float K_TRACCION = 0.09f, K_FRENO = 0.22f, K_ARRASTRE = 0.0009f, K_RODADURA = 0.015f;
    float a = K_TRACCION * tps - K_FRENO * brake * 100.0f
              - K_ARRASTRE * s->v_prev * s->v_prev - K_RODADURA * 100.0f;
    float piso = -K_FRENO * brake * 100.0f - 15.0f;
    if (a < piso) a = piso;

    float v = s->v_prev + a;
    if (v < 0.0f) v = 0.0f;
    float rpm = 800.0f + v * 32.0f;

    s->v_prev = v;
    s->rpm_prev = rpm;
    *out_v = v;
    *out_rpm = rpm;
}

/* =============================================================================
 * Bateria (P0562) -- puerto de _base_bateria + _falla_bateria.
 * ========================================================================== */
static float bateria_tick(RunState_t *s, uint32_t t)
{
    float v = 14.0f + 0.1f * rng_gauss(&s->rng);

    if (s->escenario == ESC_BATERIA_PROGRESIVA && (float)t >= s->t0_bateria) {
        v = 14.0f - s->pend_bateria * ((float)t - s->t0_bateria) + 0.1f * rng_gauss(&s->rng);
    } else if (s->escenario == ESC_BATERIA_SUBITA && (float)t >= s->t0_bateria) {
        v = rng_uniform(&s->rng, 10.3f, 11.0f) + 0.15f * rng_gauss(&s->rng);
    }
    return v;
}

/* =============================================================================
 * TODO -- mismo patron para las 4 senales restantes de CONFIG_DTC.
 * Cada una es tan corta como bateria_tick(); portalas cuando las necesites:
 *
 *   temperatura_tick()  <- _base_temperatura() + _falla_temperatura()   (linea ~107, ~265)
 *   aceite_tick()       <- _base_aceite()      + _falla_aceite()        (linea ~110, ~276)
 *   ckp_tick()          <- _base_ckp()         + _falla_ckp()           (linea ~113, ~289)
 *   llanta_tick()       <- _base_llanta()      + _falla_llanta()        (linea ~116, ~305)
 *
 * Todas siguen la MISMA receta: valor base + gauss(), y si (escenario activo
 * Y t >= t0) se sobreescribe con la formula de _falla_*. combustible_tick()
 * no necesita falla dedicada (solo _base_combustible, con nivel_inicial ya
 * guardado en RunState_t.fuel_nivel_inicial).
 * ========================================================================== */
static float temperatura_tick(void) { return 90.0f; }   /* placeholder: portar cuando lo necesites */
static float aceite_tick(void)      { return 45.0f; }   /* placeholder */
static float ckp_tick(void)         { return 98.0f; }   /* placeholder */
static float llanta_tick(void)      { return 32.0f; }   /* placeholder */

static float combustible_tick(RunState_t *s, uint32_t t)
{
    float consumo = s->fuel_nivel_inicial - 0.02f * (float)t;
    return clampf(consumo + 0.3f * rng_gauss(&s->rng), 0.0f, 100.0f);
}

/* =============================================================================
 * Empaquetado y transmision de UN tick completo, usando las funciones _pack()
 * generadas desde bcm_sim.dbc (ver integracion previa del ComStack).
 * ========================================================================== */
/* Espeja la trama por UART en el MISMO formato que candump -L, para que
 * can_log_to_dataframe.py la lea sin cambios -- sin necesitar CANable ni
 * ningun adaptador CAN fisico. Se manda por el mismo puerto del ST-Link
 * que ya usas para ver los prints de RPM/CoolantTemp/etc. */
static void uart_mirror_frame(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
    char linea[64];
    char hexbuf[17];
    uint8_t i;

    for (i = 0; i < dlc; i++) {
        snprintf(&hexbuf[i * 2], 3, "%02X", data[i]);
    }
    hexbuf[dlc * 2] = '\0';

    /* timestamp relativo al arranque (segundos.milisegundos) -- suficiente
     * para ordenar/replayear; no necesita ser epoch real. */
    uint32_t ms = osKernelGetTickCount();
    snprintf(linea, sizeof(linea), "(%lu.%06lu) can0 %lX#%s\r\n",
             (unsigned long)(ms / 1000u), (unsigned long)((ms % 1000u) * 1000u),
             (unsigned long)std_id, hexbuf);
    UART_Print(linea);
}

static void transmitir_tick(RunState_t *s, uint32_t t)
{
    CAN_TxHeaderTypeDef hdr;
    uint32_t mailbox;
    uint8_t buf[8];

    /* --- señales base de este tick --- */
    float bateria_v    = bateria_tick(s, t);
    float temp_c        = temperatura_tick();
    float aceite_psi     = aceite_tick();
    float ckp_pct        = ckp_tick();
    float llanta_psi     = llanta_tick();
    float fuel_pct       = combustible_tick(s, t);

    float app    = perfil_pedal_tick(&s->rng, t);
    float brake  = freno_normal_tick(t);
    float tps_real, tps_esperado;
    acelerador_tick(s, t, app, brake, &tps_real, &tps_esperado);
    float residuo = tps_real - tps_esperado;

    float v_kmh, rpm;
    dinamica_vehiculo_tick(s, tps_real, brake, &v_kmh, &rpm);

    uint8_t brake_override = (s->brake_override_contador >= 1) ? 1 : 0;

    /* --- 0x0F0 RunMeta --- */
    struct bcm_sim_run_meta_t meta;
    meta.scenario_id = (uint8_t)s->escenario;
    meta.run_id       = s->run_id;
    meta.seed_low16   = (uint16_t)(s->seed & 0xFFFFu);
    meta.time_s       = (uint16_t)t;
    hdr.StdId = BCM_SIM_RUN_META_FRAME_ID; hdr.ExtId = 0; hdr.IDE = CAN_ID_STD;
    hdr.RTR = CAN_RTR_DATA; hdr.DLC = BCM_SIM_RUN_META_LENGTH; hdr.TransmitGlobalTime = DISABLE;
    if (bcm_sim_run_meta_pack(buf, &meta, sizeof(buf)) >= 0)
        HAL_CAN_AddTxMessage(&hcan1, &hdr, buf, &mailbox);
    uart_mirror_frame(hdr.StdId, buf, hdr.DLC);

    /* --- 0x100 EngineStatus --- */
    struct bcm_sim_engine_status_t eng;
    eng.rpm            = bcm_sim_engine_status_rpm_encode(rpm);
    eng.coolant_temp    = bcm_sim_engine_status_coolant_temp_encode(temp_c);
    eng.engine_running  = 1;
    hdr.StdId = BCM_SIM_ENGINE_STATUS_FRAME_ID; hdr.DLC = BCM_SIM_ENGINE_STATUS_LENGTH;
    if (bcm_sim_engine_status_pack(buf, &eng, sizeof(buf)) >= 0)
        HAL_CAN_AddTxMessage(&hcan1, &hdr, buf, &mailbox);
    uart_mirror_frame(hdr.StdId, buf, hdr.DLC);

    /* --- 0x101 VehicleSpeed --- */
    struct bcm_sim_vehicle_speed_t spd;
    spd.speed = bcm_sim_vehicle_speed_speed_encode(v_kmh);
    hdr.StdId = BCM_SIM_VEHICLE_SPEED_FRAME_ID; hdr.DLC = BCM_SIM_VEHICLE_SPEED_LENGTH;
    if (bcm_sim_vehicle_speed_pack(buf, &spd, sizeof(buf)) >= 0)
        HAL_CAN_AddTxMessage(&hcan1, &hdr, buf, &mailbox);
    uart_mirror_frame(hdr.StdId, buf, hdr.DLC);

    /* --- 0x102 BrakeStatus --- */
    struct bcm_sim_brake_status_t brk;
    brk.brake_pedal_pct    = bcm_sim_brake_status_brake_pedal_pct_encode((double)(brake * 100.0f));
    brk.brake_override_flag = brake_override;
    hdr.StdId = BCM_SIM_BRAKE_STATUS_FRAME_ID; hdr.DLC = BCM_SIM_BRAKE_STATUS_LENGTH;
    if (bcm_sim_brake_status_pack(buf, &brk, sizeof(buf)) >= 0)
        HAL_CAN_AddTxMessage(&hcan1, &hdr, buf, &mailbox);
    uart_mirror_frame(hdr.StdId, buf, hdr.DLC);

    /* --- 0x110 PowertrainHealth --- */
    struct bcm_sim_powertrain_health_t pth;
    pth.battery_voltage = bcm_sim_powertrain_health_battery_voltage_encode(bateria_v);
    pth.oil_pressure     = bcm_sim_powertrain_health_oil_pressure_encode(aceite_psi);
    pth.crank_signal_quality = bcm_sim_powertrain_health_crank_signal_quality_encode(ckp_pct);
    hdr.StdId = BCM_SIM_POWERTRAIN_HEALTH_FRAME_ID; hdr.DLC = BCM_SIM_POWERTRAIN_HEALTH_LENGTH;
    if (bcm_sim_powertrain_health_pack(buf, &pth, sizeof(buf)) >= 0)
        HAL_CAN_AddTxMessage(&hcan1, &hdr, buf, &mailbox);
    uart_mirror_frame(hdr.StdId, buf, hdr.DLC);

    /* --- 0x111 ChassisHealth --- */
    struct bcm_sim_chassis_health_t cha;
    cha.tire_pressure = bcm_sim_chassis_health_tire_pressure_encode(llanta_psi);
    cha.fuel_level_pct = bcm_sim_chassis_health_fuel_level_pct_encode(fuel_pct);
    hdr.StdId = BCM_SIM_CHASSIS_HEALTH_FRAME_ID; hdr.DLC = BCM_SIM_CHASSIS_HEALTH_LENGTH;
    if (bcm_sim_chassis_health_pack(buf, &cha, sizeof(buf)) >= 0)
        HAL_CAN_AddTxMessage(&hcan1, &hdr, buf, &mailbox);
    uart_mirror_frame(hdr.StdId, buf, hdr.DLC);

    /* --- 0x120 ThrottleSubsystem --- */
    struct bcm_sim_throttle_subsystem_t thr;
    thr.accel_pedal_pct       = bcm_sim_throttle_subsystem_accel_pedal_pct_encode(app);
    thr.throttle_position_pct = bcm_sim_throttle_subsystem_throttle_position_pct_encode(tps_real);
    thr.throttle_expected_pct = bcm_sim_throttle_subsystem_throttle_expected_pct_encode(tps_esperado);
    thr.throttle_residual_pct = bcm_sim_throttle_subsystem_throttle_residual_pct_encode(residuo);
    hdr.StdId = BCM_SIM_THROTTLE_SUBSYSTEM_FRAME_ID; hdr.DLC = BCM_SIM_THROTTLE_SUBSYSTEM_LENGTH;
    if (bcm_sim_throttle_subsystem_pack(buf, &thr, sizeof(buf)) >= 0)
        HAL_CAN_AddTxMessage(&hcan1, &hdr, buf, &mailbox);
    uart_mirror_frame(hdr.StdId, buf, hdr.DLC);
}

/* =============================================================================
 * Tarea FreeRTOS: cicla los escenarios en el MISMO orden que TIPOS_ESCENARIO,
 * una corrida completa (601 ticks, 1 por segundo) por escenario, e
 * indefinidamente -- igual que loop_ciclico_multidtc.py.
 * ========================================================================== */
void StartDtcSynthGenTask(void *argument)
{
    (void)argument;
    Escenario_t escenario = ESC_NORMAL;
    uint8_t run_id = 0;
    /* Semilla "base de validacion", igual espiritu que SEMILLA_BASE_VALIDACION
     * en Python -- aqui simplemente se deriva del tick de arranque, para que
     * cada sesion de banco sea distinta. Si necesitas EXACTA repetibilidad
     * entre sesiones, fija SEED_BASE a una constante en vez de leer el tick. */
    uint32_t seed_base = 100000u + (osKernelGetTickCount() & 0xFFFFu);

    char msg[80];
    snprintf(msg, sizeof(msg), "DtcSynthGenTask iniciada, seed_base=%lu\r\n", (unsigned long)seed_base);
    UART_Print(msg);

    for (;;) {
        run_id++;
        uint32_t seed = seed_base + run_id * 37u + 11u;

        RunState_t state;
        run_state_init(&state, escenario, seed, run_id);

        snprintf(msg, sizeof(msg), "SYNTH run_id=%u escenario=%u seed=%lu\r\n",
                 run_id, (unsigned)escenario, (unsigned long)seed);
        UART_Print(msg);

        for (uint32_t t = 0; t < N_TICKS; t++) {
            transmitir_tick(&state, t);
            g_task_health.can_tx_alive_tick = osKernelGetTickCount();
            g_task_health.vehicle_tx_alive_tick = osKernelGetTickCount();
            osDelay(1000);   /* 1 tick = 1 segundo, igual granularidad que el dataset */
        }

        escenario = (Escenario_t)((escenario + 1) % ESC_COUNT);
    }
}
