/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "can.h"
#include "usart.h"
#include "iwdg.h"
#include "bcm_sim.h"   /* generado con: cantools generate_c_source bcm_sim.dbc */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint16_t rpm;
	int8_t coolant_temp_c;
	uint8_t engine_running;
	uint32_t rx_count;
} EngineStatus_t;

typedef struct {
	uint16_t vehicle_speed_kph_x10;
	uint8_t  brake_pedal_pressed;
	uint16_t brake_pressure_bar;
	uint32_t rx_count;
} VehicleDynamics_t;

typedef struct {
	volatile uint32_t led_alive_tick;
	volatile uint32_t can_tx_alive_tick;
	volatile uint32_t can_rx_alive_tick;
	volatile uint32_t vehicle_tx_alive_tick;
} TaskHealth_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Los IDs de mensaje ya no se hardcodean aqui: vienen de bcm_sim.h
 * (BCM_SIM_ENGINE_STATUS_FRAME_ID, etc.), generados directo desde la DBC.
 * Si la DBC cambia, este codigo se recompila con los IDs correctos --
 * ya no hay que sincronizar numeros a mano en dos lugares. */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
EngineStatus_t    g_engine_status    = { 0 };
VehicleDynamics_t g_vehicle_dynamics = { 0 };
TaskHealth_t      g_task_health      = { 0 };

osMutexId_t uartMutexHandle;
osMutexId_t canDataMutexHandle;
osSemaphoreId_t buttonSemHandle;

osThreadId_t ledTaskHandle;
osThreadId_t canTxTaskHandle;
osThreadId_t vehicleTxTaskHandle;
osThreadId_t canRxPrintTaskHandle;
osThreadId_t buttonTaskHandle;
osThreadId_t watchdogTaskHandle;

const osThreadAttr_t ledTask_attributes = { .name = "LedTask", .stack_size = 256
		* 4, .priority = (osPriority_t) osPriorityLow, };
const osThreadAttr_t canTxTask_attributes = { .name = "CanTxTask", .stack_size =
		512 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t vehicleTxTask_attributes = { .name = "VehicleTxTask",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t canRxPrintTask_attributes = { .name = "CanRxPrintTask",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
const osThreadAttr_t buttonTask_attributes =
		{ .name = "ButtonTask", .stack_size = 256 * 4, .priority =
				(osPriority_t) osPriorityAboveNormal, };
const osThreadAttr_t watchdogTask_attributes = { .name = "WatchdogTask",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void StartLedTask(void *argument);
void StartCanTxTask(void *argument);
void StartVehicleTxTask(void *argument);
void StartCanRxPrintTask(void *argument);
void StartButtonTask(void *argument);
void StartWatchdogTask(void *argument);
void UART_Print(const char *msg); /* usada tambien desde main.c */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	const osMutexAttr_t uartMutex_attr = { .name = "uartMutex" };
	uartMutexHandle = osMutexNew(&uartMutex_attr);

	const osMutexAttr_t canMutex_attr = { .name = "canDataMutex" };
	canDataMutexHandle = osMutexNew(&canMutex_attr);
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	buttonSemHandle = osSemaphoreNew(1, 0, NULL); /* binario, inicia en 0 */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);
	if (ledTaskHandle == NULL) {
	    UART_Print("ERROR: no se pudo crear LedTask\r\n");
	}
	canTxTaskHandle = osThreadNew(StartCanTxTask, NULL, &canTxTask_attributes);
	if (canTxTaskHandle == NULL) {
	    UART_Print("ERROR: no se pudo crear CanTxTask\r\n");
	}
	vehicleTxTaskHandle = osThreadNew(StartVehicleTxTask, NULL, &vehicleTxTask_attributes);
	if (vehicleTxTaskHandle == NULL) {
	    UART_Print("ERROR: no se pudo crear VehicleTxTask\r\n");
	}
	canRxPrintTaskHandle = osThreadNew(StartCanRxPrintTask, NULL,&canRxPrintTask_attributes);
	if (canRxPrintTaskHandle == NULL) {
	    UART_Print("ERROR: no se pudo crear CanRxPrintTask\r\n");
	}
	buttonTaskHandle = osThreadNew(StartButtonTask, NULL,&buttonTask_attributes);
	if (buttonTaskHandle == NULL) {
	    UART_Print("ERROR: no se pudo crear ButtonTask\r\n");
	}
	watchdogTaskHandle = osThreadNew(StartWatchdogTask, NULL,&watchdogTask_attributes);
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void UART_Print(const char *msg) {
	if (osMutexAcquire(uartMutexHandle, 100) == osOK) {
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
		osMutexRelease(uartMutexHandle);
	}
}

void StartLedTask(void *argument)
{
	UART_Print("LedTask iniciada\r\n");   // <-- agrega esto
	for (;;) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		g_task_health.led_alive_tick = osKernelGetTickCount();
		osDelay(500);
	}
}

void StartCanTxTask(void *argument) {
	CAN_TxHeaderTypeDef txHeader;
	uint8_t txData[BCM_SIM_ENGINE_STATUS_LENGTH];
	uint32_t txMailbox;
	uint16_t sim_rpm = 800;
	uint8_t direction_up = 1;
	struct bcm_sim_engine_status_t msg;

	txHeader.StdId = BCM_SIM_ENGINE_STATUS_FRAME_ID;
	txHeader.ExtId = 0;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = BCM_SIM_ENGINE_STATUS_LENGTH;
	txHeader.TransmitGlobalTime = DISABLE;

	for (;;) {
		if (direction_up) {
			sim_rpm += 50;
			if (sim_rpm >= 3000)
				direction_up = 0;
		} else {
			sim_rpm -= 50;
			if (sim_rpm <= 800)
				direction_up = 1;
		}

		/* Valores fisicos -> raw de bus, via las funciones _encode() generadas
		 * desde la DBC (aplican escala/offset por ti, igual que Com_SendSignal). */
		msg.rpm            = bcm_sim_engine_status_rpm_encode((double)sim_rpm);
		msg.coolant_temp   = bcm_sim_engine_status_coolant_temp_encode(90.0);
		msg.engine_running = bcm_sim_engine_status_engine_running_encode(1.0);

		if (bcm_sim_engine_status_pack(txData, &msg, sizeof(txData)) < 0) {
			UART_Print("WARN: bcm_sim_engine_status_pack fallo\r\n");
		} else if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox)
				!= HAL_OK) {
			UART_Print("WARN: fallo al transmitir CAN (engine status)\r\n");
		}

		g_task_health.can_tx_alive_tick = osKernelGetTickCount();
		osDelay(100);
	}
}

/**
 * @brief Simula y transmite las senales de dinamica del vehiculo que
 *        necesita el proyecto de deteccion de fallas (0x101 velocidad,
 *        0x102 freno). En un vehiculo real estas vienen del modulo ABS/ESP
 *        y del switch/pedal de freno, tipicamente en un mensaje CAN
 *        distinto al del ECM -- por eso se simulan en una tarea aparte,
 *        replicando la segmentacion real del bus de carroceria.
 */
void StartVehicleTxTask(void *argument) {
	CAN_TxHeaderTypeDef txHeaderSpeed;
	CAN_TxHeaderTypeDef txHeaderBrake;
	uint8_t txDataSpeed[BCM_SIM_VEHICLE_SPEED_LENGTH];
	uint8_t txDataBrake[BCM_SIM_BRAKE_STATUS_LENGTH];
	uint32_t txMailbox;
	struct bcm_sim_vehicle_speed_t msgSpeed;
	struct bcm_sim_brake_status_t  msgBrake;

	uint16_t sim_speed_kph_x10 = 0;   /* arranca detenido */
	uint8_t  speed_dir_up = 1;
	uint8_t  brake_toggle_counter = 0;
	uint8_t  brake_pressed = 0;

	txHeaderSpeed.StdId = BCM_SIM_VEHICLE_SPEED_FRAME_ID;
	txHeaderSpeed.ExtId = 0;
	txHeaderSpeed.IDE = CAN_ID_STD;
	txHeaderSpeed.RTR = CAN_RTR_DATA;
	txHeaderSpeed.DLC = BCM_SIM_VEHICLE_SPEED_LENGTH;
	txHeaderSpeed.TransmitGlobalTime = DISABLE;

	txHeaderBrake.StdId = BCM_SIM_BRAKE_STATUS_FRAME_ID;
	txHeaderBrake.ExtId = 0;
	txHeaderBrake.IDE = CAN_ID_STD;
	txHeaderBrake.RTR = CAN_RTR_DATA;
	txHeaderBrake.DLC = BCM_SIM_BRAKE_STATUS_LENGTH;
	txHeaderBrake.TransmitGlobalTime = DISABLE;

	for (;;) {
		/* --- Velocidad: rampa 0-120.0 km/h, sube y baja en diente de sierra --- */
		if (speed_dir_up) {
			sim_speed_kph_x10 += 15;
			if (sim_speed_kph_x10 >= 1200)
				speed_dir_up = 0;
		} else {
			if (sim_speed_kph_x10 >= 15)
				sim_speed_kph_x10 -= 15;
			else
				speed_dir_up = 1;
		}

		/* sim_speed_kph_x10 esta en decimas de km/h -> pasar a double km/h fisico */
		msgSpeed.speed = bcm_sim_vehicle_speed_speed_encode(sim_speed_kph_x10 / 10.0);

		if (bcm_sim_vehicle_speed_pack(txDataSpeed, &msgSpeed, sizeof(txDataSpeed)) < 0) {
			UART_Print("WARN: bcm_sim_vehicle_speed_pack fallo\r\n");
		} else if (HAL_CAN_AddTxMessage(&hcan1, &txHeaderSpeed, txDataSpeed, &txMailbox)
				!= HAL_OK) {
			UART_Print("WARN: fallo al transmitir CAN (velocidad)\r\n");
		}

		/* --- Freno: se activa unos segundos cada ciclo (~3s a 100ms/vuelta) --- */
		brake_toggle_counter++;
		if (brake_toggle_counter >= 30) {
			brake_toggle_counter = 0;
			brake_pressed = !brake_pressed;
		}
		double brake_pressure = brake_pressed
				? (double)(80U + (sim_speed_kph_x10 % 40U))
				: 0.0;

		msgBrake.brake_pedal    = bcm_sim_brake_status_brake_pedal_encode((double)brake_pressed);
		msgBrake.brake_pressure = bcm_sim_brake_status_brake_pressure_encode(brake_pressure);

		if (bcm_sim_brake_status_pack(txDataBrake, &msgBrake, sizeof(txDataBrake)) < 0) {
			UART_Print("WARN: bcm_sim_brake_status_pack fallo\r\n");
		} else if (HAL_CAN_AddTxMessage(&hcan1, &txHeaderBrake, txDataBrake, &txMailbox)
				!= HAL_OK) {
			UART_Print("WARN: fallo al transmitir CAN (freno)\r\n");
		}

		g_task_health.vehicle_tx_alive_tick = osKernelGetTickCount();
		osDelay(100);
	}
}

void StartCanRxPrintTask(void *argument) {
	char buf[160];
	EngineStatus_t local_engine;
	VehicleDynamics_t local_vehicle;

	for (;;) {
		if (osMutexAcquire(canDataMutexHandle, 50) == osOK) {
			local_engine  = g_engine_status;
			local_vehicle = g_vehicle_dynamics;
			osMutexRelease(canDataMutexHandle);

			snprintf(buf, sizeof(buf),
					"RPM=%u CoolantTemp=%dC Running=%u | Speed=%u.%ukmh Brake=%u(%ubar) | RxCount=%lu\r\n",
					local_engine.rpm, local_engine.coolant_temp_c,
					local_engine.engine_running,
					(unsigned)(local_vehicle.vehicle_speed_kph_x10 / 10),
					(unsigned)(local_vehicle.vehicle_speed_kph_x10 % 10),
					local_vehicle.brake_pedal_pressed,
					local_vehicle.brake_pressure_bar,
					(unsigned long) (local_engine.rx_count + local_vehicle.rx_count));
			UART_Print(buf);
		}
		osDelay(500);
	}
}

void StartButtonTask(void *argument) {
	for (;;) {
		if (osSemaphoreAcquire(buttonSemHandle, osWaitForever) == osOK) {
			UART_Print(
					">>> Boton presionado -- forzando reporte de estado <<<\r\n");
		}
	}
}

void StartWatchdogTask(void *argument) {
	const uint32_t MAX_SILENCE_MS = 1000;

	uint32_t boot_tick = osKernelGetTickCount();
	g_task_health.led_alive_tick        = boot_tick;
	g_task_health.can_tx_alive_tick     = boot_tick;
	g_task_health.can_rx_alive_tick     = boot_tick;
	g_task_health.vehicle_tx_alive_tick = boot_tick;

	for (;;) {
		uint32_t now = osKernelGetTickCount();
		uint8_t all_healthy = 1;

		if ((now - g_task_health.led_alive_tick) > MAX_SILENCE_MS)
			all_healthy = 0;
		if ((now - g_task_health.can_tx_alive_tick) > MAX_SILENCE_MS)
			all_healthy = 0;
		if ((now - g_task_health.can_rx_alive_tick) > MAX_SILENCE_MS)
			all_healthy = 0;
		if ((now - g_task_health.vehicle_tx_alive_tick) > MAX_SILENCE_MS)
			all_healthy = 0;

		if (all_healthy) {
			HAL_IWDG_Refresh(&hiwdg);
		} else {
		    char diag[130];
		    snprintf(diag, sizeof(diag),
		             "CRITICAL: LED=%lu TX=%lu RX=%lu VEHTX=%lu now=%lu\r\n",
		             (unsigned long)g_task_health.led_alive_tick,
		             (unsigned long)g_task_health.can_tx_alive_tick,
		             (unsigned long)g_task_health.can_rx_alive_tick,
		             (unsigned long)g_task_health.vehicle_tx_alive_tick,
		             (unsigned long)now);
		    UART_Print(diag);
		}

		osDelay(200);
	}
}
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    UART_Print("STACK OVERFLOW EN: ");
    UART_Print(pcTaskName);
    UART_Print("\r\n");
    __disable_irq();
    while(1) { __NOP(); }
}

void vApplicationMallocFailedHook(void)
{
    UART_Print("MALLOC FAILED - HEAP AGOTADO\r\n");
    __disable_irq();
    while(1) { __NOP(); }
}
/* USER CODE END Application */

