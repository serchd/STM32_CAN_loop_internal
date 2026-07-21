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
#include "dtc_synth_gen.h"
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
	uint8_t  brake_pedal_pct;
	uint8_t  brake_override_flag;
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
osThreadId_t dtcSynthGenTaskHandle;
osThreadId_t canRxPrintTaskHandle;
osThreadId_t buttonTaskHandle;
osThreadId_t watchdogTaskHandle;

const osThreadAttr_t ledTask_attributes = { .name = "LedTask", .stack_size = 256
		* 4, .priority = (osPriority_t) osPriorityLow, };
const osThreadAttr_t dtcSynthGenTask_attributes = { .name = "DtcSynthGenTask",
		.stack_size = 1024 * 4, .priority = (osPriority_t) osPriorityNormal, };
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

	/* NOTA: StartCanTxTask/StartVehicleTxTask (rampa simple sin fallas) quedan
	 * REEMPLAZADAS por StartDtcSynthGenTask, que transmite las mismas
	 * EngineStatus/VehicleSpeed/BrakeStatus (0x100/0x101/0x102) MAS
	 * PowertrainHealth/ChassisHealth/ThrottleSubsystem, ciclando los 14
	 * escenarios de fallas del generador multi-DTC. No corras ambas juntas:
	 * competirian por los mismos IDs. */
	dtcSynthGenTaskHandle = osThreadNew(StartDtcSynthGenTask, NULL, &dtcSynthGenTask_attributes);
	if (dtcSynthGenTaskHandle == NULL) {
	    UART_Print("ERROR: no se pudo crear DtcSynthGenTask\r\n");
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
					"RPM=%u CoolantTemp=%dC Running=%u | Speed=%u.%ukmh BrakePedal=%u%% Override=%u | RxCount=%lu\r\n",
					local_engine.rpm, local_engine.coolant_temp_c,
					local_engine.engine_running,
					(unsigned)(local_vehicle.vehicle_speed_kph_x10 / 10),
					(unsigned)(local_vehicle.vehicle_speed_kph_x10 % 10),
					local_vehicle.brake_pedal_pct,
					local_vehicle.brake_override_flag,
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

