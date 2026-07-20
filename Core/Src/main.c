/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t rpm;
    int8_t   coolant_temp_c;
    uint8_t  engine_running;
    uint32_t rx_count;
} EngineStatus_t;

typedef struct {
    volatile uint32_t led_alive_tick;
    volatile uint32_t can_tx_alive_tick;
    volatile uint32_t can_rx_alive_tick;
} TaskHealth_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum {
    RESET_REASON_UNKNOWN = 0,
    RESET_REASON_POWER_ON,
    RESET_REASON_PIN,
    RESET_REASON_SOFTWARE,
    RESET_REASON_IWDG,
    RESET_REASON_WWDG,
    RESET_REASON_LOW_POWER
} ResetReason_t;

volatile ResetReason_t g_reset_reason = RESET_REASON_UNKNOWN;
/* Estas variables las CREA freertos.c -- aqui solo las referenciamos */
extern EngineStatus_t g_engine_status;
extern TaskHealth_t   g_task_health;
extern osMutexId_t    canDataMutexHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

static void CAN_Filter_Config(void);
static void Error_Handler_Ctx(const char *where);   // <-- AGREGA ESTA LÍNEA
extern void UART_Print(const char *msg);   /* definida en freertos.c */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static ResetReason_t Reset_Reason_Get(void)
{
    ResetReason_t reason = RESET_REASON_UNKNOWN;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        reason = RESET_REASON_IWDG;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
        reason = RESET_REASON_WWDG;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
        reason = RESET_REASON_SOFTWARE;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
        reason = RESET_REASON_POWER_ON;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
        reason = RESET_REASON_PIN;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
        reason = RESET_REASON_LOW_POWER;
    }

    __HAL_RCC_CLEAR_RESET_FLAGS();   /* limpia las banderas para el próximo reset */

    return reason;
}
static void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef filterConfig;

    filterConfig.FilterBank = 0;
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    filterConfig.FilterIdHigh = 0x0000;
    filterConfig.FilterIdLow = 0x0000;
    filterConfig.FilterMaskIdHigh = 0x0000;
    filterConfig.FilterMaskIdLow = 0x0000;
    filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    filterConfig.FilterActivation = ENABLE;
    filterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK) {
        Error_Handler_Ctx("HAL_CAN_ConfigFilter");
    }
}
static void Error_Handler_Ctx(const char *where)
{
    char diag[100];
    snprintf(diag, sizeof(diag), "ERROR_HANDLER desde: %s | CAN ErrorCode=0x%08lX State=%d\r\n",
             where, (unsigned long)hcan1.ErrorCode, (int)hcan1.State);
    HAL_UART_Transmit(&huart2, (uint8_t*)diag, strlen(diag), 100);
    Error_Handler();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */
  g_reset_reason = Reset_Reason_Get();   // <-- breakpoint aquí
  /* USER CODE END Init */
  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_IWDG_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  switch (g_reset_reason) {
      case RESET_REASON_IWDG:
          HAL_UART_Transmit(&huart2, (uint8_t*)"RESET: IWDG\r\n", 13, 100);
          break;
      case RESET_REASON_SOFTWARE:
          HAL_UART_Transmit(&huart2, (uint8_t*)"RESET: SOFTWARE\r\n", 18, 100);
          break;
      case RESET_REASON_POWER_ON:
          HAL_UART_Transmit(&huart2, (uint8_t*)"RESET: POWER ON\r\n", 18, 100);
          break;
      case RESET_REASON_PIN:
          HAL_UART_Transmit(&huart2, (uint8_t*)"RESET: PIN/NRST\r\n", 18, 100);
          break;
      default:
          HAL_UART_Transmit(&huart2, (uint8_t*)"RESET: OTRO/DESCONOCIDO\r\n", 25, 100);
          break;
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)"UART OK\r\n", 9, 100);

  CAN_Filter_Config();

  /* Forzar salida de modo Sleep y esperar confirmacion real (SLAK) */
  CLEAR_BIT(hcan1.Instance->MCR, CAN_MCR_SLEEP);

  uint32_t sleep_wait_start = HAL_GetTick();
  while ((hcan1.Instance->MSR & CAN_MSR_SLAK) != 0U) {
      if ((HAL_GetTick() - sleep_wait_start) > 100) {
          char dbg3[100];
          snprintf(dbg3, sizeof(dbg3), "DBG: SLAK nunca bajo! MCR=0x%08lX MSR=0x%08lX\r\n",
                   (unsigned long)hcan1.Instance->MCR, (unsigned long)hcan1.Instance->MSR);
          HAL_UART_Transmit(&huart2, (uint8_t*)dbg3, strlen(dbg3), 100);
          break;
      }
  }

  char dbg4[80];
  snprintf(dbg4, sizeof(dbg4), "DBG: Sleep exit tomo %lu ms, MSR=0x%08lX\r\n",
           (unsigned long)(HAL_GetTick() - sleep_wait_start), (unsigned long)hcan1.Instance->MSR);
  HAL_UART_Transmit(&huart2, (uint8_t*)dbg4, strlen(dbg4), 100);

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      Error_Handler_Ctx("HAL_CAN_Start");
  }

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        return;
    }


    if (rxHeader.StdId == 0x100) {
        //osMutexAcquire(canDataMutexHandle, 0);
    	UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

        g_engine_status.rpm            = (rxData[0] << 8) | rxData[1];
        g_engine_status.coolant_temp_c = (int8_t)rxData[2] - 40;
        g_engine_status.engine_running = rxData[3] & 0x01;
        g_engine_status.rx_count++;

        //osMutexRelease(canDataMutexHandle);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

        g_task_health.can_rx_alive_tick = osKernelGetTickCount();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {
        extern osSemaphoreId_t buttonSemHandle;  /* definida en freertos.c */
        osSemaphoreRelease(buttonSemHandle);
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
