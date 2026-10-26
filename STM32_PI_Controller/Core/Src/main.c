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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static uint8_t uart_rx_byte;
static char uart_rx_buffer[64];
//static char command_buffer[64];

static volatile uint16_t uart_rx_index = 0;
static volatile uint8_t uart_line_ready = 0;

volatile float speed_setpoint_rpm = 0.0f;
volatile float speed_measured_rpm = 0.0f;
volatile float control_output = 0.0f;
static volatile uint32_t timer_period_cycles = 0;

static float kp = 0.08f;
static float ki = 5.0f;
static float integral = 0.0f;

static volatile uint8_t control_tick = 0;
volatile uint32_t tim2_interrupt_count = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


static float PI_Controller(float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    // Proportionalanteil des Reglers
    float proportional = kp * error;

    // Unbegrenzte Stellgröße vor Anwendung der Ausgangsgrenzen
    float u_unconstrained = proportional + integral;

    // Stellgrößenbegrenzung auf den Bereich 0.0 bis 1.0
    float u_constrained = u_unconstrained;
    if (u_constrained > 1.0f) {
        u_constrained = 1.0f;
    } else if (u_constrained < 0.0f) {
        u_constrained = 0.0f;
    }

    // Back-Calculation korrigiert den Integrator anhand der Differenz
    // zwischen begrenzter und unbegrenzter Stellgröße.
    // tracking_gain entspricht 1/Tt, wobei Tt die Tracking-Zeitkonstante ist.
    float tracking_gain = 5.0f;
    float saturation_error = u_constrained - u_unconstrained;

    // Integrator mit Fehleranteil und Back-Calculation-Korrektur aktualisieren
    integral += (ki * error + tracking_gain * saturation_error) * dt;

    // Integrator auf den Bereich 0.0 bis 1.0 begrenzen
    if (integral > 1.0f) integral = 1.0f;
    if (integral < 0.0f) integral = 0.0f;

    return u_constrained;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  /* Startup message */
  char startup_message[] =
      "STM32 Controller gestartet\r\n";

  HAL_UART_Transmit(
      &huart2,
      (uint8_t*)startup_message,
      strlen(startup_message),
      HAL_MAX_DELAY
  );

  /* Start UART reception */
  HAL_UART_Receive_IT(
      &huart2,
      (uint8_t *)&uart_rx_byte,
      1
  );

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  HAL_StatusTypeDef timer_status = HAL_TIM_Base_Start_IT(&htim2);

  if (timer_status != HAL_OK)
  {
      Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
      if (uart_line_ready)
      {
          char response[96];
          char local_buffer[96];
          uint8_t send_response = 0;

          // Empfangene UART-Zeile im kritischen Abschnitt lokal kopieren.
          __disable_irq();
          strcpy(local_buffer, uart_rx_buffer);
          uart_line_ready = 0;
          __enable_irq();

          if (sscanf(local_buffer, "SET_SPEED,%f", &speed_setpoint_rpm) == 1)
          {
              snprintf(response, sizeof(response),
                       "OK,SET_SPEED,%.1f\r\n",
                       speed_setpoint_rpm);
              send_response = 1;
          }
          else
          {
              float new_kp;
              float new_ki;

              if (sscanf(local_buffer, "SET_PI,%f,%f", &new_kp, &new_ki) == 2)
              {
                  if (new_kp >= 0.0f && new_kp <= 1000.0f &&
                      new_ki >= 0.0f && new_ki <= 1000.0f)
                  {
                      __disable_irq();
                      kp = new_kp;
                      ki = new_ki;
                      integral = 0.0f;
                      __enable_irq();

                      snprintf(response, sizeof(response),
                               "OK,SET_PI,%.6f,%.6f\r\n",
                               (double)kp,
                               (double)ki);
                  }
                  else
                  {
                      snprintf(response, sizeof(response),
                               "ERR,SET_PI_RANGE\r\n");
                  }
                  send_response = 1;
              }
          }

          if (!send_response &&
              sscanf(local_buffer, "MEAS,%f", &speed_measured_rpm) == 1)
          {
              /* The measurement is read by the next timer-driven controller update. */
          }
          else if (!send_response)
          {
              snprintf(response, sizeof(response),
                       "ERR,UNKNOWN_COMMAND\r\n");
              send_response = 1;
          }

          if (send_response)
          {
              HAL_UART_Transmit(
                  &huart2,
                  (uint8_t *)response,
                  (uint16_t)strlen(response),
                  100
              );
          }

          HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
      }

      uint8_t tick_ready;
      float out_copy = 0.0f;
      uint32_t cnt_copy = 0;
      uint32_t period_copy = 0;

      __disable_irq();
      tick_ready = control_tick;
      if (tick_ready)
      {
          control_tick = 0;
          out_copy = control_output;
          cnt_copy = tim2_interrupt_count;
          period_copy = timer_period_cycles;
      }
      __enable_irq();

      if (tick_ready)
      {
          char response[96];
          snprintf(response, sizeof(response),
                   "U,%.4f,TIM2,%lu,%lu\r\n",
                   (double)out_copy,
                   (unsigned long)cnt_copy,
                   (unsigned long)period_copy);
          HAL_UART_Transmit(
              &huart2,
              (uint8_t *)response,
              (uint16_t)strlen(response),
              100
          );
      }
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t previous_was_cr = 0;

    if (huart->Instance == USART2)
    {
        if (uart_rx_byte == '\r' || uart_rx_byte == '\n')
        {
            if (uart_rx_byte == '\n' && previous_was_cr)
            {
                previous_was_cr = 0;
            }
            else
            {
                previous_was_cr = (uart_rx_byte == '\r');

                if (uart_rx_index > 0 && uart_line_ready == 0)
                {
                    uart_rx_buffer[uart_rx_index] = '\0';
                    uart_line_ready = 1;
                }

                uart_rx_index = 0;
            }
        }
        else
        {
            previous_was_cr = 0;

            if (uart_line_ready == 0)
            {
                if (uart_rx_index < sizeof(uart_rx_buffer) - 1)
                {
                    uart_rx_buffer[uart_rx_index++] =
                        (char)uart_rx_byte;
                }
                else
                {
                    uart_rx_index = 0;
                }
            }
        }

        /* Re-arm reception after the completed line has been consumed. */
        if (uart_line_ready == 0)
        {
            HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
        }
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        static uint32_t previous_cycle_count = 0;
        static uint8_t first_tick = 1;
        uint32_t current_cycle_count = DWT->CYCCNT;

        if (first_tick)
        {
            first_tick = 0;
        }
        else
        {
            timer_period_cycles = current_cycle_count - previous_cycle_count;
        }
        previous_cycle_count = current_cycle_count;

        control_output = PI_Controller(
            speed_setpoint_rpm,
            speed_measured_rpm,
            0.01f
        );

        tim2_interrupt_count++;
        control_tick = 1;

    }
}

/* USER CODE END 4 */

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
