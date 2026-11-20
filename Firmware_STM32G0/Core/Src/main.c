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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- SERVO Positions (Adjust these to match your physical gripper limits) ---
#define PWM_POS_OPEN_US         1000    // 1.0 ms pulse = Open
#define PWM_POS_CLOSE_US        2000    // 2.0 ms pulse = Closed
#define PWM_SOFT_STEP_US        10      // Step size (us) per 20ms during Soft Close

#define ST3215_ID               1       // Default Feetech Servo ID (0xFE is broadcast if unsure)
#define ST3215_POS_OPEN         1000    // Range is 0 - 4095
#define ST3215_POS_CLOSE        3000    // Range is 0 - 4095

// --- CURRENT LIMITS (in milliamps) ---
#define SOFT_GRIP_LIMIT_MA      450     // Stop closing in Soft Mode when current hits 450mA
#define HARD_OVERLOAD_LIMIT_MA  1200    // Protect 1.5A aviation cable if hard stalled
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef enum {
    SERVO_TYPE_PWM = 0,     // MG996R (5V PWM on PA6)
    SERVO_TYPE_UART = 1     // Feetech ST3215 (12V Half-Duplex UART on PB6)
} ServoType_t;

typedef enum {
    CMD_HARD_OPEN  = 0,     // D2=0, D1=0
    CMD_HARD_CLOSE = 1,     // D2=0, D1=1
    CMD_SOFT_OPEN  = 2,     // D2=1, D1=0 (Free-wheel / Backdrivable)
    CMD_SOFT_CLOSE = 3      // D2=1, D1=1 (Current-limited gentle grip)
} GripperCmd_t;

// >>> CHANGE THIS GLOBAL VARIABLE TO SWITCH BETWEEN PWM AND UART SERVO <<<
volatile ServoType_t g_active_servo = SERVO_TYPE_PWM;

GripperCmd_t g_current_cmd = CMD_SOFT_OPEN;
GripperCmd_t g_last_cmd    = 0xFF; // Force update on boot

uint32_t g_servo_current_mA = 0;
uint16_t g_pwm_current_us   = PWM_POS_OPEN_US;
bool     g_soft_grip_locked = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ======================= HELPER FUNCTIONS ====================================
// PA7 -> ADC1_IN7, Shunt = 50mOhm, Amplifier Gain = 20V/V -> 1.0V/A
uint32_t Read_Servo_Current_mA(void) {
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    // 3300mV full scale / 4095 counts -> mV equals mA directly
    uint32_t current_mA = (raw * 3300UL) / 4095UL;
    // low-pass filter
    // Integer equivalent of (0.75 * old) + (0.25 * new) using fast bit-shift (/4)
    g_servo_current_mA = ((3 * g_servo_current_mA) + current_mA) >> 2;
  }
  HAL_ADC_Stop(&hadc1);
  return g_servo_current_mA;
}

// Send a register write packet to Feetech ST3215 over Half-Duplex USART1
void ST3215_WriteReg(uint8_t id, uint8_t reg, const uint8_t *data, uint8_t len) {
  uint8_t pkt[16];
  uint8_t msg_len = len + 2; // instruciton + parameter + checksum
  uint8_t checksum = id + msg_len + 0x03 + reg; // 0x03 = INST_WRITE

  pkt[0] = 0xFF;
  pkt[1] = 0xFF;
  pkt[2] = id;
  pkt[3] = msg_len;
  pkt[4] = 0x03; // write instruction
  pkt[5] = reg;

  for (uint8_t i = 0; i < len; i++) {
    pkt[6 + i] = data[i];
    checksum += data[i];
  }
  pkt[6 + len]  = ~checksum; // bitwise NOT checksum

  HAL_HalfDuplex_EnableTransmitter(&huart1);
  HAL_UART_Transmit(&huart1, pkt, 7 + len, 10);
  HAL_HalfDuplex_EnableReceiver(&huart1);
  HAL_Delay(2);
}

// enable (1) or disable (0) ST3215 holding torque (Reg 0x28)
void ST3215_SetTorqueEnable(uint8_t id, uint8_t enable) {
  ST3215_WriteReg(id, 0x28, &enable, 1);
}

// Set ST3215 Torque Limit (Reg 0x30, 0 - 1000 where 1000 = 100%)
void ST3215_SetTorqueLimit(uint8_t id, uint16_t limit) {
  uint8_t data[2] = { (uint8_t)(limit & 0XFF), (uint8_t)((limit >> 8) & 0xFF) };
  ST3215_WriteReg(id, 0x30, data, 2);
}

void ST3215_Move(uint8_t id, uint16_t pos, uint16_t speed) {
  uint8_t data[6] = {
    (uint8_t)(pos & 0xFF),   (uint8_t)((pos >> 8) & 0xFF),   // Goal Position (0x2A)
    0x00,                    0x00,                           // Goal Time (0x2C) = 0
    (uint8_t)(speed & 0xFF), (uint8_t)((speed >> 8) & 0xFF)  // Goal Speed (0x2E)
  };
  ST3215_WriteReg(id, 0x2A, data, 6);
}
// ============================= HELPER FUNCTIONS ================================

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // ================ INITIALIZATION ===================
  // Calibrate ADC on startup for accurate current sensing
  HAL_ADCEx_Calibration_Start(&hadc1);
  // start PWM timer on PA6 (TIM3_CH1) with pulse = 0 (idle/free on boot)
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  uint32_t last_control_tick = 0;
  uint32_t last_heartbeat_tick = 0;
  // ================ INITIALIZATION ===================

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // ============== CONTROL LOOP =====================
    uint32_t now = HAL_GetTick();

    // 50Hz (20ms) control and current sensing task
    if (now - last_control_tick >= 20) {
      last_control_tick = now;
      
      // Read Dobot Inputs (Active-HIGH via optocouplers)
      // (D1_O -> PA1), (D2_O -> PA2)
      uint8_t d1_output; 
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {
        d1_output = 1;
      } else {
        d1_output = 0;
      }
      uint8_t d2_output;
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) {
        d2_output = 1;
      } else {
        d2_output = 0;
      }
      g_current_cmd = (GripperCmd_t)((d2_output << 1) | d1_output);

      // read servo current in mA
      uint32_t current_mA = Read_Servo_Current_mA();

      // execute servo logic
      bool state_changed = (g_current_cmd != g_last_cmd);

      if (g_active_servo == SERVO_TYPE_PWM) { // PWM SERVO
        switch (g_current_cmd) {
          case CMD_HARD_OPEN:
            g_pwm_current_us = PWM_POS_OPEN_US;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, g_pwm_current_us);
            break;
          case CMD_HARD_CLOSE:
            // safety cut-back if current exceeds 1.2A (cable has 1.5A rating)
            if (current_mA < HARD_OVERLOAD_LIMIT_MA) {
              g_pwm_current_us = PWM_POS_CLOSE_US;
            } else {
              // Overload detected (>1.2A)! Relax pulse or disable PWM to save servo/cable
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
              break;
            }
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, g_pwm_current_us);
            break;
          case CMD_SOFT_OPEN:
            // setting CCR1 to 0 stops PWM pulses -> servo becomes freely moveable by hand
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            g_pwm_current_us = PWM_POS_OPEN_US;
            break;
          case CMD_SOFT_CLOSE:
            if (state_changed) {
              g_soft_grip_locked = false; // reset lock on fresh command
            }
            // step slowly towards closed until current hits SOFT_GRIP_LIMIT_MA (450mA)
            if (!g_soft_grip_locked) {
              if (current_mA >= SOFT_GRIP_LIMIT_MA) {
                // object felt so back off slightly (15us) to hold gently without stalling
                if (g_pwm_current_us > (PWM_POS_OPEN_US + 15)) {
                  g_pwm_current_us -= 15;
                }
                g_soft_grip_locked = true;
              } else if (g_pwm_current_us < PWM_POS_CLOSE_US) {
                g_pwm_current_us += PWM_SOFT_STEP_US;
              }
            }
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, g_pwm_current_us);
            break;
        }
      } 
      else { // UART SERVO
        if (state_changed) {
          switch (g_current_cmd) {
            case CMD_HARD_OPEN:
              ST3215_SetTorqueLimit(ST3215_ID, 800); // 80% torque
              ST3215_SetTorqueEnable(ST3215_ID, 1);
              ST3215_Move(ST3215_ID, ST3215_POS_OPEN, 1500);
              break;
            case CMD_HARD_CLOSE:
              ST3215_SetTorqueLimit(ST3215_ID, 800); // 80% torque
              ST3215_SetTorqueEnable(ST3215_ID, 1);
              ST3215_Move(ST3215_ID, ST3215_POS_CLOSE, 1500);
              break;
            case CMD_SOFT_OPEN:
              // disable torque so users can freely move the gripper by hand
              ST3215_SetTorqueEnable(ST3215_ID, 0);
              break;
            case CMD_SOFT_CLOSE:
              // set internal torque limit to 25% for a gentle compliant grip
              ST3215_SetTorqueLimit(ST3215_ID, 250);
              ST3215_SetTorqueEnable(ST3215_ID, 1);
              ST3215_Move(ST3215_ID, ST3215_POS_CLOSE, 600); // slower speed 
              break;
          }
        }
        // extra hardware current safety check for uart servo
        if (current_mA > HARD_OVERLOAD_LIMIT_MA) {
          ST3215_SetTorqueLimit(ST3215_ID, 200);
        }
      }
      g_last_cmd = g_current_cmd;
    }

    // 1Hz (500ms) HEARTBEAT LED Task (PB8)
    if (now - last_heartbeat_tick >= 500) {
      last_heartbeat_tick = now;
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
    }

    // ============== CONTROL LOOP =====================
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D1_IN_Pin|D2_IN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HEARTBEAT_Pin */
  GPIO_InitStruct.Pin = HEARTBEAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HEARTBEAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_OUT_Pin D2_OUT_Pin */
  GPIO_InitStruct.Pin = D1_OUT_Pin|D2_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_IN_Pin D2_IN_Pin */
  GPIO_InitStruct.Pin = D1_IN_Pin|D2_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
