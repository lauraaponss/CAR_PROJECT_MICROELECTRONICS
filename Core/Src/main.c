/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Ultrasonic sensor pins
#define TRIG_PIN GPIO_PIN_1
#define ECHO_PIN GPIO_PIN_0
#define TRIG_PORT GPIOA
#define ECHO_PORT GPIOA

// Buzzer pin
#define BUZZER_PIN GPIO_PIN_0
#define BUZZER_PORT GPIOB

// Motor driver pins
#define MOTOR_RIGHT_FWD_PIN GPIO_PIN_6  // IN1 - TIM3_CH1
#define MOTOR_RIGHT_BWD_PIN GPIO_PIN_7  // IN2
#define MOTOR_LEFT_FWD_PIN  GPIO_PIN_7  // IN3 - TIM3_CH2
#define MOTOR_LEFT_BWD_PIN  GPIO_PIN_4  // IN4
#define MOTOR_RIGHT_FWD_PORT GPIOA
#define MOTOR_RIGHT_BWD_PORT GPIOA
#define MOTOR_LEFT_FWD_PORT  GPIOC
#define MOTOR_LEFT_BWD_PORT  GPIOB

// Motor speed defines
#define MOTOR_FULL_SPEED  999   // Full speed PWM value
#define MOTOR_SLOW_SPEED  200   // Slow speed PWM value
#define MOTOR_STOP        0     // Stop PWM value

#define ADC_THRESHOLD 2482  // 2V threshold (2.0/3.3 * 4095)
#define UART_BUFFER_SIZE 1  // Size of UART receive buffer
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;
uint32_t Distance = 0;
uint32_t AdcValue = 0;
uint8_t uartBuffer[UART_BUFFER_SIZE];  // Buffer for UART reception
uint8_t manualMode = 0;  // 0 for automatic, 1 for manual mode
uint8_t rxData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void HCSR04_Start(void);
void motors_forward(uint32_t speed);
void motors_backward(uint32_t speed);
void motors_turn_left(uint32_t speed);
void motors_turn_right(uint32_t speed);
void motors_stop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motors_forward(uint32_t speed)
{
    // Set right motor speed using PWM
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
    HAL_GPIO_WritePin(MOTOR_RIGHT_BWD_PORT, MOTOR_RIGHT_BWD_PIN, GPIO_PIN_RESET);

    // Set left motor speed using PWM
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
    HAL_GPIO_WritePin(MOTOR_LEFT_BWD_PORT, MOTOR_LEFT_BWD_PIN, GPIO_PIN_RESET);
}

void motors_backward(uint32_t speed)
{
    // Right motor backward
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_GPIO_WritePin(MOTOR_RIGHT_BWD_PORT, MOTOR_RIGHT_BWD_PIN, GPIO_PIN_SET);

    // Left motor backward
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(MOTOR_LEFT_BWD_PORT, MOTOR_LEFT_BWD_PIN, GPIO_PIN_SET);
}

void motors_turn_right(uint32_t speed)
{
    // Right motor backward
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_GPIO_WritePin(MOTOR_RIGHT_BWD_PORT, MOTOR_RIGHT_BWD_PIN, GPIO_PIN_SET);

    // Left motor forward
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
    HAL_GPIO_WritePin(MOTOR_LEFT_BWD_PORT, MOTOR_LEFT_BWD_PIN, GPIO_PIN_RESET);
}

void motors_turn_left(uint32_t speed)
{
    // Right motor forward
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
    HAL_GPIO_WritePin(MOTOR_RIGHT_BWD_PORT, MOTOR_RIGHT_BWD_PIN, GPIO_PIN_RESET);

    // Left motor backward
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(MOTOR_LEFT_BWD_PORT, MOTOR_LEFT_BWD_PIN, GPIO_PIN_SET);
}

void motors_stop(void)
{
    // Stop right motor
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_STOP);
    HAL_GPIO_WritePin(MOTOR_RIGHT_BWD_PORT, MOTOR_RIGHT_BWD_PIN, GPIO_PIN_RESET);

    // Stop left motor
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_STOP);
    HAL_GPIO_WritePin(MOTOR_LEFT_BWD_PORT, MOTOR_LEFT_BWD_PIN, GPIO_PIN_RESET);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)  // 300ms timer for trigger
    {
        // Start ADC conversion and read value
        HAL_ADC_Start(&hadc);
        if(HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK)
        {
            AdcValue = HAL_ADC_GetValue(&hadc);
            HAL_ADC_Stop(&hadc);

            if(AdcValue >= ADC_THRESHOLD)
            {
                if(!manualMode)  // Only trigger ultrasonic sensor in automatic mode
                {
                    HCSR04_Start();
                }
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);  // Buzzer off when voltage < 2V
                HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);  // Stop sensor when voltage is low
                HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
                motors_stop();  // Stop motors when voltage is low
            }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Send acknowledgment back to confirm command received
    	uint8_t ack[] = "OK\r\n";
        HAL_UART_Transmit(&huart1, ack, sizeof(ack)-1, 100);

        switch(rxData)
        {
            case 'M': // Manual mode
                if(!manualMode) {  // Only if not already in manual mode
                    manualMode = 1;
                    motors_stop();
                    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
                    HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
                }
                break;

            case 'A': // Automatic mode
                if(manualMode) {  // Only if not already in automatic mode
                    manualMode = 0;
                    motors_stop();
                    if(AdcValue >= ADC_THRESHOLD)
                    {
                        HCSR04_Start();
                    }
                }
                break;

            case 'F': // Forward
                if(manualMode && AdcValue >= ADC_THRESHOLD)
                {
                    motors_forward(MOTOR_FULL_SPEED);
                }
                break;

            case 'B': // Backward
                if(manualMode && AdcValue >= ADC_THRESHOLD)
                {
                    motors_backward(MOTOR_SLOW_SPEED);
                }
                break;

            case 'L': // Left
                if(manualMode && AdcValue >= ADC_THRESHOLD)
                {
                    motors_turn_left(MOTOR_FULL_SPEED);
                }
                break;

            case 'R': // Right
                if(manualMode && AdcValue >= ADC_THRESHOLD)
                {
                    motors_turn_right(MOTOR_FULL_SPEED);
                }
                break;

            case 'S': // Stop
                if(manualMode)
                {
                    motors_stop();
                }
                break;
        }
    }

    // Immediately restart reception for next command
    HAL_UART_Receive_IT(&huart1, &rxData, 1);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;  // Only process TIM2 events
    if (manualMode) return;  // Don't process if in manual mode

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (Is_First_Captured == 0)
        {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Is_First_Captured = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            if (IC_Val2 > IC_Val1)
                Difference = IC_Val2 - IC_Val1;
            else if (IC_Val2 < IC_Val1)
                Difference = ((65535 - IC_Val1) + IC_Val2) + 1;

            Distance = Difference / 58;

            // Update Buzzer and Motors based on distance only if ADC value is above threshold
            if(AdcValue >= ADC_THRESHOLD)
            {
                if(Distance >= 30) {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);  // Buzzer off
                    motors_forward(MOTOR_FULL_SPEED);  // Full speed forward
                }
                else if(Distance >= 10) {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 500);   // Buzzer medium
                    motors_forward(MOTOR_SLOW_SPEED);  // Slow speed forward
                }
                else {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);     // Buzzer full
                    motors_stop();     // Stop motors when too close
                }
            }
            else {
                motors_stop();  // Stop motors when ADC below threshold
            }

            Is_First_Captured = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

void HCSR04_Start(void)
{
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);    // Start input capture
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);    // Start output compare
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
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // Start PWM for right motor
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // Start PWM for left motor
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  // Start PWM for buzzer
  HAL_TIM_Base_Start_IT(&htim4);             // Enable update interrupt for trigger timing

  // Send initial connection message
  uint8_t connectMsg[] = "Connection Established\r\n";
  HAL_UART_Transmit(&huart1, connectMsg, sizeof(connectMsg)-1, 1000);

  // Start receiving Bluetooth commands
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxData, 1);

  // Initial ADC read and motor setup
  HAL_ADC_Start(&hadc);
  if(HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK)
  {
      AdcValue = HAL_ADC_GetValue(&hadc);
      if(AdcValue >= ADC_THRESHOLD)
      {
          motors_forward(MOTOR_FULL_SPEED);  // Start at full speed if voltage is above threshold
          HCSR04_Start();
      }
      else
      {
          motors_stop();     // Ensure motors are stopped if voltage is low
      }
  }
  HAL_ADC_Stop(&hadc);
  /* USER CODE END 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 500;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 31999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 299;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
        __disable_irq();
        while (1)
        {
        }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
