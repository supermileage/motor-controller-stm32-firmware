/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  PHASE_OFF = 0,
  PHASE_PWM_HIGH,
  PHASE_LOW_ON
} phase_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLOCKWISE        1                    // 1 = clockwise, 0 = counterclockwise (LOOKING INTO ROTOR)
#define DEBUG_LOG        0                    // 1 = enable debug telemetry prints, 0 = disable
#define MAX_DUTY         ((2047 * 95) / 100)  // max 95% duty to protect bootstrap
#define MIN_HALL_DT_US   200                  // 2 / (MIN_HALL_DT_US) = max rpm (10K RPM rn, actual motor max is 8112 RPM)
#define ADC_BUF_LEN      16                   // rolling average
#define MOTOR_POLE_PAIRS 5                    // pole pairs
#define RPM_CONST        (10000000UL / MOTOR_POLE_PAIRS)
#define RPM_TIMEOUT_US   100000               // zero rpm estimate if no hall edge for 100 ms
#define HALL_COMMUTATION_STABLE_RPM 300
#define ADC_DEADBAND     10                   // below this throttle is treated as zero
#define TARGET_DUTY_RISE_STEP 2               // max increase in target duty request per 1 kHz tick
#define TARGET_DUTY_FALL_STEP 1               // max decrease in target duty per 1 kHz tick
#define DUTY_RISE_STEP   1                    // fixed applied-duty rise step
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint32_t cycles_per_us;
uint16_t adc_buf[ADC_BUF_LEN];
volatile uint16_t current_duty = 0;
volatile uint16_t target_duty = 0;
volatile uint16_t shaped_target_duty = 0;
volatile uint8_t hallState;
volatile uint32_t lastHall;
volatile uint16_t motor_rpm = 0;
static uint8_t duty_rise_divider = 0;
#if DEBUG_LOG
volatile uint16_t dbg_adc_raw = 0;
volatile uint16_t dbg_adc_avg = 0;
volatile uint16_t dbg_adc_scaled = 0;
volatile uint16_t dbg_target_duty = 0;
volatile uint16_t dbg_current_duty = 0;
volatile uint16_t dbg_rpm = 0;
volatile uint8_t dbg_ready = 0;
static uint8_t debug_sample_divider = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static inline uint16_t filtered_adc_to_duty(void);
static inline void DWT_Init(void);
static inline uint32_t micros(void);
static inline int8_t hall_index(uint8_t hall);
static inline uint8_t hall_valid_transition(uint8_t oldHall, uint8_t newHall);
static inline uint8_t readHall(void);
static inline void commutate(uint8_t hallState, uint16_t duty);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
  return ch;
}
// 1 kHz control loop
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6)
  {
    uint32_t now = micros();
    uint16_t raw_target_duty;
    uint16_t rpm;

    if ((now - lastHall) > RPM_TIMEOUT_US) motor_rpm = 0;
    raw_target_duty = filtered_adc_to_duty();

    if (raw_target_duty <= shaped_target_duty)
    {
      shaped_target_duty = raw_target_duty;
    }
    else
    {
      uint16_t next = shaped_target_duty + TARGET_DUTY_RISE_STEP;
      shaped_target_duty = (next > raw_target_duty) ? raw_target_duty : next;
    }

    target_duty = shaped_target_duty;
    rpm = motor_rpm;

    if (target_duty < current_duty)
    {
      uint16_t step = TARGET_DUTY_FALL_STEP;

      if (++duty_rise_divider >= 4)
      {
        duty_rise_divider = 0;

        if (current_duty > step)
        {
          uint16_t next = current_duty - step;
          current_duty = (next < target_duty) ? target_duty : next;
        }
        else
        {
          current_duty = target_duty;
        }
      }
    }
    else
    {
      if (target_duty > current_duty && ++duty_rise_divider >= 4)
      {
        duty_rise_divider = 0;

        uint16_t step = DUTY_RISE_STEP;

        uint16_t next = current_duty + step;
        current_duty = (next > target_duty) ? target_duty : next;
      }
    }

    if (target_duty > 0 && rpm == 0)
    {
      uint8_t state = hallState;
      uint16_t duty = current_duty;
      commutate(state, duty);
    }

#if DEBUG_LOG
    if (++debug_sample_divider >= 100)
    {
      debug_sample_divider = 0;
      dbg_target_duty = target_duty;
      dbg_current_duty = current_duty;
      dbg_rpm = motor_rpm;
      dbg_ready = 1;
    }
#endif
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == HALL_A_Pin || GPIO_Pin == HALL_B_Pin || GPIO_Pin == HALL_C_Pin)
  {
    uint32_t now = micros();
    uint32_t dt = now - lastHall;
    uint8_t newHall = readHall();

    if (dt >= MIN_HALL_DT_US && hall_valid_transition(hallState, newHall))
    {
      lastHall = now;
      motor_rpm = RPM_CONST / dt;
      hallState = newHall;
      commutate(hallState, current_duty);
    }
  }
}
static inline uint16_t filtered_adc_to_duty(void)
{
  uint32_t sum = 0;

  for (uint32_t i = 0; i < ADC_BUF_LEN; i++) {
    sum += adc_buf[i];
  }

  uint16_t avg = sum / ADC_BUF_LEN;
  uint32_t scaled;
  uint32_t lin;
  uint32_t quad;

#if DEBUG_LOG
  dbg_adc_raw = adc_buf[0];
  dbg_adc_avg = avg;
#endif

  if (avg <= ADC_DEADBAND)
  {
#if DEBUG_LOG
    dbg_adc_scaled = 0;
#endif
    return 0;
  }

  scaled = ((uint32_t)(avg - ADC_DEADBAND) * 4095UL) / (4095UL - ADC_DEADBAND);
  lin  = (scaled * MAX_DUTY) / 4095UL;
  quad = (uint32_t)(((uint64_t)scaled * scaled * MAX_DUTY) / (4095ULL * 4095ULL));
#if DEBUG_LOG
  dbg_adc_scaled = scaled;
#endif
  return lin;
}
static inline void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  cycles_per_us = SystemCoreClock / 1000000;
}
static inline uint32_t micros(void)
{
  return DWT->CYCCNT / cycles_per_us;
}
static inline int8_t hall_index(uint8_t hall)
{
  switch (hall) {
    case 0b011: return 0;
    case 0b010: return 1;
    case 0b110: return 2;
    case 0b100: return 3;
    case 0b101: return 4;
    case 0b001: return 5;
    default:    return -1;
  }
}
static inline uint8_t hall_valid_transition(uint8_t oldHall, uint8_t newHall)
{
  int8_t oldi = hall_index(oldHall);
  int8_t newi = hall_index(newHall);

  if (oldi < 0 || newi < 0) return 0;

#if CLOCKWISE
  return newi == ((oldi + 1) % 6);
#else
  return newi == ((oldi + 5) % 6);
#endif
}
static inline uint8_t readHall(void)
{
  return ((HALL_A_GPIO_Port->IDR & HALL_A_Pin ? 1 : 0) << 2) |
         ((HALL_B_GPIO_Port->IDR & HALL_B_Pin ? 1 : 0) << 1) |
         ((HALL_C_GPIO_Port->IDR & HALL_C_Pin ? 1 : 0));
}
static inline void commutate(uint8_t hallState, uint16_t duty)
{
  TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                  TIM_CCER_CC2E | TIM_CCER_CC2NE |
                  TIM_CCER_CC3E | TIM_CCER_CC3NE);
  if (duty >= 30)
  {
    switch (hallState)
    {
#if CLOCKWISE
      case 0b101:   // C+, B-, A open
        TIM1->CCR3 = duty;
        TIM1->CCR2 = 0;
        TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
        break;

      case 0b100:   // A+, B-, C open
        TIM1->CCR1 = duty;
        TIM1->CCR2 = 0;
        TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC1E | TIM_CCER_CC1NE);
        break;

      case 0b110:   // A+, C-, B open
        TIM1->CCR1 = duty;
        TIM1->CCR3 = 0;
        TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
        break;

      case 0b010:   // B+, C-, A open
        TIM1->CCR2 = duty;
        TIM1->CCR3 = 0;
        TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
        break;

      case 0b011:   // B+, A-, C open
        TIM1->CCR2 = duty;
        TIM1->CCR1 = 0;
        TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC1E | TIM_CCER_CC1NE);
        break;

      case 0b001:   // C+, A-, B open
        TIM1->CCR3 = duty;
        TIM1->CCR1 = 0;
        TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC1E | TIM_CCER_CC1NE);
        break;
  #else
      case 0b101:   // B+, C-, A open
        TIM1->CCR2 = duty;
        TIM1->CCR3 = 0;
        TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
        break;

      case 0b100:   // B+, A-, C open
        TIM1->CCR2 = duty;
        TIM1->CCR1 = 0;
        TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC1E | TIM_CCER_CC1NE);
        break;

      case 0b110:   // C+, A-, B open
        TIM1->CCR3 = duty;
        TIM1->CCR1 = 0;
        TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC1E | TIM_CCER_CC1NE);
        break;

      case 0b010:   // C+, B-, A open
        TIM1->CCR3 = duty;
        TIM1->CCR2 = 0;
        TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
        break;

      case 0b011:   // A+, B-, C open
        TIM1->CCR1 = duty;
        TIM1->CCR2 = 0;
        TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC1E | TIM_CCER_CC1NE);
        break;

      case 0b001:   // A+, C-, B open
        TIM1->CCR1 = duty;
        TIM1->CCR3 = 0;
        TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
        break;
  #endif
      case 0b000:
      case 0b111:
      default:
        TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                        TIM_CCER_CC2E | TIM_CCER_CC2NE |
                        TIM_CCER_CC3E | TIM_CCER_CC3NE);
        break;
    }
  }

  TIM1->EGR = TIM_EGR_COMG; // generate commutation event
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_SuspendTick();
  DWT_Init();

  TIM1->CR2 |= TIM_CR2_CCPC;   // enable preload
  TIM1->CR2 &= ~TIM_CR2_CCUS;  // COM only from software
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                  TIM_CCER_CC2E | TIM_CCER_CC2NE |
                  TIM_CCER_CC3E | TIM_CCER_CC3NE);
  
  hallState = readHall();
  lastHall = micros();
  motor_rpm = 0;
  current_duty = 0;
  target_duty = 0;
  commutate(hallState, current_duty);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if DEBUG_LOG
    if (dbg_ready)
    {
      uint16_t adc_raw;
      uint16_t adc_avg;
      uint16_t adc_scaled;
      uint16_t target;
      uint16_t duty;
      uint16_t rpm;

      __disable_irq();
      adc_raw = dbg_adc_raw;
      adc_avg = dbg_adc_avg;
      adc_scaled = dbg_adc_scaled;
      target = dbg_target_duty;
      duty = dbg_current_duty;
      rpm = dbg_rpm;
      dbg_ready = 0;
      __enable_irq();

      printf("adc_raw=%u adc_avg=%u adc_scaled=%u target_duty=%u current_duty=%u rpm=%u\r\n",
             adc_raw, adc_avg, adc_scaled, target, duty, rpm);
    }
#endif
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 2047;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 20;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 40000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA6 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_C_Pin HALL_B_Pin HALL_A_Pin */
  GPIO_InitStruct.Pin = HALL_C_Pin|HALL_B_Pin|HALL_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
