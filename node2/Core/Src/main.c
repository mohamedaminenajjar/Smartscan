/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
  * @brief Threshold value for the potentiometer level to determine the motor speed.
  *        Motors will be controlled based on the potentiometer reading when it exceeds this level.
  */
#define POTENTIOMETER_LEVEL  2500

/**
  * @brief Minimum speed value for the motors.
  *        This value represents the lowest acceptable speed for the motors.
  */
#define MOTOR_SPEED_MIN  0

/**
  * @brief Maximum speed value for the motors.
  *        This value represents the highest acceptable speed for the motors.
  */
#define MOTOR_SPEED_MAX  900

/**
  * @brief Size of the received CAN message array.
  *        Specifies the number of elements in the array to receive CAN messages.
  */
#define CAN_RECIEV_MSG_SIZE 3

/**
  * @brief Size of the transmitted CAN message array.
  *        Specifies the number of elements in the array to transmit CAN messages.
  */
#define CAN_TRANSMIT_MSG_SIZE 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA_HandleTypeDef hdma_tim4_ch2;

/* USER CODE BEGIN PV */

/**
  * @brief Initialize variables for motor speed control and communication flags.
  */
uint16_t Left_Motor_Speed = 0; // Variable to store the speed of the left motor. */
uint16_t Right_Motor_Speed = 0; // Variable to store the speed of the right motor. */
volatile uint8_t adc_convert_complete = 0; // Flag to indicate completion of ADC conversion. */
volatile char Can_Transmission_Flag = 0; // Flag to indicate the status of CAN transmission. */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void turn_motor(uint32_t speed , uint8_t motor_number);
static void handle_ADC_Conversion(void);
static void variablesToByteArray(uint8_t *byteArray, int size1, void *var1, int size2, void *var2);
static void CAN1_Tx(void);
static void CAN1_Filter_Config (void);
static void CAN1_Rx (void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  /**
    * @brief Starts ADC in the interruption mode
    */
  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  {
      Error_Handler();
  }
  // Set right motor pins for forward rotation and left motor pins for forward rotation
  HAL_GPIO_WritePin(right_motor_in1_GPIO_Port, right_motor_in1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(right_motor_in2_GPIO_Port, right_motor_in2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(left_motor_in1_GPIO_Port, left_motor_in1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(left_motor_in2_GPIO_Port, left_motor_in2_Pin, GPIO_PIN_RESET);
  // Start PWM DMA for right motor speed control
  if (HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t *)&Right_Motor_Speed, 1) != HAL_OK)
  {
      Error_Handler();
  }
  // Start PWM DMA for left motor speed control
  if (HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, (uint32_t *)&Left_Motor_Speed, 1) != HAL_OK)
  {
      Error_Handler();
  }
  // Activate CAN notifications
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF) != HAL_OK)
  {
      Error_Handler();
  }
  // Start CAN communication
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /**
	    * @brief Handles ADC conversion and CAN transmission, updates GPIO pins based on transmission flag,
	    *        and introduces a delay.
	    */
	  handle_ADC_Conversion();
	  CAN1_Tx();
	  if (1 == Can_Transmission_Flag)
	  {
	      // Set GREEN pin high and others low
	      HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
	      Can_Transmission_Flag = 0;
	  }
	  else if (2 == Can_Transmission_Flag)
	  {
	      // Set BLUE pin high and others low
	      //CAN1_Rx();
	      HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
	      Can_Transmission_Flag = 0;
	  }
	  else if (3 == Can_Transmission_Flag)
	  {
	      // Set RED pin high and others low
	      HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
	      Can_Transmission_Flag = 0;
	  }
	  // Delay for 500 milliseconds
	  HAL_Delay(500);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 20;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  htim4.Init.Prescaler = 36000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, left_motor_in2_Pin|left_motor_in1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, right_motor_in2_Pin|right_motor_in1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : left_motor_in2_Pin left_motor_in1_Pin */
  GPIO_InitStruct.Pin = left_motor_in2_Pin|left_motor_in1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_Pin ORANGE_Pin RED_Pin BLUE_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : right_motor_in2_Pin right_motor_in1_Pin */
  GPIO_InitStruct.Pin = right_motor_in2_Pin|right_motor_in1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Callback function called when ADC conversion is complete.
  *        Sets the flag to indicate completion of conversion.
  * @param hadc: Pointer to ADC handle
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  adc_convert_complete = 1;
}

/**
  * @brief Turns the specified motor at the given speed.
  * @param speed: Speed value for the motor
  * @param motor_number: Motor number (1 for right motor, 2 for left motor)
  */
static void turn_motor(uint32_t speed , uint8_t motor_number)
{
	if( motor_number == 1 )
	{
	    Right_Motor_Speed = speed;
	}
	else if( motor_number == 2 )
	{
		Left_Motor_Speed = speed;
	}
}

/**
  * @brief Handles the ADC conversion process.
  *        Stops ADC interrupts, reads the ADC value, adjusts motor speeds based on the value,
  *        and restarts ADC interrupts.
  */
static void handle_ADC_Conversion(void) {
    while (adc_convert_complete != 1) ;
    HAL_ADC_Stop_IT(&hadc1);
    if( HAL_ADC_GetValue(&hadc1) < POTENTIOMETER_LEVEL)
    {
        turn_motor( MOTOR_SPEED_MIN , 1);
        turn_motor( MOTOR_SPEED_MIN ,2);
    }
    else
    {
        turn_motor( MOTOR_SPEED_MAX, 1);
        turn_motor( MOTOR_SPEED_MAX, 2);
    }
    adc_convert_complete = 0;
    HAL_ADC_Start_IT(&hadc1);
}

/**
  * @brief Converts two variables into a byte array.
  * @param byteArray: Pointer to the byte array
  * @param size1: Size of the first variable
  * @param var1: Pointer to the first variable
  * @param size2: Size of the second variable
  * @param var2: Pointer to the second variable
  */
static void variablesToByteArray(uint8_t *byteArray, int size1, void *var1, int size2, void *var2) {
    // Copie de la première variable dans le tableau d'octets
    for (int i = 0; i < size1; i++) {
        byteArray[i] = *((uint8_t *)var1 + i);
    }
    // Copie de la deuxième variable dans le tableau d'octets
    for (int i = 0; i < size2; i++) {
        byteArray[size1 + i] = *((uint8_t *)var2 + i);
    }
}

/**
  * @brief Transmits data over CAN bus.
  *        Converts motor speeds to byte array, configures CAN message header, and transmits the message.
  */
static void CAN1_Tx(void)
{
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint8_t can_msg[CAN_TRANSMIT_MSG_SIZE];
	variablesToByteArray(can_msg, sizeof(Right_Motor_Speed), &Right_Motor_Speed, sizeof(Left_Motor_Speed), &Left_Motor_Speed);
	TxHeader.StdId= 0x002;
	TxHeader.RTR= CAN_RTR_DATA;
	TxHeader.IDE= CAN_ID_STD;
	TxHeader.DLC= CAN_TRANSMIT_MSG_SIZE;
	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, can_msg, &TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}
}


/**
  * @brief Configures CAN filter.
  *        Configures the filter for CAN messages.
  */
static void CAN1_Filter_Config (void)
{
	CAN_FilterTypeDef can1_filter_init;
	can1_filter_init.FilterActivation = ENABLE;
	can1_filter_init.FilterBank = 0;
	can1_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
	can1_filter_init.FilterIdHigh = 0xFF1;
	can1_filter_init.FilterIdLow = 0x000;
	can1_filter_init.FilterMaskIdHigh = 0xFF1;
	can1_filter_init.FilterMaskIdLow = 0x000;
	can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;
	if(HAL_CAN_ConfigFilter(&hcan1, &can1_filter_init) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief Receives data over CAN bus.
  *        Processes received CAN message and takes appropriate action.
  */
static void CAN1_Rx (void)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t Reciev_Msg[CAN_RECIEV_MSG_SIZE];
	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, Reciev_Msg) != HAL_OK)
	{
		Error_Handler();
	}
	if( 0xFF1 == RxHeader.StdId)
	  {
		if( Reciev_Msg[0] == 1)
		 {
				//turn motor 1
		 }
		else if( Reciev_Msg[0] == 2)
		 {
				//turn motor 2
		 }
	  }
}

/**
  * @brief Callback function called when CAN transmission mailbox 0 is complete.
  *        Sets the transmission flag accordingly.
  * @param hcan: Pointer to CAN handle
  */
void HAL_CAN_TxMailbox0CompleteCallback (CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1)
	{
	   Can_Transmission_Flag=1;
	}
}

/**
  * @brief Callback function called when CAN transmission mailbox 1 is complete.
  *        Sets the transmission flag accordingly.
  * @param hcan: Pointer to CAN handle
  */
void HAL_CAN_TxMailbox1CompleteCallback (CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1)
	{
		Can_Transmission_Flag=1;
	}
}

/**
  * @brief Callback function called when CAN transmission mailbox 2 is complete.
  *        Sets the transmission flag accordingly.
  * @param hcan: Pointer to CAN handle
  */
void HAL_CAN_TxMailbox2CompleteCallback (CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1)
	{
		Can_Transmission_Flag=1;
	}
}

/**
  * @brief Callback function called when CAN FIFO 0 has a pending message.
  *        Sets the transmission flag accordingly.
  * @param hcan: Pointer to CAN handle
  */
void HAL_CAN_RxFifo0MsgPendingCallback (CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1)
	{
		Can_Transmission_Flag = 2;
	}
}

/**
  * @brief Callback function called when a CAN error occurs.
  *        Sets the transmission flag accordingly.
  * @param hcan: Pointer to CAN handle
  */
void HAL_CAN_ErrorCallback (CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1)
	{
		Can_Transmission_Flag=3;
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
