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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
  * @brief MAX_RETRIES: Maximum number of retries for communication with the DHT11 sensor.
  *        If the sensor fails to respond within these retries, the communication is aborted.
  */
#define MAX_RETRIES 3

/**
  * @brief CAN_TRANSMIT_MSG_SIZE: Size of the CAN message payload in bytes.
  *        It specifies the number of data bytes to be transmitted over the CAN bus.
  */
#define CAN_TRANSMIT_MSG_SIZE 3

/**
  * @brief FUEL_LEVEL_MIN: Minimum fuel level threshold.
  *        If the fuel level falls below this threshold, it indicates a low fuel condition.
  */
#define FUEL_LEVEL_MIN 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/**
  * @brief hum_integer_part: Integer part of the humidity value from the DHT11 sensor.
  */
uint8_t hum_integer_part = 0;

/**
  * @brief hum_decimal_part: Decimal part of the humidity value from the DHT11 sensor.
  */
uint8_t hum_decimal_part = 0;

/**
  * @brief temp_integer_part: Integer part of the temperature value from the DHT11 sensor.
  */
uint8_t temp_integer_part = 0;

/**
  * @brief temp_decimal_part: Decimal part of the temperature value from the DHT11 sensor.
  */
uint8_t temp_decimal_part = 0;

/**
  * @brief sum: Sum of the received data from the DHT11 sensor used for verification.
  */
uint16_t sum = 0;

/**
  * @brief Adc_Conversion_Complete: Flag indicating the completion of ADC conversion.
  *        It's volatile as it's accessed from an interrupt service routine (ISR).
  */
volatile char Adc_Conversion_Complete = 0;

/**
  * @brief Can_Transmission_Flag: Flag indicating the completion of CAN transmission.
  *        It's volatile as it's accessed from an interrupt service routine (ISR).
  */
volatile char Can_Transmission_Flag = 0;

/**
  * @brief FUEL_level: Variable to store the current fuel level reading.
  *        Initialized to 0 and updated with the value obtained from the ADC conversion.
  */
uint16_t FUEL_level = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void micro_Delay(uint16_t time);
static void Set_Pin_output(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
static void Set_Pin_Input(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
static void dht11_start (void);
static uint8_t check_response (void);
static uint8_t dht11_read (void);
static bool temp_seuil (uint8_t temp_FUEL_levelue );
static void FUEL_Sensor_Function (void);
static void Dht11_function (void);
static void CAN1_Tx(void);
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
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  /**
    * @brief Starts the TIM6 base timer.
    *        If the start operation fails, it triggers an error handler.
    */
  if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
  {
      Error_Handler();
  }

  /**
    * @brief Starts ADC conversion in interrupt mode.
    *        If the start operation fails, it triggers an error handler.
    */
  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  {
      Error_Handler();
  }

  /**
    * @brief Activates notifications for CAN events including TX mailbox empty,
    *        received FIFO0 message pending, and bus-off condition.
    *        If the activation fails, it triggers an error handler.
    */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF) != HAL_OK)
  {
      Error_Handler();
  }

  /**
    * @brief Starts the CAN peripheral.
    *        If the start operation fails, it triggers an error handler.
    */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /**
	    * @brief Executes the functions to read data from sensors (DHT11 and fuel level sensor),
	    *        transmit data over CAN bus, and handle the CAN transmission flag.
	    */
	  Dht11_function();                // Read data from DHT11 sensor
	  FUEL_Sensor_Function();         // Read data from fuel level sensor
	  CAN1_Tx();                      // Transmit data over CAN bus

	  // Handle CAN transmission flag to control LED indicators
	  if (1 == Can_Transmission_Flag)
	  {
	      HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);  // Set BLUE LED
	      HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);  // Reset RED LED
	      Can_Transmission_Flag = 0;                                  // Reset flag
	  }
	  else if (3 == Can_Transmission_Flag)
	  {
	      HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);  // Reset BLUE LED
	      HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);      // Set RED LED
	      Can_Transmission_Flag = 0;                                    // Reset flag
	  }

	  HAL_Delay(500);  // Delay to control the loop frequency
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
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
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
  htim6.Init.Prescaler = 50-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(dht11_GPIO_Port, dht11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : dht11_Pin */
  GPIO_InitStruct.Pin = dht11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(dht11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_Pin ORANGE_Pin RED_Pin BLUE_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief HAL_ADC_ConvCpltCallback: Callback function executed when ADC conversion is complete.
  * @param hadc: pointer to ADC_HandleTypeDef structure that contains the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  Adc_Conversion_Complete = 1;
}

/**
  * @brief micro_Delay: Function to introduce a microsecond delay.
  * @param time: the time in microseconds to delay.
  * @retval None
  */
static void micro_Delay(uint16_t time){
	  __HAL_TIM_SET_COUNTER(&htim6, 0);
	  while (__HAL_TIM_GET_COUNTER(&htim6) < time);
}

/**
  * @brief Set_Pin_output: Function to configure a GPIO pin as output.
  * @param GPIOx: GPIO port to which the pin belongs.
  * @param GPIO_Pin: specifies the port bit to be written.
  * @retval None
  */
static void Set_Pin_output(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_PULLUP;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
  * @brief Set_Pin_Input: Function to configure a GPIO pin as input.
  * @param GPIOx: GPIO port to which the pin belongs.
  * @param GPIO_Pin: specifies the port bit to be written.
  * @retval None
  */
static void Set_Pin_Input(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
  * @brief dht11_start: Function to start DHT11 sensor communication.
  * @retval None
  */
static void dht11_start (void)
{
	Set_Pin_output( dht11_GPIO_Port, dht11_Pin);
	HAL_GPIO_WritePin( dht11_GPIO_Port, dht11_Pin, 0);   // pull the pin low
	micro_Delay(18000);
	HAL_GPIO_WritePin( dht11_GPIO_Port, dht11_Pin, 1);   // pull the pin high
	micro_Delay(20);
	Set_Pin_Input( dht11_GPIO_Port, dht11_Pin);
}

/**
  * @brief check_response: Function to check the response from DHT11 sensor.
  * @retval Response: returns 1 if response is received, -1 if not.
  */
static uint8_t check_response (void)
{
	uint8_t Response = 0;
	micro_Delay(40);
	if (!(HAL_GPIO_ReadPin (dht11_GPIO_Port, dht11_Pin)))
	{
		micro_Delay(80);
		if ((HAL_GPIO_ReadPin (dht11_GPIO_Port, dht11_Pin)))
		{
			Response = 1;
		}
		else
			{
			Response = -1;
			}
	}
	while ((HAL_GPIO_ReadPin (dht11_GPIO_Port, dht11_Pin)));   // wait for the pin to go low

	return Response;
}

/**
  * @brief dht11_read: Function to read data from DHT11 sensor.
  * @retval i: the received data byte.
  */
static uint8_t dht11_read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (dht11_GPIO_Port, dht11_Pin)));   // wait for the pin to go high
		micro_Delay(40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (dht11_GPIO_Port, dht11_Pin)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (dht11_GPIO_Port, dht11_Pin)));  // wait for the pin to go low
	}
	return i;
}

/**
  * @brief temp_seuil: Function to check if the temperature exceeds a certain threshold.
  * @param temp_level: the temperature value to check.
  * @retval true if temperature is below or equal to the threshold, false otherwise.
  */
static bool temp_seuil (uint8_t temp_level )
{
	if( 28 >= temp_level )
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
  * @brief Dht11_function: Function to read and process data from the DHT11 sensor.
  * @retval None
  */
static void Dht11_function (void)
{
	uint8_t retries = 0;
	dht11_start();
	while ( !check_response())
     {
        retries++;
        if (retries >= MAX_RETRIES)
        {
  		  HAL_GPIO_WritePin(GPIOD, BLUE_Pin, GPIO_PIN_SET);
            retries = 0;
            break;  // Break out of the inner loop to avoid infinite retry loop
        }
        HAL_Delay(1000); // Delay before retrying
     }

    if (retries < MAX_RETRIES)
     {
	    hum_integer_part = dht11_read();
	    hum_decimal_part = dht11_read();
	    temp_integer_part = dht11_read();
	    temp_decimal_part = dht11_read();
	    sum = dht11_read();
	    while (sum != (hum_integer_part + hum_decimal_part + temp_integer_part + temp_decimal_part))
	    {
		    hum_integer_part = dht11_read();
		    hum_decimal_part = dht11_read();
		    temp_integer_part = dht11_read();
		    temp_decimal_part = dht11_read();
	    }
	    if(temp_seuil(temp_integer_part))
	    {
		    HAL_GPIO_WritePin(GPIOD, ORANGE_Pin, GPIO_PIN_SET);
		    HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
	    }
	    else
	    {
		    HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
		    HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin, GPIO_PIN_RESET);
	    }
     }
}

/**
  * @brief FUEL_Sensor_Function: Function to read and process data from the fuel sensor.
  * @retval None
  */
static void FUEL_Sensor_Function (void)
{
    FUEL_level = (uint16_t) HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop_IT(&hadc1);
    HAL_GPIO_WritePin(GPIOD, GREEN_Pin | RED_Pin, GPIO_PIN_RESET);

	  if ( 1 == Adc_Conversion_Complete)
	  {
	        if (FUEL_level < FUEL_LEVEL_MIN)
	        {
	            //HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
	            //HAL_GPIO_WritePin(GPIOD, GREEN_Pin, GPIO_PIN_RESET);
	        } else
	        {
	            //HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
	            //HAL_GPIO_WritePin(GPIOD, RED_Pin, GPIO_PIN_RESET);
	        }
		  Adc_Conversion_Complete = 0;
		  if(HAL_ADC_Start_IT(&hadc1) != HAL_OK )
		  {
			  Error_Handler();
		  }
	  }
}

/**
  * @brief CAN1_Tx: Function to transmit sensor data via CAN protocol.
  * @retval None
  */
static void CAN1_Tx (void)
{
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint8_t can_msg[CAN_TRANSMIT_MSG_SIZE];
	can_msg[0] = temp_integer_part;
	can_msg[1] = hum_integer_part;
	can_msg[2] = FUEL_level;
	if(28 < temp_integer_part)
	{
		TxHeader.StdId= 0x001;
	}
	else
	{
		TxHeader.StdId= 0x003;
	}
	TxHeader.RTR= CAN_RTR_DATA;
	TxHeader.IDE= CAN_ID_STD;
	TxHeader.DLC= CAN_TRANSMIT_MSG_SIZE;
	TxHeader.TransmitGlobalTime= DISABLE;

	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, can_msg, &TxMailbox) != HAL_OK)
	{

		Error_Handler();
	}
}

/**
  * @brief HAL_CAN_TxMailbox0CompleteCallback: Callback function executed when CAN Tx mailbox 0 is complete.
  * @param hcan: pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox0CompleteCallback (CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1)
	{
	   Can_Transmission_Flag=1;
	}
}

/**
  * @brief HAL_CAN_TxMailbox1CompleteCallback: Callback function executed when CAN Tx mailbox 1 is complete.
  * @param hcan: pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox1CompleteCallback (CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1)
	{
		Can_Transmission_Flag=1;
	}
}

/**
  * @brief HAL_CAN_TxMailbox2CompleteCallback: Callback function executed when CAN Tx mailbox 2 is complete.
  * @param hcan: pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox2CompleteCallback (CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1)
	{
		Can_Transmission_Flag=1;
	}
}

/**
  * @brief HAL_CAN_ErrorCallback: Callback function executed when CAN error occurs.
  * @param hcan: pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
  * @retval None
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
     ex: printf("Wrong parameters FUEL_levelue: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
