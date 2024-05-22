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
#include "math.h"
#define PI 3.1415926;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "as5047p.h"
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
as5047p_handle_t as5047p;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t prev_capture = 0;
uint32_t current_capture = 0;
uint32_t frequency = 0;


#define TIMCLOCK   72000000
#define PRESCALAR  72

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
int Is_First_Captured = 0;

/* Measure Frequency */
//float frequency = 0;
//
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//	{
//		if (Is_First_Captured==0) // if the first rising edge is not captured
//		{
//			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
//			Is_First_Captured = 1;  // set the first captured as true
//		}
//
//		else   // If the first rising edge is captured, now we will capture the second edge
//		{
//			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
//
//			if (IC_Val2 > IC_Val1)
//			{
//				Difference = IC_Val2-IC_Val1;
//			}
//
//			else if (IC_Val1 > IC_Val2)
//			{
//				Difference = (0xffff - IC_Val1) + IC_Val2;
//			}
//
//			float refClock = TIMCLOCK/(PRESCALAR);
//
//			frequency = refClock/Difference;
//
//			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
//			Is_First_Captured = 0; // set it back to false
//		}
//	}
//}

/* Measure Width */
uint32_t usWidth = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if (Is_First_Captured == 0)
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			Is_First_Captured = 1;
		}
		else
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2 - IC_Val1;
			}
			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xFFFF - IC_Val1) + IC_Val2;
			}

			float refClock = HAL_RCC_GetPCLK1Freq();  // ?��??�PCLK1频�??
			float mFactor = 1000000 / refClock;

			float usWidth = Difference * mFactor;

			printf("Pulse Width: %.2f us\r\n", usWidth);

			__HAL_TIM_SET_COUNTER(htim, 0);
			Is_First_Captured = 0;
		}
	}
}


void printf_ENTER(){
	printf("\r\n");
}
void as5047p_spi_send(uint16_t data);
uint16_t as5047p_spi_read(void);
void as5047p_spi_select(void);
void as5047p_spi_deselect(void);
void as5047p_delay(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	void as5047p_spi_send(uint16_t data)
	{
	  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);
	}

	uint16_t as5047p_spi_read(void)
	{
	  uint16_t data = 0;
	  HAL_SPI_Receive(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);
	  return data;
	}

	void as5047p_spi_select(void)
	{
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	}

	void as5047p_spi_deselect(void)
	{
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	}

	// For 't_CSn': High time of CSn between two transmissions, >350 ns.
	void as5047p_delay(void)
	{
	  HAL_Delay(1);
	}
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
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	TIM1->CCR1 = 9999;
	TIM1->CCR2 = 9999;
	TIM1->CCR3 = 900;
	TIM2->CCR2 = 900;
	TIM1->CCR4 = 900;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  //HAL_TIM_Base_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	printf("------------1104 DUAL BLDC speed control and encoder test--------------------------\r\n");
	as5047p_handle_t as5047p;

	as5047p_make_handle(&as5047p_spi_send,
					  &as5047p_spi_read,
					  &as5047p_spi_select,
					  &as5047p_spi_deselect,
					  &as5047p_delay,
					  &as5047p);

	as5047p_config(&as5047p, 0b00100101, 0b00000000);
	as5047p_set_zero(&as5047p, 0);

	__HAL_TIM_SET_COUNTER(&htim2, 300000);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim4, 0);



	printf("\r\nSTM32-AS5047P, Ready\r\n");
	/* USER CODE END 2 */
	__HAL_TIM_SetCounter(&htim1, 100);
	__HAL_TIM_SetCounter(&htim2, 100);


	int prev_left = 0, prev_right = 0, fly_mode = 0, right_speed = 0, left_speed= 0;

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
	//	  printf("test");
	//	  printf_ENTER();
	//	    // 读�?��?�获??
	//	    current_capture = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
	//
	//	    // 计�?��?��??
	//	    if (current_capture >= prev_capture) {
	//	      frequency = HAL_RCC_GetPCLK1Freq() / (current_capture - prev_capture);
	//	    } else {
	//	      frequency = 0;  // ?��??�溢?��等问�??????????
	//	    }
	//
	//	    // ??�印频�??
	//	    printf("Frequency: %lu Hz\r\n", frequency);
	//
	//	    prev_capture = current_capture;
	//
		  // ?��?��?�延迟�?�可以�?�整

		//void as5047p_spi_select();
		//	  float angle;
		//	  as5047p_get_angle(&as5047p, without_daec, &angle);
		//	  printf("Angle: %3i\r\n", (int)angle);

		/*wing speed and phase control*/
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, TIM1->CCR3);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, TIM1->CCR4);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, TIM2->CCR2);
		float left =__HAL_TIM_GET_COUNTER(&htim3);
		float right =__HAL_TIM_GET_COUNTER(&htim4);

		if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
		{
			fly_mode++;

			while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);
			HAL_Delay(50);
			// while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET){}
		}

		/*phase detect*/
		float left_angle = left / 5.5555555;//3600 degree MAX
//		if(left_angle > 1800)
//			left_angle = 3600 - left_angle;
		float right_angle = right / 5.5555555;//3600 degree MAX
//		if(right_angle > 1800)
//			right_angle = 3600 - right_angle;

		int phase_diff = (left_angle > right_angle)?
				(left_angle - right_angle):(right_angle - left_angle);
		if(phase_diff > 1800)
		{
			if(left_angle > right_angle)
				phase_diff = 3600 - left_angle + right_angle;
			else
				phase_diff = 3600 - right_angle + left_angle;
		}

		if(left_angle < prev_left)
		{
			if(prev_left - left_angle > 1800)
				left_speed = (3600 - prev_left + left_angle)/2.5;//rpm
			else
				left_speed = (prev_left - left_angle)/2.5;//rpm
		}
		else
		{
			if(left_angle - prev_left > 1800)
				left_speed = (3600 - left_angle + prev_left)/2.5;//rpm
			else
				left_speed = (left_angle - prev_left)/2.5;//rpm
		}

		if(right_angle < prev_right)
		{
			if(prev_right - right_angle > 1800)
				right_speed = (3600 - prev_right + right_angle)/2.5;//rpm
			else
				right_speed = (prev_right - right_angle)/2.5;// rpm
		}
		else
		{
			if(right_angle - prev_right > 1800)
				right_speed = (3600 - right_angle + prev_right)/2.5;//rpm
			else
				right_speed = (right_angle - prev_right)/2.5;// rpm
		}

//		int speed_diff = (left_speed > right_speed)?
//						(left_speed - right_speed):(right_speed - left_speed);

		prev_left = left_angle;
		prev_right = right_angle;

		printf("------------------------------------------------------\r\n");
		printf("left angle : %.1f\r\n", left_angle);
		printf("right angle : %.1f\r\n", right_angle);
		printf("phase difference : %d\r\n", phase_diff);
		printf("left wing speed : %d per min\rright wing speed : %d per min\r\n",
				left_speed, right_speed);
//		printf("speed difference : %d\r\n", speed_diff);
		printf("flying mode is %d\n", fly_mode);

		/*phase control*/
		switch(fly_mode)
		{
		case 1://wing sync
			/*fore wing speed*/
			TIM1->CCR3 = 970;//left wing speed
			TIM2->CCR2 = 970;//right wing speed
			TIM1->CCR4 = 970;
//			if(left_angle != right_angle)
//				fly_mode = 2;
//			break;
			/*correction of the balanced speed on both wings*/
		case 2:
			if(right_speed < 300)
				TIM1->CCR3++;
			else if(right_speed > 300)
				TIM1->CCR3--;	//left wing speed
			if(left_speed < 300)
				TIM2->CCR2++;
			else if(left_speed > 300)
				TIM2->CCR2--;	//left wing speed
			if(phase_diff != 450 || phase_diff != 1250)
			{
				if(right_angle > left_angle)
				{
					TIM2->CCR2++;
					TIM1->CCR3--;	//left wing speed
				}
				else
				{
					TIM2->CCR2--;	//left wing speed
					TIM1->CCR3++;
				}
			}
			break;
		case 3:
			if(right_speed < 300)
				TIM1->CCR3++;
			else if(right_speed > 300)
				TIM1->CCR3--;	//left wing speed
			if(left_speed < 300)
				TIM2->CCR2++;
			else if(left_speed > 300)
				TIM2->CCR2--;	//left wing speed
			if(phase_diff != 225 || phase_diff != 1125)
			{
				if(right_angle > left_angle)
				{
					TIM2->CCR2++;
					TIM1->CCR3--;	//left wing speed
				}
				else
				{
					TIM2->CCR2--;	//left wing speed
					TIM1->CCR3++;
				}
			}
			break;
			break;
//		case 2:
//			if(right_speed < 6)
//				TIM1->CCR3++;
//			else if(right_speed > 6)
//				TIM1->CCR3--;	//left wing speed
//			if(left_speed < 6)
//				TIM2->CCR2++;
//			else if(left_speed > 6)
//				TIM2->CCR2--;	//left wing speed
//			break;



//
//		case 2://left wing ahead 90 degree
//			if(TIM1->CCR3 < 1300)
//				TIM1->CCR3 += 10;
//			else
//				fly_mode = 3;//decrease the speed.
//			break;
//		case 3:
//			if(TIM1->CCR3 > 1150)
//				TIM1->CCR3 -= 10;
//			else
//				 fly_mode = 1;//back to sync.
//			break;
			//////////////////////////////////////////
//		case 2://right wing ahead 90 degree
//		  	if(phase_diff != 90)
//		  	{
//		  		if(phase_diff < 90)
//		  			TIM2->CCR2 += 10;
//		  		if(phase_diff > 90)
//		  			TIM2->CCR2 -= 10;
//		  	}
//			else
//				fly_mode = 1;//decrease the speed.
//			break;
////		  case 3:
//			if(TIM2->CCR2 > 970)
//				TIM2->CCR2 -= 10;
//			else
//				 fly_mode = 1;//back to sync.
//			break;
			/////////////////////////////////////////////
		/*case 2://hind wing ahead 180 degree
		 * 	if(TIM2->CCR2 < 1300)
				TIM2->CCR2 += 10;
			else
				fly_mode = 3;//decrease the speed.
			break;
		  case 3:
			if(TIM2->CCR2 > 1150)
				TIM2->CCR2 -= 10;
			else
				 fly_mode = 1;//back to sync.
			break;*/
			/////////////////////////////////////////////
		/*case 2://fore wing ahead 180 degree
		 * 	if(TIM2->CCR2 < 1300)
				TIM2->CCR2 += 10;
			else
				fly_mode = 3;//decrease the speed.
			break;
		  case 3:
			if(TIM2->CCR2 > 1150)
				TIM2->CCR2 -= 10;
			else
				 fly_mode = 1;//back to sync.
			break;*/
			/////////////////////////////////////////////
		/*case 2://fore wing ahead 180 degree
		 * 	if(TIM2->CCR2 < 1300)
				TIM2->CCR2 += 10;
			else
				fly_mode = 3;//decrease the speed.
			break;
		  case 3:
			if(TIM2->CCR2 > 1150)
				TIM2->CCR2 -= 10;
			else
				 fly_mode = 1;//back to sync.
			break;*/
		default:
			break;
		}

		/*speed detect*/

		HAL_Delay(100);
		/*float angle_deg;
		int8_t error = as5047p_get_angle(&as5047p, 1, &angle_deg);

		if (error == 0)
			printf("Angle: %f\r\n", angle_deg);

		uint16_t position;
		error = as5047p_get_position(&as5047p, without_daec, &position);

		if (error == 0)
			printf("Raw: %5i\r\n", position);*/
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE
{
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
return ch;
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
