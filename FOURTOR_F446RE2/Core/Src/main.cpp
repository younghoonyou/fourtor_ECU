/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "CANSPI.h"
#include "mcp_can.h"
#include "CANopen.h"


#define Slave1 0x0F6
#define Slave2 0x036
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
CAN_FilterTypeDef sFilterConfig;
CAN_RxHeaderTypeDef RxHeader;
SPI_HandleTypeDef hspi2;
//CanTxMsg msg;
//uint8_t RxData[8];
//uint8_t TxData[8];
//volatile uint8_t Flag_Rx = 0;
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

//	uint8_t init_bool = 0;
//	init_bool = can_bus.begin(CAN_250KBPS);

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
  MX_I2C1_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
//  bno055_assignI2C(&hi2c1); //i2c_1?�� ?��?��?�� 2�???? 바꾸?��?��?�� ?��?��?��?�� ?��?��브러�???? ?��?�� I2C_HandleTypeDef hi2c2 good!
//  bno055_setup();
//  bno055_setOperationMode(BNO055_OPERATION_MODE_ACCGYRO);//?��?�� 모드
//  bno055_setPowerMode(BNO55_POWER_MODE_LOWPOWER);//?��?�� 모드

//  	RxHeader.DLC = 8;				//Data length
//  	RxHeader.IDE = CAN_ID_STD;		// Extended frame mode
//  	RxHeader.RTR = CAN_RTR_DATA;	//Data Frame
//  	RxHeader.StdId = 0xF6;			// Replace zero when using extended frames

//  	sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;	// Filter enable
//  	sFilterConfig.FilterBank = 0;						// Filter 0
//  	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;// Specify the FIFO (0 or 1U) that will be assigned to the filter.
//  	sFilterConfig.FilterIdHigh = 0x0000;	// ID 16 digits
//  	sFilterConfig.FilterIdLow = 0x0000;	// ID of 16 bits, only receive extended frame mode, data frame
//  	sFilterConfig.FilterMaskIdHigh = 0x0000;// Filter mask High Low Byte Data is 1 Time Table must match the ID, 0xffffffFFF represents receiving screening must pass the same
//  	sFilterConfig.FilterMaskIdLow = 0x0000;
//  	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;// Mask mode
//  	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;// 32
//  	sFilterConfig.SlaveStartFilterBank = 0;
//
//
//  	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
//   	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);// Activate CAN_IT_RX_FIFO0_MSG_PENDING Interrupt
//   	HAL_CAN_Start(&hcan1);
  CANSPI_Initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(CANSPI_Receive(&rxMessage))
	 	      {

		  if(Slave1 == rxMessage.frame.id)
		  {
			  printf("The message id is 0x0F6 \r\n");
			  printf("ID:0x%2x DLC:%d \r\n",rxMessage.frame.id,rxMessage.frame.dlc);
			  printf("0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x \r\n",rxMessage.frame.data0,rxMessage.frame.data1,rxMessage.frame.data2,rxMessage.frame.data3,rxMessage.frame.data4,rxMessage.frame.data5,rxMessage.frame.data6,rxMessage.frame.data7);
		  }
		  else if(Slave2 == rxMessage.frame.id)
		  {
			  printf("The message id is 0x036 \r\n");
			  printf("ID:0x%2x DLC:%d \r\n",rxMessage.frame.id,rxMessage.frame.dlc);
			  printf("0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x \r\n",rxMessage.frame.data0,rxMessage.frame.data1,rxMessage.frame.data2,rxMessage.frame.data3,rxMessage.frame.data4,rxMessage.frame.data5,rxMessage.frame.data6,rxMessage.frame.data7);

		  }
		  printf("\n");

	 //	        txMessage.frame.idType = rxMessage.frame.idType;
	 //	        txMessage.frame.id = rxMessage.frame.id;
	 //	        txMessage.frame.dlc = rxMessage.frame.dlc;
	 //	        txMessage.frame.data0++;
	 //	        txMessage.frame.data1 = rxMessage.frame.data1;
	 //	        txMessage.frame.data2 = rxMessage.frame.data2;
	 //	        txMessage.frame.data3 = rxMessage.frame.data3;
	 //	        txMessage.frame.data4 = rxMessage.frame.data4;
	 //	        txMessage.frame.data5 = rxMessage.frame.data5;
	 //	        txMessage.frame.data6 = rxMessage.frame.data6;
	 //	        txMessage.frame.data7 = rxMessage.frame.data7;
	 //	        CANSPI_Transmit(&txMessage);
//	 		  printf("ID:0x%2x DLC:%d \r\n",rxMessage.frame.id,rxMessage.frame.dlc);
//	 		  printf("0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x \r\n",rxMessage.frame.data0,rxMessage.frame.data1,rxMessage.frame.data2,rxMessage.frame.data3,rxMessage.frame.data4,rxMessage.frame.data5,rxMessage.frame.data6,rxMessage.frame.data7);
	 		  HAL_Delay(1000);
	 	      }
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
