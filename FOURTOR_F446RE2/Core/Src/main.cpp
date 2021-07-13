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
//#include "mcp_can.h"
//#include "CANopen.h"
#include "stdio.h"
#include "string.h"
#include "CANSPI.h"
#include "MCP2515.h"
#include <stdlib.h>
#define Slave1 0x0F6
#define Slave2 0x036
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
//MCP_CAN can_bus(12);
//CANopen motor;
uint16_t rpm;
uint8_t len = 0;
uint32_t canId = 0;
uint32_t timestamp = 0;
uint8_t buf[20];
uint8_t i=0;
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
//char *rx;
rx_reg_t rxReg;
int a;
float b;

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
uint8_t init_bool = 0;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
//  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
//  uint8_t init_bool = 0;
//  can_bus.mcp2515_initCANBuffers;
  CANSPI_Initialize();
  /* USER CODE END 2 */
//  do {
//      init_bool = can_bus.begin(CAN_250KBPS);
//      if (init_bool==CAN_OK) {
//        printf("CAN init ok!!\r\n");
//      } else {
//        printf("CAN init failed!!\r\n");
//      }
//      HAL_Delay(100);
//    } while (init_bool!=CAN_OK);
  /* Infinite loop */
  while (1)
  {
  /* USER CODE BEGIN WHILE */
//	  can_bus.readMsgBuf(&len, buf); // read data, len: data length, buf: data buf
//	  canId = can_bus.getCanId();
	  txMessage.frame.id=0x80;//0x81
	  txMessage.frame.idType=0x00;
	  txMessage.frame.dlc=8;
	  txMessage.frame.data0=0x00;
	  txMessage.frame.data1=0x00;
	  txMessage.frame.data2=0x00;
	  txMessage.frame.data3=0x00;
	  txMessage.frame.data4=0x00;
	  txMessage.frame.data5=0x00;
	  txMessage.frame.data6=0x00;
	  txMessage.frame.data7=0x00;
	  CANSPI_Transmit(&txMessage);
	  HAL_Delay(100);
	  if(CANSPI_Receive(&rxMessage))
	  {
		  uint16_t torque_buff= ((uint16_t)rxMessage.frame.data5 << 8) | rxMessage.frame.data4;
		  uint16_t temp_buff= ((uint16_t)rxMessage.frame.data1 << 8) | rxMessage.frame.data0;
		  uint16_t vol_buff= ((uint16_t)rxMessage.frame.data3 << 8) | rxMessage.frame.data2;
		  uint16_t current_buff= ((uint16_t)rxMessage.frame.data7 << 8) | rxMessage.frame.data6;

//		  for(int i=0;i;i++){
//		  printf("%x",rxMessage.array[i]);
//		  printf("\r\n");
//		  HAL_Delay(1000);
//		  printf("ID:0x%08x DLC:%d \r\n",rxMessage.frame.id,rxMessage.frame.dlc);
//		  printf("%2x %2x %2x %2x %2x %2x %2x %2x \r\n",rxMessage.frame.data0,rxMessage.frame.data1,rxMessage.frame.data2,rxMessage.frame.data3,rxMessage.frame.data4,rxMessage.frame.data5,rxMessage.frame.data6,rxMessage.frame.data7);
		  printf("Temp:%d ",temp_buff);
		  printf("Current:%d ",current_buff);
		  printf("Voltage:%.2f ",(vol_buff)*0.0625);
		  printf("Torque:%.2f \r\n",(torque_buff)*0.1);

//
		 	 		  HAL_Delay(100);
	  }
    /* USER CODE END WHILE */
//	  motor.read16bit(0x1803, 0x00, &rpm, DEFAULT_NODE_ID);

//	  HAL_UART_Receive(&huart4,(uint8_t *)rx,sizeof(rx),100);
//	  printf("%d ",rx);
//	  HAL_Delay(500);
//
//	  printf("\r\n");
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
