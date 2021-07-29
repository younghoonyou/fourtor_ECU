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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "bno055.h"
#include "bno055_stm32.h"
#include "string.h"
#include "CANSPI.h"
#include "MCP2515.h"
#include <stdlib.h>
#include "i2c-lcd.h"
#include "stm32_tm1637.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c1;
/* USER CODE END Includes */
uint16_t rpm;
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
int row=0;
int col=0;
/* USER CODE END PTD */
uint8_t a[5];
int RPM_RR;
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
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
//  CANSPI_Initialize();
  /* USER CODE END 2 */
//  lcd_init();
//  HAL_Delay(20);
//  lcd_clear();
//  HAL_Delay(20);
//  lcd_put_cur(0,0);
//  HAL_Delay(20);
//  lcd_send_string((char*)"M.Tq:");
//    lcd_put_cur(1,0);
//    HAL_Delay(20);
//    lcd_send_string((char*)"B.Vol:");
//  tm1637Init();
//  tm1637SetBrightness(3);
  /* Infinite loop */
  bno055_assignI2C(&hi2c1); //i2c_2이 안되서 1로 바꾸었더니 잘되었다 라이브러리 선언 I2C_HandleTypeDef hi2c2
   bno055_setup();
   bno055_setOperationMode(BNO055_OPERATION_MODE_IMU);//작동 모드
   bno055_setPowerMode(BNO55_POWER_MODE_LOWPOWER);//파워 모드
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  printf("HI \r\n");
    /* USER CODE END WHILE */
//	  printf("HI");
//	  HAL_UART_Receive_IT(&huart4,a,5);
//	  HAL_UART_Transmit(&huart2, a, 5, 100);
//	  printf("\r\n");
//	  lcd_put_cur(1,7);
//	  HAL_Delay(20);
//	  lcd_send_string((char*)a);
//	  HAL_Delay(500);
//	  bno055_vector_t gyro = bno055_getVectorGyroscope();// gyro라는 벡터에 각각 x,y,z값의 자이로값
//		  bno055_vector_t acc = bno055_getVectorAccelerometer();//acc라는 벡터에 각각 x,y,z값의 가속도
		  bno055_vector_t mag = bno055_getVectorEuler();//mag라는 벡터에 각각 x,y,z값의 지자기

//		  printf("X_gyro: %.2f Y_gyro: %.2f Z_gyro: %.2f\r\n",gyro.x,gyro.y,gyro.z);
//		  printf("X_acc: %.2f Y_acc: %.2f Z_acc: %.2f\r\n",acc.x,acc.y,acc.z);
//		  printf("X_mag: %.2f Y_mag: %.2f Z_mag: %.2f\r\n",mag.x,mag.y,mag.z);
		  printf("%.2f %.2f %.2f\r\n",mag.x,mag.y,mag.z);
		  	HAL_Delay(100);
//	  txMessage.frame.id=0x80;//0x81
//	 	  txMessage.frame.idType=0x00;
//	 	  txMessage.frame.dlc=8;
//	 	  txMessage.frame.data0=0x00;
//	 	  txMessage.frame.data1=0x00;
//	 	  txMessage.frame.data2=0x00;
//	 	  txMessage.frame.data3=0x00;
//	 	  txMessage.frame.data4=0x00;
//	 	  txMessage.frame.data5=0x00;
//	 	  txMessage.frame.data6=0x00;
//	 	  txMessage.frame.data7=0x00;
//	 	  CANSPI_Transmit(&txMessage);
//	 	  HAL_Delay(50);
//	 	  if(CANSPI_Receive(&rxMessage))
//	 	  {
//	 		  uint16_t RPM_1= ((uint16_t)rxMessage.frame.data1 << 8) | rxMessage.frame.data0;
//	 		  uint16_t RPM_2= ((uint16_t)rxMessage.frame.data3 << 8) | rxMessage.frame.data2;
//	 		  uint32_t RPM=	((uint32_t)RPM_2 << 16) | RPM_1;
//	 		  uint16_t torque_buff= ((uint16_t)rxMessage.frame.data5 << 8) | rxMessage.frame.data4;
////	 		  uint16_t temp_buff= ((uint16_t)rxMessage.frame.data1 << 8) | rxMessage.frame.data0;
////	 		  uint16_t vol_buff= ((uint16_t)rxMessage.frame.data3 << 8) | rxMessage.frame.data2;
//	 		  uint16_t temp_buff= ((uint16_t)rxMessage.frame.data7 << 8) | rxMessage.frame.data6;
//	 		  printf("RPM:%d ",RPM);
//	 		  sprintf(RPM_RR,"%d",RPM);
//	 		  printf("Temp:%.1f ",temp_buff);
////	 		  printf("Voltage:%.2f ",(vol_buff)*0.0625);
//	 		  printf("Torque:%.1f \r\n",(torque_buff)*0.1);
//	 	  	  HAL_Delay(50);
//	  tm1637DisplayDecimal(RPM_RR,1);
//	 	  }
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
