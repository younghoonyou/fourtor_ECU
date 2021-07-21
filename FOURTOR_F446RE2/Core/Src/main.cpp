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
#include "stdio.h"
#include "bno055.h"
#include "bno055_stm32.h"
#include "string.h"
#include "CANSPI.h"
#include "MCP2515.h"
#include <stdlib.h>
#include "i2c-lcd.h"
#include "stm32_tm1637.h"
#define SLAVE_ADDRESS_LCD 0x27
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c3;
//MCP_CAN can_bus(12);
//CANopen motor;
uint16_t rpm;
uint8_t len = 0;
uint32_t canId = 0;
uint32_t timestamp = 0;
uint8_t buf[20];
uint8_t i=0;
int row=0;
int col=0;
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
char s[10];
rx_reg_t rxReg;
int a=0;
int b=0;
//int c=0;
//int a;
//float b;

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
//	uint8_t a[30];
//	int size;
//	int* arr = (int*)malloc(sizeof(int) * size);
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
	uint8_t buffer[100];
	int str=1000;
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
  MX_I2C1_Init();//BNO055
  MX_CAN1_Init();
  MX_SPI2_Init();//Motor data logging
  MX_UART4_Init();//BMS data logging
  MX_UART5_Init();//7-Segment
  MX_I2C3_Init();//LCD print
  /* USER CODE BEGIN 2 */

  CANSPI_Initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  lcd_init();
  HAL_Delay(20);
  lcd_clear();
//  HAL_Delay(20);
//  lcd_put_cur(0,0);
//  HAL_Delay(20);
//  lcd_send_string((char*)"M.Tq:");
//  HAL_Delay(20);
//  lcd_put_cur(1,0);
//  HAL_Delay(20);
//  lcd_send_string((char*)"M.Tq:");


//  for(int i=0;i<100;i++){
//	  HAL_Delay(100);
//	  lcd_put_cur(1,5);
//	  sprintf((char*)buffer,"%d",i);
//	  lcd_send_string((char*)buffer);
//  }
//  bno055_assignI2C(&hi2c2); //i2c_1이 안되서 2로 바꾸었더니 잘되었다 라이브러리 선언 I2C_HandleTypeDef hi2c2
//  bno055_setup();
//  bno055_setOperationMode(BNO055_OPERATION_MODE_ACCGYRO);//작동 모드
//  bno055_setPowerMode(BNO55_POWER_MODE_LOWPOWER);//파워 모드
//  lcd_init();
//  HAL_Delay(30);
//  lcd_clear();
//  HAL_Delay(30);
//  lcd_send_string("B.Temp:");
//tm1637Init();
//tm1637SetBrightness(5);
//for(int i=0;i<1000;i++){
//tm1637DisplayDecimal(str-i, 1);
//}
  while (1)
  {

//	  bno055_vector_t gyro = bno055_getVectorGyroscope();// gyro라는 벡터에 각각 x,y,z값의 자이로값
//	  bno055_vector_t acc = bno055_getVectorAccelerometer();//acc라는 벡터에 각각 x,y,z값의 가속도
//	  printf("X_gyro: %.2f Y_gyro: %.2f Z_gyro: %.2f\r\n",gyro.x,gyro.y,gyro.z);
//	  printf("X_acc: %.2f Y_acc: %.2f Z_acc: %.2f\r\n",acc.x,acc.y,acc.z);
//	  HAL_Delay(1000);
  /* USER CODE BEGIN WHILE */
//	  txMessage.frame.id=0x80;//0x81
//	  txMessage.frame.idType=0x00;
//	  txMessage.frame.dlc=8;
//	  txMessage.frame.data0=0x00;
//	  txMessage.frame.data1=0x00;
//	  txMessage.frame.data2=0x00;
//	  txMessage.frame.data3=0x00;
//	  txMessage.frame.data4=0x00;
//	  txMessage.frame.data5=0x00;
//	  txMessage.frame.data6=0x00;
//	  txMessage.frame.data7=0x00;
//	  CANSPI_Transmit(&txMessage);
//	  HAL_Delay(50);
	  Data_Print(b);
//	  if(CANSPI_Receive(&rxMessage))
//	  {
//		  uint16_t torque_buff= ((uint16_t)rxMessage.frame.data5 << 8) | rxMessage.frame.data4;
//		  uint16_t temp_buff= ((uint16_t)rxMessage.frame.data1 << 8) | rxMessage.frame.data0;
//		  uint16_t vol_buff= ((uint16_t)rxMessage.frame.data3 << 8) | rxMessage.frame.data2;
//		  uint16_t current_buff= ((uint16_t)rxMessage.frame.data7 << 8) | rxMessage.frame.data6;
//		  printf("Temp:%d ",temp_buff);
//		  printf("Current:%d ",current_buff);
//		  printf("Voltage:%.2f ",(vol_buff)*0.0625);
//		  printf("Torque:%.2f \r\n",(torque_buff)*0.1);
//	  	  HAL_Delay(50);
//	  }
    /* USER CODE END WHILE */
//	  unsigned char data[16];
//	  HAL_UART_Receive_IT(&huart4,a,30);
//	  HAL_UART_Transmit(&huart2, &a[3], 3, 100);
	  HAL_Delay(10);
//
//	  LCD_Print(a);
//	  HAL_Delay(100);

	  //case 1:
//	  if(c==0){
//		  lcd_init();
//	  lcd_put_cur(0,0);
//	    lcd_send_string("M.Tq:");
//	    HAL_Delay(100);
//	    lcd_put_cur(1,0);
//	    lcd_send_string("M.Temp:");
//	  //  lcd_put_cur(0,7);
//	  //  lcd_send_string((int*)torque_buff);
//	  //  lcd_put_cur(1,7);
//	  //  lcd_send_string((int*)temp_buff);
//	  }
////
////	  //case 2:
//	  else if(c==1){
//		  lcd_init();
//		lcd_put_cur(0,0);
//	    lcd_send_string("B.Volt:");
//	    HAL_Delay(50);
//	    lcd_put_cur(1,0);
//	    lcd_send_string("B.Temp:");
//	    lcd_put_cur(0,7);
//	    lcd_send_string((char*)torque_buff);
//	    lcd_put_cur(1,7);
//	    lcd_send_string((char*)temp_buff);
	  }
  }


//    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if(GPIO_Pin==GPIO_PIN_7){
//		HAL_Delay(10);
//		        if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)==0)
//		        {
//		                printf("this is exti\r\n");
//		        }
//		printf("please\r\n");
//		a+=1;
//		a%=2;
//		printf("%d\r\n",a);
//	}
//	LCD_Print(a);
//}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_7)
    {
    	HAL_Delay(20);
//         printf("please\r\n");
        if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)==0)
        {
        	lcd_clear();
        	HAL_Delay(20);
		 	printf("%d\r\n",a);
			HAL_Delay(20);
			a+=1;
			b+=1;
			HAL_Delay(20);
			a%=2;
			b%=2;
			LCD_Print(a);
        }
    }
}
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
