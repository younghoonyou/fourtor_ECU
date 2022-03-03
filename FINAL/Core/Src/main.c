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
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "bno055.h"
#include "string.h"
#include "CANSPI.h"
#include "MCP2515.h"
#include <stdlib.h>
#include "i2c-lcd.h"
#include "stm32_tm1637.h"
#include "MY_NRF24.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart6;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi3;
TIM_HandleTypeDef htim1;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t receive_data[7];

const uint64_t TxpipeAddrs = 0xE8E8F0F0E1LL; // RF pipe to send
struct {
	int rpm;//for Motor Data
	float imu_x,imu_y,imu_z,motor_torq;//for IMU and Motor Data
	uint8_t B_vol_int,B_vol_div,temp1,temp2,temp3,temp4,temp_max,motor_temp;//for telemetry
}mxTxData;//To send data
char AckPayload[32];
char string_voltage[6];
char string_temperature[6];
char RPM_RR[10];
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
int row=0;
int col=0;
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
  MX_SPI3_Init();
  MX_I2C2_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Infinite loop */
  enum notes {// For RTDS ready to drive sound
      C4 = 26162,
      D4 = 29366,
      E4 = 32962,
      F4 = 34922,
      G4 = 39199,
	  B4 = 49388,
	  bE4 = 31112,
	  bA4 = 41530,
	  bB4 = 46616,
      C5 = 52325,
	  B5 = 98776,
	  D5 = 58733,
	  bE5 = 62225,
	  E5 = 65925,
	  F5 = 69845,
	  G5 = 78399,
	  bA5 = 83060,
	  bB5 = 93232,
    };
  enum notes A[] = {C4,D4,E4,F4,G4,C5,D5,E5,F5,G5};// To play music

  while(1){// For start buzzer
	GPIO_PinState pin_state= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9);
	if(pin_state==GPIO_PIN_SET){
		HAL_Delay(200);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		int PCLK1_Freq = HAL_RCC_GetPCLK2Freq();//initial PCLK
		__HAL_TIM_SET_PRESCALER(&htim1,899);//Set to prescaler
		int size = sizeof(A/sizeof(enum notes));
		for (int i = 0 ; i < size ; i++) {
		      __HAL_TIM_SET_AUTORELOAD(&htim1, PCLK1_Freq / A[i]*0.1 );
		      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PCLK1_Freq / A[i]*0.1 / 2);
		      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		      HAL_Delay(500);
		      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		      if(i==size - 1) HAL_Delay(750); // if music is done
		    }
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		break;
	}
	HAL_Delay(1000);
}
  //------------------------------------ **** TRANSMIT - ACK ****------------------------------------//
	nrf24_DebugUART_Init(huart2);
    NRF24_begin(GPIOB, GPIO_PIN_6, GPIO_PIN_4, hspi3);

    NRF24_setPALevel(RF24_PA_0dB);
  	NRF24_setAutoAck(false);
  	NRF24_setChannel(80);
  	NRF24_setPayloadSize(28);
  	NRF24_setDataRate(RF24_2MBPS);
  	NRF24_openWritingPipe(TxpipeAddrs);
  	NRF24_stopListening();

  	//------------------------------------ **** MCP2515 Setting ****------------------------------------//
    CANSPI_Initialize();
	//------------------------------------ **** LCD Setting ****------------------------------------//
    lcd_init();
	lcd_clear();
  	HAL_Delay(20);
  	lcd_put_cur(0,0);
  	HAL_Delay(20);
  	lcd_send_string((char*)"B.Temp:");
  	lcd_put_cur(1,0);
  	HAL_Delay(20);
  	lcd_send_string((char*)"B.Vol:");
  	//------------------------------------ **** Segment Setting ****------------------------------------//
  	tm1637Init();
  	tm1637SetBrightness(3);
  	//------------------------------------ **** IMU Setting ****------------------------------------//
  	bno055_calibration_data_t calData;
  	bno055_assignI2C(&hi2c1);
  	bno055_setup();
	bno055_setCalibrationData(calData);//Set init
  	bno055_setOperationMode(BNO055_OPERATION_MODE_IMU);//작동 모드
  	bno055_setPowerMode(BNO55_POWER_MODE_LOWPOWER);//파워 모드
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //recieve_data : Highest Temperature , Voltage Int, Voltage float , 1~4 segment Temperature Data
	  HAL_UART_Receive(&huart4,receive_data,4,200);// Read Voltage and Celus temperature Data
	  sprintf(string_voltage,"%d.%dV",receive_data[1],receive_data[2]);//Make String Voltage Data
	  sprintf(string_temperature,"%d'C",receive_data[0]);//Make String temperature Data
	  lcd_put_cur(0,7);//Move Cursor
	  HAL_Delay(50);
	  lcd_send_string((char*)string_temperature); // Battery Temperature
	  lcd_put_cur(1,7);//Move Cursor
	  HAL_Delay(50);
	  lcd_send_string((char*)string_voltage); // Battery Voltage
	  	//------------------------------------ **** IMU ****------------------------------------//
	  	  	  bno055_vector_t gyro = bno055_getVectorGyroscope();// gyro vector for x,y,z gryoscope
	  		  bno055_vector_t acc = bno055_getVectorAccelerometer();//acc vector for x,y,z accelerometer
//	  		//------------------------------------ **** CAN_Motor ****------------------------------------//
	  		  HAL_Delay(200);
	  		  txMessage.frame.id=0x80;// To Listen Motor PDO data
	  	 	  txMessage.frame.idType=0x00;
	  	 	  txMessage.frame.dlc=8;// Data length
	  	 	  txMessage.frame.data0=0x00;// To get PDO data
	  	 	  txMessage.frame.data1=0x00;
	  	 	  txMessage.frame.data2=0x00;
	  	 	  txMessage.frame.data3=0x00;
	  	 	  txMessage.frame.data4=0x00;
	  	 	  txMessage.frame.data5=0x00;
	  	 	  txMessage.frame.data6=0x00;
	  	 	  txMessage.frame.data7=0x00;
	  	 	  CANSPI_Transmit(&txMessage);// Send Tx Message to get PDO data
	  	 	  HAL_Delay(200);
	  	 	  if(CANSPI_Receive(&rxMessage))// If Message sent well
	  	 	  {
	  	 		  uint16_t RPM_1= ((uint16_t)rxMessage.frame.data1 << 8) | rxMessage.frame.data0;
	  	 		  uint16_t RPM_2= ((uint16_t)rxMessage.frame.data3 << 8) | rxMessage.frame.data2;
	  	 		  int RPM=	((int)RPM_2 << 16) | RPM_1; // Motor RPM data 4bytes
	  	 		  uint16_t torque_buff= ((uint16_t)rxMessage.frame.data5 << 8) | rxMessage.frame.data4;// Motor Torque data 2bytes
	  	 		  uint16_t temp_buff= ((uint16_t)rxMessage.frame.data1 << 8) | rxMessage.frame.data0;// Motor Temperature 2bytes
	  	 		  sprintf(RPM_RR,"%d",RPM);
	  	 	  	  HAL_Delay(50);
	  	 	 }
	  	//------------------------------------ **** 7Segment ****------------------------------------//

	  	 		  tm1637DisplayDecimal(RPM_RR,1);//Display 7-segment for motor data

	  	//------------------------------------ **** RF_Trans ****------------------------------------//
	  	 	mxTxData.rpm = RPM;

	  	 	mxTxData.imu_x=acc.x;
			mxTxData.imu_y=acc.x;
			mxTxData.imu_z=acc.z;

			mxTxData.motor_temp = temp_buff;
			mxTxData.motor_torq = torque_buff * 0.1;

			mxTxData.B_vol_div = receive_data[2];
			mxTxData.B_vol_int = receive_data[1];
			mxTxData.temp_max= receive_data[0];
			mxTxData.temp1= receive_data[3];
			mxTxData.temp2= receive_data[4];
			mxTxData.temp3= receive_data[5];
			mxTxData.temp4= receive_data[6];
	  if(NRF24_write(txPackage, 28)){// Total Data is 28 bytes and if that goes well
		HAL_UART_Transmit(&huart2, (uint8_t *)"Transmitted Successfully\r\n", strlen("Transmitted Successfully\r\n"), 10);
	  }
	  		  HAL_Delay(100);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
//}
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
