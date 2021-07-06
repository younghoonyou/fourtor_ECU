#include "MotorCtrl.h"
#include "stdio.h"
#include "spi.h"

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "stm32f4xx_hal_uart.h"

CANopen MotorCtrl::can = CANopen();


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//		if(huart->Instance==USART2){
//			if(Rx_indx==0){for(int i=0; i<100;i++)Rx_Buffer[i]=0;} // clear Rx_buffer before receiving new data
//			Rx_Buffer[Rx_indx++]=Rx_data[0];
//			if(Rx_indx==100){Rx_indx=0;EndOfTrans=1;}
//		}
//	}

void MotorCtrl::setup() {
//	extern SPI_HandleTypeDef hspi2;
	HAL_UART_RxCpltCallback(&huart2);
	printf("\n");
#ifdef DEBUG
  printf("DEBUG MODE\n");
#endif
  printf("Motor control setting up\n");
//  pinMode(IN_CONTROL_ACITVATE,OUTPUT);
//  pinMode(IN_MOTOR_ACTIVATE,OUTPUT);

  can.setup();

//  digitalWrite(IN_CONTROL_ACITVATE,HIGH);
#ifndef DEBUG
  do { // wait for bootup msg
    can.readCanBus();
    Serial.println(F("waiting for boot msg"));
  } while (can.canId!=0x700);
#endif

#ifndef DEBUG
  startController(); // send NMT msg to start connection
#endif

  setOperationMode(MODE_POSITION);

  setMotorReady(); // power ready mode
//  digitalWrite(IN_MOTOR_ACTIVATE,HIGH);

  HAL_Delay(10);
  printf("Motor control set up done\n");
  return;
}//setup


void MotorCtrl::startController() {
  can.sendSyncMsg();
  can.readCanBus();
  can.startOperational();
  can.readCanBus();
  return;
}

uint8_t MotorCtrl::setMotorReady() {
  can.write16bit(ADR_CONTROLWORD,0x00,0x06);
//  uint16_t status = 0;
#ifndef DEBUG
  do {
    can.read16bit(ADR_STATUSWORD,0x00,&status);
    Serial.println(F("waiting setMotorReady"));
  } while ((status&0x6F)!=0x21);
#endif
  return SUCCESS;
}

uint8_t MotorCtrl::setOperationMode(int8_t mode) {
  can.write8bit(ADR_MODES_OF_OPERATION,0x00,(uint8_t)mode);
//  uint8_t newMode = 255;
#ifndef DEBUG
  do {
    // read modes of operation display
    can.read8bit(ADR_MODES_OF_OPERATION_DISPLAY,0x00,&newMode);
    newMode = (int8_t)newMode;
    Serial.println(F("waiting setOperationMode"));
  } while (newMode!=mode);
#endif
  return SUCCESS;
}

uint8_t MotorCtrl::enableOperation() {
  // power motor on and enable operation
  can.write16bit(ADR_CONTROLWORD,0x00,0x0F);
//  uint16_t status = 0;
#ifndef DEBUG
  do {
    can.read16bit(ADR_STATUSWORD,0x00,&status);
    Serial.println(F("waiting enableOperation"));
  } while ((status&0x6F)!=0x27);
#endif
  return SUCCESS;
}

uint8_t MotorCtrl::newSetpoint(int32_t position, uint32_t velocity) {
  can.write32bit(ADR_TARGET_POSITION,0,(uint32_t)position);
  can.write32bit(ADR_PROFILE_VELOCITY,0,velocity);
  can.write32bit(ADR_END_VELOCITY,0,0x0);
  // can.write32bit(ADR_PROFILE_ACCELERATION,0,acc);
  // can.write32bit(ADR_PROFILE_DECELERATION,0,dec);
  // can.write16bit(ADR_MOTION_PROFILE_TYPE,0,RAMP_LINEAR);
  can.write16bit(ADR_MOTION_PROFILE_TYPE,0,RAMP_LIMITED_JERK);

  return 0; // 에러가 생겨서 수정한 부분.
}

uint8_t MotorCtrl::activateNewSetpoint() {
  can.write16bit(ADR_CONTROLWORD,0x00,0x1F);
  uint16_t status = 0;
  do {
    // wait until it is acknoledged in statusword
    can.read16bit(ADR_STATUSWORD,0x00,&status);
  } while ((status&0x1000)!=0x1000);
  // set the new setpoint low, since it is acknoledged
  can.write16bit(ADR_CONTROLWORD,0x00,0x0F);
  return SUCCESS;
}

uint8_t MotorCtrl::motorHalt() {
  // save way to bring the motor to a halt
  // triggers the target reached flag
  return can.write16bit(ADR_CONTROLWORD,0x00,0x8F);
}
