#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "LT_I2C.h"          
#include "QuikEval_EEPROM.h"
#include "UserInterface.h"   
#include "LTC681x.h"
#include "LTC6811.h"
#include <stdio.h>
#include "stdlib.h"
/************************* Defines *****************************/
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1

#define DATALOG_DISABLED 0
#define limit_v 117.6
/**************** Local Function Declaration *******************/
uint8_t buff[30];
void measurement_loop(uint8_t datalog_en);
void print_cells(uint8_t datalog_en);
void check_error(int error);
/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
const uint8_t TOTAL_IC = 4;//!< Number of ICs in the daisy chain
const uint8_t ADC_OPT = ADC_OPT_DISABLED; //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED; //!< Discharge Permitted 
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL; //!< Register Selection 
const uint8_t SEL_REG_A = REG_1; //!< Register Selection 
const uint8_t SEL_REG_B = REG_2; //!< Register Selection 

const uint16_t MEASUREMENT_LOOP_TIME = 500; //!< Loop Time in milliseconds(ms)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = 30000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

//Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL CAPS
const uint8_t WRITE_CONFIG = ENABLED;  //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = ENABLED; //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED; //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = ENABLED; //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = ENABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t PRINT_PEC = ENABLED; //!< This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop


cell_asic BMS_IC[TOTAL_IC]; //!< Global Battery Variable


bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool GPIOBITS_A[5] = {true,true,true,true,true}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
uint16_t UV=UV_THRESHOLD; //!< Under-voltage Comparison Voltage
uint16_t OV=OV_THRESHOLD; //!< Over-voltage Comparison Voltage
bool DCCBITS_A[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool DCTOBITS[4] = {false, false, false, true}; //!< Discharge time value // Dcto 0,1,2,3 // Programed for 4 min 
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */


/* 여기는 배터리 온도 인자*/
int s01 = 6;
int s11 = 7;
int s21 = 8;
int s31 = 9;
int SIG_pin1 = A0;

int s02 = 2;
int s12 = 3;
int s22 = 4;
int s32 = 5;
int SIG_pin2 = A1;

float R1 = 10000;
float logR2, R2, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.0192026e-07;

void setup()
{ //BMS slave 
  pinMode(8,OUTPUT);
  uint8_t streg=0;
  int8_t error = 0;
  uint32_t conv_time = 0;
  int8_t s_pin_read=0;
  Serial.begin(9600);
  spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock
  LTC6811_init_cfg(TOTAL_IC, BMS_IC);
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
  {
    LTC6811_set_cfgr(current_ic,BMS_IC,REFON,ADCOPT,GPIOBITS_A,DCCBITS_A, DCTOBITS, UV, OV);
  }
  
  LTC6811_reset_crc_count(TOTAL_IC,BMS_IC);
  LTC6811_init_reg_limits(TOTAL_IC,BMS_IC);

    wakeup_sleep(TOTAL_IC);
      for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
      {
        BMS_IC[current_ic].pwm.tx_data[0]= 0x88; // Duty cycle for S pin 2 and 1
        BMS_IC[current_ic].pwm.tx_data[1]= 0x88; // Duty cycle for S pin 4 and 3
        BMS_IC[current_ic].pwm.tx_data[2]= 0x88; // Duty cycle for S pin 6 and 5
        BMS_IC[current_ic].pwm.tx_data[3]= 0x88; // Duty cycle for S pin 8 and 7
        BMS_IC[current_ic].pwm.tx_data[4]= 0x88; // Duty cycle for S pin 10 and 9
        BMS_IC[current_ic].pwm.tx_data[5]= 0x88; // Duty cycle for S pin 12 and 11
        LTC6811_wrcfg(TOTAL_IC,BMS_IC);
      }          
      LTC6811_wrpwm(TOTAL_IC,0,BMS_IC);
      wakeup_idle(TOTAL_IC);


      wakeup_sleep(TOTAL_IC);
      /**************************************************************************************
         S pin control. 
         1)Ensure that the pwm is set according to the requirement using the previous case.
         2)Choose the value depending on the required number of pulses on S pin. 
         Refer to the data sheet. 
      ***************************************************************************************/
      for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
      {
        BMS_IC[current_ic].sctrl.tx_data[0]=0xFF; // No. of high pulses on S pin 2 and 1
        BMS_IC[current_ic].sctrl.tx_data[1]=0xFF; // No. of high pulses on S pin 4 and 3
        BMS_IC[current_ic].sctrl.tx_data[2]=0xFF; // No. of high pulses on S pin 6 and 5
        BMS_IC[current_ic].sctrl.tx_data[3]=0xFF; // No. of high pulses on S pin 8 and 7
        BMS_IC[current_ic].sctrl.tx_data[4]=0xFF; // No. of high pulses on S pin 10 and 9
        BMS_IC[current_ic].sctrl.tx_data[5]=0xFF; // No. of high pulses on S pin 12 and 11
        LTC6811_wrcfg(TOTAL_IC,BMS_IC);
      }
      LTC6811_wrsctrl(TOTAL_IC,streg,BMS_IC);
      // Start S Control pulsing
      wakeup_idle(TOTAL_IC);
      LTC6811_stsctrl();

      // Read S Control Register Group 
      wakeup_idle(TOTAL_IC);
      error=LTC6811_rdsctrl(TOTAL_IC,streg,BMS_IC);
//      check_error(error);


/*먹스 초기 설정*/
  pinMode(s01, OUTPUT);
  pinMode(s11, OUTPUT);
  pinMode(s21, OUTPUT);
  pinMode(s31, OUTPUT);

  pinMode(s02, OUTPUT);
  pinMode(s12, OUTPUT);
  pinMode(s22, OUTPUT);
  pinMode(s32, OUTPUT);
  digitalWrite(s01,LOW);
  digitalWrite(s11,LOW);
  digitalWrite(s21,LOW);
  digitalWrite(s31,LOW);

  digitalWrite(s02,LOW);
  digitalWrite(s12,LOW);
  digitalWrite(s22,LOW);
  digitalWrite(s32,LOW);

  
}

/*!*********************************************************************
 \brief Main loop
 @return void
***********************************************************************/
void loop()
{
   wakeup_sleep(TOTAL_IC); // 절전 모드 
   LTC6811_wrcfg(TOTAL_IC,BMS_IC);
   measurement_loop(DATALOG_ENABLED);
}

void measurement_loop(uint8_t datalog_en)
{
  int8_t error = 0;
  char input = 0;
  
  while (input != 'm')
  {
    for (uint8_t current_ic =0 ; current_ic < TOTAL_IC; current_ic++){
    if(BMS_IC[current_ic].stat.flags[0]==0xAA){
      digitalWrite(10,LOW);
//      Serial.println("good!");
      }
    else
    {
      digitalWrite(10,HIGH);
    }
 }
     if (Serial.available() > 0)
      {
        input = read_char();
      }
    if (MEASURE_CELL == ENABLED)
    {
      //////////////LTC6811_set_cfgr_dis(TOTAL_IC,BMS_IC,DCCBITS_A);
      wakeup_idle(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
      LTC6811_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,BMS_IC);
//      check_error(error);
      print_temp(datalog_en);
      print_cells(datalog_en);
    }
    delay(MEASUREMENT_LOOP_TIME);

    
  }
}

/*!*********************************
  \brief Prints the main menu
 @return void
***********************************/


/*!******************************************************************************
 \brief Prints the configuration data that is going to be written to the LTC6811
 to the serial port.
 @return void
 ********************************************************************************/

void print_cells(uint8_t datalog_en)
{
  float percent_of_volt=0;
  int real_vol=0;
  uint8_t temp_max = 0,temp_min = 0;
  double total_v=0;
  double maxi=-99,mini=99;
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++){
//    double total_v=0;
    Serial.print("["); //입증용
    Serial.print(current_ic+1);  //입증용
    Serial.print("]");  //입증용
    Serial.print(":");  //입증용
    if (datalog_en == 0){

    }
    else{
//      double total_v=0;
      for (int i=0; i<BMS_IC[0].ic_reg.cell_channels; i++){
        total_v +=(double)(BMS_IC[current_ic].cells.c_codes[i] * 0.0001);
        maxi = max(maxi,(double)(BMS_IC[current_ic].cells.c_codes[i] * 0.0001));
        mini = max(mini,(double)(BMS_IC[current_ic].cells.c_codes[i] * 0.0001));
        Serial.print("["); //입증용
        Serial.print((double)(BMS_IC[current_ic].cells.c_codes[i] * 0.0001)); //입증용
        Serial.print("]");  //입증용
      }
    Serial.print("["); //입증용
        Serial.print(total_v); //입증용
        Serial.print("]");  //입증용       
    }
    Serial.print("["); //입증용
        Serial.print(total_v); //입증용
        Serial.print("]");  //입증용
    Serial.println();//입증용
    Serial.println();//입증용
  }
//real_vol = (total_v)*10;
temp_max = buff[0];
temp_min = buff[1];
buff[2] = real_vol/10;
buff[3] = real_vol%10;
//Serial.write(buff,4);
if(maxi>4.2||mini<2.8||temp_max>60||temp_min<-20){
  digitalWrite(8,LOW);
}
else{
  digitalWrite(8,HIGH);
}
      delay(500);
}
void check_error(int error)
{
  if (error == -1)
  {
//    Serial.println(F("A PEC error was detected in the received data"));
  }
}
/*온도 소자 먹스*/
void print_temp(uint8_t datalog_en){
  int maxT1 = -99;
  int minT1 = 99;
  int maxT2 = -99;
  int minT2 = 99;
  if(datalog_en == 0){
    
  }
  else{
    for(int i=0; i<14; i++){
    Serial.print("[");//입증용
    Serial.print("Mux1");//입증용
    Serial.print("]");//입증용
    Serial.print(":");//입증용
    Serial.print(readMux1(i));//입증용
    Serial.print(" ");//입증용
    maxT1 = max(maxT1,readMux1(i));
    minT1 = max(minT1,readMux1(i));
    }
//    Serial.println();//입증용
    for(int i=0; i<14; i++){
      Serial.print("[");//입증용
    Serial.print("Mux2");//입증용
    Serial.print("]");//입증용
    Serial.print(":");//입증용
    Serial.print(readMux2(i));//입증용
    Serial.print(" ");//입증용
    maxT2 = max(maxT2,readMux2(i));
    minT2 = max(minT2,readMux2(i));
    }
    Serial.println();//입증용
    Serial.println();//입증용
  }
    int real_max = max(maxT1,maxT2);
    int real_min = min(minT1,minT2);
    buff[0] = real_max + 20;
    buff[1] = real_min + 20;
    delay(500);
}



int readMux1(int channel){
  int controlPin[] = {s01,s11,s21,s31};
  int muxChannel[14][4] = 
  {
    {0,0,0,0}, //channel 0
    {1,0,0,0}, //channel 1
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6 no use
    
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1}  //channel 15
      };
 
  for(int i =0; i<4; i++){
    digitalWrite(controlPin[i],muxChannel[channel][i]);
      
  }
  int  Vo = analogRead(SIG_pin1);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  Tf = (Tc * 9.0)/5.0 + 32.0;
  return Tc;
 
}

int readMux2(int channel){
  int controlPin[] = {s02,s12,s22,s32};
  int muxChannel[14][4] = 
  {
    {0,0,0,0}, //channel 0
    {1,0,0,0}, //channel 1
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6 no use
    
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1}  //channel 15
      };
 
  for(int i =0; i<4; i++){
    digitalWrite(controlPin[i],muxChannel[channel][i]);
      
  }
  int  Vo = analogRead(SIG_pin2);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  Tf = (Tc * 9.0)/5.0 + 32.0;
  return Tc;
 
}