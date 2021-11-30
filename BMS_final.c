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
#include <SoftwareSerial.h>
/************************* Defines *****************************/
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1

#define DATALOG_DISABLED 0
#define limit_v 117.6
/**************** Local Function Declaration *******************/
SoftwareSerial temp(2,3);
uint8_t data[30];
uint8_t send_data[4];
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


void setup()
{ //BMS slave 
  pinMode(8,OUTPUT);
  uint8_t streg=0;
  int8_t error = 0;
  uint32_t conv_time = 0;
  int8_t s_pin_read=0;
  Serial.begin(9600);
  temp.begin(9600);
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
        BMS_IC[current_ic].pwm.tx_data[0]= 0xFF; // Duty cycle for S pin 2 and 1
        BMS_IC[current_ic].pwm.tx_data[1]= 0xFF; // Duty cycle for S pin 4 and 3
        BMS_IC[current_ic].pwm.tx_data[2]= 0xFF; // Duty cycle for S pin 6 and 5
        BMS_IC[current_ic].pwm.tx_data[3]= 0xFF; // Duty cycle for S pin 8 and 7
        BMS_IC[current_ic].pwm.tx_data[4]= 0xFF; // Duty cycle for S pin 10 and 9
        BMS_IC[current_ic].pwm.tx_data[5]= 0xFF; // Duty cycle for S pin 12 and 11
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
      LTC6811_set_cfgr_dis(TOTAL_IC,BMS_IC,DCCBITS_A);
      wakeup_idle(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
      LTC6811_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC,BMS_IC);
//      check_error(error);
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
  double mini=99,maxi=-99;
  uint8_t temp_max = 0,temp_min = 0;
  temp.readBytes(data,30);
//  for(int i=0;i<30;++i){
//     Serial.println(data[i]);
//  }
double total_v=0;
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++){
    if (datalog_en == 0){

    }
    else{
      for (int i=0; i<BMS_IC[0].ic_reg.cell_channels; i++){
        total_v +=(double)(BMS_IC[current_ic].cells.c_codes[i] * 0.0001);
        maxi = max(maxi,(double)(BMS_IC[current_ic].cells.c_codes[i] * 0.0001));
        mini = min(mini,(double)(BMS_IC[current_ic].cells.c_codes[i] * 0.0001));
      }
    }
  }
Serial.println(total_v);
real_vol = (total_v)*10;
//Serial.println(real_vol);
temp_max = data[0]-20;
temp_min = data[1]-20;
send_data[0] = data[0];
send_data[1] = data[1];
send_data[2] = real_vol/10;
send_data[3] = real_vol%10;
Serial.write(send_data,4);
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
    Serial.println(F("A PEC error was detected in the received data"));
  }
}