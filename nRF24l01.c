#include <SPI.h> 
#include <nRF24L01.h>
#include <RF24.h>
#include <string.h>
RF24 radio(9, 6); // in SPI bus to set nRF24L01 radio 

const uint64_t address = 0xE8E8F0F0E1LL;// Pipeline
struct data{
  int RPM;
  float imu_x,imu_y,imu_z,motor_torq;
  uint8_t B_vol_int,B_vol_div,temp1,temp2,temp3,temp4,temp_max,motor_temp;
};//Data struct
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MAX); // To set about voltage level. If module is close you should set at least

  radio.setAutoAck(false);
  radio.openReadingPipe(1,address);//you can set RF24_PA_MIN / RF24_PA_LOW / RF24_PA_HIGH / RF24_PA_MAX
  //for your module distance
  //For MAX level to operate you should use by-pass capacitor between 3.3V and GND
  radio.startListening(); //Setting Module to Listener
}
void loop() {//Read ECU Data and print to use Node.js
  if (radio.available()){ 
  radio.read(data,28);
  Serial.print(data.RPM);
  Serial.print(" ");
  Serial.print(data.imu_x);
  Serial.print(" ");
  Serial.print(data.imu_y);
  Serial.print(" ");
  Serial.print(data.imu_z);
  Serial.print(" ");
  Serial.print(data.B_vol_int);
  Serial.print(" ");
  Serial.print(data.B_vol_div);
  Serial.print(" ");
  Serial.print(data.temp1);
  Serial.print(" ");
  Serial.print(data.temp2);
  Serial.print(" ");
  Serial.print(data.temp3);
  Serial.print(" ");
  Serial.print(data.temp4);
  Serial.print(" ");
  Serial.print(data.temp_max);
  Serial.print(" ");
  Serial.print(data.motor_temp);
  Serial.print(" ");
  Serial.print(data.motor_torq);
  Serial.println();
}
delay(500);
}