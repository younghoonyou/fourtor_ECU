#include <SPI.h> 
#include <nRF24L01.h>
#include <RF24.h>
#include <string.h>
RF24 radio(9, 6); // SPI 버스에 nRF24L01 라디오를 설정하기 위해 CE, CSN 선언.

const uint64_t address = 0xE8E8F0F0E1LL;
struct data{
  int RPM;
  float imu_x,imu_y,imu_z,motor_torq;
  uint8_t B_vol_int,B_vol_div,temp1,temp2,temp3,temp4,temp_max,motor_temp;
};
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MAX); //전원공급에 관한 파워레벨을 설정합니다. 모듈 사이가 가까우면 최소로 설정합니다.
  radio.setAutoAck(false);
  radio.openReadingPipe(1,address);
//거리가 가까운 순으로 RF24_PA_MIN / RF24_PA_LOW / RF24_PA_HIGH / RF24_PA_MAX 등으로 설정할 수 있습니다.
//높은 레벨(거리가 먼 경우)은 작동하는 동안 안정적인 전압을 가지도록 GND와 3.3V에 바이패스 커패시터 사용을 권장함
  radio.startListening(); //모듈을 수신기로 설정
}
void loop() {
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