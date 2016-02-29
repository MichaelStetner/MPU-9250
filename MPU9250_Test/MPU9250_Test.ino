/*

Communication test with MPU-9250
Michael Stetner
February 23, 2016

Communicate with the MPU-9250 over I2C and print the output to the Serial
monitor. Every 100ms, the Arduino asks the MPU-9250 for the contents of its
WHO_AM_I register. The response should be 0x71.

This uses Kris Winer's MPU9250 library (https://github.com/kriswiner/MPU-9250).

*/

#include <Wire.h>

#define ADO 1
#if ADO
#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#endif  
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71

unsigned int cnt = 0;

void setup() {
  Wire.begin();

  Serial.begin(9600);
  Serial.println("The MPU9250 should say that it is 0x71.");

}

void loop() {
  cnt = cnt + 1;
  Serial.print("Loop ");
  Serial.print(cnt);
  Serial.print(" - ");
  Serial.print("MPU9250 says it is ");

  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(WHO_AM_I_MPU9250);
  Wire.endTransmission();
  Wire.requestFrom(MPU9250_ADDRESS, 1);
  int value = Wire.read();
  Wire.endTransmission();
  Serial.println(value, HEX);
  delay(100);
}
