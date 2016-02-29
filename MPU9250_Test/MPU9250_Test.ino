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
#include <MPU9250.h>

void setup() {
  Wire.begin();

  Serial.begin(9600);
  Serial.println("The MPU9250 should say that it is 0x71.");

  MPU9250 mpu9250;

  unsigned int cnt = 0;
}

void loop() {
  Serial.print("Loop ");
  Serial.print(cnt);
  Serial.print(" - ");
  Serial.print("MPU9250 says it is ");
  unsigned int whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.println(whoami, HEX);
  delay(100);
}
