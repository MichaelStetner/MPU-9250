#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define INT_PIN_CFG 0x37
#define WHO_AM_I_AK8963  0x00 // should return 0x48

#include <Wire.h>

const int loop_delay_ms = 10;

void setup() {
  Serial.begin(38400);
  Wire.begin();

  // Configure MPU9250 for pass thru. Now we can communicate directly with AK8963.
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x02);
  Wire.endTransmission();
  delay(100);

  // who am i for ak8963
  Wire.beginTransmission(AK8963_ADDRESS);
  Wire.write(WHO_AM_I_AK8963);
  Wire.endTransmission();
  Wire.requestFrom(AK8963_ADDRESS, 1);
  int wia_ak8963 = Wire.read();
  Serial.print("AK8963 says it is ");
  Serial.println(wia_ak8963, HEX);
  Serial.println("It should be 0x48");
  Serial.println("");
  delay(100);
}

void loop() {
  delay(loop_delay_ms);

  // Check if data is ready

  // If data is ready, read it
  

}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  uint8_t s = Wire.endTransmission();             // Send the Tx buffer, but send a restart to keep connection alive FIXME no more restart
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}
