#define MPU9250_ADDRESS_0 0x68
#define MPU9250_ADDRESS_1 0x69
#define INT_PIN_CFG     0x37


//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#include <Wire.h>

byte status1, status2, who;
bool dataReady, dataOverrun, sensorOverflow;
uint8_t magData[6];
const int num = 1;

void setup() {
  Serial.begin(38400);
  Wire.begin();
  delay(100);

  // Configure MPU9250 for pass thru. Now we can communicate directly with AK8963 magnetometer.
  if (num == 0) {
    writeByte(MPU9250_ADDRESS_0, INT_PIN_CFG, 0x02);
    writeByte(MPU9250_ADDRESS_1, INT_PIN_CFG, 0x00);
    Serial.print("Talking to magnetometer in MPU9250 with address 0x");
    Serial.println(MPU9250_ADDRESS_0, HEX);
  } else {
    writeByte(MPU9250_ADDRESS_0, INT_PIN_CFG, 0x00);
    writeByte(MPU9250_ADDRESS_1, INT_PIN_CFG, 0x02);
    Serial.print("Talking to magnetometer in MPU9250 with address 0x");
    Serial.println(MPU9250_ADDRESS_1, HEX);
  }
  delay(100);

  // Check communication with AK8963 magnetometer.
  who = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  Serial.print("AK8963 says it is 0x");
  Serial.println(who, HEX);
  Serial.println("It should be 0x48");
  Serial.println("");
  if (who != 0x48)
    while(1);
  delay(100);

  // Set up AK8963 for data acquisition. This puts it in continuous measurement mode 1 (8Hz) with 14-bit output. 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, B00000010);
}

void loop() {
  // Check for data ready and overrun
  status1 = readByte(AK8963_ADDRESS, AK8963_ST1);
  dataReady   = status1 & B00000001;
  dataOverrun = status1 & B00000010;
  status2 = readByte(AK8963_ADDRESS, AK8963_ST2);
  sensorOverflow = status2 & B00001000;
  if (!dataReady) {
    //printWithMillis("not ready");
    delay(100);
    return;
  }
  if (dataOverrun)
    printWithMillis("overrun");
  if (sensorOverflow)
    printWithMillis("sensor overflow");

  // If data is ready, read it
  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 6, &magData[0]);
  printWithMillis("data read");
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission();             // Send the Tx buffer, but send a restart to keep connection alive FIXME no more restart
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
  /*
   * Read multiple bytes from registers on an I2C slave device. Store the data in *dest.
   * Returns 0
   */
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission();
  Wire.requestFrom(address, count);
  for (int i=0; i<count; i++) {
    dest[i] = Wire.read();
  }
  return 0;
}

void printWithMillis(String message) {
  Serial.print(millis());
  Serial.print(" ");
  Serial.println(message);
}

uint8_t writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  /*
   * Write one byte of data to I2C slave register
   * Returns 0 on success
   */
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
  return 0;
}
