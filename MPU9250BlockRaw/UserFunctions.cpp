#include "UserTypes.h"
#include "Registers.h"
#include "I2C.h"
// User data functions.  Modify these functions for your data items.

// Start time for data
static uint32_t startMicros;

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  /*
   * Read one byte of data from a register in an I2C slave device.
   */
  uint8_t s = I2c.read(address, subAddress, 1u); // read one byte and store it in
                                                // I2c library's internal buffer
  if(s == 0 && I2c.available() >= 1) {
    return I2c.receive();
  } else { // if something went wrong, reset I2C communications
    I2c.end();
    I2c.begin();
    return 0;
  }
}

uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
  /*
   * Read multiple bytes from registers on an I2C slave device. Store the data in *dest.
   */
  uint8_t s = I2c.read(address, subAddress, count, dest);
  if(s != 0) { // if something went wrong, reset I2C communications
    I2c.end();
    I2c.begin();
  }
  return s;
}

// Acquire a data record.
void acquireData(data_t* data) {
  data->time = millis();
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, ADC_DIM, &data->adc[0]);
}

// Sensor setup
void userSetup() {
  // I2C (two-wire interface) for communicating with MPU-9250
  I2c.begin();
  I2c.timeOut(10);
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  if(c != 0x71) {
    Serial.println("MPU9250 did not respond correctly to who am i.");
  }
}
