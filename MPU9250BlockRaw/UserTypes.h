#ifndef UserTypes_h
#define UserTypes_h
#include "Arduino.h"
// User data types.  Modify for your data items.
#define FILE_BASE_NAME "adc4pin"
const uint8_t IMU_DIM = 20;
const uint8_t ADC_DIM = 2 * IMU_DIM + 1;
struct data_t {
  uint32_t time;
  uint8_t adc[ADC_DIM];
};

void acquireData1(data_t* data);
void acquireData2(data_t* data);
void userSetup();


// I2C functions
uint8_t writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);
uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
void requestBytes(uint8_t address, uint8_t subAddress, uint8_t count);
uint8_t getI2cErrorCount();
uint8_t rdd();
void print_i2c_status(void);
void i2c_finish(void);


// IMU functions
void initMPU9250(uint8_t MPU9250_ADDRESS);
void initAK8963(uint8_t MPU9250_ADDRESS);
void checkWho(uint8_t deviceAddr, uint8_t regAddr, uint8_t correct);

#endif  // UserTypes_h
