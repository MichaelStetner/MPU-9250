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
void acquireData(data_t* data);
void printData(Print* pr, data_t* data);
void printHeader(Print* pr);
void userSetup();


uint8_t getI2cErrorCount();
void initMPU9250(uint8_t MPU9250_ADDRESS);
void initAK8963(uint8_t MPU9250_ADDRESS);
void checkWho(uint8_t deviceAddr, uint8_t regAddr, uint8_t correct);

#endif  // UserTypes_h
