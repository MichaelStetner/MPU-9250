#ifndef UserTypes_h
#define UserTypes_h
#include "Arduino.h"
// User data types.  Modify for your data items.
#define FILE_BASE_NAME "adc4pin"
const uint8_t ADC_DIM = 21;
struct data_t {
  uint32_t time;
  uint8_t adc[ADC_DIM];
};
void acquireData(data_t* data);
void printData(Print* pr, data_t* data);
void printHeader(Print* pr);
void userSetup();
uint8_t getI2cErrorCount();
#endif  // UserTypes_h
