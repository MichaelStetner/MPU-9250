#include "UserTypes.h"
#include "Registers.h"
#include <i2c_t3.h>

#define SYNC_PIN 0
#define SYNC_RATE  50 // integer between 0 (no sync pulses) and 10000 (sync pulse on every cycle)
#define SYNC_MICROSECONDS 30

#define GYRO_SCALE 3 // 0=250dps, 1=500dps, 2=1000dps, 3=2000dps
#define ACCEL_SCALE 1 // 0=2g, 1=4g, 2=8g, 3=16g


// User data functions.  Modify these functions for your data items.

unsigned long myrand;
uint8_t syncNow;
uint8_t i2cErrorCount = 0;

uint8_t getI2cErrorCount() {
  return i2cErrorCount;
}

uint8_t writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  /*
   * Write one byte of data to I2C slave register
   * Returns 0 on success
   *         1 if data is too long
   *         2 if received NACK on address
   *         3 if received NACK on data
   *         4 if other error
   */
  Wire1.beginTransmission(address);
  Wire1.write(subAddress);
  Wire1.write(data);
  Wire1.endTransmission();
  uint8_t i2cErrorCode = Wire1.getError();
  if(i2cErrorCode != 0) {  // if write was not successful
    Wire1.resetBus();
  }
  return i2cErrorCode;
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  /*
   * Read one byte of data from a register in an I2C slave device.
   * Returns the data. If no data is received, returns 0.
   */
  Wire1.beginTransmission(address);
  Wire1.write(subAddress);
  Wire1.endTransmission();
  Wire1.requestFrom(address, (uint8_t) 1);
  if (Wire1.available()) {
    return Wire1.readByte();
  } else {
    return 0;
  }
}

uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
  /*
   * Read multiple bytes from registers on an I2C slave device. Store the data in *dest.
   * Returns 0
   */
  Wire1.beginTransmission(address);
  Wire1.write(subAddress);
  Wire1.endTransmission();
  Wire1.requestFrom(address, count);
  for (int i=0; i<count; i++) {
    dest[i] = Wire1.read();
  }
  return 0;
}


void initMPU9250(uint8_t MPU9250_ADDRESS) {
  checkWho(MPU9250_ADDRESS, WHO_AM_I_MPU9250, 0x71);
  
  // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                    // determined inset in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | GYRO_SCALE << 3; // Set full scale range for the gyro (3 for 2000 dps)
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

 // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | ACCEL_SCALE << 3; // Set full scale range for the accelerometer (1 for 4g)
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}


// Acquire a data record.
void acquireData(data_t* data) {
  data->time = millis();
  readBytes(MPU9250_ADDRESS_0, ACCEL_XOUT_H, IMU_DIM, &data->adc[0]);
  readBytes(MPU9250_ADDRESS_1, ACCEL_XOUT_H, IMU_DIM, &data->adc[IMU_DIM]);

  // Emit sync pulses at random
  myrand = random(10000);
  syncNow = (myrand < SYNC_RATE);
  if(syncNow) {
    digitalWrite(SYNC_PIN, HIGH);
    delayMicroseconds(SYNC_MICROSECONDS);
    digitalWrite(SYNC_PIN, LOW);
  }
  data->adc[ADC_DIM - 1] = syncNow;
}

// Sensor setup
void userSetup() {
  // I2C (two-wire interface) for communicating with MPU-9250
  Wire1.begin();
  Wire1.setDefaultTimeout(10000);  // ten milliseconds
  initMPU9250(MPU9250_ADDRESS_0);
  initAK8963(MPU9250_ADDRESS_0);
  initMPU9250(MPU9250_ADDRESS_1);
  initAK8963(MPU9250_ADDRESS_1);
  pinMode(SYNC_PIN, OUTPUT);
}

/*
 * Checks that device responds correctly to WHO_AM_I. If it fails, 
 * an error message is printed to Serial and execution stops.
 */
void checkWho(uint8_t deviceAddr, uint8_t regAddr, uint8_t correct) {
  byte whoIAm = readByte(deviceAddr, regAddr);
  if (whoIAm != correct) {
    Serial.print("Device at address 0x");
    Serial.print(deviceAddr, HEX);
    Serial.println(" did not respond correctly to WHO_AM_I.");
    Serial.print("Expected 0x");
    Serial.print(correct, HEX);
    Serial.print(" but recieved 0x");
    Serial.println(whoIAm, HEX);
    while(1);
  }
}

/*
 * Initialize AK8963 magnetometer that is inside the MPU9250 and
 * set it up to be an I2C slave
 */
void initAK8963(uint8_t MPU9250_ADDRESS) {  
  // Tell MPU9250 to let us talk to AK8963 directly
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02); // enable pass thru when i2c master disabled
  writeByte(MPU9250_ADDRESS, USER_CTRL, B00000000); // turn off i2c master on mpu9250
  delay(500);

  
  checkWho(AK8963_ADDRESS, WHO_AM_I_AK8963, 0x48);
  
  writeByte(AK8963_ADDRESS, AK8963_CNTL, B00000010);

  // Enable I2C master functionality on MPU9250
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, B00001000);
  //  Bit   Name           Description
  //   7    MULT_MST_EN    I2C multimaster (1 to enable)
  //   6    WAIT_FOR_ES    DRDY interrupt waits for external data (1 to enable)
  //   5    SLV_3_FIFO_EN  Write SLV_3 data to FIFO (1 to enable)
  //   4    I2C_MST_P_NSR  Behavior between reads (0 to stop, 1 to reset)
  //  3:0   I2C_MST_CLK    I2C master clock speed (B1000 for 258kHz, slowest)

  // Configure to read from WHO_AM_I register of AK8963
  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, 128 + AK8963_ADDRESS);
  // First bit is "1" for read

  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AK8963_XOUT_L);
  // Value read should be 0x48

  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, B10000111);
  //  Bit   Name              Description
  //   7    I2C_SLV0_EN       Use this slave device (1 to enable)
  //   6    I2C_SLV0_BYTE_SW  Swap bytes in a word? (1 to enable)
  //   5    I2C_SLV0_REG_DIS  "When set, the transaction does not write a register value, it will only read data, or write data"
  //   4    I2C_SLV0_GRP      Grouping of bytes into words (0 for [0 1] [2 3]...; 1 for 0 [1 2] [3 4]...)
  //  3:0   I2C_MST_CLK       Number of bytes to read (seven bytes = B0111)

  // Enable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, B00100000);  
}
