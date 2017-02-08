/*
  AK8963BasicSlave.ino
  Read from AK8963 compass as a slave to the MPU-9250
*/

#include "I2C.h"

#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define AK8963_XOUT_L    0x03  // data
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define MPU9250_ADDRESS  0x69
#define INT_PIN_CFG      0x37
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define EXT_SENS_DATA_00 0x49
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71

#define I2C_TIMEOUT_MS 10

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino online");

  // I2C (two-wire interface) for communicating with MPU-9250
  I2c.begin();
  I2c.timeOut(I2C_TIMEOUT_MS);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 says I AM 0x");
  Serial.println(c, HEX);
  Serial.println("MPU9250 should be 0x71");
  delay(500);

  // Set up magnetometer
  Serial.println("");
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02); // enable pass thru when i2c master disabled
  writeByte(MPU9250_ADDRESS, USER_CTRL, B00000000); // turn off i2c master on mpu9250
  delay(500);
  c = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  Serial.print("AK8963 says I AM 0x");
  Serial.println(c, HEX);
  Serial.println("AK8963 should be 0x48");
  writeByte(AK8963_ADDRESS, AK8963_CNTL, B00000010);

  // Enable I2C master functionality
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, B00001000);
  //  Bit   Name           Description
  //   7    MULT_MST_EN    I2C multimaster (1 to enable)
  //   6    WAIT_FOR_ES    DRDY interrupt waits for external data (1 to enable)
  //   5    SLV_3_FIFO_EN  Write SLV_3 data to FIFO (1 to enable)
  //   4    I2C_MST_P_NSR  Behavior between reads (0 to stop, 1 to reset)
  //  3:0   I2C_MST_CLK    I2C master clock speed (B1000 for 258kHz, slowest)
  
  // FIXME experiment with bit 4 - what to do between reads

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
/*
  // FIXME Set sample rate in register 25?
  // Optionally use Slave Delay Enable Registers in Register 103 and 52

  // Can use I2C_MST_RST bit (Register 106) to reset MPU9250's I2C master functionality

  // Data is in EXT_SENS_DATA_00 (Register 73)
  */
}

void loop(){
  Serial.print(millis());
  //Serial.print(" External sensor says");
  for (int regaddr=73; regaddr<=78; regaddr++) {
    byte c = readByte(MPU9250_ADDRESS, regaddr);
    Serial.print(',');
    Serial.print(c, DEC);
  }
  Serial.println("");
  delay(100);
}


uint8_t writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  /*
   * Write one byte of data to I2C slave register
   */
  uint8_t s = I2c.write(address, subAddress, data);
  if(s != 0) { // if something went wrong, reset I2C communications
    I2c.end();
    I2c.begin();
  }
  return s;
}

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

