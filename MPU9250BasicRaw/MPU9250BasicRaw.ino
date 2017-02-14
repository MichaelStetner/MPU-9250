#include "I2C.h"
#include <SPI.h>
#include <SD.h>
#include "Registers.h"

// Pin assignments
// On Arduino UNO, Pins 11, 12, 13 are used for SPI with the data logger shield
#define CHIP_SELECT_PIN 10
#define SYNC_PIN 9

#define SYNC_RATE  50 // integer between 0 (no sync pulses) and 10000 (sync pulse on every cycle)
#define GYRO_SCALE 3 // 0=250dps, 1=500dps, 2=1000dps, 3=2000dps
#define ACCEL_SCALE 1 // 0=2g, 1=4g, 2=8g, 3=16g
#define I2C_TIMEOUT_MS 10
#define SYNC_MICROSECONDS 30
#define BYTES_PER_DATA_SAMPLE 18 // 9 readings * 2 bytes per reading
#define FIFO_POLLING_INTERVAL 20 // milliseconds

#define DS1307_ADDRESS  0x68

// MPU9250 Registers
#define MPU9250_ADDRESS 0x69
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38

// AK8963 Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define AK8963_XOUT_L    0x03  // data
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define INT_PIN_CFG      0x37
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define EXT_SENS_DATA_00 0x49
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP

File myFile;
char filename[] = "GD_XXXX.LOG";
uint8_t rtcout[7]; // output from real time clock
const int SAMPLES_PER_BLOCK = 251 / BYTES_PER_DATA_SAMPLE;
const int FIFO_BLOCK_SIZE = SAMPLES_PER_BLOCK * BYTES_PER_DATA_SAMPLE;
const int DATA_LEN = FIFO_BLOCK_SIZE + 5; // DATA_LEN is maxiumum 256
uint8_t data[DATA_LEN];
uint8_t syncPulseEmitted;
unsigned long ms;


uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }

void dateTime(uint16_t* date, uint16_t* time) {
  readBytes(DS1307_ADDRESS, 0, 7, &rtcout[0]);
  uint8_t ss = bcd2bin(rtcout[0] & 0x7F);
  uint8_t mm = bcd2bin(rtcout[1]);
  uint8_t hh = bcd2bin(rtcout[2]);
  uint8_t d  = bcd2bin(rtcout[4]);
  uint8_t m  = bcd2bin(rtcout[5]);
  uint16_t y = bcd2bin(rtcout[6]) + 2000;


  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(y, m, d);

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hh, mm, ss);
}

void setup() {
  Serial.begin(115200); //debug

  // I2C (two-wire interface) for communicating with MPU-9250
  I2c.begin();
  I2c.timeOut(I2C_TIMEOUT_MS);

  // SD card for data logging
  SdFile::dateTimeCallback(dateTime);
  if (!SD.begin(CHIP_SELECT_PIN)) {
    error("SD card initialization failed!");
    return;
  }
  initLogFile();

  initMPU9250();

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  if(c != 0x71) {
    error("MPU9250 did not respond correctly to who am i.");
  }

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

  pinMode(SYNC_PIN, OUTPUT);
}


void loop() {
  while (getFifoCount() < FIFO_BLOCK_SIZE) delay(FIFO_POLLING_INTERVAL);
  readBytes(MPU9250_ADDRESS, FIFO_R_W, FIFO_BLOCK_SIZE, &data[0]);
  if (doSyncNow()) {
    emitSyncPulse();
    syncPulseEmitted = 1;
  } else {
    syncPulseEmitted = 0;
  }
  ms = millis();
  data[DATA_LEN - 5] = syncPulseEmitted
  data[DATA_LEN - 4] = (byte) ms;
  data[DATA_LEN - 3] = (byte) ms >> 8;
  data[DATA_LEN - 2] = (byte) ms >> 16;
  data[DATA_LEN - 1] = (byte) ms >> 24;
  myFile.write(&data[0], DATA_LEN);
}


uint16_t getFifoCount() {
  // Number of bytes in the MPU9250's FIFO
  uint8_t fifo_cnt_high = readByte(MPU9250_ADDRESS, FIFO_COUNTH) & B00001111;
  uint8_t fifo_cnt_low = readByte(MPU9250_ADDRESS, FIFO_COUNTL);
  return ((int16_t)fifo_cnt_high << 8) | fifo_cnt_low;
}


bool doSyncNow() {
  // Returns true if should emit a sync pulse now
  return random(10000) < SYNC_RATE
}


void emitSyncPulse() {
  // Emits a logical high for SYNC_MICROSECONDS on SYNC_PIN
  digitalWrite(SYNC_PIN, HIGH);
  delayMicroseconds(SYNC_MICROSECONDS);
  digitalWrite(SYNC_PIN, LOW);
}


void initMPU9250()
{
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

   writeByte(MPU9250_ADDRESS, USER_CTRL, B01000000); // Enable FIFO
   writeByte(MPU9250_ADDRESS, FIFO_EN, B01000000); // Write GyroX only to FIFO

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
//34567890123456789012345678901234567890123456789012345678901234567890123456789
uint8_t initLogFile() {
  /*
   * Create a new log file.
   *
   * Makes a new file of the format GD_XXXX.DAT where XXXX is replaced
   * by a number between 0001 and 9999. It will use the lowest number
   * that does not already exist.
   *
   * After creating the file, write a
   * header. If you are changing the format of the log file you should
   * change the header. In particular, the "ColumnNames" field should
   * be changed to reflect the columns in your log file entry.
   */

  // Close the current file because we can only have one file open at a time. This is
  // a limitation of the SD library.
  if(myFile) {
    myFile.close();
  }

  // Make a new filename in the form GD_XXXX.LOG
  for(int xxxx = 0; xxxx < 10000; xxxx++) {
    filename[3] = xxxx        / 1000 + '0';
    filename[4] = xxxx % 1000 /  100 + '0';
    filename[5] = xxxx %  100 /   10 + '0';
    filename[6] = xxxx %   10        + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      myFile = SD.open(filename, FILE_WRITE);
      numlines = 0;
      break;
    }
  }

  if (!myFile) {
    error("couldnt create file");
  }
}


void error(char *msg) {
  myFile.println(msg);
  myFile.close();
  Serial.println(msg);
  while(1);
}
