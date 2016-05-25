#include "I2C.h"
#include <SPI.h>
#include <SD.h>

// Pin assignments
// On Arduino UNO, Pins 11, 12, 13 are used for SPI with the data logger shield
#define CHIP_SELECT_PIN 10
#define SYNC_PIN 9

#define SYNC_RATE  50 // integer between 0 (no sync pulses) and 10000 (sync pulse on every cycle)
#define GYRO_SCALE 3 // 0=250dps, 1=500dps, 2=1000dps, 3=2000dps
#define ACCEL_SCALE 1 // 0=2g, 1=4g, 2=8g, 3=16g
#define I2C_TIMEOUT_MS 10
#define MAX_FILE_LINES 16000000UL  // 16 million lines/file * 56 bytes/line = 896MB/file
#define SYNC_MICROSECONDS 30
#define BUFFER_LINES 20 // number of lines to buffer before flushing to SD card

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

File myFile;
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
bool syncNow;
unsigned long numlines = 0;
unsigned long myrand;
char filename[] = "GD_XXXX.LOG";
uint8_t rtcout[7]; // output from real time clock  
char entry[] = "!tttttttttt,Gxxxxx,Gyyyyy,Gzzzzz,Axxxxx,Ayyyyy,Azzzzz,S";
unsigned int entrypos;
// template log file entry.
// To change the format of the log file, you need to change this, as well as printLogEntry() and updateEntry() and initLogFile()

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
  initMPU9250();

  // SD card for data logging
  SdFile::dateTimeCallback(dateTime);
  if (!SD.begin(CHIP_SELECT_PIN)) {
    error("SD card initialization failed!");
    return;
  }
  initLogFile();

  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  if(c != 0x71) {
    error("MPU9250 did not respond correctly to who am i.");
  }

  pinMode(SYNC_PIN, OUTPUT);
}

void loop() {
  readAccelData(accelCount);
  readGyroData(gyroCount);

  // emit sync pulses at random
  myrand = random(10000);
  syncNow = (myrand < SYNC_RATE);
  if(syncNow) {
    digitalWrite(SYNC_PIN, HIGH);
    delayMicroseconds(SYNC_MICROSECONDS);
    digitalWrite(SYNC_PIN, LOW);

      // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 says: I am 0x");
  Serial.println(c, HEX);
  }
  
  printLogEntry();
  numlines++;
  if((numlines % BUFFER_LINES) == 0) {
    myFile.flush();
  }

  // Make a new log file if the current file is too long
  if(numlines > MAX_FILE_LINES) {
    initLogFile();
  }
}

void printLogEntry() {
  // log entry is like:
  // "!1234567890,-12345, 12345,-12345, 12345,-12345, 12345,1"
  //     millis   GyroX  GyroY  GyroZ  AccelX AccelY AccelZ Sync
  entrypos = 1;
  updateEntry(millis());
  entrypos++; // skip comma
  for(int i = 0; i < 3; i++) {
    updateEntry(gyroCount[i]);
    entrypos++; // skip comma
  }
  for(int j = 0; j < 3; j++) {
    updateEntry(accelCount[j]);
    entrypos++; // skip comma
  }
  updateEntry(syncNow);
  myFile.println(entry);
  Serial.println(entry);
}

void updateEntry(int16_t y) {
  if(y < 0) {
    entry[entrypos++] = '-';
  } else {
    entry[entrypos++] = ' ';
  }
  entry[entrypos++] = abs(y         / 10000) + '0';
  entry[entrypos++] = abs(y % 10000 /  1000) + '0';
  entry[entrypos++] = abs(y %  1000 /   100) + '0';
  entry[entrypos++] = abs(y %   100 /    10) + '0';
  entry[entrypos++] = abs(y %    10        ) + '0';
}

void updateEntry(bool y) {
  if(y) {
    entry[entrypos++] = '1';
  } else {
    entry[entrypos++] = '0';
  }
}

void updateEntry(unsigned long y) {
  entry[entrypos++] = y                / 1000000000UL + '0';
  entry[entrypos++] = y % 1000000000UL /  100000000UL + '0';
  entry[entrypos++] = y %  100000000UL /   10000000UL + '0';
  entry[entrypos++] = y %   10000000UL /    1000000UL + '0';
  entry[entrypos++] = y %    1000000UL /     100000UL + '0';
  entry[entrypos++] = y %     100000UL /      10000UL + '0';
  entry[entrypos++] = y %      10000UL /       1000UL + '0';
  entry[entrypos++] = y %       1000UL /        100UL + '0';
  entry[entrypos++] = y %        100UL /         10UL + '0';
  entry[entrypos++] = y %         10UL                + '0';
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
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

  // Read current time from real time clock so we can record file start time 
  // in the header below.
  readBytes(DS1307_ADDRESS, 0, 7, &rtcout[0]); 
  uint8_t ss = bcd2bin(rtcout[0] & 0x7F);
  uint8_t mm = bcd2bin(rtcout[1]);
  uint8_t hh = bcd2bin(rtcout[2]);
  uint8_t d  = bcd2bin(rtcout[4]);
  uint8_t m  = bcd2bin(rtcout[5]);
  uint16_t y = bcd2bin(rtcout[6]) + 2000;

  // Write header
  myFile.println("Arduino data");
  myFile.print("BytesPerLine=");
  myFile.println(sizeof(entry));
  myFile.print("Columns=Milliseconds,GyroX,GyroY,GyroZ,"); // change this with 
  myFile.println("AccelX,AccelY,AccelZ,Sync");             // entry format 
  myFile.print("FileStartTime=");
  myFile.print(y);
  myFile.print("-");
  myFile.print(m);
  myFile.print("-");
  myFile.print(d);
  myFile.print(" ");
  myFile.print(hh);
  myFile.print(":");
  myFile.print(mm);
  myFile.print(":");
  myFile.println(ss);
  myFile.println("DATA BEGINS ON NEXT LINE");
}


void error(char *msg) {
  Serial.println(msg);
  while(1);
}

