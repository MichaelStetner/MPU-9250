/*
  AK8963BasicSlave.ino
  Read from AK8963 compass as a slave to the MPU-9250
*/
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define EXT_SENS_DATA_00 0x49
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71


void setup() {
  Serial.begin(115200);
  Serial.println("Arduino online");

  Wire.begin();

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 says I AM 0x");
  Serial.println(c, HEX);
  Serial.print("MPU9250 should be 0x71");
  delay(500);

  // Enable I2C master functionality
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, B00001000);
  /*  Bit   Name           Description
       7    MULT_MST_EN    I2C multimaster (1 to enable)
       6    WAIT_FOR_ES    DRDY interrupt waits for external data (1 to enable)
       5    SLV_3_FIFO_EN  Write SLV_3 data to FIFO (1 to enable)
       4    I2C_MST_P_NSR  Behavior between reads (0 to stop, 1 to reset)
      3:0   I2C_MST_CLK    I2C master clock speed (B1000 for 258kHz, slowest)
  */
  // FIXME experiment with bit 4 - what to do between reads

  // Configure to read from WHO_AM_I register of AK8963
  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, 128 + AK8963_ADDRESS);
  // First bit is "1" for read

  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, WHO_AM_I_AK8963);
  // Value read should be 0x48

  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, B10000001);
  // FIXME What does bit 5 do? "When set, the transaction does not write a register value, it will only read data, or write data"

  // Enable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, B0010000);

  // FIXME Set sample rate in register 25?
  // Optionally use Slave Delay Enable Registers in Register 103 and 52

  // Can use I2C_MST_RST bit (Register 106) to reset MPU9250's I2C master functionality

  // Data is in EXT_SENS_DATA_00 (Register 73)
}

void loop(){
  int c = readByte(MPU9250_ADDRESS, 73);
  Serial.print(millis());
  Serial.print(" External sensor says 0x");
  Serial.println(c, HEX)
  delay(500);
}
