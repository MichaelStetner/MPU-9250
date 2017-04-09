/**
 * This program logs data to a binary file.  Functions are included
 * to convert the binary file to a csv text file.
 *
 * Samples are logged at regular intervals.  The maximum logging rate
 * depends on the quality of your SD card and the time required to
 * read sensor data.  This example has been tested at 500 Hz with
 * good SD card on an Uno.  4000 HZ is possible on a Due.
 *
 * If your SD card has a long write latency, it may be necessary to use
 * slower sample rates.  Using a Mega Arduino helps overcome latency
 * problems since 12 512 byte buffers will be used.
 *
 * Data is written to the file using a SD multiple block write command.
 */
#include "Registers.h"

#include "SdFat.h"
#include "FreeStack.h"
#include "UserTypes.h"

#ifdef __AVR_ATmega328P__
#include "MinimumSerial.h"
MinimumSerial MinSerial;
#define Serial MinSerial
#endif  // __AVR_ATmega328P__
//==============================================================================
// Start of configuration constants.
//==============================================================================
// Abort run on an overrun.  Data before the overrun will be saved.
#define ABORT_ON_OVERRUN 1
//------------------------------------------------------------------------------
//Interval between data records in microseconds.
const uint32_t LOG_INTERVAL_USEC =  5000;
//------------------------------------------------------------------------------
// Set USE_SHARED_SPI non-zero for use of an SPI sensor.
// May not work for some cards.
#ifndef USE_SHARED_SPI
#define USE_SHARED_SPI 0
#endif  // USE_SHARED_SPI
//------------------------------------------------------------------------------
// Pin definitions.
//
//
// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for
// overrun errors and logging continues unless ABORT_ON_OVERRUN
// is non-zero.
#ifdef ERROR_LED_PIN
#undef ERROR_LED_PIN
#endif  // ERROR_LED_PIN
const int8_t ERROR_LED_PIN = -1;
const int8_t BUTTON_PIN = 11; // button to terminate recording
//------------------------------------------------------------------------------
// File definitions.
//
// Maximum file size in blocks.
// The program creates a contiguous file with FILE_BLOCK_COUNT 512 byte blocks.
// This file is flash erased using special SD commands.  The file will be
// truncated if logging is stopped early.
const uint32_t FILE_BLOCK_COUNT = 65535;
//
// log file base name if not defined in UserTypes.h
#ifndef FILE_BASE_NAME
#define FILE_BASE_NAME "data"
#endif  // FILE_BASE_NAME
//------------------------------------------------------------------------------
// Buffer definitions.
//
// The logger will use SdFat's buffer plus BUFFER_BLOCK_COUNT-1 additional
// buffers.
//
#ifndef RAMEND
// Assume ARM. Use total of ten 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 10;
//
#elif RAMEND < 0X8FF
#error Too little SRAM
//
#elif RAMEND < 0X10FF
// Use total of two 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 2;
//
#elif RAMEND < 0X20FF
// Use total of four 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;
//
#else  // RAMEND
// Use total of 12 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 12;
#endif  // RAMEND
//==============================================================================
// End of configuration constants.
//==============================================================================

// Size of file base name.
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
const uint8_t FILE_NAME_DIM  = BASE_NAME_SIZE + 7;
char binName[FILE_NAME_DIM] = FILE_BASE_NAME "00.bin";

SdFatSdioEX sd;

SdBaseFile binFile;

// Number of data records in a block.
const uint16_t DATA_DIM = (512 - 4)/sizeof(data_t);

//Compute fill so block size is 512 bytes.  FILL_DIM may be zero.
const uint16_t FILL_DIM = 512 - 4 - DATA_DIM*sizeof(data_t);

struct block_t {
  uint16_t count;
  uint16_t overrun;
  data_t data[DATA_DIM];
  uint8_t fill[FILL_DIM];
};
//==============================================================================
// Error messages stored in flash.
#define error(msg) {sd.errorPrint(&Serial, F(msg));fatalBlink();}
//------------------------------------------------------------------------------
//
void fatalBlink() {
  while (true) {
    SysCall::yield();
    if (ERROR_LED_PIN >= 0) {
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(200);
      digitalWrite(ERROR_LED_PIN, LOW);
      delay(200);
    }
  }
}
//------------------------------------------------------------------------------
void createBinFile() {
  // max number of blocks to erase per erase call
  const uint32_t ERASE_SIZE = 262144L;
  uint32_t bgnBlock, endBlock;
  char TMP_FILE_NAME[] = "myfile.dat";
  // Delete old tmp file.
  if (sd.exists(TMP_FILE_NAME)) {
    Serial.print(F("Deleting tmp file "));
    Serial.println(TMP_FILE_NAME);
    if (!sd.remove(TMP_FILE_NAME)) {
      error("Can't remove tmp file");
    }
  }
  // Create new file.
  Serial.println(F("\nCreating new file"));
  binFile.close();
  if (!binFile.createContiguous(TMP_FILE_NAME, 512 * FILE_BLOCK_COUNT)) {
    error("createContiguous failed");
  }
  // Get the address of the file on the SD.
  if (!binFile.contiguousRange(&bgnBlock, &endBlock)) {
    error("contiguousRange failed");
  }
  // Flash erase all data in the file.
  Serial.println(F("Erasing all data"));
  uint32_t bgnErase = bgnBlock;
  uint32_t endErase;
  while (bgnErase < endBlock) {
    endErase = bgnErase + ERASE_SIZE;
    if (endErase > endBlock) {
      endErase = endBlock;
    }
    if (!sd.card()->erase(bgnErase, endErase)) {
      error("erase failed");
    }
    bgnErase = endErase + 1;
  }
}
//------------------------------------------------------------------------------
// log data
void logData() {
  createBinFile();
  recordBinFile();
  renameBinFile();
}
//------------------------------------------------------------------------------
void renameBinFile() {
  if (binFile.fileSize() == 0) {
    return;
  }
  while (sd.exists(binName)) {
    if (binName[BASE_NAME_SIZE + 1] != '9') {
      binName[BASE_NAME_SIZE + 1]++;
    } else {
      binName[BASE_NAME_SIZE + 1] = '0';
      if (binName[BASE_NAME_SIZE] == '9') {
        error("Can't create file name");
      }
      binName[BASE_NAME_SIZE]++;
    }
  }
  if (!binFile.rename(sd.vwd(), binName)) {
    error("Can't rename file");
    }
  Serial.print(F("File renamed: "));
  Serial.println(binName);
  Serial.print(F("File size: "));
  Serial.print(binFile.fileSize()/512);
  Serial.println(F(" blocks"));
}
//------------------------------------------------------------------------------
void recordBinFile() {
  const uint8_t QUEUE_DIM = BUFFER_BLOCK_COUNT + 1;
  // Index of last queue location.
  const uint8_t QUEUE_LAST = QUEUE_DIM - 1;

  // Allocate extra buffer space.
  block_t block[BUFFER_BLOCK_COUNT - 1];

  block_t* curBlock = 0;

  block_t* emptyStack[BUFFER_BLOCK_COUNT];
  uint8_t emptyTop;
  uint8_t minTop;

  block_t* fullQueue[QUEUE_DIM];
  uint8_t fullHead = 0;
  uint8_t fullTail = 0;

  // Use SdFat's internal buffer.
  emptyStack[0] = (block_t*)sd.vol()->cacheClear();
  if (emptyStack[0] == 0) {
    error("cacheClear failed");
  }
  // Put rest of buffers on the empty stack.
  for (int i = 1; i < BUFFER_BLOCK_COUNT; i++) {
    emptyStack[i] = &block[i - 1];
  }
  emptyTop = BUFFER_BLOCK_COUNT;
  minTop = BUFFER_BLOCK_COUNT;

  // Start a multiple block write.
  if (!sd.card()->writeStart(binFile.firstBlock(), FILE_BLOCK_COUNT)) {
    error("writeStart failed");
  }
  Serial.print(F("FreeStack: "));
  Serial.println(FreeStack());
  Serial.println(F("Logging - type any character to stop"));
  bool closeFile = false;
  uint32_t bn = 0;
  uint32_t maxLatency = 0;
  uint32_t overrun = 0;
  uint32_t overrunTotal = 0;
  uint32_t logTime = micros();
  while(1) {
     //////////////////////////////////////////////////////////////////////////
     // Time for next data record.
    logTime += LOG_INTERVAL_USEC;
    //////////////////////////////////////////////////////////////////////////
    // Close file
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println("Stopping due to user input");
      closeFile = true;
    }
    if (getI2cErrorCount() > 100) {
      Serial.println("Stopping because too many I2C errors");
      closeFile = true;
    }
    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("Stopping because button pressed");
      closeFile = true;
    }
    if (closeFile) {
      if (curBlock != 0) {
        // Put buffer in full queue.
        fullQueue[fullHead] = curBlock;
        fullHead = fullHead < QUEUE_LAST ? fullHead + 1 : 0;
        curBlock = 0;
      }
    //////////////////////////////////////////////////////////////////////////
    // If not closing file, acquire data and maybe write block to file
    } else {
      // If we don't have a current blokc and there is an empty block in the
      // stack, take an empty block off the empty stack to be our current
      // block.
      if (curBlock == 0 && emptyTop != 0) {
        curBlock = emptyStack[--emptyTop];
        if (emptyTop < minTop) {
          minTop = emptyTop;
        }
        curBlock->count = 0;
        curBlock->overrun = overrun;
        overrun = 0;
      }
      // Check rate
      if ((int32_t)(logTime - micros()) < 0) {
        closeFile = true;
        Serial.println("Rate too fast");
        while(1);
      }
      // Wait for next sample
      int32_t delta;
      do {
        delta = micros() - logTime;
      } while (delta < 0);
      // If we don't have a block and couldn't get an empty block off the
      // stack, we have overrun our buffer!
      if (curBlock == 0) {
        overrun++;
        overrunTotal++;
        if (ERROR_LED_PIN >= 0) {
          digitalWrite(ERROR_LED_PIN, HIGH);
        }
#if ABORT_ON_OVERRUN
        Serial.println(F("Overrun abort"));
        break;
 #endif  // ABORT_ON_OVERRUN
      } else {
#if USE_SHARED_SPI
        sd.card()->spiStop();
#endif  // USE_SHARED_SPI
        ////////////////
        // Get the data
        acquireData(&curBlock->data[curBlock->count++]);
#if USE_SHARED_SPI
        sd.card()->spiStart();
#endif  // USE_SHARED_SPI
        // If the current block is full, move it to the full queue. Then, we
        // don't have a current block, but we will try to get a new one at the
        // beginning of the loop
        if (curBlock->count == DATA_DIM) {
          fullQueue[fullHead] = curBlock;
          fullHead = fullHead < QUEUE_LAST ? fullHead + 1 : 0;
          curBlock = 0;
        }
      }
    }
    ///////////////////////////////////////////////////////////////////////////
    // If there are no full blocks and we are trying to close the file, end the
    // acquisition loop
    if (fullHead == fullTail) {
      // Exit loop if done.
      if (closeFile) {
        break;
      }
    ///////////////////////////////////////////////////////////////////////////
    // If SD card is not busy, and we have full blocks, write full blocks to
    // file
    } else if (!sd.card()->isBusy()) {
      // Get address of block to write.
      block_t* pBlock = fullQueue[fullTail];
      fullTail = fullTail < QUEUE_LAST ? fullTail + 1 : 0;
      // Write block to SD.
      uint32_t usec = micros();
      if (!sd.card()->writeData((uint8_t*)pBlock)) {
        error("write data failed");
      }
      usec = micros() - usec;
      if (usec > maxLatency) {
        maxLatency = usec;
      }
      // Move block to empty queue.
      emptyStack[emptyTop++] = pBlock;
      bn++;
      if (bn == FILE_BLOCK_COUNT) {
        // File full so stop
        break;
      }
    }
  }
  /* Now we are done putting data in the file, but the SD card will continue
   * doing the write command until it receives FILE_BLOCK_COUNT blocks. We
   * cannot use writestop() to end it now because that is not supported. To
   * force it to finish, we will write empty blocks to the file until we hit the
   * end.
   */
  // Make empty block
  uint8_t* zeroBlock = (uint8_t*) emptyStack[emptyTop];
  for (size_t i=0; i<512; i++)
    zeroBlock[i] = 0;
  // Write empty block to file until file is full
  while(bn < FILE_BLOCK_COUNT) {
    if (!sd.card()->writeData(zeroBlock)) {
        error("write zero data failed");
    }
    bn++;
  }
  Serial.print(F("Min Free buffers: "));
  Serial.println(minTop);
  Serial.print(F("Max block write usec: "));
  Serial.println(maxLatency);
  Serial.print(F("Overruns: "));
  Serial.println(overrunTotal);
  // Truncate file if recording stopped early.
  if (bn != FILE_BLOCK_COUNT) {
    Serial.println(F("Truncating file"));
    if (!binFile.truncate(512L * bn)) {
      error("Can't truncate file");
    }
  }
}
//------------------------------------------------------------------------------
void setup(void) {
  if (ERROR_LED_PIN >= 0) {
    pinMode(ERROR_LED_PIN, OUTPUT);
  }
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(9600);

  // Wait for USB Serial
  while (!Serial) {
    SysCall::yield();
  }
  Serial.print(F("\nFreeStack: "));
  Serial.println(FreeStack());
  Serial.print(F("Records/block: "));
  Serial.println(DATA_DIM);
  Serial.print(F("Record bytes: "));
  Serial.println(sizeof(data_t));
  Serial.print(F("Fill bytes: "));
  Serial.println(FILL_DIM);
  if (sizeof(block_t) != 512) {
    error("Invalid block size");
  }

  // Setup sensors.
  userSetup();

  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin()) {
    sd.initErrorPrint(&Serial);
    fatalBlink();
  }
}
//------------------------------------------------------------------------------
void loop(void) {
  logData();
}

/*
current structure is

1. set closing flag
2. get block
3. wait for sample time
4. check overrun
5. acquire data (and increment block count)
6. check for full block
7. write blocks

NEW:
(1-4 are the same)
5. request data
6. write blocks (data is being received in background)
7. read data (and increment block count)
8. check for full block

goal is to save time by writing blocks while receiving data in background.
receiving data takes about 4ms just because of i2c communication time. Split
"acquire data" step into two new steps: request data and read data. Request
data happens at step 5. Moved up block writing to step 6. At this point, data
is being received in the background. Then, at step 7 we get the data that was
received in the background and add it to the block. Notice that we don't
incement the block counter until the read data step (7), so the block we are
writing to in this step is never written between requesting and read the data.
*/
