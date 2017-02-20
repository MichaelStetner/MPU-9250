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

#include <SPI.h>
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
const uint32_t LOG_INTERVAL_USEC = 3333;
//------------------------------------------------------------------------------
// Set USE_SHARED_SPI non-zero for use of an SPI sensor.
// May not work for some cards.
#ifndef USE_SHARED_SPI
#define USE_SHARED_SPI 0
#endif  // USE_SHARED_SPI
//------------------------------------------------------------------------------
// Pin definitions.
//
// SD chip select pin.
const uint8_t SD_CS_PIN = SS;
//
// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for
// overrun errors and logging continues unless ABORT_ON_OVERRUN
// is non-zero.
#ifdef ERROR_LED_PIN
#undef ERROR_LED_PIN
#endif  // ERROR_LED_PIN
const int8_t ERROR_LED_PIN = -1;
//------------------------------------------------------------------------------
// File definitions.
//
// Maximum file size in blocks.
// The program creates a contiguous file with FILE_BLOCK_COUNT 512 byte blocks.
// This file is flash erased using special SD commands.  The file will be
// truncated if logging is stopped early.
const uint32_t FILE_BLOCK_COUNT = 256000;
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

SdFat sd;

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
  if (!sd.card()->writeStart(binFile.firstBlock())) {
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
     // Time for next data record.
    logTime += LOG_INTERVAL_USEC;
    if (Serial.available()) {
      closeFile = true;
    }  
    if (closeFile) {
      if (curBlock != 0) {
        // Put buffer in full queue.
        fullQueue[fullHead] = curBlock;
        fullHead = fullHead < QUEUE_LAST ? fullHead + 1 : 0;
        curBlock = 0;
      }
    } else {
      if (curBlock == 0 && emptyTop != 0) {
        curBlock = emptyStack[--emptyTop];
        if (emptyTop < minTop) {
          minTop = emptyTop;
        }
        curBlock->count = 0;
        curBlock->overrun = overrun;
        overrun = 0;
      }
      if ((int32_t)(logTime - micros()) < 0) {
        error("Rate too fast");             
      }
      int32_t delta;
      do {
        delta = micros() - logTime;
      } while (delta < 0);
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
        acquireData(&curBlock->data[curBlock->count++]);
#if USE_SHARED_SPI
        sd.card()->spiStart();
#endif  // USE_SHARED_SPI      
        if (curBlock->count == DATA_DIM) {
          fullQueue[fullHead] = curBlock;
          fullHead = fullHead < QUEUE_LAST ? fullHead + 1 : 0;
          curBlock = 0;
        } 
      }
    }
    if (fullHead == fullTail) {
      // Exit loop if done.
      if (closeFile) {
        break;
      }
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
  if (!sd.card()->writeStop()) {
    error("writeStop failed");
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
  Serial.begin(9600);
  
  // Wait for USB Serial 
  while (!Serial) {
    SysCall::yield();
  }
  Serial.print(F("\nFreeStack: "));
  Serial.println(FreeStack());
  Serial.print(F("Records/block: "));
  Serial.println(DATA_DIM);
  if (sizeof(block_t) != 512) {
    error("Invalid block size");
  }
  // Allow userSetup access to SPI bus.
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  
  // Setup sensors.
  userSetup();
  
  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(50))) {
    sd.initErrorPrint(&Serial);
    fatalBlink();
  }
}
//------------------------------------------------------------------------------
void loop(void) {
  logData();
  while(1);
}