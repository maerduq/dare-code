/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2017 Semtech
	
Description: Decoding functions
License: Revised BSD License, see LICENSE file included in the project
By: Paul Marcelis
*/
#include "DaReDecode.h"

void DaReDecode::init(uint8_t dataPointSizeIn, uint32_t simulationLength) {
  dataPointSize = dataPointSizeIn;
  totalDataPoints = simulationLength;
  dataPointsReceived = new uint8_t[simulationLength * dataPointSize]();
  dataPointsDelay = new uint32_t[simulationLength * dataPointSize]();
  isDataPointReceived = new bool[simulationLength]();
#if DEBUG >= 0
  dataPointsDebug = new uint8_t[simulationLength * dataPointSize]();
#endif
}

void DaReDecode::destroy() {
  free(dataPointsReceived);
  free(isDataPointReceived);
}

void DaReDecode::flushBuffers() {
  checkBuffersForSubmatrix(0, true, totalDataPoints);
}

#if DEBUG >= 0
void DaReDecode::debugData(uint32_t fcntup, uint8_t *dataPoint) {
  uint8_t i;
  for (i = 0; i < dataPointSize; i++) {
    dataPointsDebug[(fcntup - 1) * dataPointSize + i] = dataPoint[i];
  }
}
#endif

void DaReDecode::storeDataPoint(uint32_t fcntup, uint8_t *dataPoint, uint32_t currentFcntup, int phase) {
  uint8_t i;
  bool wrong = false;
  for (i = 0; i < dataPointSize; i++) {
    dataPointsReceived[(fcntup - 1) * dataPointSize + i] = dataPoint[i];
    if (dataPoint[i] != dataPointsDebug[(fcntup - 1) * dataPointSize + i]) {
      wrong = true;
#if DEBUG >= 0
      std::printf("FOUT! d[%d_%d](%d) = %02x != %02x. ", (fcntup-1), i, phase, dataPoint[i], dataPointsDebug[(fcntup - 1) * dataPointSize + i]);
#endif
    }
  }

  if (!wrong) {
    dataPointsDelay[fcntup - 1] = currentFcntup - fcntup;
    isDataPointReceived[fcntup - 1] = true;
    recovered += 1;
    recoverPhase[phase-1] += 1;
  }

#if DEBUG >= 1
  std::cout << "++ Received d[" << (fcntup - 1) << "]: ";
  displayCharArray(&dataPointsReceived[(fcntup - 1)*dataPointSize], dataPointSize, 1, ' ');
  std::cout << ", delay = " << dataPointsDelay[fcntup - 1] << std::endl;
#endif
}

void DaReDecode::clearBuffer(uint32_t bufferI) {
  //free(buffers[bufferI].parityCheck);
  //free(buffers[bufferI].generatorLine);
  buffers[bufferI].inUse = false;
}

void DaReDecode::decode(DaRe::Payload payload, uint32_t fcntup) {
  uint8_t windowSize, W, R, dataPointOffset;
  uint32_t dataPointOffsetPointer;
  bool *generatorLine;
  bool previousDataRecovered = false;
  uint8_t R_i, dataPoint_i;
  int bufferI;

  DaRe::R_VALUE enumR = (DaRe::R_VALUE) (payload.payload[0] >> 4);
  DaRe::W_VALUE enumW = (DaRe::W_VALUE) (payload.payload[0] & 0xf);
  W = DaRe::getW(enumW);
  R = DaRe::getR(enumR);


  // This data point is never of use for parity checks in the buffer
  //** STAGE 1 DATA RECOVERY | NORMAL RECOVERY **//
  storeDataPoint(fcntup, &payload.payload[1], fcntup, 1);

  // Check if there is something missing...
  if (lastFcntup < (fcntup - 1)) {
    tryToRecover = true;
#if DEBUG >= 2
    std::cout << "!!! There is something missing!" << std::endl;
#endif
  }
  lastFcntup = fcntup;

  // If there is something missing, let's get checking..
  if (tryToRecover) {
#if DEBUG >= 2
    std::cout << "Interpret parity check." << std::endl;
#endif

    windowSize = DaRe::getWindowSize(W, fcntup);
    for (R_i = 0; R_i < R - 1; R_i++) {
      generatorLine = DaRe::prlg(W, fcntup, R_i);

#if DEBUG >= 3
      displayBoolArray(generatorLine, windowSize); 
      std::cout << std::endl;
#endif
      for (dataPointOffset = 1; dataPointOffset <= windowSize; dataPointOffset++) {
        // If there is a one, XOR a previous data point with
        dataPointOffsetPointer = ((fcntup - 1) - dataPointOffset); // Calculate pointer for previous data point
        if (isDataPointReceived[dataPointOffsetPointer]) {
#if DEBUG >= 3
          std::cout << (int)dataPointOffsetPointer << ", ";
#endif
          if (generatorLine[dataPointOffset - 1] == 1) {
            generatorLine[dataPointOffset - 1] = 0;
#if DEBUG >= 3
            std::cout << "0x";
            displayCharArray(&dataPointsReceived[dataPointOffsetPointer * dataPointSize], dataPointSize);
            std::cout << std::endl;
#endif

            for (dataPoint_i = 0; dataPoint_i < dataPointSize; dataPoint_i++) {
              payload.payload[1 + dataPointSize * (1 + R_i) + dataPoint_i] ^= dataPointsReceived[dataPointOffsetPointer * dataPointSize + dataPoint_i]; // XOR it
            }
          }
        }
      }
      
      int generatorLineOnes = 0;
      int newDataOffset = 0;
      uint32_t j;
      for (j = 0; j < windowSize; j++) {
        if (generatorLine[j] == 1) {
          generatorLineOnes += 1;
          newDataOffset = j + 1;
        }
      }
#if DEBUG >= 3
      displayBoolArray(generatorLine, windowSize); 
      std::cout << std::endl;
#endif

      switch (generatorLineOnes) {
      case 0:
#if DEBUG >= 2
        std::cout << "No new data" << std::endl;
#endif
        break;
      case 1:
        //** STAGE 2 DATA RECOVERY | DIRECTLY FROM PARITY CHECK **//
        storeDataPoint(fcntup - newDataOffset, &payload.payload[1 + dataPointSize * (1 + R_i)], fcntup, 2);
        previousDataRecovered = true;
        break;
      default:
        // so a new buffer entry. First to check if there is a submatrix in the buffers.

        uint32_t newOldestDataPointId = 0;
        for (j = windowSize - 1; j > 0; j--) {
          if (generatorLine[j] == 1) {
            newOldestDataPointId = fcntup - 1 - (j + 1);
            break;
          }
        }

        // If there is information left in the parity check, put it in the buffer...

        bool emptyBufferFound = false;
        bufferI = 0;
        // If buffers not full, get next empty buffer
        for (j = 0; j < DARE_DECODING_BUFFERS; j++) {
          if (!buffers[j].inUse) {
            bufferI = j;
            emptyBufferFound = true;
            break;
          }
        }
        // If the buffers are full, try to solve part of it, is possible
        if (!emptyBufferFound) {
#if DEBUG >= 2
          std::cout << "BUFFER FULL!!!, try to solve what's possible, check if something comes clear" << std::endl;
#endif
          // If buffers not full, get next empty buffer
          for (j = 0; j < DARE_DECODING_BUFFERS; j++) {
            if (!buffers[j].inUse) {
              bufferI = j;
              emptyBufferFound = true;
              break;
            }
          }
          if (!emptyBufferFound) {
#if DEBUG >= 2
            std::cout << "Noting solved in the buffers WHAT!!!!" << std::endl;
#endif
            // take oldest buffer entry and replace it.
            bufferI = 0;
            uint32_t bufferDataI = buffers[0].fcntup;
            for (j = 0; j < DARE_DECODING_BUFFERS; j++) {
              if (bufferDataI > buffers[j].fcntup) {
                bufferI = j;
                bufferDataI = buffers[j].fcntup;
              }
            }
          }
        }
        // out: bufferI

        buffers[bufferI].inUse = true;
        buffers[bufferI].fcntup = fcntup;
        buffers[bufferI].parityCheck = new uint8_t[windowSize]();
        for (j = 0; j < windowSize; j++) {
          buffers[bufferI].parityCheck[j] = payload.payload[1 + dataPointSize * (1 + R_i) + j];
        }
        buffers[bufferI].generatorLine = generatorLine;
        buffers[bufferI].windowSize = windowSize;
#if DEBUG >= 2
        std::cout << "Intermediate result saved in BUFFER[" << bufferI << "]." << std::endl;
#endif
      }
    }

    // Now, if there is data received, other then the current data point
    while (previousDataRecovered) {
      previousDataRecovered = false;
      for (bufferI = 0; bufferI < DARE_DECODING_BUFFERS; bufferI++) {
        if (!buffers[bufferI].inUse) {
          continue;
        }
#if DEBUG >= 2
        std::cout << "-- Checking BUFFER[" << bufferI << "]" << std::endl;
#endif
        for (dataPointOffset = 1; dataPointOffset <= buffers[bufferI].windowSize; dataPointOffset++) {
          // If there is a one, XOR a previous data point with
          dataPointOffsetPointer = (((buffers[bufferI].fcntup - 1) - dataPointOffset)); // Calculate pointer for previous data point
          if (buffers[bufferI].generatorLine[dataPointOffset - 1] == 1 && isDataPointReceived[dataPointOffsetPointer]) {
            buffers[bufferI].generatorLine[dataPointOffset - 1] = 0;
            for (dataPoint_i = 0; dataPoint_i < dataPointSize; dataPoint_i++) {
              // std::cout << std::hex << (unsigned int)dataPointHistory[dataPointOffsetPointer + dataPointI] << std::endl; /*DEBUG*/
              buffers[bufferI].parityCheck[dataPoint_i] ^= dataPointsReceived[dataPointOffsetPointer * dataPointSize + dataPoint_i]; // XOR it
            }
          }
        }

        int generatorLineOnes = 0;
        int newDataOffset = 0;
        int j;
        for (j = 0; j < buffers[bufferI].windowSize; j++) {
          if (buffers[bufferI].generatorLine[j] == 1) {
            generatorLineOnes += 1;
            newDataOffset = j + 1;
          }
        }

        switch (generatorLineOnes) {
        case 0:
#if DEBUG >= 2
          std::cout << "No new data in this buffer anymore" << std::endl;
#endif
          clearBuffer(bufferI);
          break;
        case 1:
          //** STAGE 3 DATA RECOVERY | FROM A BUFFER **//
          storeDataPoint(buffers[bufferI].fcntup - newDataOffset, buffers[bufferI].parityCheck, fcntup, 3);
          previousDataRecovered = true;

          clearBuffer(bufferI);
          break;
#if DEBUG >= 2
        default:
          std::cout << "Still not more info" << std::endl;
#endif
        }
      }
    }

    checkBuffersForSubmatrix(0, false, fcntup);
  }

  if (recovered == fcntup && tryToRecover) {
#if DEBUG >= 2
    std::cout << "--------- We are complete!" << std::endl;
#endif
    tryToRecover = false;
  }
}

void DaReDecode::displayReceivedData(uint8_t *dataToCheck) {
  uint32_t i;
  uint8_t dataI;

  std::cout << std::endl << "Received data: ";
  for (i = 0; i < lastFcntup - 1; i++) {
    std::cout << "d[" << i << "]=";
    if (isDataPointReceived[i]) {
      displayCharArray(&dataPointsReceived[i * dataPointSize], dataPointSize, 1, ' ');
      for (dataI = 0; dataI < dataPointSize; dataI++) {
        if (dataPointsReceived[i*dataPointSize + dataI] != dataToCheck[i*dataPointSize + dataI]) {
          std::printf("\n FOUT! %02x != %02x\n", dataPointsReceived[i*dataPointSize + dataI], dataToCheck[i*dataPointSize + dataI]);
        }
      }
    }
    else {
      std::cout << "x ";
    }
    std::cout << ", ";
  }
}

void DaReDecode::displayReceivedDataIds() {
  uint32_t i;

  for (i = 0; i < lastFcntup - 1; i++) {
    if (isDataPointReceived[i]) {
      std::cout << i << ",";
    }
  }
  std::cout << std::endl << std::endl;
}

void DaReDecode::displayResults() {
  uint32_t dataPoint_i;
  uint32_t delaySum = 0, delayVarSum = 0, delayCount = 0;

  
  for (dataPoint_i = 0; dataPoint_i < totalDataPoints; dataPoint_i++) {
    if (isDataPointReceived[dataPoint_i]) {
      delaySum += dataPointsDelay[dataPoint_i];
      delayCount += 1;
      //std::cout << " " << dataPointsDelay[dataPoint_i];
    }
  }
  //std::cout << std::endl;

  double avg_delay = (double)delaySum / delayCount;
  for (dataPoint_i = 0; dataPoint_i < totalDataPoints; dataPoint_i++) {
    if (isDataPointReceived[dataPoint_i]) {
      delayVarSum += (dataPointsDelay[dataPoint_i] - avg_delay) * (dataPointsDelay[dataPoint_i] - avg_delay);
      //std::cout << " " << dataPointsDelay[dataPoint_i];
    }
  }
  double var_delay = (double)delayVarSum / delayCount;

  double p_rr = (double)100 * recovered / totalDataPoints;
#if DEBUG > 0
  std::cout << std::endl
    << "p_rr: \t\t" << p_rr << std::endl
    << "Recovered: \t" << recovered << std::endl
    << "Avg delay: \t" << avg_delay << std::endl
    << "Var[delay]: \t" << var_delay << std::endl
    << "delay count: \t" << delayCount << std::endl
    << "Rec phase1: \t" << recoverPhase[0] << std::endl
    << "Rec phase2: \t" << recoverPhase[1] << std::endl
    << "Rec phase3: \t" << recoverPhase[2] << std::endl
    << "Rec phase4: \t" << recoverPhase[3] << std::endl
    << "Rec phase5: \t" << recoverPhase[4] << std::endl;
#else
  std::cout
    << p_rr << "\t"
    << recovered << "\t"
    << recoverPhase[0] << "\t"
    << recoverPhase[1] << "\t"
    << recoverPhase[2] << "\t"
    << recoverPhase[3] << "\t"
    << recoverPhase[4] << "\t"
    << avg_delay << "\t"
    << var_delay << std::endl;
#endif
}

void DaReDecode::checkBuffersForSubmatrix(uint32_t newOldestDataPointId, bool clearBuffers, uint32_t fcntup) {
  uint32_t currentNewestDataPointId = 0, currentOldestDataPointId = 0, buffersInUse = 0;
  uint32_t bufferI, dataPointOffset, dataPointOffsetPointer, dataPoint_i, j;
  bool currentOldestDataPointIdSet = false;

  for (bufferI = 0; bufferI < DARE_DECODING_BUFFERS; bufferI++) {
    if (!buffers[bufferI].inUse) {
      continue;
    }
    buffersInUse += 1;

    for (dataPointOffset = 1; dataPointOffset <= buffers[bufferI].windowSize; dataPointOffset++) {
      if (buffers[bufferI].generatorLine[dataPointOffset - 1] == 1) {
        dataPointOffsetPointer = (((buffers[bufferI].fcntup - 1) - dataPointOffset)); // Calculate pointer for previous data point
#if DEBUG >= 3
        std::cout << "d[" << (unsigned int)dataPointOffsetPointer << "], ";
#endif
        if (dataPointOffsetPointer > currentNewestDataPointId) {
          currentNewestDataPointId = dataPointOffsetPointer;
        }
        if (dataPointOffsetPointer < currentOldestDataPointId || !currentOldestDataPointIdSet) {
          currentOldestDataPointIdSet = true;
          currentOldestDataPointId = dataPointOffsetPointer;
        }
      }
    }
  }
#if DEBUG >= 2
  std::cout << "In " << (unsigned int)buffersInUse << " buffers:" << std::endl
    << "- oldest current data point: " << (unsigned int)currentOldestDataPointId << std::endl
    << "- newest current data point: " << (unsigned int)currentNewestDataPointId << std::endl
    << "- oldest new data point: " << (unsigned int)newOldestDataPointId << std::endl;
#endif

  if (buffersInUse == 0) {
#if DEBUG >= 2
    std::cout << "No buffers in use.." << std::endl;
#endif
    return;
  }
  // if there is no overlap in the current buffer and the new data point..
  else if (newOldestDataPointId > 0 && newOldestDataPointId <= currentNewestDataPointId) {
#if DEBUG >= 2
    std::cout << "The new buffer entry has overlapping information with the current buffers." << std::endl;
#endif
    return;
  }
  else if (buffersInUse == 1) {
#if DEBUG >= 2
    std::cout << "Only one buffer in use, so discard it.." << std::endl;
#endif
    if (clearBuffers) {
      for (bufferI = 0; bufferI < DARE_DECODING_BUFFERS; bufferI++) {
        if (!buffers[bufferI].inUse) {
          continue;
        }
        clearBuffer(bufferI);
        break;
      }
    }
    return;
  }
  else {
    uint32_t subMatrixWidth = (currentNewestDataPointId - currentOldestDataPointId + 1);
    bool *subMatrix = new bool[subMatrixWidth * buffersInUse]();
    uint8_t *X = new uint8_t[buffersInUse * dataPointSize]();
    uint32_t nrBufferInUse = 0;
    for (bufferI = 0; bufferI < DARE_DECODING_BUFFERS; bufferI++) {
      if (!buffers[bufferI].inUse) {
        continue;
      }
      for (dataPointOffset = 1; dataPointOffset <= buffers[bufferI].windowSize; dataPointOffset++) {
        if (buffers[bufferI].generatorLine[dataPointOffset - 1] == 1) {
          dataPointOffsetPointer = (((buffers[bufferI].fcntup - 1) - dataPointOffset)); // Calculate pointer for previous data point
          subMatrix[nrBufferInUse*subMatrixWidth + dataPointOffsetPointer - currentOldestDataPointId] = 1;
          for (dataPoint_i = 0; dataPoint_i < dataPointSize; dataPoint_i++) {
            X[nrBufferInUse * dataPointSize + dataPoint_i] = buffers[bufferI].parityCheck[dataPoint_i];
          }
        }
      }
      nrBufferInUse++;
    }
#if DEBUG >= 3
    displayBoolArray(subMatrix, subMatrixWidth * buffersInUse, subMatrixWidth);
    displayCharArray(X, buffersInUse * dataPointSize, dataPointSize, ' ');
    std::cout << std::endl;
#endif
    DaReDecode::g2rref(subMatrix, subMatrixWidth, buffersInUse, X);
#if DEBUG >= 3
    displayBoolArray(subMatrix, subMatrixWidth * buffersInUse, subMatrixWidth);
    displayCharArray(X, buffersInUse * dataPointSize, dataPointSize, ' ');
    std::cout << std::endl;
#endif

    uint32_t pointFound;
    uint8_t sum;
    bool foundOne = true;
    while (foundOne) {
      foundOne = false;
      for (nrBufferInUse = 0; nrBufferInUse < buffersInUse; nrBufferInUse++) {
        sum = 0;
        pointFound = 0;
        for (j = 0; j < subMatrixWidth; j++) {
          if (subMatrix[nrBufferInUse*subMatrixWidth + j] == 1) {
            sum += 1;
            pointFound = j;
          }
        }
        if (sum == 1) {
          //** STAGE 4 DATA RECOVERY | FROM A SOLVED SUBMATRIX **//
          storeDataPoint(currentOldestDataPointId + pointFound + 1, &X[nrBufferInUse*dataPointSize], fcntup, (newOldestDataPointId == 0) ? 5 : 4);
          subMatrix[nrBufferInUse*subMatrixWidth + pointFound] = 0;
          for (j = 0; j < buffersInUse; j++) {
            if (subMatrix[j*subMatrixWidth + pointFound] == 1) {
              subMatrix[j*subMatrixWidth + pointFound] = 0;
              for (dataPoint_i = 0; dataPoint_i < dataPointSize; dataPoint_i++) {
                X[j*dataPointSize + dataPoint_i] ^= X[nrBufferInUse*dataPointSize + dataPoint_i];
              }
            }
          }
#if DEBUG >= 3
          displayBoolArray(subMatrix, subMatrixWidth * buffersInUse, subMatrixWidth);
          displayCharArray(X, buffersInUse * dataPointSize, dataPointSize, ' ');
          std::cout << std::endl;
#endif
          foundOne = true;
          break;
        }
      }
    }

    for (bufferI = 0; bufferI < DARE_DECODING_BUFFERS; bufferI++) {
      if (!buffers[bufferI].inUse) {
        continue;
      }
      clearBuffer(bufferI);
    }

    if (clearBuffers) {
      tryToRecover = false;
    }
    else {
      // fill the buffers again with the result
#if DEBUG >= 2
      std::cout << "Save part of the buffers again, which still have information" << std::endl;
#endif
      int newBuffers = 0;
      bufferI = 0;
      uint32_t firstOne, lastOne;
      bool firstOneFound, thisValueIsDoomed;
      uint32_t oldestDataPointStillReceivable = ((fcntup - 1) > DARE_MAX_W) ? ((fcntup - 1) - DARE_MAX_W) : 0;
      for (nrBufferInUse = 0; nrBufferInUse < buffersInUse; nrBufferInUse++) {
        firstOne = 0, lastOne = 0;
        firstOneFound = false;
        thisValueIsDoomed = false;
        for (j = 0; j < subMatrixWidth; j++) {
          if (subMatrix[nrBufferInUse*subMatrixWidth + j] == 1) {
            if (!firstOneFound) {
              firstOne = j;
              firstOneFound = true;
              // als het oudste datapunt in deze parity check niet meer ontvangen kan worden, dan discarten..
              if ((currentOldestDataPointId + firstOne) < oldestDataPointStillReceivable) {
                thisValueIsDoomed = true;
              }
            }
            lastOne = j;
          }
        }

        if (!firstOneFound) {
#if DEBUG >= 2
          std::cout << "Discard empty row" << std::endl;
#endif
        }
        else if (thisValueIsDoomed) {
#if DEBUG >= 1
            std::cout << "-- d[" << currentOldestDataPointId + firstOne << "] is forever lost!" << std::endl;
#endif
          }
        else {
          buffers[bufferI].inUse = true;
          buffers[bufferI].fcntup = currentOldestDataPointId + lastOne + 2;

          uint8_t *newParityCheck = new uint8_t[dataPointSize]();
          for (j = 0; j < dataPointSize; j++) {
            newParityCheck[j] = X[nrBufferInUse*dataPointSize + j];
          }
          buffers[bufferI].parityCheck = newParityCheck;

          buffers[bufferI].windowSize = lastOne - firstOne + 1;
          bool *newGeneratorLine = new bool[buffers[bufferI].windowSize]();
          for (j = 0; j < buffers[bufferI].windowSize; j++) {
            newGeneratorLine[buffers[bufferI].windowSize - 1 - j] = subMatrix[nrBufferInUse*subMatrixWidth + firstOne + j];
          }
          buffers[bufferI].generatorLine = newGeneratorLine;

#if DEBUG >= 2
          std::cout << "New buffer[" << (int)bufferI
            << "]: fcnt = " << (int)buffers[bufferI].fcntup
            << ", firstOne = " << (int)firstOne << ", lastOne = " << (int)lastOne
            << ", windowSize = " << (int)buffers[bufferI].windowSize
            << ", parity check = ";
          displayCharArray(buffers[bufferI].parityCheck, dataPointSize, 1, ' ');
          std::cout << std::endl;
          displayBoolArray(buffers[bufferI].generatorLine, buffers[bufferI].windowSize);
          std::cout << std::endl;
#endif

          bufferI++;
          newBuffers += 1;
        }
      }
#if DEBUG >= 2
      std::cout << "There are now still " << newBuffers << " buffers with information left" << std::endl;
#endif
    }

    free(subMatrix);
    free(X);
  }
}

//#define DEBUG_G2RREF
void DaReDecode::g2rref(bool* matrix, uint32_t width, uint32_t height, uint8_t *X) {
  uint32_t n = width, m = height, i = 0, j = 0, a, b, k;
  bool kFound, tempBool;
  uint8_t tempChar, dataPoint_i;

  while ((i < m) && (j < n)) {
    // Find value and index of largest element in the remainder of column j.
    kFound = false;
    while (!kFound) {
      for (a = i; a < m; a++) {
        if (matrix[j + width * a] == 1) {
          kFound = true;
          k = a;
          break;
        }
      }
      if (!kFound) {
        j += 1;
        if (j >= n) {
          return;
        }
      }
    }

    if (kFound) {
#ifdef DEBUG_G2RREF
      std::cout << "k = " << k;
      std::cout << std::endl;
#endif

      // Swap i - th and k - th rows.
      for (b = j; b < n; b++) {
        tempBool = matrix[b + width * k];
        matrix[b + width * k] = matrix[b + width * i];
        matrix[b + width * i] = tempBool;
      }
      for (dataPoint_i = 0; dataPoint_i < dataPointSize; dataPoint_i++) {
        tempChar = X[dataPoint_i + dataPointSize*k];
        X[dataPoint_i + dataPointSize*k] = X[dataPoint_i + dataPointSize*i];
        X[dataPoint_i + dataPointSize*i] = tempChar;
      }

#ifdef DEBUG_G2RREF
      std::cout << "After swap: ";
      std::cout << std::endl;
      displayBoolArray(matrix, width*height, width);
      displayCharArray(X, height * dataPointSize, dataPointSize, ' ');
      std::cout << std::endl;
#endif
    }

    // Save the right hand side of the pivot row
    bool *aijn = new bool[n - j]();
    for (b = j; b < n; b++) {
      aijn[b - j] = matrix[b + width * i];
    }
#ifdef DEBUG_G2RREF
    std::cout << "aijn: ";
    displayBoolArray(aijn, n - j);
    std::cout << std::endl;
#endif

    // Column we're looking at
    bool *col = new bool[m]();
    for (a = 0; a < m; a++) {
      col[a] = matrix[j + width * a];
    }

    // Never Xor the pivot row against itself
    col[i] = 0;

#ifdef DEBUG_G2RREF
    std::cout << "col: ";
    displayBoolArray(col, m);
    std::cout << std::endl;
#endif
    
    for (a = 0; a < m; a++) {
      for (b = j; b < n; b++) {
        matrix[b + width * a] = matrix[b + width * a] ^ (col[a] & aijn[b - j]);
      }

      if (col[a]) {
        for (dataPoint_i = 0; dataPoint_i < dataPointSize; dataPoint_i++) {
          X[dataPoint_i + dataPointSize*a] ^= X[dataPoint_i + dataPointSize*i];
        }
      }
    }
#ifdef DEBUG_G2RREF
    std::cout << "After XOR: ";
    std::cout << std::endl;
    displayBoolArray(matrix, width*height, width);
    displayCharArray(X, height * dataPointSize, dataPointSize, ' ');
    std::cout << std::endl;
    std::cout << std::endl;
#endif

    i += 1;
    j += 1;

    free(aijn);
    free(col);
  }
}
