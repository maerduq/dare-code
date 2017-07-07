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

/*
 * initialise a DaRe decoder. 
 * @param dataPointSizeIn - the size in bytes of the data points that will be transmitted. should be constant during runtime. zero padding is possible to keep the size constant, then maximum data point size should be used here
 * @param simulationLength - required to allocate sufficient memory for results
 */
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

/*
 * destroy the DaRe decoder
 */
void DaReDecode::destroy() {
  free(dataPointsReceived);
  free(isDataPointReceived);
}

/*
 * helper function to clear all buffers from intermediate decoded data
 */
void DaReDecode::flushBuffers() {
  checkBuffersForSubmatrix(true, totalDataPoints);
}

#if DEBUG >= 0
/*
 * store the known value for a certain data point for later correctness comparison of the decoded value
 */
void DaReDecode::debugData(uint32_t fcntup, uint8_t *dataPoint) {
  uint8_t i;
  for (i = 0; i < dataPointSize; i++) {
    dataPointsDebug[(fcntup - 1) * dataPointSize + i] = dataPoint[i];
  }
}
#endif

/*
 * store the decoded value at a certain position
 * @param fcntup - frame counter value of the frame the decoded data point is originally from
 * @param dataPoint - the decoded value
 * @param currentFcntup - frame counter value of the current received frame, to compute the decoding delay for this data point
 * @param phase - the phase at which the data point was decoded, for statistics purposes
 */
void DaReDecode::storeDataPoint(uint32_t fcntup, uint8_t *dataPoint, uint32_t currentFcntup, int phase) {
  uint8_t i;
  bool wrong = false; //to verify the decoded value
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

/*
 * clear the intermediate decoded value from a certain buffer. freeing memory does not work (yet)
 */
void DaReDecode::clearBuffer(uint32_t bufferI) {
  //free(buffers[bufferI].parityCheck);
  //free(buffers[bufferI].generatorLine);
  buffers[bufferI].inUse = false;
}

/*
 * Main function to decode the payload from a certain frame
 * @param payload - the payload from the frame to be decoded
 * @param fcntup - the frame counter
 */
void DaReDecode::decode(DaRe::Payload payload, uint32_t fcntup) {
  uint8_t windowSize, W, R, dataPointOffset;
  uint32_t dataPointOffsetPointer;
  bool *generatorLine;
  bool previousDataRecovered = false;
  uint8_t R_i, dataPoint_i;
  int bufferI;

  // get coding paramter values, code rate R and window size W from the first byte in the payload
  DaRe::R_VALUE enumR = (DaRe::R_VALUE) (payload.payload[0] >> 4);
  DaRe::W_VALUE enumW = (DaRe::W_VALUE) (payload.payload[0] & 0xf);
  W = DaRe::getW(enumW);
  R = DaRe::getR(enumR);


  //** STAGE 1 DATA RECOVERY | NORMAL RECOVERY **//
  // store the current data point from the payload
  storeDataPoint(fcntup, &payload.payload[1], fcntup, 1);

  // Check if a previous frame was not received...
  if (lastFcntup < (fcntup - 1)) {
    tryToRecover = true; //if so, try to recover
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
    // the code rate indicates the number of parity checks included in the frame payload for R = 2, one parity check is included, for R = 3, two parity checks, etc.
    for (R_i = 0; R_i < R - 1; R_i++) {
      generatorLine = DaRe::prlg(W, fcntup, R_i); // recalculate the generator line for this parity check

#if DEBUG >= 3
      displayBoolArray(generatorLine, windowSize); 
      std::cout << std::endl;
#endif
      // iterate over the window size
      for (dataPointOffset = 1; dataPointOffset <= windowSize; dataPointOffset++) {
        dataPointOffsetPointer = ((fcntup - 1) - dataPointOffset); // Calculate pointer for this previous data point
        if (isDataPointReceived[dataPointOffsetPointer]) { // if the value for this data point is known, received or decoded...
#if DEBUG >= 3
          std::cout << (int)dataPointOffsetPointer << ", ";
#endif
          //... and if the data point is included in the parity check ..
          if (generatorLine[dataPointOffset - 1] == 1) {
            generatorLine[dataPointOffset - 1] = 0; //... remove the data point from the generator line ...
#if DEBUG >= 3
            std::cout << "0x";
            displayCharArray(&dataPointsReceived[dataPointOffsetPointer * dataPointSize], dataPointSize);
            std::cout << std::endl;
#endif
            // ... and remove the data point from the parity check by XORing the value with the parity check value, bytewise
            for (dataPoint_i = 0; dataPoint_i < dataPointSize; dataPoint_i++) {
              payload.payload[1 + dataPointSize * (1 + R_i) + dataPoint_i] ^= dataPointsReceived[dataPointOffsetPointer * dataPointSize + dataPoint_i]; // XOR it
            }
          }
        }
      }
      
      // now check how much data points are still included in the parity check
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
      case 0: //if no data points are left in the parity check, no new information is received
#if DEBUG >= 2
        std::cout << "No new data" << std::endl;
#endif
        break;
      case 1: //if one data point is left in the parity check, a data point is recovered!
        //** STAGE 2 DATA RECOVERY | DIRECTLY FROM PARITY CHECK **//
        storeDataPoint(fcntup - newDataOffset, &payload.payload[1 + dataPointSize * (1 + R_i)], fcntup, 2);
        previousDataRecovered = true; // set flag for data point recovered to continue the iterative decoding
        break;
      default: //if more than one data point is left in the parity check, the intermediate result should be stored in a buffer instance
        // so a new buffer entry. First to check if there is a submatrix in the buffers.

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
        // If the buffers are full, take oldest buffer entry and replace it. The oldest buffer has the smallest probability of being solved ever again
        if (!emptyBufferFound) {
#if DEBUG >= 2
          std::cout << "BUFFER FULL!!!, try to solve what's possible, check if something comes clear" << std::endl;
#endif
          bufferI = 0;
          uint32_t bufferDataI = buffers[0].fcntup;
          for (j = 0; j < DARE_DECODING_BUFFERS; j++) {
            if (bufferDataI > buffers[j].fcntup) {
              bufferI = j;
              bufferDataI = buffers[j].fcntup;
            }
          }
        }
        // result of this functipn part: bufferI

        // fill the selected buffer instance
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

    // This is the iterative decoding part: if there is data recovered previously in decoding, 
    // the buffers with parity checks that contain this new value should be updated in order to check whether
    // they now contain only one data point an can be used to recover a certain data point.
    while (previousDataRecovered) {
      previousDataRecovered = false; //reset flag
      for (bufferI = 0; bufferI < DARE_DECODING_BUFFERS; bufferI++) {
        // only process buffers in use
        if (!buffers[bufferI].inUse) {
          continue;
        }
#if DEBUG >= 2
        std::cout << "-- Checking BUFFER[" << bufferI << "]" << std::endl;
#endif
        for (dataPointOffset = 1; dataPointOffset <= buffers[bufferI].windowSize; dataPointOffset++) {
          dataPointOffsetPointer = (((buffers[bufferI].fcntup - 1) - dataPointOffset)); // Calculate pointer for previous data point
          if (buffers[bufferI].generatorLine[dataPointOffset - 1] == 1 && isDataPointReceived[dataPointOffsetPointer]) { // If the data point is known and in the generator line ...
            buffers[bufferI].generatorLine[dataPointOffset - 1] = 0; // ... remove the data point from the generator line ...
            for (dataPoint_i = 0; dataPoint_i < dataPointSize; dataPoint_i++) {
              // .. and remove the data point from the parity check by XORing bytewise
              buffers[bufferI].parityCheck[dataPoint_i] ^= dataPointsReceived[dataPointOffsetPointer * dataPointSize + dataPoint_i];
            }
          }
        }

        int generatorLineOnes = 0;
        int newDataOffset = 0;
        int j;
        // check the number of data points in the parity check
        for (j = 0; j < buffers[bufferI].windowSize; j++) {
          if (buffers[bufferI].generatorLine[j] == 1) {
            generatorLineOnes += 1;
            newDataOffset = j + 1;
          }
        }

        switch (generatorLineOnes) {
        case 0: //if no data points in the parity check left, empty the buffer
#if DEBUG >= 2
          std::cout << "No new data in this buffer anymore" << std::endl;
#endif
          clearBuffer(bufferI);
          break;
        case 1: //if one data point in the parity check, save this as a decoded value
          //** STAGE 3 DATA RECOVERY | FROM A BUFFER **//
          storeDataPoint(buffers[bufferI].fcntup - newDataOffset, buffers[bufferI].parityCheck, fcntup, 3);
          previousDataRecovered = true; //and raise flag that another data point is recovered

          clearBuffer(bufferI);
          break;
#if DEBUG >= 2
        default:
          std::cout << "Still not more info" << std::endl;
#endif
        }
      }
    }

    // finally, try to find more data points in all buffers
    checkBuffersForSubmatrix(false, fcntup);
  }

  // reset the try to recover flag if all previous data points are recovered
  if (recovered == fcntup && tryToRecover) {
#if DEBUG >= 2
    std::cout << "--------- We are complete!" << std::endl;
#endif
    tryToRecover = false;
  }
}

/*
 * perform Gaussian elimination on the results in the buffers in order to eliminate linear dependence between buffer values and to find more data points
 * @param flushBuffers - whether the buffers should be flushed after being processed
 * @param fcntup - frame counter of current frame (used to compute recovery delay)
 */
void DaReDecode::checkBuffersForSubmatrix(bool flushBuffers, uint32_t fcntup) {
  uint32_t currentNewestDataPointId = 0, currentOldestDataPointId = 0, buffersInUse = 0;
  uint32_t bufferI, dataPointOffset, dataPointOffsetPointer, dataPoint_i, j;
  bool currentOldestDataPointIdSet = false;

  // loop through all buffers to determine the newest and oldest data point in the buffers
  for (bufferI = 0; bufferI < DARE_DECODING_BUFFERS; bufferI++) {
    // only consider buffers in use
    if (!buffers[bufferI].inUse) {
      continue;
    }
    buffersInUse += 1; // determine number of buffers in use

    for (dataPointOffset = 1; dataPointOffset <= buffers[bufferI].windowSize; dataPointOffset++) {
      if (buffers[bufferI].generatorLine[dataPointOffset - 1] == 1) {
        dataPointOffsetPointer = (((buffers[bufferI].fcntup - 1) - dataPointOffset)); // Calculate pointer for this previous data point that is included in the parity check
#if DEBUG >= 3
        std::cout << "d[" << (unsigned int)dataPointOffsetPointer << "], ";
#endif
        // determine the newest data point id of all datapoints included in all buffers
        if (dataPointOffsetPointer > currentNewestDataPointId) {
          currentNewestDataPointId = dataPointOffsetPointer;
        }
        // determine the oldest data point id of all datapoints included in all buffers
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
    << "- newest current data point: " << (unsigned int)currentNewestDataPointId << std::endl;
#endif

  // quit the function if no buffers are used
  if (buffersInUse == 0) {
#if DEBUG >= 2
    std::cout << "No buffers in use.." << std::endl;
#endif
    return;
  }
  // if there is only one buffer in use, Gaussian elimination cannot be performed
  else if (buffersInUse == 1) { 
#if DEBUG >= 2
    std::cout << "Only one buffer in use, so discard it.." << std::endl;
#endif
    if (flushBuffers) {
      for (bufferI = 0; bufferI < DARE_DECODING_BUFFERS; bufferI++) {
        if (!buffers[bufferI].inUse) {
          continue;
        }
        clearBuffer(bufferI);
        break;
      }
    }
    return;
  // if more than one buffer in use, perform Gaussian elimination
  } else {
    // create submatrix that expresses relation between data points and the parity checks in buffers
    uint32_t subMatrixWidth = (currentNewestDataPointId - currentOldestDataPointId + 1);
    bool *subMatrix = new bool[subMatrixWidth * buffersInUse]();
    // create array to contain the parity check values
    uint8_t *X = new uint8_t[buffersInUse * dataPointSize]();

    // variable that will hold the number of parity checks in the submatrix
    uint32_t nrBufferInUse = 0;
    for (bufferI = 0; bufferI < DARE_DECODING_BUFFERS; bufferI++) {
      if (!buffers[bufferI].inUse) {
        continue;
      }
      // for each buffer, fill the submatrix using the generator line, and fill X with the parity check values
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
    // now perform Gaussian elimination in GF(2) over the submatrix
    DaReDecode::g2rref(subMatrix, subMatrixWidth, buffersInUse, X);
#if DEBUG >= 3
    displayBoolArray(subMatrix, subMatrixWidth * buffersInUse, subMatrixWidth);
    displayCharArray(X, buffersInUse * dataPointSize, dataPointSize, ' ');
    std::cout << std::endl;
#endif

    uint32_t dataPointFoundIndex;
    uint8_t nrDataPointsInParityCheck;
    bool foundOne = true;
    while (foundOne) {
      foundOne = false;
      for (nrBufferInUse = 0; nrBufferInUse < buffersInUse; nrBufferInUse++) {
        nrDataPointsInParityCheck = 0;
        dataPointFoundIndex = 0;
        // determine how much data points are in the parity check (and store the index of the last one)
        for (j = 0; j < subMatrixWidth; j++) {
          if (subMatrix[nrBufferInUse*subMatrixWidth + j] == 1) {
            nrDataPointsInParityCheck += 1;
            dataPointFoundIndex = j;
          }
        }

        // if only one data point is in the parity check, store it!
        if (nrDataPointsInParityCheck == 1) {
          //** STAGE 4 DATA RECOVERY | FROM A SOLVED SUBMATRIX **//
          storeDataPoint(currentOldestDataPointId + dataPointFoundIndex + 1, &X[nrBufferInUse*dataPointSize], fcntup, 4);
          subMatrix[nrBufferInUse*subMatrixWidth + dataPointFoundIndex] = 0;
          // remove the known data point value from parity checks that had this data point included
          for (j = 0; j < buffersInUse; j++) {
            if (subMatrix[j*subMatrixWidth + dataPointFoundIndex] == 1) {
              subMatrix[j*subMatrixWidth + dataPointFoundIndex] = 0;
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
          foundOne = true; // flag to trigger iterative decoding
          break;
        }
      }
    }

    // clear all current buffers in use
    for (bufferI = 0; bufferI < DARE_DECODING_BUFFERS; bufferI++) {
      if (!buffers[bufferI].inUse) {
        continue;
      }
      clearBuffer(bufferI);
    }

    // if flush buffers flag was set, don't refill the buffers with the result of the Gaussian elimination, and reset the flag to try to recover data points
    if (flushBuffers) {
      tryToRecover = false;
    } else {
      // fill the buffers again with the result of the Gaussian elimination
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
              // if the oldest data point in the parity check cannot be included in a to be received parity check, discard the parity check
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
        } else if (thisValueIsDoomed) {
#if DEBUG >= 1
          std::cout << "-- d[" << currentOldestDataPointId + firstOne << "] is forever lost!" << std::endl;
#endif
        } else {
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
/*
 * Function to perform Gaussian elimination in GF(2)
 */
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

/*
 * Debug function for displaying data
 */
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

/*
* Debug function for displaying data point ids
*/
void DaReDecode::displayReceivedDataIds() {
  uint32_t i;

  for (i = 0; i < lastFcntup - 1; i++) {
    if (isDataPointReceived[i]) {
      std::cout << i << ",";
    }
  }
  std::cout << std::endl << std::endl;
}

/*
* Debug function for displaying decoding result
*/
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
