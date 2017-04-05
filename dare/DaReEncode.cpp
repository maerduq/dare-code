/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2017 Semtech
	
Description: Encoding functions
License: Revised BSD License, see LICENSE file included in the project
By: Paul Marcelis
*/
#include "DaReEncode.h"

void DaReEncode::init(DaRe::Payload *payload, uint8_t dataPointSizeIn, DaRe::R_VALUE maxR, DaRe::W_VALUE maxW) {
  MaxR = maxR;
  MaxW = maxW;
  DataPointSize = dataPointSizeIn;
  DataPointHistorySize = DataPointSize * DaRe::getW(MaxW);

  payload->payload = new uint8_t[1 + 2 * DataPointSize*DaRe::getR(MaxR)]();
  DataPointHistory = new uint8_t[DataPointHistorySize]();
}

void DaReEncode::destroy() {
}

bool DaReEncode::set(DaRe::R_VALUE inR, DaRe::W_VALUE inW) {
  return (setR(inR) & setW(inW));
}

bool DaReEncode::setW(DaRe::W_VALUE inW) {
  if ((uint8_t)inW <= (uint8_t)MaxW) {
    SetW = inW;
    return true;
  }
  return false;
}

bool DaReEncode::setR(DaRe::R_VALUE inR) {
  if ((uint8_t)inR <= (uint8_t)MaxR) {
    SetR = inR;
    return true;
  }
  return false;
}

DaRe::R_VALUE DaReEncode::getR() {
  return SetR;
}

DaRe::W_VALUE DaReEncode::getW() {
  return SetW;
}

void DaReEncode::encode(DaRe::Payload *transmit, uint8_t *dataPoint, uint32_t fcntup) {
  uint8_t dataPoint_i, R_i, W, R, windowSize, dataPointOffset;
  uint32_t dataPointOffsetPointer;
  bool *generatorLine;

  W = DaRe::getW(SetW);

#if DEBUG >= 3
  displayCharArray(DataPointHistory, DataPointHistorySize, DataPointSize, ' ');
  std::cout << std::endl;
#endif

  R = DaRe::getR(SetR);
  transmit->payloadSize = 1 + DataPointSize * R;

  for (dataPoint_i = 0; dataPoint_i < transmit->payloadSize; dataPoint_i++) {
    transmit->payload[dataPoint_i] = 0;
  }

  transmit->payload[0] = ((SetR & 0xf) << 4) | (SetW & 0xf);

  // Set d[i]
  for (dataPoint_i = 0; dataPoint_i < DataPointSize; dataPoint_i++) {
    transmit->payload[1 + dataPoint_i] = dataPoint[dataPoint_i];
  }

  // Calculate x[i]
  windowSize = DaRe::getWindowSize(W, fcntup); // Limit window size to number of previous data points
  for (R_i = 0; R_i < R - 1; R_i++) {
    generatorLine = DaRe::prlg(W, fcntup, R_i);
#if DEBUG >= 3
    displayBoolArray(generatorLine, windowSize);
    std::cout << std::endl;
#endif
    // Loop through generator line
    for (dataPointOffset = 1; dataPointOffset <= windowSize; dataPointOffset++) {
      // If there is a one, XOR a previous data point with
      if (generatorLine[dataPointOffset - 1] == 1) {
        dataPointOffsetPointer = (((fcntup - 1) - dataPointOffset) * DataPointSize) % DataPointHistorySize; // Calculate pointer for previous data point
        for (dataPoint_i = 0; dataPoint_i < DataPointSize; dataPoint_i++) {
#if DEBUG >= 3
          std::cout << std::hex << (unsigned int)DataPointHistory[dataPointOffsetPointer + dataPoint_i] << std::endl;
#endif
          transmit->payload[1 + DataPointSize * (1 + R_i) + dataPoint_i] ^= DataPointHistory[dataPointOffsetPointer + dataPoint_i]; // XOR it
        }
      }
    }
  }

  // Write new data point to history
  for (dataPoint_i = 0; dataPoint_i < DataPointSize; dataPoint_i++) {
    DataPointHistory[((fcntup - 1) * DataPointSize + dataPoint_i) % DataPointHistorySize] = dataPoint[dataPoint_i];
  }
}
