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
#include "DaRe.h"

#ifndef __DARE_DECODE_H
#define __DARE_DECODE_H

#define DARE_DECODING_BUFFERS 50 // finite number of buffers to store intermediate data point recovery results

class DaReDecode {
  uint8_t dataPointSize;
  uint32_t totalDataPoints;
  uint8_t *dataPointsReceived;
  uint32_t *dataPointsDelay;
#if DEBUG >= 0
  uint8_t *dataPointsDebug;
#endif
  bool *isDataPointReceived;
  uint32_t lastFcntup = 0;
  bool tryToRecover = false;

  int recovered = 0;
  int recoverPhase[5] = { 0, 0, 0, 0, 0 };

  struct buffer {
    bool inUse = false;
    uint32_t fcntup;
    uint8_t *parityCheck;
    bool *generatorLine;
    uint8_t windowSize;
  };
  buffer buffers[DARE_DECODING_BUFFERS];

  void storeDataPoint(uint32_t fcntup, uint8_t *dataPoint, uint32_t currentFcntup, int phase);
  void g2rref(bool* matrix, uint32_t width, uint32_t height, uint8_t *X);
  void clearBuffer(uint32_t bufferI);
  void checkBuffersForSubmatrix(bool flushBuffers, uint32_t fcntup);

public:
  void init(uint8_t dataPointSizeIn, uint32_t simulationLength);
  void destroy();
  void decode(DaRe::Payload payload, uint32_t fcntup);
  void displayReceivedData(uint8_t *dataToCheck);
  void displayReceivedDataIds();
  void displayResults();
  void flushBuffers();
#if DEBUG >= 0
  void debugData(uint32_t fcntup, uint8_t *dataPoint);
#endif
};

#endif
