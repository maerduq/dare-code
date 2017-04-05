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
#include "DaRe.h"

#ifndef __DARE_ENCODE_H
#define __DARE_ENCODE_H

class DaReEncode {
  DaRe::R_VALUE MaxR, SetR;
  DaRe::W_VALUE MaxW, SetW;
  uint8_t DataPointSize;
  uint8_t *DataPointHistory;
  uint32_t DataPointHistorySize;

public:
  void init(DaRe::Payload *payload, uint8_t dataPointSizeIn, DaRe::R_VALUE maxR, DaRe::W_VALUE maxW);
  bool set(DaRe::R_VALUE setR, DaRe::W_VALUE setW);
  bool setR(DaRe::R_VALUE setR);
  bool setW(DaRe::W_VALUE setW);
  DaRe::R_VALUE getR();
  DaRe::W_VALUE getW();
  void encode(DaRe::Payload *transmit, uint8_t *dataPoint, uint32_t fcntup);
  void destroy();
};

#endif
