/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2017 Semtech
	
Description: General functions for DaRe
License: Revised BSD License, see LICENSE file included in the project
By: Paul Marcelis
*/

#include <iostream>
#include "utilities.h"

#ifndef __DARE_H
#define __DARE_H

#define DEBUG 0 // Amount of debug data to print to std::out. 0 = none, 1 = only result, 2 = process, 3 = all (with matrices)
#define DARE_MAX_W 64 //absolute maximal supported value for window size W
//#define CONVENTIONAL_CODING //uncomment to apply a repetition coding scheme instead of DaRe

class DaRe {
public:
  enum R_VALUE { R_1_2, R_1_3, R_1_4, R_1_5 }; // Coding rate enumerate values
  enum W_VALUE { W_0, W_1, W_2, W_4, W_8, W_16, W_32, W_64 }; // Window size enumerate values
  struct Payload {
    uint8_t *payload;
    uint8_t payloadSize;
  };

  static bool *prlg(uint8_t W, uint32_t fcntup, uint8_t R);
  static uint8_t prng(uint8_t max, uint32_t index, uint32_t seed);
  static uint8_t getW(W_VALUE);
  static uint8_t getR(R_VALUE);
  static double w2d(uint8_t W);
  static uint8_t getWindowSize(uint8_t W, uint32_t fcntup);
};

#define W2D_A 0.75
#define W2D_B -0.0625
#define W2D_C 0.25

#endif
