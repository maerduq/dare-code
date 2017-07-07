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
#include "DaRe.h"

/*
 * Calculate the optimal degree (relative number of data units in your parity check) from the window size
 * 
 */
double DaRe::w2d(uint8_t W) {
  return W2D_A * exp(W2D_B * W) + W2D_C;
}

/*
 * If the window size W is larger than the history (calculated from the frame counter), return the maximum possible window size
 */
uint8_t DaRe::getWindowSize(uint8_t W, uint32_t fcntup) {
  return ((fcntup - 1) < W) ? (fcntup - 1) : W;
}

/*
 * Convert a window size W enumerate value to the corresponding integer value, returns 0 if incorrect
 */
uint8_t DaRe::getW(W_VALUE enumW) {
  uint8_t W = 0;
  switch (enumW) {
  case W_0:
    W = 0;
    break;
  case W_1:
    W = 1;
    break;
  case W_2:
    W = 2;
    break;
  case W_4:
    W = 4;
    break;
  case W_8:
    W = 8;
    break;
  case W_16:
    W = 16;
    break;
  case W_32:
    W = 32;
    break;
  case W_64:
    W = 64;
    break;
  }

  return W;
}

/*
 * Convert a code rate R enumerate value to the corresponding integer value
 */
uint8_t DaRe::getR(R_VALUE enumR) {
  uint8_t R = 0;
  switch (enumR) {
  case R_1_2:
    R = 2;
    break;
  case R_1_3:
    R = 3;
    break;
  case R_1_4:
    R = 4;
    break;
  case R_1_5:
    R = 5;
    break;
  }

  return R;
}
#ifdef CONVENTIONAL_CODING
/*
 * Pseudo random line generator function for conventional coding. 
 */
bool *DaRe::prlg(uint8_t W, uint32_t fcntup, uint8_t R) {
  bool *line = new bool[W]();
  if (R > W) {
    return line;
  }

  line[R] = 1;

  return line;
}
#else
/*
 * Pseudo random line generator for DaRe. This function is used to calculate the generator lines that are used to calculate the parity checks for certain frames
 * @param W - window size
 * @param fcntup - frame counter value for the frame to calculate the generator line for
 * @param R - code rae
 */
bool *DaRe::prlg(uint8_t W, uint32_t fcntup, uint8_t R) {
  double d = w2d(W); //calculate relative degree
  uint8_t D = (uint8_t) round(W * d); //determine absolute degree, number of previous data units to use in the parity check
#if DEBUG >= 3
  std::cout << "d = " << d << ", D = " << (unsigned int)D << std::endl;
#endif
  bool *line = new bool[W]();
  uint32_t index = fcntup, indexNew, indexTemp;
  uint8_t onesAdded = 0;

  // determine pseudo-randomly the index of the previous data units to use in the parity check
  while (onesAdded < D) {
    indexNew = prng(W, index, fcntup + (R << 3));
    indexTemp = index;

    // if the pseudo random number generator returns an already included data unit, retry until a new one is selected
    while (line[indexNew]) {
      indexTemp += 7;
      indexNew = prng(W, indexTemp, fcntup + (R << 3));
    }
    line[indexNew] = 1;
    index = indexNew;
    onesAdded++;
  }

  return line;
}
#endif

/*
 * calculate a pseudo random number on the interval [0, max] with index and seed as seeds
 * implemented as a linear feedback shift register with period 255
 */
uint8_t DaRe::prng(uint8_t max, uint32_t index, uint32_t seed) {
  uint8_t period = 255;
  uint8_t lfsrOut;
  uint8_t lfsr;
  uint8_t step;
  uint8_t bitMask;

  index = index % period;
  seed = (seed % (period - 1)) + 1;

  lfsr = seed;
  step = 0;
  while (step < index) {
    //x^8+x^6+x^5+x^4+1
    lfsrOut = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 4)) & 1;
    lfsr = (lfsr >> 1) | (lfsrOut << 7);
    step++;
  }

  bitMask = max - 1;
  return (lfsr & bitMask);
}
