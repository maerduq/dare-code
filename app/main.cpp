/*
/ _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
\____ \| ___ |    (_   _) ___ |/ ___)  _ \
_____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
(C)2017 Semtech

Description: Simulation program to show DaRe Coding encoding and decoding
License: Revised BSD License, see LICENSE file included in the project
By: Paul Marcelis
*/

#include <iostream>
#include <time.h>
#include "utilities.h"
#include "DaRe.h" // the DEBUG flag in this file determines the simulation output
#include "DaReEncode.h"
#include "DaReDecode.h"

#define SIMULATION_LENGTH 100000 // Number of frames to send for one run
#define DATA_POINT_SIZE 2

uint8_t *getDataPoint();
void simulation(DaRe::R_VALUE, DaRe::W_VALUE, int);

int main() {
  // Set random seed
  srand((unsigned int)time(NULL));

  // Start!
  std::cout << "DaRe Coding for LoRaWAN" << std::endl;
  std::cout << "Data point size: " << DATA_POINT_SIZE << " bytes" << std::endl;
    
  std::cout << std::endl;
  std::cout << "R \tW \tp_e \tp_rr \trec \tphase1 \tphase2 \tphase3 \tphase4 \tphase5 \tavg_delay \tvar_delay" << std::endl;


  simulation(DaRe::R_1_2, DaRe::W_8, 10);
  return hang();
}

// if the p_e_percent parameter gives the percentage of frames to drop randomly.
void simulation(DaRe::R_VALUE R, DaRe::W_VALUE W, int p_e_percent) {
  uint32_t framesReceived = 0, fcntup;
  uint8_t *dataPoint;
  DaRe::Payload payload;
  DaReEncode encoding;
  DaReDecode decoding;

  // Initialisation of encoder and decoder
  encoding.init(&payload, DATA_POINT_SIZE, DaRe::R_1_5, DaRe::W_64);
  encoding.set(R, W);
  decoding.init(DATA_POINT_SIZE, SIMULATION_LENGTH);

  // simulate SIMULATION_LENGTH frames
  for (fcntup = 1; fcntup <= SIMULATION_LENGTH; fcntup++) {
    // get a random value
    dataPoint = getDataPoint();

#if DEBUG >= 2
    std::cout << std::endl << "-------- fcntup " << fcntup << ", d[" << (fcntup - 1) << "] --------";
    std::cout << std::endl << "Data: ";
    displayCharArray(dataPoint, DATA_POINT_SIZE, 1, ' ');
    std::cout << std::endl;
#endif

#if DEBUG >= 0
    // Pass original data unit to decover to be able to compare results for debugging
    decoding.debugData(fcntup, dataPoint);
#endif

    // encode
    encoding.encode(&payload, dataPoint, fcntup);


#if DEBUG >= 2
    std::cout << "Transmitted payload: ";
    displayCharArray(payload.payload, payload.payloadSize, 1, ' ');
    std::cout << std::endl << std::endl;
#endif

    // TRANSMISSION: fcntup & payload

    // simulate lost frames
    if ((rand() % 1000) < p_e_percent * 10) {
#if DEBUG >= 2
      std::cout << "-- Packet lost.." << std::endl;
#endif
      continue;
    }

    // decode
    decoding.decode(payload, fcntup);
    framesReceived++;
  }
  decoding.flushBuffers();

#if DEBUG >= 1
  std::cout << std::endl
    << "Send: \t\t" << SIMULATION_LENGTH << std::endl
    << "p_e: \t\t" << p_e_percent << std::endl;
#else
  std::cout << (int)DaRe::getR(R) << "\t" << (int)DaRe::getW(W) << "\t" << p_e_percent << "\t";
#endif
  decoding.displayResults();

  encoding.destroy();
  decoding.destroy();
}

uint8_t *getDataPoint() {
  uint8_t dataI;
  uint8_t *data = new uint8_t[DATA_POINT_SIZE]();
  for (dataI = 0; dataI < DATA_POINT_SIZE; dataI++) {
    data[dataI] = (rand() % 0xFF);
  }
  return data;
}
