/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2017 Semtech
	
Description: Utility functions for debugging / testing
License: Revised BSD License, see LICENSE file included in the project
By: Paul Marcelis
*/
#ifndef __DARE_UTILITIES_H
#define __DARE_UTILITIES_H

void displayCharArray(unsigned char data[], int dataSize, int breaks = 0, char breakChar = '\n');
void displayBoolArray(bool* data, int dataSize, int breaks = 0);
int hang();
int hexCharToDecimal(char);

#endif
