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
#include <iostream>
#include <iomanip>

/*!
* \brief   Function to display a char array in hexadecimal representation
*
* \param   [IN] data - the data array to be displayed
* \param   [IN] dataSize - the size of the data array
* \param   [IN] breaks - optional newline brakes after x data elements
*/
void displayCharArray(unsigned char data[], int dataSize, int breaks, char breakChar) {
	std::cout << std::setfill('0');
	for (int i = 0; i < dataSize; i++) {
		if (breaks != 0 && i != 0 && (i % breaks) == 0) {
			std::cout << breakChar;
		}
		std::cout << std::hex << std::setw(2) << (unsigned int)data[i];
	}
	std::cout << std::dec;
}

void displayBoolArray(bool* data, int dataSize, int breaks = 0) {
  for (int i = 0; i < dataSize; i++) {
    if (breaks != 0 && i != 0 && (i % breaks) == 0) {
      std::cout << "\n";
    }
    std::cout << ((data[i]) ? "1" : "0") << " ";
  }
}

/*!
* \brief   Function to wait for the return key before continuing
*/
int hang() {
	std::cin.ignore(32767, '\n');
	return 0;
}

/*!
* \brief   Function to convert a hexadecimal character (0-1,A-F) to its integer value
*
* \param   [IN] ch - the hexadecimal character to return
* \return  the integer value (0 - 15)
*/
int hexCharToDecimal(char ch) {
	ch = toupper(ch); // Change it to uppercase
	if (ch >= 'A' && ch <= 'F')
		return 10 + ch - 'A';
	else // ch is '0', '1', ..., or '9'
		return ch - '0';
}
