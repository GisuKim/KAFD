/*
 * UTIL_CRC8.h
 *
 *  Created on: 2022. 8. 29.
 *      Author: syslabs
 */

#ifndef UTIL_INCLUDE_UTIL_CRC8_H_
#define UTIL_INCLUDE_UTIL_CRC8_H_


#include "f2838x_Device.h"


#define CRC8_INIT_VALUE        0x0000
#define CRC8_XOR_VALUE        0x0000

void CRC8_InitChecksum(unsigned char *crcVal);

void CRC8_Update( unsigned char *crcvalue, const unsigned char data );
void CRC8_UpdateChecksum( unsigned char *crcvalue, const void *data, int length );

void CRC8_FinishChecksum( unsigned char *crcvalue );

unsigned char CRC8_BlockChecksum( const void *data, int length );




#endif /* UTIL_INCLUDE_UTIL_CRC8_H_ */
