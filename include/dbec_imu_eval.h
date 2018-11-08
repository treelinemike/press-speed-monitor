/* dbec_imu_eval.h
 *
 * Definitions and functions specific to DBEC IMU
 * evaluation prototype system. These functions enable
 * SPI communication with IMU device, interpretation of
 * gyro and accelerometer data for debug purposes, and
 * transmission of serial packets via RS232 (USART).
 *
 * Date: 	2016-06-22
 * Author: 	M. Kokko
 *
 */

#ifndef DBEC_IMU_EVAL_H_
#define DBEC_IMU_EVAL_H_

// includes
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include "xmega_usart.h"

// application definitions
#define PKT_TYPE_ST_DATA 		0x01
#define PKT_TYPE_BOSCH_DATA 	0x02
#define PKT_TYPE_ST_CONFIG		0xA1
#define PKT_TYPE_BOSH_CONFIG	0xA2
#define PKT_TYPE_ENCODER_DATA	0xAA
#define DLE 					0x10
#define STX						0x02
#define ETX						0x03
#define CONFIG_MSG_DT   		10    // period of configuration message delivery (# of overflows of TCC0)

// define "no-op" macro to waste a clock cycle
#ifndef NOP
#define NOP() asm("nop")
#endif

/* * * * * Function Prototypes * * * * */

// initialize SPI peripheral and ST IMU device
uint8_t initST(void);

// write a byte to the ST device over SPI bus
uint8_t writeST(uint8_t regAddr, uint8_t writebyte);

// read a byte from the ST device over SPI bus
uint8_t readST(uint8_t regAddr, uint8_t *readbyte);

// compute CRC value from an array of bytes (can be one byte in length)
uint8_t crcAddBytes(uint8_t *CRC, uint8_t *byteArray, uint16_t numBytes);

// write a byte to the serial port, twice in the case of a DLE byte
uint8_t usartWriteDLEStuff(USART_t * usartPort, char out);

// send a packet of IMU measurement or configuration data out over the serial port
uint8_t sendIMUSerialPacket(uint8_t packetType, uint16_t microTime, uint8_t *imuTimeBytes, uint8_t imuTimeBytesSize, uint8_t *imuDataBytes, uint8_t imuDataBytesSize);

// send a packet of encoder data out over the serial port
uint8_t sendEncoderSerialPacket(uint8_t packetType, uint16_t microTime, uint32_t encoderCount);

// conversion functions for reference and debugging
// not for use in actual application (do conversions on PC side)
float uint16_to_float16(uint16_t uint16_value);
float imuVal16(uint16_t twosVal,float dataRange);


#endif
