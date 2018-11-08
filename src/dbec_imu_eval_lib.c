/* dbec_imu_eval.h
 *
 * Functions specific to DBEC IMU
 * evaluation prototype system. These functions enable
 * SPI communication with IMU device, interpretation of
 * gyro and accelerometer data for debug purposes, and
 * transmission of serial packets via RS232 (USART).
 *
 * Date: 	2016-06-22
 * Author: 	M. Kokko
 *
 */

// includes
#include "dbec_imu_eval.h"

// initialize SPI peripheral and ST IMU device
// TODO: pass settings as parameters to this function or #define them in header file, hard-coded for now...
// TODO: add error handling
uint8_t initST(void){

	// temporary variable for building SPI control register value
	uint8_t spid_ctrl_val;

	// configure SPI CS pin (ST IMU) as an output and pull high
	PORTE.DIR |= PIN0_bm;
	PORTE.OUT |= PIN0_bm;

	// configure SPI for master mode(SPID on PORTD)
	// SPID MOSI = PD5 #31
	// SPID MISO = PD6 #32
	// SPID CLK  = PD7 #33
	// CS for each device controlled with GPIO pins
	PORTD.DIR |= PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm; // NOTE: *must* make AVR hardwared SS pin (PD4) an output, even if not using!
	PORTD.OUT |= PIN5_bm | PIN6_bm | PIN7_bm;			// pull SPI lines high

	// set control parameters for SPI peripheral
	// for some reason the registers need to be written as a single line
	// so we use a temporary variable for line-by-line setting
	spid_ctrl_val = 0x00;								// reset SPI control register
	spid_ctrl_val &= ~SPI_CLK2X_bm;						// DO NOT double SPI clock speed
	spid_ctrl_val |= SPI_ENABLE_bm;						// enable SPI
	spid_ctrl_val |= SPI_MASTER_bm;						// configure as SPI master
	spid_ctrl_val |= SPI_MODE_0_gc;						// mode 0: sample on rising edge of clock, setup data on falling edge
	spid_ctrl_val |= SPI_PRESCALER_DIV128_gc;			// SPICLK = SYSCLK/128 = 125kHz; this looks good leaving ~50% of 100Hz cycle in idle
	SPID.CTRL = spid_ctrl_val;							// set SPID.CTRL register
	SPID.INTCTRL = 0x00;								// no SPI interrupts

	// configure ST device
	//readST(0x0F,&spi_data_a); 						// TODO: assert if device doesn't return correct ID
	writeST(0x12,0x01); 								// reset chip - not certain that this is necessary but it helped resolve an issue where after power cycles the "data ready" flags would never be set in register 0x1B
	_delay_ms(10);      								// wait for chip to reset, see note above
	writeST(0x12,0x40);									// set BDU bit, ensuring all pairs of data (high and low registers) are from the same sample
	writeST(0x18,0x38); 								// enable accelerometers
	writeST(0x10,0x60); 								// set accelerometer ODR
	writeST(0x19,0x38); 								// enable gyros
	writeST(0x11,0x68); 								// set gyro ODR
	writeST(0x58,0x80); 								// enable timer for timestamp generation
	_delay_ms(2);
	writeST(0x5C,0x10); 								// set timestamp resolution to HIGH (25us / 7min limit) if desired; NOTE: ST COUNTER DOES NOT OVERFLOW (Bosch timer does overflow)
	//writeST(0x5C,0x00); 								// set timestamp resolution to LOW (6.4ms / 29hr limit) if desired; NOTE: ST COUNTER DOES NOT OVERFLOW (Bosch timer does overflow)
	_delay_ms(2);
	writeST(0x42,0xAA); 								// restart counter
	_delay_ms(2);

	// return, no error handling
	return 0;
}

// write a byte to the ST IMU
// TODO: add error handling
uint8_t writeST(uint8_t regAddr, uint8_t writebyte){

	// pull CS line low
	PORTE.OUT &= ~PIN0_bm;

	// send register address
	SPID.DATA = (0x00 | regAddr);
	while(!(SPID.STATUS & SPI_IF_bm)){};

	// send data
	SPID.DATA = (0x00 | writebyte);
	while(!(SPID.STATUS & SPI_IF_bm)){};

	// pull CS line high
	PORTE.OUT |= PIN0_bm;

	// return, no error handling
	return 0;
}

// read a byte from the ST IMU
// TODO: add error handling
uint8_t readST(uint8_t regAddr, uint8_t *readbyte){

	// pull CS line low
	PORTE.OUT &= ~PIN0_bm;

	// send register address
	SPID.DATA = (0x80 | regAddr);
	while(!(SPID.STATUS & SPI_IF_bm)){};

	// read data
	*readbyte = SPID.DATA;
	SPID.DATA = (0x80);
	while(!(SPID.STATUS & SPI_IF_bm)){};
	*readbyte = SPID.DATA;

	// pull CS line high
	PORTE.OUT |= PIN0_bm;

	// return, no error handling
	return 0;
}

// compute CRC value from an array of bytes (can be one byte in length)
uint8_t crcAddBytes(uint8_t *CRC, uint8_t *byteArray, uint16_t numBytes){

	uint8_t bitNum, thisBit, doInvert;
	uint8_t byteNum = 0;
	uint8_t *pInputByte = byteArray;

	// cycle through all bytes in input string
	while(byteNum < numBytes){

		// cycle through all bits in current byte, starting with the most significant bit
		for(bitNum = 8; bitNum > 0; bitNum--){
			thisBit = !(0 == (*pInputByte & (1<<(bitNum-1))));	// input bit
			doInvert = thisBit ^ ((*CRC & 0x80) >> 7);			// new bit0
			*CRC = *CRC << 1;									// bitshift left by 1 position
			if(doInvert){
				*CRC = *CRC ^ 0b00110001;						// if input bit is 1, invert using x^8 + x^5 + x^4 + 1 polynomial
			}
		}

		// increment input byte counter and pointer
		++byteNum;
		++pInputByte;
	}

	// return, no error handling
	return 0;
}

// send a byte, stuffing an extra DLE character if necessary
uint8_t usartWriteDLEStuff(USART_t * usartPort, char out){

	// write character
	usartWriteGeneric(usartPort, out);

	// repeat if character is DLE
	if(out == DLE){
		usartWriteGeneric(usartPort, out);
	}

	return 0;
}

// send a binary (non-ASCII) serial message with IMU measurement or configuration data
uint8_t sendIMUSerialPacket(uint8_t packetType, uint16_t microTime, uint8_t *imuTimeBytes, uint8_t imuTimeBytesSize, uint8_t *imuDataBytes, uint8_t imuDataBytesSize){

	uint8_t i, microTimeByte;
	uint8_t CRC = 0x00;

	// send start sequence
	usartWriteGeneric(&BINARY_USART,DLE);
	usartWriteGeneric(&BINARY_USART,STX);

	// send packet type
	usartWriteDLEStuff(&BINARY_USART, packetType);
	crcAddBytes(&CRC,&packetType,1);

	// send MICRO time (low byte first)
	microTimeByte = (uint8_t)((microTime & 0x00FF));
	usartWriteDLEStuff(&BINARY_USART, microTimeByte);
	crcAddBytes(&CRC,&microTimeByte,1);

	microTimeByte = (uint8_t)((microTime & 0xFF00) >> 8);
	usartWriteDLEStuff(&BINARY_USART, microTimeByte);
	crcAddBytes(&CRC,&microTimeByte,1);

	// send IMU time (low byte first)
	// these bytes come directly from chip so we read
	// them out of an array rather than from a 24- or 32-bit
	// unsigned integer
	for(i = 0; i < imuTimeBytesSize; i++){
		usartWriteDLEStuff(&BINARY_USART, imuTimeBytes[i]);
	}
	crcAddBytes(&CRC,imuTimeBytes,imuTimeBytesSize);

	// send actual IMU data
	// these data bytes come directly from IMU and are just passed on without processing
	for(i = 0; i < imuDataBytesSize; i++){
		usartWriteDLEStuff(&BINARY_USART, imuDataBytes[i]);
	}
	crcAddBytes(&CRC,imuDataBytes,imuDataBytesSize);

	// send CRC8 checksum
	usartWriteDLEStuff(&BINARY_USART, CRC);

	// send end sequence
	usartWriteGeneric(&BINARY_USART, DLE);
	usartWriteGeneric(&BINARY_USART, ETX);

	// done
	return 0;
}

// send a binary (non-ASCII) serial message with encoder count data
uint8_t sendEncoderSerialPacket(uint8_t packetType, uint16_t microTime, uint32_t encoderCount){

	uint8_t microTimeByte, encoderByte;
	uint8_t CRC = 0x00;

	// send start sequence
	usartWriteGeneric(&BINARY_USART,DLE);
	usartWriteGeneric(&BINARY_USART,STX);

	// send packet type
	usartWriteDLEStuff(&BINARY_USART, packetType);
	crcAddBytes(&CRC,&packetType,1);

	// send MICRO time (low byte first)
	microTimeByte = (uint8_t)((microTime & 0x00FF));
	usartWriteDLEStuff(&BINARY_USART, microTimeByte);
	crcAddBytes(&CRC,&microTimeByte,1);

	microTimeByte = (uint8_t)((microTime & 0xFF00) >> 8);
	usartWriteDLEStuff(&BINARY_USART, microTimeByte);
	crcAddBytes(&CRC,&microTimeByte,1);

	// send encoder count (low byte first)
	encoderByte = (uint8_t)((encoderCount & 0x000000FF));
	usartWriteDLEStuff(&BINARY_USART, encoderByte);
	crcAddBytes(&CRC,&encoderByte,1);

	encoderByte = (uint8_t)((encoderCount & 0x0000FF00) >> 8);
	usartWriteDLEStuff(&BINARY_USART, encoderByte);
	crcAddBytes(&CRC,&encoderByte,1);

	encoderByte = (uint8_t)((encoderCount & 0x00FF0000) >> 16);
	usartWriteDLEStuff(&BINARY_USART, encoderByte);
	crcAddBytes(&CRC,&encoderByte,1);

	// send CRC8 checksum
	usartWriteDLEStuff(&BINARY_USART, CRC);

	// send end sequence
	usartWriteGeneric(&BINARY_USART, DLE);
	usartWriteGeneric(&BINARY_USART, ETX);

	// done
	return 0;
}

// convert two bytes to half precision float
// this function is for debug purposes only, data should not
// be converted on embedded device
float uint16_to_float16(uint16_t uint16_value){
	float float16_value = 0x0000;
	float signbit  = (float)((uint16_value & 0x8000)>>15);
	float exponent = (float)((uint16_value & 0x7C00)>>10);
	float mantissa = (float)((uint16_value & 0x03FF)>>0);

	float signfactor = pow(-1,signbit);

	if(exponent == 0){
		if(mantissa == 0){
			float16_value = (signfactor) * 0;
		} else {
			float16_value = (float)(signfactor * (pow(2,-14)) * (0 + mantissa*pow(2,-10)));
		}
	} else if(exponent == 31){
		if(mantissa == 0){
			float16_value = (signfactor) * INFINITY;
		} else {
			float16_value = NAN;
		}
	} else {
		float16_value = (double)(signfactor * pow(2,(exponent-15)) * (1+mantissa*pow(2,-10)));
	}
	return float16_value;
}

// convert twos complement data to scaled floating point value
// this function is for debug purposes only, data should not
// be converted on embedded device
//
// Examples:
//   timeStamp = (uint32_t)((((uint32_t)(timeStampDataST[2]))<<16) + (((uint32_t)(timeStampDataST[1]))<<8) + ((uint16_t)(timeStampDataST[0])) );
//   Gx = imuVal16( (uint16_t)((((uint16_t)(allIMUDataST[1]))<<8) + ((uint16_t)(allIMUDataST[0]))), 1000.0F);
//   Gy = imuVal16( (uint16_t)((((uint16_t)(allIMUDataST[3]))<<8) + ((uint16_t)(allIMUDataST[2]))), 1000.0F);
//   Gz = imuVal16( (uint16_t)((((uint16_t)(allIMUDataST[5]))<<8) + ((uint16_t)(allIMUDataST[4]))), 1000.0F);
//   Ax = imuVal16( (uint16_t)((((uint16_t)(allIMUDataST[7]))<<8) + ((uint16_t)(allIMUDataST[6]))), 2.0F);
//   Ay = imuVal16( (uint16_t)((((uint16_t)(allIMUDataST[9]))<<8) + ((uint16_t)(allIMUDataST[8]))), 2.0F);
//   Az = imuVal16( (uint16_t)((((uint16_t)(allIMUDataST[11]))<<8) + ((uint16_t)(allIMUDataST[10]))), 2.0F);
float imuVal16(uint16_t twosVal,float dataRange){
	float dataVal;
	if(twosVal & 0x8000){
		// negative value
		dataVal = ((float)(~((uint16_t)(twosVal-1)))*(-1*dataRange))/(32768);
	} else {
		// positive value
		dataVal = (twosVal*dataRange)/32767;
	}

	return dataVal;
}
