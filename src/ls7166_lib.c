/* ls7166_lib.c
 *
 * Functions for controlling and reading
 * data from LS7166 quadrature counter.
 *
 * Date: 	2016-06-22
 * Author: 	M. Kokko
 *
 */

// includes
#include "ls7166.h"

// initialize LS7166 quadrature counter chip
// TODO: include parameters that allow various configurations; configuration is hard-coded now...
uint8_t LS7166Init(void){

	// set port directions and initialize output to zero
	LS7166_SET_DDIR_WRITE();
	LS7166_DATA_PORT.OUT = 0x00;
	LS7166_CTRL_PORT.DIR = 0x00 | LS7166_PIN_CS_bm | LS7166_PIN_CD_bm | LS7166_PIN_RD_bm | LS7166_PIN_WR_bm;
	LS7166_CTRL_PORT.OUT = 0x00 | LS7166_PIN_CS_bm | LS7166_PIN_CD_bm | LS7166_PIN_RD_bm | LS7166_PIN_WR_bm;

	// MSR: soft reset
	LS7166WriteCtrl(0x20);
	LS7166WriteCtrl(0x04);

	// QR: 4x quadrature
	LS7166WriteCtrl(0xC3);

	// OCCR: default settings, not using divide-by-n mode b/c doesn't handle over/underflow properly,
	// this is OK because we only care about velocity and can check for 24-bit wrap downstream
	LS7166WriteCtrl(0x80);

	// ICR: enable AB inputs
	LS7166WriteCtrl(0x48);

	// return, no error handling
	return 0;
}


// write byte to LS7166 control register
uint8_t LS7166WriteCtrl(uint8_t ctrlData){

	// set micro port for output
	LS7166_SET_DDIR_WRITE();

	// write data to port
	LS7166_DATA_PORT.OUT = ctrlData;

	// enable LS7166 for control write
	LS7166_CD_UP();
	LS7166_CS_DN();

	// delay
	NOP(); NOP();

	// latch data to control register
	LS7166_WR_DN();

	// delay
	NOP(); NOP();

	// complete write
	LS7166_WR_UP();
	LS7166_CD_UP();
	LS7166_CS_UP();

	// return, no error handling
	return 0;
}


// read encoder count from LS7166
uint8_t LS7166ReadOL(uint32_t *count){

	// reset count variable to zero
	*count = 0x00000000;

	// transfer count to OL
	LS7166WriteCtrl(0x03);

	// set micro port for input
	LS7166_SET_DDIR_READ();

	// enable LS7166 for read from OL
	LS7166_CD_DN();
	LS7166_CS_DN();

	// read byte0
	NOP(); NOP(); NOP(); NOP();
	LS7166_RD_DN();
	NOP(); NOP(); NOP(); NOP();
	*count |= ((uint32_t)(LS7166_DATA_PORT.IN) << 0);
	LS7166_RD_UP();

	// read byte1
	NOP(); NOP(); NOP(); NOP();
	LS7166_RD_DN();
	NOP(); NOP(); NOP(); NOP();
	*count |= ((uint32_t)(LS7166_DATA_PORT.IN) << 8);
	LS7166_RD_UP();

	// read byte2
	NOP(); NOP(); NOP(); NOP();
	LS7166_RD_DN();
	NOP(); NOP(); NOP(); NOP();
	*count |= ((uint32_t)(LS7166_DATA_PORT.IN) << 16);
	LS7166_RD_UP();

	// complete read
	LS7166_CD_UP();
	LS7166_CS_UP();

	// return, no error handling
	return 0;
}
