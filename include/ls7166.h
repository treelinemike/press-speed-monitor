/* ls7166.h
 *
 * Definitions and functions for controlling and reading
 * data from LS7166 quadrature counter.
 *
 * Date: 	2016-06-22
 * Author: 	M. Kokko
 *
 */

#ifndef LS7166_H_
#define LS7166_H_

// includes
#include <avr/io.h>

// define "no-op" macro to waste a clock cycle
#ifndef NOP
#define NOP() asm("nop")
#endif

// port and pin definitions, determined by hardware connections
#define LS7166_DATA_PORT 		PORTA
#define LS7166_CTRL_PORT 		PORTB
#define LS7166_PIN_CS_bm    	PIN0_bm
#define LS7166_PIN_CD_bm    	PIN1_bm
#define LS7166_PIN_RD_bm    	PIN2_bm
#define LS7166_PIN_WR_bm   		PIN3_bm

// macros for common LS7166 operations
#define LS7166_SET_DDIR_READ()  LS7166_DATA_PORT.DIR = 0x00;
#define LS7166_SET_DDIR_WRITE() LS7166_DATA_PORT.DIR = 0xFF;
#define LS7166_CS_UP() 			LS7166_CTRL_PORT.OUT |= LS7166_PIN_CS_bm
#define LS7166_CS_DN() 			LS7166_CTRL_PORT.OUT &= ~LS7166_PIN_CS_bm
#define LS7166_CD_UP() 			LS7166_CTRL_PORT.OUT |= LS7166_PIN_CD_bm
#define LS7166_CD_DN() 			LS7166_CTRL_PORT.OUT &= ~LS7166_PIN_CD_bm
#define LS7166_RD_UP() 			LS7166_CTRL_PORT.OUT |= LS7166_PIN_RD_bm
#define LS7166_RD_DN() 			LS7166_CTRL_PORT.OUT &= ~LS7166_PIN_RD_bm
#define LS7166_WR_UP() 			LS7166_CTRL_PORT.OUT |= LS7166_PIN_WR_bm
#define LS7166_WR_DN() 			LS7166_CTRL_PORT.OUT &= ~LS7166_PIN_WR_bm

/* * * * * Function Prototypes * * * * */

// initialize LS7166 device for application
uint8_t LS7166Init(void);

// write a control byte to the LS7166
uint8_t LS7166WriteCtrl(uint8_t ctrlData);

// read the counter value on the output latch of the LS7166
uint8_t LS7166ReadOL(uint32_t *count);

#endif
