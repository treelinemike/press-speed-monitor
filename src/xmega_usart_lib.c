/* xmega_usart_lib.h
 *
 * Functions for controlling USART
 * peripherals on XMEGA devices.
 *
 * Date: 	2016-06-22
 * Author: 	M. Kokko
 *
 */

// includes
#include "xmega_usart.h"

// initialize USART for use as stdout (for printf/scanf)
// requires #defines for STDOUT_USART_PORT and STDOUT_USART
// TODO: make settings parameters of this function or #defines in header file, configuration is hard-coded now...
int initStdOutUSART(void){
	// RXD0 = PE2
	// TXD0 = PE3
	STDOUT_USART_PORT.OUTSET = 0x08; 							//TODO: unnecessary? remove this?
	STDOUT_USART_PORT.DIRSET = 0x08; 							// need TX line to be set for output
	STDOUT_USART.BAUDCTRLA = 0xD7; 								// 0xD7 for 115200 @ 16MHz Fosc (no CLK2X)
	STDOUT_USART.BAUDCTRLB = 0x93; 								// 0x93 for 115200 @ 16MHz Fosc (no CLK2X)
	//STDOUT_USART.BAUDCTRLA = 131;  							// 131 for 57,600 @ 16MHz Fosc (no CLK2X)
	//STDOUT_USART.BAUDCTRLB = 0xD0; 							// 0xD0 for 57,600 @ 16MHz Fosc (no CLK2X)
	//STDOUT_USART.BAUDCTRLA = 135;  							// 135 for 57,600 @ 32MHz Fosc (no CLK2X)
	//STDOUT_USART.BAUDCTRLB = 0xE0; 							// 0xE0 for 57,600 @ 32MHz Fosc (no CLK2X)
	STDOUT_USART.CTRLA = 0x00;									// no USART interrupts
	STDOUT_USART.CTRLB = 0x00 | USART_RXEN_bm | USART_TXEN_bm; 	// enable RX and TX
	STDOUT_USART.CTRLC = 0x03;									// 8-bit data size

	// map STDIN and STDOUT to this USART
	fdevopen(&usartWriteStdOut, &usartReadStdOut,0);

	// return, no error handling
	return 0;
}

// generic USART write character function
int usartWriteGeneric(USART_t * usartPort, char out){
	while (!(usartPort->STATUS & USART_DREIF_bm)){};
	usartPort->DATA = out;
	return 0;
}

// generic USART read character function
int usartReadGeneric(USART_t * usartPort){
	while (!(usartPort->STATUS & USART_RXCIF_bm)){};
	return (int)usartPort->DATA;
}

// function for writing a character to stdout (called by printf)
int usartWriteStdOut(char out){
	return usartWriteGeneric(&STDOUT_USART, out);
}

// function for reading a character from stdout (called by scanf)
int usartReadStdOut(void){
	return usartReadGeneric(&STDOUT_USART);
}
