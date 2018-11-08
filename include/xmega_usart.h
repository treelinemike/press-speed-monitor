/* xmega_usart.h
 *
 * Definitions and functions for controlling USART
 * peripherals on XMEGA devices.
 *
 * Date: 	2016-06-22
 * Author: 	M. Kokko
 *
 */

#ifndef XMEGA_USART_H_
#define XMEGA_USART_H_

// enable use of STDIO streams (for printf())
#define __STDIO_FDEVOPEN_COMPAT_12

// includes
#include <stdio.h>
#include <avr/io.h>

// definitions for USART interface mapped from STDOUT
#define BINARY_USART        USARTE0
#define STDOUT_USART        USARTE0
#define STDOUT_USART_PORT   PORTE

/* * * * * Function Prototypes * * * * */

// USART initialization and mapping to STDOUT/STDIN
int initStdOutUSART(void);

// low-level byte write function for the xmega USART peripheral
int usartWriteGeneric(USART_t * usartPort, char out);

// low-level byte read function for the xmega USART peripheral
int usartReadGeneric(USART_t * usartPort);

// serial write wrapper for STDOUT mapping
int usartWriteStdOut(char out);

// serial read wrapper for STDIN mapping
int usartReadStdOut(void);

#endif
