/* press_speed_monitor.c
 *
 * Firmware for ATxmega256A3BU to capture data from string potentiometer
 * to monitor angular extrusion press speed. Data are streamed back
 * to PC via RS232 link in ASCII format.
 *
 * Date: 	2018-11-13
 * Author: 	M. Kokko
 *
 * Timer Configuration:
 * 1. TCC0: 1.00Hz   (dt = 1s) used for consistent loop/sampling period
 *
 * TODO: pull all port/pin definitions, configuration settings, and "magic numbers" out of code, pass as parameters or #define
 * TODO: add watchdog timer
 *
 */

// uncomment this line to allow Eclipse IDE to index device-specific symbols
//#include <avr/iox256a3b.h>

// set default CPU clock frequency to 16MHz (external crystal)
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// required includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include "xmega_usart.h"
#include <avr/pgmspace.h>   // needed for EEPROM access to get ADC calibration
#include <stddef.h>

// define "no-op" macro to waste a clock cycle
#ifndef NOP
#define NOP() asm("nop")
#endif

// global variables for ADC
// TODO: replace globals with another mechanism for passing data to/from ISR
#define ADC_WINDOW_SIZE 10
volatile uint16_t adc_result_vector[ADC_WINDOW_SIZE];
volatile uint16_t  *adc_result_vector_pos = adc_result_vector;
volatile uint8_t adc_result_vector_full = 0;
volatile uint8_t adc_result_vector_lock = 0;

// interrupt service routine prototype(s)
ISR(ADCA_CH0_vect);

// function prototypes
uint8_t ReadCalibrationByte( uint8_t index );

// simple main function to read from string pot and compute speed
// TODO: replace all "magic numbers", hard-coded ports, pins, registers, and values with #define statements
int main(void) {

	// declare variables for use in main() scope
	volatile uint16_t  *adc_vector_read_pointer;
	uint8_t loopcount = 0;
	uint32_t adc_sum;
	uint16_t pos_prev = 0, pos_cur = 0;
	float speed;

	// change to 16MHz external oscillator
	NOP(); NOP();NOP(); NOP();  				// wait a few clock cycles to be safe
	OSC.XOSCCTRL = 0xDB; 						// OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc; // select frequency of external oscillator
	OSC.CTRL = OSC_XOSCEN_bm;  					// turn on external oscillator
	while(!(OSC.STATUS & OSC_XOSCRDY_bm)){}; 	// wait for oscillator to stabilize
	CCP = CCP_IOREG_gc;  						// write correct signature (0xD8) to change protection register first
	CLK.CTRL = CLK_SCLKSEL_XOSC_gc;  			// use external clock source (0x03); 0x01 selects 32MHz internal RC oscillator

	// configure TCC0 to cycle at 1.00Hz
	TCC0.CTRLA    = 0x00 | TC_CLKSEL_DIV256_gc; // prescaler = 256; 16MHz/256 = 62.5kHz -> 16us/tick
	TCC0.CTRLB    = 0x00 | TC_WGMODE_NORMAL_gc; // normal operation (expire at PER)
	TCC0.CTRLC    = 0x00;
	TCC0.CTRLD    = 0x00;
	TCC0.CTRLE    = 0x00;
	TCC0.INTCTRLA = 0x00 | TC_OVFINTLVL_LO_gc; 	// enable timer overflow interrupt
	TCC0.INTCTRLB = 0x00;
	TCC0.PER      = 62500;     				    // 62500 = 1.00s (1.00Hz) with 16MHz clock and prescaler = 256

	// initialize USART, configure for STDOUT, and send ASCII boot message
	initStdOutUSART();
	printf("Press Speed Monitor\r\n");

	// Initialize ADCA
	PORTA_DIR = 0x00;  // configure ADC on PORTA to be all input
	PR.PRPA   = 0x05; // Clear ADC bit in Power Reduction Register, do not send clock to DAC or AC, write n/c bits to 0 per datasheet
	ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
	ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );
	ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
	ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );

	ADCA.CTRLB      =  (0x00 | 0x80 | 0x08);  // low impedance source (opamp output), freerunning mode
	ADCA.REFCTRL    =  0x20;
	ADCA.EVCTRL     =  0x00;                  // sweep channel ADC0 only
	ADCA.PRESCALER  =  0x01;		          // clk_adc = (clk_per / 8) = (16,000,000/8) = 2MHz; ADC clock should be 100kHz - 2MHz

	ADCA.CH0.CTRL    = 0x01; 	              // single ended (in unsigned mode)
	ADCA.CH0.MUXCTRL = 0x08; 	              // input on pin PA1/ADC1 (#63)
	ADCA.CH0.INTCTRL = 0x03;  		          // High-level interrupt, triggered on conversion completion
	ADCA.CTRLA       = 0x01;                  //enable ADC

	// enable interrupts
	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();

	// continually read string pot position
	// and report linear velocity
	while(1){

		// initialize pointer for copying ADC results
		// and reset ADC sum to zero
		adc_vector_read_pointer = adc_result_vector;
		adc_sum = 0;

		// wait for lock to become available
		while(adc_result_vector_lock);

		// acquire lock on ADC result vector
		adc_result_vector_lock = 1;

		// copy ADC results
		while(adc_vector_read_pointer < (adc_result_vector + ADC_WINDOW_SIZE) ){
			adc_sum += *adc_vector_read_pointer;
			++adc_vector_read_pointer;
		}

		// release lock on ADC result vector
		adc_result_vector_lock = 0;

		// compute speed
		// TODO: magic numbers!
		pos_cur = (uint16_t)(adc_sum/((uint32_t)ADC_WINDOW_SIZE)); // note: integer division
		speed = (( ((float)pos_cur) - ((float)pos_prev) ) *((float)5.9))/( ((float)3873) );  // [in/sec]
		pos_prev = pos_cur;

		// display results intermittently
		// TODO: fix magic number!
		if(loopcount == 0){
			printf("ADC Counts: %04u     Speed: %+06.3f\r",pos_prev, speed);
			loopcount = 0;
		} else{
			++loopcount;
		}

		// force loop timing with TCC0
		// TODO: throw error if this timer has already expired when we get here
		// potential source of error in computed speed
		while(!(TCC0.INTFLAGS & TC0_OVFIF_bm));
		TCC0.INTFLAGS |= TC0_OVFIF_bm;

	}
}

// ADC interrupt vector
// note: interrupt flag bit automatically cleared on execution
ISR(ADCA_CH0_vect){

	// only read ADC if we can take the lock
	if( !adc_result_vector_lock ){

		// acquire lock
		adc_result_vector_lock = 1;

		// store result in ADC array
		*adc_result_vector_pos = ADCA.CH0.RES;

		// increment pointer
		++adc_result_vector_pos;
		if(adc_result_vector_pos >= (adc_result_vector + ADC_WINDOW_SIZE)){
			adc_result_vector_pos = adc_result_vector;
			adc_result_vector_full = 1;
		}

		// release lock
		adc_result_vector_lock = 0;

	}
}

// function to get ADC calibration from non-volatile memory
// directly from: https://www.avrfreaks.net/forum/xmega-production-signature-row
uint8_t ReadCalibrationByte( uint8_t index ){
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return( result );
}
