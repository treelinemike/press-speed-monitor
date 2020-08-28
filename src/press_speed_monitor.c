/* press_speed_monitor.c
 *
 * Firmware for ATxmega256A3BU to capture data from string potentiometer
 * to monitor angular extrusion press speed. Data are streamed back
 * to PC via RS232 link in ASCII format.
 *
 * Date: 	2018-11-13
 * Updated: 2020-08-27
 * Author: 	M. Kokko
 *
 * Timer Configuration:
 * 1. TCC0: 10kHz   (dt = 100us) used to start A/D conversions
 * 2. TCC1: 0.33Hz   (dt = 3s) used for consistent loop/sampling period
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

// DEBUG_MODE toggles PA2 on each speed computation
// allowing evaluation of timing with scope
#define DEBUG_MODE 0

// SPEED SCALING CONSTANT [count*sec/in]
// string pot was measured at 19.181" extension from 260 to 4095 (delta = 3835 counts) on 29-NOV-18
// (3835 counts * 3sec)/(19.181 in) = 599.81 count*sec/in -> ASSUMING SPEED CALCULATED AT 3sec INTERVALS
// assumes good linearity
// TODO: remove magic numbers!

#define STRING_POT_SPEED_CONST 599.81   // COMPUTED AS ABOVE

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

// moving average window size
// want this to equate to 200ms s.t. window covers 12 full cycles of 60Hz and 10 full cycles of 50Hz
#define ADC_WINDOW_SIZE 2000

// global variables for ADC
// TODO: replace globals with another mechanism for passing data to/from ISR
volatile uint16_t adc_result_vector[ADC_WINDOW_SIZE];
volatile uint16_t  *adc_result_vector_pos = adc_result_vector;
volatile uint8_t adc_result_vector_full = 0;
volatile uint32_t adc_sum = 0;

#if(DEBUG_MODE)
	volatile uint8_t led_flag = 0;
#endif

// interrupt service routine prototype(s)
ISR(ADCA_CH0_vect);
ISR(TCC0_OVF_vect);

// function prototypes
uint8_t ReadCalibrationByte( uint8_t index );  // for reading factory ADC calibration bits, unfortunately zeroed-out on ATxmega256A3BU

// simple main function to read from string pot and compute speed
// TODO: replace all "magic numbers", hard-coded ports, pins, registers, and values with #define statements
int main(void) {

	// declare variables for use in main() scope
	uint8_t loopcount = 0;
	uint8_t temp = 0;
	uint16_t pos_prev = 0, pos_cur = 0;
	int32_t delta_pos = 0;
	float speed;

	/*********** CHANGE TO 16MHz EXTERNAL OSCILLATOR ***********/
	NOP(); NOP();NOP(); NOP();  				// wait a few clock cycles to be safe
	OSC.XOSCCTRL = 0xDB; 						// OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc; // select frequency of external oscillator
	OSC.CTRL = OSC_XOSCEN_bm;  					// turn on external oscillator
	while(!(OSC.STATUS & OSC_XOSCRDY_bm)){}; 	// wait for oscillator to stabilize
	CCP = CCP_IOREG_gc;  						// write correct signature (0xD8) to change protection register first
	CLK.CTRL = CLK_SCLKSEL_XOSC_gc;  			// use external clock source (0x03); 0x01 selects 32MHz internal RC oscillator

	/*********** CONFIGURE TCC0 TO SAMPLE THE ADC IN ONE-SHOT (NOT FREERUNNING) CONFIGURATION ***********/
	TCC0.CTRLA    = 0x00 | TC_CLKSEL_DIV8_gc; // prescaler = 8; 16MHz/8= 2MHz -> 64us/tick
	TCC0.CTRLB    = 0x00 | TC_WGMODE_NORMAL_gc; // normal operation (expire at PER)
	TCC0.CTRLC    = 0x00;
	TCC0.CTRLD    = 0x00;
	TCC0.CTRLE    = 0x00;
	TCC0.INTCTRLA = 0x00 | TC_OVFINTLVL_HI_gc; 	// enable timer overflow interrupt, seems we need a different level than for TCC1 to make them both work?
	TCC0.INTCTRLB = 0x00;
	TCC0.PER      = 200;     				    // 200 = 10kHz -> 100us with 16MHz clock at prescaler = 8

	/*********** CONFIGURE TCC1 TO AVERAGE AND REPORT READINGS AT LOWER FREQUENCY ***********/
	TCC1.CTRLA    = 0x00 | TC_CLKSEL_DIV1024_gc; // prescaler = 8; 16MHz/8= 2MHz -> 64us/tick
	TCC1.CTRLB    = 0x00 | TC_WGMODE_NORMAL_gc; // normal operation (expire at PER)
	TCC1.CTRLC    = 0x00;
	TCC1.CTRLD    = 0x00;
	TCC1.CTRLE    = 0x00;
	TCC1.INTCTRLA = 0x00 | TC_OVFINTLVL_LO_gc; 	// enable timer overflow interrupt
	TCC1.INTCTRLB = 0x00;
//	TCC1.PER      = 15625;     				    // 15625 = 1Hz -> 1s with 16MHz clock at prescaler = 1024 ... DON'T FORGET TO CHANGE SPEED CONSTANT
	TCC1.PER      = 46875;                      // 46875 = 0.33Hz -> 3s with 16MHz clock at prescaler = 1024 ... DON'T FORGET TO CHANGE SPEED CONSTANT

	/*********** INITIALIZE USART, CONFIGURE FOR STDOUT, AND SEND ASCII BOOT MESSAGE ***********/
	initStdOutUSART();
	printf("Press Speed Monitor\r\nUsing speed constant %6.2f count*sec/in\r\n",((float)STRING_POT_SPEED_CONST));

	/*********** INITIALIZE ADC ***********/
	PORTA_DIR = 0x00;  // configure ADC on PORTA to be all input
	PR.PRPA   = 0x05; // Clear ADC bit in Power Reduction Register, do not send clock to DAC or AC, write n/c bits to 0 per datasheet

	// load calibration bytes
	// note: first read needs to be discarded, so do it twice...
	// BUT! atxmega256a3bu has 0x00 for both calibration values! https://www.avrfreaks.net/forum/signature-row-reading-issue

	/* to test that we're reading production row...
	temp = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, LOTNUM4) );
	temp = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, LOTNUM4) );
	printf("LOTNUM4 = 0x%02X\r\n",temp);
	 */

	//printf("starting cal:     L = 0x%02X, H = 0x%02X\r\n",ADCA.CALL,ADCA.CALH);
	ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
	ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );
	//printf("after first read: L = 0x%02X, H = 0x%02X\r\n",ADCA.CALL,ADCA.CALH);
	ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
	ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );
	//printf("after next read:  L = 0x%02X, H = 0x%02X\r\n",ADCA.CALL,ADCA.CALH);

	ADCA.CTRLB      =  (0x00 | 0x80 | 0x00	);  // low impedance source (opamp output), freerunning mode
	ADCA.REFCTRL    =  0x10;
	ADCA.EVCTRL     =  0x00;                  // sweep channel ADC0 only
	ADCA.PRESCALER  =  0x01;		          // clk_adc = (clk_per / 8) = (16,000,000/8) = 2MHz; ADC clock should be 100kHz - 2MHz
	ADCA.CH0.CTRL    = 0x01; 	              // single ended (in unsigned mode)
	ADCA.CH0.MUXCTRL = 0x08; 	              // input on pin PA1/ADC1 (#63)
	ADCA.CH0.INTCTRL = 0x03;  		          // High-level interrupt, triggered on conversion completion
	ADCA.CTRLA       = 0x01;                  //enable ADC

	/*********** ENABLE INTERRUPTS ***********/
	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();

	/*********** ENABLE DEBUG I/O OUTPUT ***********/
#if(DEBUG_MODE)
	// configure and clear debug pin
	PORTA_DIR |= 0x04;
	PORTA.OUT &= ~0x04;
	led_flag = 0;
#endif

	/*********** LOOP FOREVER TO CONTINUALLY MEASURE POSITION AND REPORT SPEED ***********/
	while(1){

		// wait until we've collected enough data to average
		// this should only delay on the first run through the main loop
		while(!adc_result_vector_full){
			NOP(); // don't optimize loop away!
		}

		// compute speed
		pos_cur = (uint16_t)(adc_sum/((uint32_t)ADC_WINDOW_SIZE)); // note: integer division
		delta_pos = ((int32_t)pos_cur) - ((int32_t)pos_prev);      // in ADC counts
		speed = ((float)delta_pos) / ((float)STRING_POT_SPEED_CONST);  // [in/sec]
		pos_prev = pos_cur;

		// display speed
		printf("ADC value: %10u; Speed [in/s]: %+08.5f\r",pos_cur,speed);

		// force loop timing with TCC1
		// TODO: throw error if this timer has already expired when we get here
		// potential source of error in computed speed
		while(!(TCC1.INTFLAGS & TC1_OVFIF_bm));
		TCC1.INTFLAGS |= TC1_OVFIF_bm;

	}
}

// start an A/D conversion when TCC0 expires
ISR(TCC0_OVF_vect){
#if(DEBUG_MODE)
	PORTA.OUT |= 0x04;
	led_flag = 1;
#endif

	// start a conversion
	ADCA.CTRLA  |= 0x04;

	// clear interrupt flag (this may happen automatically)
	TCC0.INTFLAGS |= TC0_OVFIF_bm;
}

// ADC interrupt vector
// note: interrupt flag bit automatically cleared on execution
ISR(ADCA_CH0_vect){

	// storage for ADC value
	uint16_t thisADCVal;

	// read ADC
	thisADCVal = ADCA.CH0.RES;

	// adjust ADC sum
	if(adc_result_vector_full){
		adc_sum -= *adc_result_vector_pos;
	}
	adc_sum += thisADCVal;
	//adc_sum = thisADCVal;

	// store result in ADC array
	*adc_result_vector_pos = thisADCVal;

	// increment pointer, resetting if necessary
	if(adc_result_vector_pos < (adc_result_vector + ADC_WINDOW_SIZE -1)){
		++adc_result_vector_pos;
	} else {
		adc_result_vector_pos = adc_result_vector;
		adc_result_vector_full = 1;  // set flag high once we've filled the vector for the first time
	}

#if(DEBUG_MODE)
	PORTA.OUT &= ~0x04;
	led_flag = 0;
#endif
}

// function to get ADC calibration from non-volatile memory
// directly from: https://www.avrfreaks.net/forum/xmega-production-signature-row
// NOTE: for the ATxmega256A3BU the calibration bytes are both 0x00 which isn't so useful
uint8_t ReadCalibrationByte( uint8_t index ){
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return( result );
}
