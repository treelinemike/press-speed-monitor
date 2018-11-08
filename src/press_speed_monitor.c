/* imu-eval-main.c
 *
 * Firmware for ATxmega256A3BU to capture data from gyro/accel
 * IMU chip(s) and stream back to PC via RS232 link. Communicates
 * with ST #LSM6DS3 using SPI bus and accepts optical encoder input
 * via LS7166 quadrature counting interface for gyro calibration.
 *
 * Date: 	2016-06-22
 * Author: 	M. Kokko
 *
 * Code Flow:
 * 1. Configure peripherals and sensor interfaces
 * 2. Infinite loop:
 * 	  2A: If new IMU data available, read and pass to PC as a packet including an MCU-derived timestamp
 * 	  2B: If in "calibration" mode, send another packet of encoder position data with an MCU-derived timestamp
 * 	  2C: Send "configuration" packet to PC every so often with IMU and MCU configuration parameters for record keeping
 *
 * Timer Configuration:
 * 1. TCC0: 100Hz   (dt = 10ms) used for consistent loop/sampling period
 * 2. TCC1: 62.5kHz (dt = 16us); 16-bit counter overflows in 0.000016*65535 = 1.04856s
 *          used for timestamping data read from sensors (IMU and encoder), interrupts
 *          on overflow/reset
 *
 * TODO: pull all port/pin definitions, configuration settings, and "magic numbers" out of code, pass as parameters or #define
 * TODO: add watchdog timer
 *
 */

// set default CPU clock frequency to 16MHz (external crystal)
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// calibration mode control
// 0: normal operating mode, encoder not read
// 1: calibration mode, encoder data captured and transmitted
#define CALIBRATION_MODE 1

// required includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include "xmega_usart.h"
#include "dbec_imu_eval.h"

#if(CALIBRATION_MODE)
#include "ls7166.h"
#endif

// define "no-op" macro to waste a clock cycle
#ifndef NOP
#define NOP() asm("nop")
#endif

// global variables for timer state
volatile uint8_t tcc1IntState = 0;
volatile uint8_t tcc1IntCount = 0;
volatile uint8_t configMsgDue = 0;

// interrupt service routine prototype(s)
ISR(TCC1_OVF_vect);

// simple main function to configure IMUs and stream data via USART
// TODO: replace all "magic numbers", hard-coded ports, pins, registers, and values with #define statements
int main(void) {

	// declare variables for use in main() scope
	uint8_t allIMUDataST[12];
	uint8_t *pAllIMUDataST;
	uint8_t timeStampDataST[3];
	uint8_t *pTimeStampDataST;
	uint8_t configDataST[5];
	uint8_t *pConfigDataST;
	uint8_t readByte;
	uint8_t thisRegAddr = 0x00;
	uint16_t thisMicroTime;

#if(CALIBRATION_MODE)
	uint32_t encoderCount;
#endif

	// Step 1: Configuration and sensor initialization

	// change to 16MHz external oscillator
	NOP(); NOP();NOP(); NOP();  				// wait a few clock cycles to be safe
	//PORTR.DIR = 0xFF; 						// set port R for output to drive crystal (not sure if necessary)
	OSC.XOSCCTRL = 0xDB; 						// OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc; // select frequency of external oscillator
	OSC.CTRL = OSC_XOSCEN_bm;  					// turn on external oscillator
	while(!(OSC.STATUS & OSC_XOSCRDY_bm)){}; 	// wait for oscillator to stabilize
	CCP = CCP_IOREG_gc;  						// write correct signature (0xD8) to change protection register first
	CLK.CTRL = CLK_SCLKSEL_XOSC_gc;  			// use external clock source (0x03); 0x01 selects 32MHz internal RC oscillator

	// configure TCC0 to cycle at 100Hz
	TCC0.CTRLA = 0x00 | TC_CLKSEL_DIV4_gc;   	// prescaler = 4; 16MHz/4 = 4MHz -> 250ns/tick
	TCC0.CTRLB = 0x00 | TC_WGMODE_NORMAL_gc; 	// normal operation (expire at PER)
	TCC0.CTRLC = 0x00;
	TCC0.CTRLD = 0x00;
	TCC0.CTRLE = 0x00;
	TCC0.INTCTRLA = 0x00 | TC_OVFINTLVL_LO_gc; 	// enable timer overflow interrupt
	TCC0.INTCTRLB = 0x00;
	TCC0.PER = 40000;     						// 40000 = 0.01s (100Hz) with 16MHz clock and prescaler = 4

	// configure PC0 for toggling by counter if desired (manually, not with hardware output compare - done in ISR)
	PORTC.DIR |= PIN0_bm;
	PORTC.OUT &= ~PIN0_bm;

	// configure TCC1 to increment at 16us for comparison to IMU timestamps
	TCC1.CTRLA = 0x00 | TC_CLKSEL_DIV256_gc; 	// prescaler = 256, 16us per count
	TCC1.CTRLB = 0x00 | TC_WGMODE_NORMAL_gc; 	// normal operation (expire at PER)
	TCC1.CTRLC = 0x00;
	TCC1.CTRLD = 0x00;
	TCC1.CTRLE = 0x00;
	TCC1.INTCTRLA = 0x00 | TC_OVFINTLVL_HI_gc; 	// enable timer overflow interrupt
	TCC1.INTCTRLB = 0x00;
	TCC1.PER = 65535;							// let counter overflow at 2^16 (0.000016s*65535 = 1.05s between overflows)
	//TCC1.PER = 0;								// for debugging, set TCC1.PER = 0 and scope PC0

	// enable interrupts
	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();

	// initialize USART, configure for STDOUT, and send ASCII boot message
	initStdOutUSART();
	printf("IMU Evaluation\r\n");

	// initialize ST IMU
	initST();

	// initialize quadrature counter for encoder if in calibration mode
#if(CALIBRATION_MODE)
	LS7166Init();
#endif

	// Step 2: collect and transmit data forever
	while(1){

		// Step 2A: if new data available, read from IMU and pass to PC along with timestamp from TCC1
		readST(0x1E,&readByte);
		if(1 && ((readByte & 0x02) || (readByte & 0x01))){

			// grab a timestamp from the microcontroller TCC1 counter
			// this timestamp is as close as we can get to IMU device data capture time
			// TODO: Block internal IMU update before read & unblock after read to ensure data is very close to TCC1 timestamp
			thisMicroTime = TCC1.CNT;

			// read timestamp from IMU device
			// TODO: pull out register addresses as #defines
			pTimeStampDataST = timeStampDataST;
			for(thisRegAddr = 0x40; thisRegAddr <= 0x42; thisRegAddr++){
				readST(thisRegAddr,pTimeStampDataST);
				++pTimeStampDataST;
			}

			// read IMU data
			// TODO: pull out register addresses as #defines
			pAllIMUDataST = allIMUDataST;
			for(thisRegAddr=0x22; thisRegAddr <= 0x2D; thisRegAddr++){
				readST(thisRegAddr,pAllIMUDataST);
				++pAllIMUDataST;
			}

			// send IMU data packet to PC over RS232 link
			sendIMUSerialPacket(PKT_TYPE_ST_DATA, thisMicroTime, timeStampDataST, 3, allIMUDataST, 12);
		}

		// Step 2B: if in calibration mode, read encoder and transmit packet with position data
#if(CALIBRATION_MODE)

		// grab a timestamp from the microcontroller TCC1 counter
		thisMicroTime = TCC1.CNT;

		// read encoder count from output latch of LS7166
		LS7166ReadOL(&encoderCount);
		//printf("Encoder Count: %09lu\r\n",encoderCount);

		// send encoder data via serial port
		sendEncoderSerialPacket(PKT_TYPE_ENCODER_DATA, thisMicroTime, encoderCount);

#endif

		// Step 2C: send a configuration message if one is due
		// message timing based on TCC1 (see ISR)
		if(configMsgDue){

			// grab a timestamp from the microcontroller TCC1 counter
			thisMicroTime = TCC1.CNT;

			// read timestamp from IMU device
			// TODO: pull out register addresses as #defines
			pTimeStampDataST = timeStampDataST;
			for(thisRegAddr = 0x40; thisRegAddr <= 0x42; thisRegAddr++){
				readST(thisRegAddr,pTimeStampDataST);
				++pTimeStampDataST;
			}

			// read device configuration from appropriate registers
			// TODO: pull out register addresses as #defines
			// TODO: make this more elegant
			pConfigDataST = configDataST;
			readST(0x10,pConfigDataST); 	// REG_VAL_A = accelerometer ODR & range
			++pConfigDataST;
			*pConfigDataST = 0x00;			// REG_VAL_B = 0x00
			++pConfigDataST;
			readST(0x11,pConfigDataST); 	// REG_VAL_C = gyro ODR & range
			++pConfigDataST;
			*pConfigDataST = 0x00;			// REG_VAL_D = 0x00
			++pConfigDataST;
			readST(0x5C,pConfigDataST); 	// REG_VAL_E = timestamp resolution

			// send configuration message
			sendIMUSerialPacket(PKT_TYPE_ST_CONFIG, thisMicroTime, timeStampDataST, 3, configDataST, 5);

			// reset configuration message due flag
			configMsgDue = 0;
		}

		// set output pin high at start of idle period
		PORTC.OUT |= PIN0_bm;

		// force loop timing with TCC0
		// TODO: throw error if this timer has already expired when we get here; for now we can check this using timestamps in transmitted data
		while(!(TCC0.INTFLAGS & TC0_OVFIF_bm));
		TCC0.INTFLAGS |= TC0_OVFIF_bm;

		// clear output pin at end of idle period
		PORTC.OUT &= ~PIN0_bm;

	} // end while(1)

	// done, but program will never get here
	return 0;
}

// interrupt service routine for TCC1 timer
// keeps track of when configuration messages
// need to be sent and toggles an output pin
// for debug purposes
// note: TC1_OVFIF bit in TCC1.INTFLAGS is
// automatically cleard upon ISR execution
ISR(TCC1_OVF_vect){

	// check to see whether we need to send a configuration message
	++tcc1IntCount;
	if(tcc1IntCount >= CONFIG_MSG_DT){
		tcc1IntCount = 0;
		configMsgDue = 1;
	}

	// toggle output pin for debugging purposes
	/*
	if(tcc1IntState){
		PORTC.OUT &= ~PIN0_bm;
		tcc1IntState = 0;
	} else {
		PORTC.OUT |= PIN0_bm;
		tcc1IntState = 1;
	}
	 */
}
