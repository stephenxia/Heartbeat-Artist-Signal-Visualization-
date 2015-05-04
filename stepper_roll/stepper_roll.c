/**
 * stepper_roll.c
 * Controls the stepper motor used to roll and move the paper being drawn on.
 * Acts as the time-axis in the graphing system.
 */

#include "msp430g2553.h"

// Coil control pins.  These are on port 2.
#define COIL_1A BIT0
#define COIL_2A BIT3
#define COIL_1B BIT1
#define COIL_2B BIT4

// Used to determine which coil input pattern to energize.
int counter = 0;

/**
 * Main function
 */
void main(void) {
	WDTCTL = WDTPW + WDTHOLD;  // Turn off watchdog timer for now

	// If calibration constant erased
	if (CALBC1_16MHZ==0xFF) {

		// do not load, trap CPU!!
		while(1);
	}
	
	// Set up clock and DCO.  Run at 16 MHz to produce noticeable rotation speed.
	DCOCTL = 0;              // Select lowest DCOx and MODx settings
	BCSCTL1 = CALBC1_16MHZ;  // Set range
	DCOCTL = CALDCO_16MHZ;   // Set DCO step + modulation

	// WDT is used to control changes in input to the stepper motor.
	WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL;  // Set WDT as an interval timer for every 32767 clock cycles, producing a step rate of approximately 500 Hz.
	IE1 |= WDTIE;                          // Enable  WDT interrupts

	// Set up output pins for controlling stepper coils.
	P2DIR |= COIL_1A + COIL_2A + COIL_1B + COIL_2B;     // I/O direction of all stepper coil control pins will be output.
	P2OUT &= ~(COIL_1A + COIL_2A + COIL_1B + COIL_2B);  // Set all outputs to low (off) for now.
	_bis_SR_register(GIE);                              // Enable global interrupts to allow for WDT interrupts.
	
	// Main loop.
	while(1) {
		
		// Enter LPM0 mode and wait for next cycle.
		_bis_SR_register(LPM0_bits);

		// Determine which step and therefore, which coil pattern to create.
		if(counter == 0) {

			// First step should energize Coil 1A and Coil 2A.
			counter++;
			P2OUT |= COIL_1A + COIL_2A;
			P2OUT &= ~(COIL_1B + COIL_2B);
		}
		else if(counter == 1) {

			// Second step should energize Coil 2A and Coil 1B.
			counter++;
			P2OUT |= COIL_2A + COIL_1B;
			P2OUT &= ~(COIL_1A + COIL_2B);
		}
		else if(counter == 2) {

			// Third step should energize Coil 1B and Coil 2B.
			counter++;
			P2OUT |= COIL_1B + COIL_2B;
			P2OUT &= ~(COIL_1A + COIL_2A);
		}
		else {

			// Fourth step should energize Coil 1A and Coil 2B.
			counter = 0;
			P2OUT |= COIL_1A + COIL_2B;
			P2OUT &= ~(COIL_2A + COIL_1B);
		}
	}
}

// WDT used to wake up system to change inputs to the stepper motor coils.
#pragma vector = WDT_VECTOR
__interrupt void watchdog_timer(void){
	_bic_SR_register_on_exit(LPM0_bits);
}
