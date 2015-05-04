/**
 * servo_pen.c
 * Used to control the two servo motors carrying the pen or writing utensil in the signal visualization system.
 */

#include "msp430g2553.h"

// Port 2 interrupt constants for controlling starting and stopping servos.
#define BUTTON BIT1  // Button port 2
#define ON 1         // value of state variable when data is being collected and visualized.
#define OFF 0        // value of state variable when the machine is not visualizing data inputs.

// PWM and servo constants
#define MCU_CLOCK           1100000   // SMCLK
#define PWM_FREQUENCY       46        // Frequency of PWM
#define SERVO_STEPS         180       // 180 steps for 180 degrees.
#define SERVO_MAX_STEPS     60        // Of the SERVO_STEPS, this is the number of steps that the servos will be restricted to (180 steps produces too large of a waveform)
#define SERVO_MIN           700       // The minimum duty cycle for servo motors
#define SERVO_MAX           3000      // The maximum duty cycle for servo motors
#define SERVO_PINS          BIT2+BIT4 // Ports used to control PWM to servo motors.  On port 2.

// ADC10 Constants
#define NUM_AVG 5           // Number of values to average from ADC10

unsigned int PWM_Period     = (MCU_CLOCK / PWM_FREQUENCY);  // PWM Period for servos.

// ADC10 Variables
int sensor_data = 0;      // Used to store data obtained from ADC10

// State variables
unsigned int state = OFF; // 1 if data being read and drawn.  0 to stop data visualization.  Button toggles this flag.

/**
 * collectData:
 * Input: None
 * Output: Amplitude of the NUM_AVG averaged sensor readings.  This amplitude is a value between 0 and SERVO_MAX_STEPS, which corresponds to the degree in which
 *         one of the servos motors must rotate to in order to draw the correct amplitude.  The other servo (forming the other half of the vehicle) is the complement of
 *         this value, such that if you add the degree of the servo motors up together, the value will equal SERVO_MAX_STEPS.
 */
int collectData(void) {

	// variable declaration
	int long data = 0;  // Return value of the averaged and scaled sensor readings.
	int i = 0;          // Counter.
	sensor_data = 0;    // Reset the variable used to store raw sensor data.

	// Read in NUM_AVG sensor values to average (low-pass filter).
	for (i = 0; i < NUM_AVG; i ++) {

		// Begin sampling and conversion and wait for it to finish in LPM0 mode.
		ADC10CTL0 |= ENC + ADC10SC;
		_bis_SR_register(LPM0_bits);

		// Sum the values up
		data += sensor_data;
	}

	// Divide by NUM_AVG to find average of data.
	data = (int) data * 1.0 / i;

	// There is an offset of around 400, meaning the lowest value read from the sensor is around 400 (noise and true signal).  Subtract if off here.
	data = data - 400;

	// In case the averaged value is less than 400, we make sure we do not have any negative values because servos do not take in negative degrees.
	if (data < 0) {
		data = 0;
	}

	// After subtracting away the offset, the data has a range between 0 and 400.  Here, we scale it down so that it is between 0 and SERVO_MAX_STEPS, which corresponds
	// to the degree that one of the servos must rotate to.  The other servo will turn to the angle such that the sum of both servos' angles adds up to SERVO_MAX_STEPS
	data = data / 400 * SERVO_MAX_STEPS;
	return data;
}

/**
 * Main function
 */
void main (void){

    unsigned int servo_stepval, servo_stepnow;     // Used for calculating the duty cycles for all the servo step angles.
    unsigned int servo_lut[SERVO_STEPS + 1];       // Step look up table.  servo_lut[i] refers to the duty cycle for the servo to turn to angle i degree.
    unsigned int i;                                // Counter.
    unsigned int servo_index;                      // The degree to turn the servo to

    WDTCTL  = WDTPW + WDTHOLD;  // Kill watchdog timer for now.

    // If calibration constant erased
    if (CALBC1_1MHZ==0xFF) {

    	// do not load, trap CPU!!
    	while(1);
    }

    // Set up clock and DCO.
    DCOCTL = 0;                 // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;      // Set range
    DCOCTL = CALDCO_1MHZ;       // Set DCO step + modulation

    // ADC10 initialization
    ADC10CTL0 = (SREF_1 +       // 1.5V reference voltage
    		     REFON +        // Internal ref
    		     ADC10ON +      // Enable adc10
    		     ADC10SHT_1 +   // Sample every 8 clock cycles
    		     ADC10IE);      // Enable ADC10 interrupts.

    ADC10CTL1 = (INCH_0 +                   // Read in values from sensor
    		     ADC10SSEL0 + ADC10SSEL1 +  // Source from SMCLK.
    		     CONSEQ_0);                 // single channel mode.

    // Calculate the step value and define the current step, defaults to minimum.
    servo_stepval   = ((SERVO_MAX - SERVO_MIN) / SERVO_STEPS);  // the difference in duty cycle between each step or degree in a servo motor.
    servo_stepnow   = SERVO_MIN;                                // The duty cycle of the very first step or degree.

    // Fill up the look up table.  Eliminates any calculations later on when determining what duty cycle to use to turn the motor to the specified angle.
    for (i = 0; i < SERVO_STEPS; i++) {
        servo_stepnow += servo_stepval;   // Increment the duty cycle by the servo_stepval calculated previously to find the duty cycle for the next rotation degree.
        servo_lut[i] = servo_stepnow;     // Then store this value for faster lookups later.
    }

    // Setup the PWM for the servos.
    TA1CCTL1 = OUTMOD_7;            // TA1CCR1 reset/set.  Corresponds to the first servo.
    TA1CCTL2 = OUTMOD_7;            // TA1CCR2 reset/set.  Corresponds to the second servo.
    TA1CTL   = TASSEL_2 + MC_1;     // SMCLK, upmode
    TA1CCR0  = PWM_Period-1;        // PWM Period
    TA1CCR1  = 0;                   // Keep off in the beginning
    TA1CCR2  = 0;                   // Keep off in the beginning
    P2DIR   |= BIT2 + BIT4;         // PWM setup for servo control pins.
    P2SEL   |= BIT2 + BIT4;         // PWM setup for servo control pins.

	// Set up on/off button.
	P2DIR &= ~(BUTTON); // Enable buttons as input.
	P2REN |= BUTTON;    // Enable internal pull-up/down resistors for buttons.
	P2OUT |= BUTTON;    // Select pull-up mode for buttons (default state is high).
	P2IES |= BUTTON;    // Interrupt enable on falling edge.

    // Enable watchdog timer and global interrupts
    WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL;     // Set WDT as an interval timer for every 32767 clock cycles (sample at 30 Hz)
    IE1 |= WDTIE;                             // Enable  WDT interrupts
    _bis_SR_register(GIE);                    // Enable global interrupts.

    // Main loop
    while(1) {
    	_bis_SR_register(LPM0_bits);                            // Enter LPM0 mode and wait for the next sample.
    	servo_index = collectData();                            // Obtain next reading.
    	TA1CCR1 = servo_lut[servo_index];                       // The reading obtained corresponds to the angle that one of the servos should be in.
    	TA1CCR2 = servo_lut[SERVO_MAX_STEPS - 1 - servo_index]; // The other servo should be at an angle that is complementary to the first servo in order for the vehicular system to move in one direction.
    }
}

// Button is pushed to change state.
#pragma vector=PORT2_VECTOR
__interrupt void button_interrupt(void){
	// Debouncing: disable button interrupts to wait for button to restabilize.  Buttons will be reenabled in WDT.
	P2IE &= ~BUTTON;

	// Interrupt on falling edge (button is pushed).  State flag is toggled here.
	if(P2IES & BUTTON) {

		// Toggle interrupt for debouncing.  When the button is released, another interrupt is triggered.
		P2IES &= ~BUTTON;

		// Toggle state flag.
		state ^= ON;
	}

	// Otherwise, the interrupt is on the rising edge (button is released).  Only for debouncing.
	else {

		// Toggle interrupt so that it triggers on falling edge again.  An interrupt triggered on rising edge is used only for debouncing
		// for when bounces occur after releasing the button.
		P2IES |= BUTTON;
	}
}

// Controls sampling rate from the sensor
#pragma vector = WDT_VECTOR
__interrupt void watchdog_timer(void){
	P2IE |= BUTTON;   // Re-enable button interrupts.
	P2IFG &= 0;       // clear button flags.

	// Only exit and begin sampling from sensor if the device is in "ON" mode.
	if(state) {

		// Exit from LPM3 mode to begin sampling from sensor.
		_bic_SR_register_on_exit(LPM0_bits);
	}
}

// Read value from sensor
#pragma vector = ADC10_VECTOR
__interrupt void adc10_timer(void){

	// Disable further sampling and conversion
	ADC10CTL0 &= ~(ENC + ADC10SC);

	// Read in sampled value
	sensor_data = ADC10MEM;

	// Exit from LPM0 mode to begin computations on obtained value.
	_bic_SR_register_on_exit(LPM0_bits);
}
