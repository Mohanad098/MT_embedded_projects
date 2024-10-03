/*
 * Module: PWM
 *
 * File Name: PWM_Driver.h
 *
 * Description: header file for PWM driver
 *
 * Author: Mohanad Hany
 */

#include "PWM_Driver.h"
#include <avr/io.h>
#include "gpio.h"

void PWM_Timer0_Start(uint8 duty_cycle){

	/*Set Timer Initial Value to 0*/
	TCNT0 = 0;

	/*Get compare value from user*/
	OCR0  = duty_cycle;

	/*Set the pwm timer0 pin, which is pin 3 in port B to output*/
	GPIO_setupPinDirection(PORTB_ID, PIN3_ID, PIN_OUTPUT);

	/* Configure timer control register
	 * 1. Fast PWM mode FOC0=0
	 * 2. Fast PWM Mode WGM01=1 & WGM00=1
	 * 3. Clear OC0 when match occurs (non inverted mode) COM00=0 & COM01=1
	 * 4. clock = F_CPU/8 CS00=0 CS01=1 CS02=0
	 */
	TCCR0 = (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS01);

	/* Fpwm= Fcpu/256*N=(1000000)/256*8=500Hz*/

}



