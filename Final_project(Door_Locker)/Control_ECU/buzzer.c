 /******************************************************************************
 *
 * Module: Buzzer
 *
 * File Name: buzzer.c
 *
 * Description: Source file for the buzzer
 *
 * Author: Mohanad Hany
 *
 *******************************************************************************/

#include "buzzer.h"
#include "gpio.h"
#include "std_types.h"

void Buzzer_init(void){
	GPIO_setupPinDirection(PORTD_ID, PIN4_ID, PIN_OUTPUT);
	GPIO_writePin(PORTD_ID, PIN4_ID, LOGIC_LOW);
}

void Buzzer_on(void){
	GPIO_writePin(PORTD_ID, PIN4_ID, LOGIC_HIGH);
}

void Buzzer_off(void){
	GPIO_writePin(PORTD_ID, PIN4_ID, LOGIC_LOW);
}
