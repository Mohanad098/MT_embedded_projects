/******************************************************************************
 *
 * Module: Ultrasonic sensor
 *
 * File Name: Ultrasonic.c
 *
 * Description: Source file for the Ultrasonic sensor driver
 *
 * Author:Mohanad Hany
 *
 *******************************************************************************/

#include "Ultrasonic.h"
#include "std_types.h"
#include "gpio.h"
#include "icu.h"
#include <util/delay.h>

extern Icu_ConfigType *Ptr_config;
uint8 g_edgeCount = 0;
uint16 g_timeHigh = 0;


void Ultrasonic_init(void){

	/* Set the Call back function pointer in the ICU driver */
	Icu_setCallBack(Ultrasonic_edgeProcessing);
	Icu_init(Ptr_config);

	GPIO_setupPinDirection(PORTB_ID, PIN5_ID, PIN_OUTPUT);
}

void Ultrasonic_Trigger(void){
	GPIO_writePin(PORTB_ID, PIN5_ID, LOGIC_HIGH);
	_delay_us(10);
	GPIO_writePin(PORTB_ID, PIN5_ID, LOGIC_LOW);
}

uint16 Ultrasonic_readDistance(void){
	uint16 distance=0;
	Ultrasonic_Trigger();
	/* calculate the period */
	distance=(uint16)(g_timeHigh/58.8);
	return distance;
}

void Ultrasonic_edgeProcessing(void){
	g_edgeCount++;
	if(g_edgeCount == 1)
	{
		/*
		 * Clear the timer counter register to start measurements from the
		 * first detected rising edge
		 */
		Icu_clearTimerValue();
		/* Detect falling edge */
		Icu_setEdgeDetectionType(FALLING);
	}
	else if(g_edgeCount == 2)
	{
		/* Store the High time value */
		g_timeHigh = Icu_getInputCaptureValue();
		/* Detect rising edge */
		Icu_setEdgeDetectionType(RISING);
		g_edgeCount=0;
	}
}

