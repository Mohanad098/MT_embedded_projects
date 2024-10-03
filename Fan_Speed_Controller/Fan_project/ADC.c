/*
 * Module: ADC
 *
 * File Name: ADC.c
 *
 * Description: source file for ADC driver
 *
 * Author: Mohanad Hany
 */

#include "ADC.h"
#include "common_macros.h"
#include <avr/io.h>
#include "gpio.h"

//////*Initialize the ADC*///////
void ADC_init (const ADC_ConfigType * Config_Ptr){

	/*Configure the ADC parameters*/
	/* If you want to use AVCC, put *Config_Ptr->ref_volt=AVCC in main
	 * If you want to use internal ref volt, put *Config_Ptr->ref_volt=internal_ref_volt in main
	 */

	ADMUX|=Config_Ptr->ref_volt;

	/* channel 0 */

	SET_BIT(ADCSRA,ADEN);
	CLEAR_BIT(ADCSRA,ADIF);

	/* Adjusting prescaler */

	ADCSRA|=Config_Ptr->prescaler;

}

uint16 ADC_readChannel (uint8 ch_num){

	// insert ch num
	ch_num&= 0x07; // input from 0 to 7
	ADMUX&= 0XE0; // 0 first 5 places in MUX
	ADMUX|= ch_num;

	SET_BIT(ADCSRA,ADSC); // start conversion
	while (BIT_IS_CLEAR(ADCSRA,ADIF)); //polling
	SET_BIT(ADCSRA,ADIF); //clear the flag


	return ADC;


}
