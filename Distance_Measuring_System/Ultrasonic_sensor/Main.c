/******************************************************************************
 *
 * Module: Application
 *
 * File Name: Main.c
 *
 * Description: Main file for using the ultrasonic sensor
 *
 * Author:Mohanad Hany
 *
 *******************************************************************************/

#include "Ultrasonic.h"
#include <avr/io.h>
#include "lcd.h"
#include "icu.h"

Icu_ConfigType *Ptr_config=0;

int main()
{
	uint16 distance=0;
	/* Create configuration structure for ICU driver */
	Icu_ConfigType Icu_Config = {F_CPU_8,RISING};


	/* Enable Global Interrupt I-Bit */
	SREG |= (1<<7);
	Ptr_config=&Icu_Config;

	/* Initialize both the LCD and ICU driver */
	Ultrasonic_init();
	LCD_init();

	//distance=Ultrasonic_readDistance();

	while(1)
	{
			//Icu_DeInit(); /* Disable ICU Driver */
			LCD_moveCursor(0, 0);
			LCD_displayString("Distance = ");
			distance=Ultrasonic_readDistance();
			/* display the period on LCD screen */
			LCD_moveCursor(0, 11);
			LCD_intgerToString(distance);
			LCD_displayString(" cm");
	}
}


/*
Icu_ConfigType *g_Icu_Configuration = 0;
void main(void){
	uint16 value =0;
	Icu_ConfigType IcuConfiguration = {F_CPU_8, RISING};
	g_Icu_Configuration = &IcuConfiguration;
	SREG |= (1<<7);
	Ultrasonic_init();
	LCD_init();


	while(1){
		LCD_moveCursor(0, 0);
	//	LCD_displayString("Distance = ");
		// display the distance on LCD screen
		value = Ultrasonic_readDistance();
		LCD_intgerToString(value);
		LCD_displayString(" cm");
	//	_delay_ms(1);
	}

}
*/
