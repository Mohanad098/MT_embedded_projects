#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Prototypes.h"

unsigned int tick=0;
unsigned int secondBit=0;
unsigned int thirdBit=0;
unsigned int fourthBit=0;
unsigned int fifthBit=0;
unsigned int sixthBit=0;


ISR (TIMER1_COMPA_vect){

	tick++;

}

ISR (INT0_vect){

	// Reseting all variables of the clock
	TCNT1=0;
	tick=0;
	secondBit=0;
	thirdBit=0;
	fourthBit=0;
	fifthBit=0;
	sixthBit=0;

}

ISR (INT1_vect){

	TCCR1B&= ~(1<<CS12) & ~(1<<CS11) & ~(1<<CS10);

}

ISR (INT2_vect){

	TCCR1B|=(1<<CS12)|(1<<CS10);

}


int main(void){

	DDRC|= 0x0F;
	PORTC&= 0xF0;    // initializing first 4 bits 0
	DDRA|= 0x3F;
	PORTA|= 0x3F;    // Enabling all 7 segments

	// push buttons for interrupts at their respective functions

	Timer1_Init();  //start clock

	//call the interrupts
	INT0_Init();
	INT1_Init();
	INT2_Init();

	for(;;){

		if (tick>9)
		{
			secondBit++;
			if (secondBit>5)
			{
				thirdBit++;
				if (thirdBit>9)
				{
					fourthBit++;
					if(fourthBit>5)
					{
						fifthBit++;
						if(fifthBit>9)
						{
							sixthBit++;
							fifthBit=0;
						}
						fourthBit=0;
					}
					thirdBit=0;
				}
				secondBit=0;
			}
			tick=0;
		}

		PORTA= (PORTA & 0xC0) | 0x01;
		PORTC= (PORTC& 0xF0) | tick;
		_delay_ms(1);
		PORTA= (PORTA & 0xC0) | 0x02;
		PORTC= (PORTC& 0xF0) | secondBit;
		_delay_ms(1);
		PORTA= (PORTA & 0xC0) | 0x04;
		PORTC= (PORTC& 0xF0) | thirdBit;
		_delay_ms(1);
		PORTA= (PORTA & 0xC0) | 0x08;
		PORTC= (PORTC& 0xF0) | fourthBit;
		_delay_ms(1);
		PORTA= (PORTA & 0xC0) | 0x10;
		PORTC= (PORTC& 0xF0) | fifthBit;
		_delay_ms(1);
		PORTA= (PORTA & 0xC0) | 0x20;
		PORTC= (PORTC& 0xF0) | sixthBit;
		_delay_ms(1);


	}

}
