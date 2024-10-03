#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void INT0_Init (void){

	/*
	Configure External Interrupt INT0 with falling edge. Connect a push button with the
	internal pull-up resistor. If a falling edge detected the Stop Watch time should be
	reset
	 */
	DDRD &=~(1<<PD2); //input
	PORTD |=(1<<2); // internal pull up

	SREG|=(1<<7);
	MCUCR|=(1<<ISC01);
	GICR|=(1<<INT0);

}

void INT1_Init(void){

	/*
	 Configure External Interrupt INT1 with raising edge. Connect a push button with the
	external pull-down resistor. If a raising edge detected the Stop Watch time should be
	paused.
	 */
	DDRD &=~(1<<PD3);  //input , external pull down

	SREG|=(1<<7);
	MCUCR|=(1<<ISC11)|(1<<ISC10);
	GICR|=(1<<INT1);

}

void INT2_Init(void){

	/*
	 Configure External Interrupt INT2 with falling edge. Connect a push button with the
	internal pull-up resistor. If a falling edge detected the Stop Watch time should be
	resumed.
	 */
	DDRB &=~(1<<PB2);  //input
	PORTB |=(1<<2);    // internal pull up

	SREG|=(1<<7);
	MCUCSR &=~(1<<ISC2);   //for falling edge
	GICR|=(1<<INT2);

}

void Timer1_Init (void){

	TCNT1=0;
	TCCR1A=0;

	/*
	 1) CTC operation at OCR1A , WGM12=1
	 2) Prescaler 1024, 1MHZ/1024= 976.5 times to reach 1 sec
	 CS12=1, CS10=1
	 */

	TCCR1B=(1<<WGM12)|(1<<CS12)|(1<<CS10);
	OCR1A=976;
	SREG |=(1<<7);
	TIMSK= (1<<OCIE1A);

}

