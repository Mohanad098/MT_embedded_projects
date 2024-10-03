/*
 * Application code
 *
 * File Name: Main.c
 *
 * Description: Application file for a fan system
 *
 * Author: Mohanad Hany
 */

#include "ADC.h"
#include "lcd.h"
#include "DC_Motor_Driver.h"
#include "lm35_sensor.h"

int main(void){

	uint8 temp;
	/*Configure ADC*/
	ADC_ConfigType ADC_Prameters={internal_ref_volt,Pre8};
	ADC_init (&ADC_Prameters);

	/*Initialize LCD*/
	LCD_init();

	/*Initialize DC motor*/
	DcMotor_Init();

	LCD_moveCursor(1,0);
	LCD_displayString("Temp =    C");


	for(;;){

		temp = LM35_getTemperature();

		if (temp<30){
			DcMotor_Rotate(Stop,0);
			LCD_moveCursor(0,0);
			LCD_displayString("Fan is OFF");
			LCD_moveCursor(1,7);
			LCD_intgerToString(temp);
		}
		else if (temp<60){
			LCD_moveCursor(0,0);
			LCD_displayString("Fan is ON ");
			DcMotor_Rotate(Cw,25);
			LCD_moveCursor(1,7);
			LCD_intgerToString(temp);
		}
		else if (temp<90){
			LCD_moveCursor(0,0);
			LCD_displayString("Fan is ON ");
			DcMotor_Rotate(Cw,50);
			LCD_moveCursor(1,7);
			LCD_intgerToString(temp);
		}
		else if (temp<120){
			LCD_moveCursor(0,0);
			LCD_displayString("Fan is ON ");
			DcMotor_Rotate(Cw,75);
			LCD_moveCursor(1,7);
			LCD_intgerToString(temp);
		}
		else if (temp>=120){
			LCD_moveCursor(0,0);
			LCD_displayString("Fan is ON ");
			DcMotor_Rotate(Cw,100);
			LCD_moveCursor(1,7);
			LCD_intgerToString(temp);
			LCD_displayCharacter(' ');
		}

	}
}
