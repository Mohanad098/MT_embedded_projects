 /******************************************************************************
 *
 * Module: Buzzer
 *
 * File Name: buzzer.h
 *
 * Description: header file for the buzzer
 *
 * Author: Mohanad Hany
 *
 *******************************************************************************/

#ifndef BUZZER_H_
#define BUZZER_H_

/*************************************Description****************************************/
/*Setup the direction for the buzzer pin as output pin through the GPIO driver
 * Turn off the buzzer through the GPIO
 * Inputs: None
 * Return: None
 */

void Buzzer_init(void);

/*************************************Description****************************************/
/* Function to enable the Buzzer through the GPIO.
 * Inputs: None
 * Return: None
 */

void Buzzer_on(void);

/*************************************Description****************************************/
/* Function to disable the Buzzer through the GPIO.
 * Inputs: None
 * Return: None
 */

void Buzzer_off(void);

#endif /* BUZZER_H_ */
