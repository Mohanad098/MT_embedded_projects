 /******************************************************************************
 *
 * Module: Ultrasonic sensor
 *
 * File Name: Ultrasonic.h
 *
 * Description: Header file for the Ultrasonic sensor driver
 *
 * Author:Mohanad Hany
 *
 *******************************************************************************/

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_
#include "std_types.h"

void Ultrasonic_init(void);
void Ultrasonic_Trigger(void);
uint16 Ultrasonic_readDistance(void);
void Ultrasonic_edgeProcessing(void);

#endif /* ULTRASONIC_H_ */
