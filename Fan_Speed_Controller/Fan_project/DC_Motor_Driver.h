/*
 * Module: DC Motor
 *
 * File Name: DC_Motor_Driver.h
 *
 * Description: header file for DC Motor driver
 *
 * Author: Mohanad Hany
 */

#ifndef DC_MOTOR_DRIVER_H_
#define DC_MOTOR_DRIVER_H_
#include "std_types.h"

/*
 * Declaring DC motor states
 * Stop=0x00, Anti clock wise =0x01, clock wise=0x02
 */
typedef enum{
	Stop, A_Cw, Cw
}DcMotor_State;

 void DcMotor_Init(void);
 void DcMotor_Rotate(DcMotor_State state,uint8 speed);

#endif /* DC_MOTOR_DRIVER_H_ */
