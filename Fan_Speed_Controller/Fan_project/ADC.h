/*
 * Module: ADC
 *
 * File Name: ADC.h
 *
 * Description: header file for ADC driver
 *
 * Author: Mohanad Hany
 */

#ifndef ADC_H_
#define ADC_H_
#include "std_types.h"

#define ADC_MAXIMUM_VALUE    1023
#define ADC_REF_VOLT_VALUE   5
#define ADC_Internal_Ref_Volt 2.56

/* Declaring AVCC (REFS0=1) and internal reference voltage (REFS0=1, REFS1=1) for use of any*/
typedef enum{
	AREF,AVCC=0x40,Reserved=0x80,internal_ref_volt=0xC0
}ADC_ReferenceVolatge;

/* Putting prescaler values in order*/
typedef enum{
	Pre2_1,Pre2_2,Pre4,Pre8,Pre16,Pre32,Pre64,Pre128
}ADC_Prescaler;

typedef struct{
	ADC_ReferenceVolatge ref_volt;
	ADC_Prescaler prescaler;
}ADC_ConfigType;

void ADC_init (const ADC_ConfigType * Config_Ptr);
uint16 ADC_readChannel (uint8 ch_num);




#endif /* ADC_H_ */
