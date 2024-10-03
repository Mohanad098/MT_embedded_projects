 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohanad Hany
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR
 * for example Mohanad Hany's college ID = 4080 */
#define PORT_VENDOR_ID    (4080U)

/* Port Module Id */
#define PORT_MODULE_ID    (124U)

/* Port Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (2U)
#define PORT_AR_RELEASE_MINOR_VERSION   (3U)
#define PORT_AR_RELEASE_PATCH_VERSION   (0U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* Std_Types and port driver are different versions */

/* AUTOSAR checking between Std Types and Port Modules */
/*#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif
*/

/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
   
/* Service ID for PORT Init */
#define PORT_INIT_SID                   (uint8)0x00   
   
/* Service ID for PORT Set Pin Direction */
#define PORT_SET_PIN_DIRECTION_SID      (uint8)0x01

/* Service ID for PORT Refresh Port Direction */
#define PORT_REFRESH_PORT_DIRECTION     (uint8)0x02

/* Service ID for PORT Get Version Info */
#define PORT_GET_VERSION_INFO           (uint8)0x03

/* Service ID for PORT Set Pin Mode */   
#define PORT_SET_PIN_MODE               (uint8)0x04
   
/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/

/* Det code to report Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN                        0x0A 
   
/* Det code to report Port Pin not configured as changeable*/ 
#define PORT_E_DIRECTION_UNCHANGEABLE           0x0B 

/*Det code to report API Port_Init service called with wrong parameter*/
#define PORT_E_PARAM_CONFIG                    0x0C 

/* Det code to report API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_PARAM_INVALID_MODE               0x0D 
#define PORT_E_MODE_UNCHANGEABLE                0x0E 

/* Det code to report API service called without module initialization */
#define PORT_E_UNINIT                           0x0F 

/* Det code to report APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER                    0x10 
   
/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/

/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40004000
#define GPIO_PORTB_BASE_ADDRESS           0x40005000
#define GPIO_PORTC_BASE_ADDRESS           0x40006000
#define GPIO_PORTD_BASE_ADDRESS           0x40007000
#define GPIO_PORTE_BASE_ADDRESS           0x40024000
#define GPIO_PORTF_BASE_ADDRESS           0x40025000

/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET              0x3FC
#define PORT_DIR_REG_OFFSET               0x400
#define PORT_ALT_FUNC_REG_OFFSET          0x420
#define PORT_PULL_UP_REG_OFFSET           0x510
#define PORT_PULL_DOWN_REG_OFFSET         0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET    0x51C
#define PORT_LOCK_REG_OFFSET              0x520
#define PORT_COMMIT_REG_OFFSET            0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET   0x528
#define PORT_CTL_REG_OFFSET               0x52C

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirection;

/* Description: Enum to hold PIN initial value */
typedef enum{
  PORT_PIN_LEVEL_LOW,PORT_PIN_LEVEL_HIGH
}PortPinLevelValue;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Description: Enum to hold PIN initial mode */
typedef enum
{
  PORT_PIN_MODE_ADC,
  PORT_PIN_MODE_DIO,
  PORT_PIN_MODE_UART,
  PORT_PIN_MODE_UART_MODULE1,
  PORT_PIN_MODE_SPI,
  PORT_PIN_MODE_SPI_MODULE3,
  PORT_PIN_MODE_I2C,
  PORT_PIN_MODE_PWM_MODULE0,
  PORT_PIN_MODE_PWM_MODULE1,
  PORT_PIN_MODE_GPT,
  PORT_PIN_MODE_CAN,
  PORT_PIN_MODE_IDX,
  PORT_PIN_MODE_MOTOR_PHASE,
  PORT_PIN_MODE_TIMER_PWM,
  PORT_PIN_MODE_USB,
    
}PortPinMode;

/* Description: Shall cover max available port pins (8) for TM4C */
typedef uint8 Port_PinType;

/* Declerations for port pin direction if it should be changeable or not */
#define PORT_PIN_DIRECTION_CHANGEABLE           0U
#define PORT_PIN_DIRECTION_UNCHANGEABLE         1U

/* Declerations for port pin mode if it should be changeable or not */
#define PORT_PIN_MODE_CHANGEABLE                0U
#define PORT_PIN_MODE_UNCHANGEABLE              1U

/* Index of pins present in each port */
#define PORTA_PIN0_                             0U
#define PORTA_PIN1_                             1U
#define PORTA_PIN2_                             2U
#define PORTA_PIN3_                             3U
#define PORTA_PIN4_                             4U
#define PORTA_PIN5_                             5U
#define PORTA_PIN6_                             6U
#define PORTA_PIN7_                             7U
#define PORTB_PIN0_                             8U
#define PORTB_PIN1_                             9U
#define PORTB_PIN2_                             10U
#define PORTB_PIN3_                             11U
#define PORTB_PIN4_                             12U
#define PORTB_PIN5_                             13U
#define PORTB_PIN6_                             14U
#define PORTB_PIN7_                             15U
#define PORTC_PIN4_                             16U
#define PORTC_PIN5_                             17U
#define PORTC_PIN6_                             18U
#define PORTC_PIN7_                             19U
#define PORTD_PIN0_                             20U
#define PORTD_PIN1_                             21U
#define PORTD_PIN2_                             22U
#define PORTD_PIN3_                             23U
#define PORTD_PIN4_                             24U
#define PORTD_PIN5_                             25U
#define PORTD_PIN6_                             26U
#define PORTD_PIN7_                             27U
#define PORTE_PIN0_                             28U
#define PORTE_PIN1_                             29U
#define PORTE_PIN2_                             30U
#define PORTE_PIN3_                             31U
#define PORTE_PIN4_                             32U
#define PORTE_PIN5_                             33U
#define PORTF_PIN0_                             34U
#define PORTF_PIN1_                             35U
#define PORTF_PIN2_                             36U
#define PORTF_PIN3_                             37U
#define PORTF_PIN4_                             38U


/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the internal resistor --> Disable, Pull up or Pull down
 */
typedef struct
{
    uint8 port_num; 
    Port_PinType pin_num; 
    Port_PinDirection direction;
    Port_InternalResistor resistor;
    PortPinLevelValue initial_value;
    PortPinMode pin_mode;
    uint8 port_mode_changeable;
    uint8 port_direction_changeable;
}Port_ConfigType;

typedef struct
{
  Port_ConfigType Pins[PORT_CONFIGURED_PINS];
}Port_Conf;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/* Initializes the port driver module */
void Port_init(const Port_Conf *ConfigPtr );

/* Sets the port pin direction */
void Port_SetPinDirection(Port_PinType Pin_Num, Port_PinDirection Direction);

/* Refreshes port direction */
void Port_RefreshPortDirection(void);

/* Returns the version information of this module */
#if (PORT_VERSION_INFO_API==STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo);
#endif

/* Sets the port pin mode */
void Port_SetPinMode(Port_PinType Pin_Num, PortPinMode Pin_Mode);

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port and other modules */
extern const Port_Conf Port_Configuration;

#endif /* PORT_H */
