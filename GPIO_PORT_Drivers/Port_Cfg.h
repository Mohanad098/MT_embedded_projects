 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Mohanad Hany
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (2U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (3U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (0U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_OFF)

/* Pre-compile option for presence of Port_Set_Pin_Direction API */
#define Port_Set_Pin_Direction_Api           (STD_ON)

/* Pre-compile option for presence of Port_Set_Pin_Mode API */
#define Port_Set_Pin_Mode_Api                (STD_ON)

/* Number of the configured Port Channels */
#define PORT_CONFIGURED_PINS             (39U)
/* Port c pins 0,1,2,3 are JTAG pins so they have been excluded from the total 43 pins */

/* Port Index in the array of structures in Port_PBcfg.c */
#define PortConf_PortA                          (uint8)0x00
#define PortConf_PortB                          (uint8)0x01
#define PortConf_PortC                          (uint8)0x02
#define PortConf_PortD                          (uint8)0x03
#define PortConf_PortE                          (uint8)0x04
#define PortConf_PortF                          (uint8)0x05

/* Pin Index in the array of structures in Port_PBcfg.c */
#define PortConf_Pin0                           (uint8)0x00
#define PortConf_Pin1                           (uint8)0x01
#define PortConf_Pin2                           (uint8)0x02
#define PortConf_Pin3                           (uint8)0x03
#define PortConf_Pin4                           (uint8)0x04
#define PortConf_Pin5                           (uint8)0x05
#define PortConf_Pin6                           (uint8)0x06
#define PortConf_Pin7                           (uint8)0x07

/*  Port Mode instance ID */
#define MODE_0                                  (uint8)0x00
#define MODE_1                                  (uint8)0x01   
#define MODE_2                                  (uint8)0x02
#define MODE_3                                  (uint8)0x03
#define MODE_4                                  (uint8)0x04
#define MODE_5                                  (uint8)0x05
#define MODE_6                                  (uint8)0x06
#define MODE_7                                  (uint8)0x07
   
#endif /* PORT_CFG_H */
