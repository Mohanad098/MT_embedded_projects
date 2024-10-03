/******************************************************************************
*
* Module: Port
*
* File Name: Port.c
*
* Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
*
* Author: Mohanad Hany
******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"


/*#if (PORT_DEV_ERROR_DETECT == STD_ON)*/

#include "Det.h"
/* Different autosar versions */
/* AUTOSAR Version checking between Det and Port Modules */
/*
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
|| (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
|| (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Det.h does not match the expected version"
#endif

#endif
*/
STATIC const Port_ConfigType * Port_PinsSettings = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/************************************************************************************
* Service Name: Port_init
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initializes the port driver module:
*              - Setup the direction of the GPIO pin
*              - Setup the internal resistor for i/p pin
*              - Setup the initial value for o/p pin
*              - Setup the GPIO pin mode
************************************************************************************/
void Port_init(const Port_Conf * ConfigPtr )
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* check if the input configuration pointer is not a NULL_PTR */
  if (NULL_PTR == ConfigPtr)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID, PORT_E_PARAM_CONFIG);
  }
  else
#endif
  {
    /*
    * Set the module state to initialized and point to the PB configuration structure using a global pointer.
    * This global pointer is global to be used by other functions to read the PB configuration structures
    */
    Port_Status  = PORT_INITIALIZED;
    Port_PinsSettings = ConfigPtr->Pins; /* address of the first Channels structure --> Channels[0] */
  }
  volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
  volatile uint32 delay = 0;
  for(uint8 i=0; i<PORT_CONFIGURED_PINS;i++)
  {
    
    /******************** Port number ********************/
    
    switch(Port_PinsSettings[i].port_num)
    {
    case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
    break;
    case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
    break;
    case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
    break;
    case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
    break;
    case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
    break;
    case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
    break;
    default: /* Do Nth */ break;
    }
    
    /*********** Enable clock for PORT and allow time for clock to start **************/
    SYSCTL_REGCGC2_REG |= (1<<Port_PinsSettings[i].port_num);
    delay = SYSCTL_REGCGC2_REG;
    
    /************************* Unlocking pins ****************************/
    /* PD7 or PF0 */
    if( ((Port_PinsSettings[i].port_num == 3) && (Port_PinsSettings[i].pin_num == 7)) || ((Port_PinsSettings[i].port_num == 5) && (Port_PinsSettings[i].pin_num == 0)) ) 
    {
      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                                       /* Unlock the GPIOCR register */   
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PinsSettings[i].pin_num);          /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    /* PC0 to PC3 */
    else if( (Port_PinsSettings[i].port_num == 2) && (Port_PinsSettings[i].pin_num <= 3) ) 
    {
      /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
      /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    
    
    /********* Enable analog for ADC mode and disable it for the rest **********/
    if(Port_PinsSettings[i].pin_mode==PORT_PIN_MODE_ADC)
    {
      /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PinsSettings[i].pin_num);
      /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PinsSettings[i].pin_num);
    }
    /********* Enabling Digital for all modes except ADC *************/
    else
    {
      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PinsSettings[i].pin_num);
      /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PinsSettings[i].pin_num);
    }
    
    
    /**************************** PORT MODES ****************************/
    
    switch(Port_PinsSettings[i].pin_mode)
    {
    case PORT_PIN_MODE_ADC:
      if(Port_PinsSettings[i].direction==PORT_PIN_IN)
      {
        if((Port_PinsSettings[i].port_num==PortConf_PortB && (Port_PinsSettings[i].pin_num==PortConf_Pin4 || Port_PinsSettings[i].pin_num==PortConf_Pin5)) \
          || (Port_PinsSettings[i].port_num==PortConf_PortC) || (Port_PinsSettings[i].port_num==PortConf_PortE) \
            || ((Port_PinsSettings[i].port_num==PortConf_PortD) && (Port_PinsSettings[i].pin_num==PortConf_Pin0 || Port_PinsSettings[i].pin_num==PortConf_Pin1 || Port_PinsSettings[i].pin_num==PortConf_Pin2 || Port_PinsSettings[i].pin_num==PortConf_Pin3)))
        {
          /* Disable Alternative function for this pin by clearing the corresponding bit in GPIOAFSEL register */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
          /* Clear the PMCx bits for this pin */
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        }
        else
        {
          /*Wrong pin chosen*/
        }
      }
      else
      {
        /*Wrong direction chosen*/
      }
      break;
      
    case PORT_PIN_MODE_DIO:
      /* Disable Alternative function for this pin by clearing the corresponding bit in GPIOAFSEL register */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
      /* Clear the PMCx bits for this pin */
      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));     
      break;
      
    case PORT_PIN_MODE_UART:
      if((Port_PinsSettings[i].port_num==PortConf_PortA && (Port_PinsSettings[i].pin_num==PortConf_Pin0 || Port_PinsSettings[i].pin_num==PortConf_Pin1)) \
        || (Port_PinsSettings[i].port_num==PortConf_PortC) \
          || (Port_PinsSettings[i].port_num==PortConf_PortD && (Port_PinsSettings[i].pin_num==PortConf_Pin4 || Port_PinsSettings[i].pin_num==PortConf_Pin5 || Port_PinsSettings[i].pin_num==PortConf_Pin6 || Port_PinsSettings[i].pin_num==PortConf_Pin7)) \
            || (Port_PinsSettings[i].port_num==PortConf_PortE && (Port_PinsSettings[i].pin_num!=PortConf_Pin2 && Port_PinsSettings[i].pin_num!=PortConf_Pin3)) \
              || (Port_PinsSettings[i].port_num==PortConf_PortF && (Port_PinsSettings[i].pin_num==PortConf_Pin0 || Port_PinsSettings[i].pin_num==PortConf_Pin1)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_PinsSettings[i].pin_num * 4));
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_UART_MODULE1:
      if (Port_PinsSettings[i].port_num==PortConf_PortB && (Port_PinsSettings[i].pin_num==PortConf_Pin0 || Port_PinsSettings[i].pin_num==PortConf_Pin1))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_PinsSettings[i].pin_num * 4));
      }
      else if (Port_PinsSettings[i].port_num==PortConf_PortC && (Port_PinsSettings[i].pin_num==PortConf_Pin4 || Port_PinsSettings[i].pin_num==PortConf_Pin5))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_PinsSettings[i].pin_num * 4));
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_SPI:
      if((Port_PinsSettings[i].port_num==PortConf_PortA && (Port_PinsSettings[i].pin_num==PortConf_Pin2 || Port_PinsSettings[i].pin_num==PortConf_Pin3 || Port_PinsSettings[i].pin_num==PortConf_Pin4 || Port_PinsSettings[i].pin_num==PortConf_Pin5)) \
        || (Port_PinsSettings[i].port_num==PortConf_PortB && (Port_PinsSettings[i].pin_num==PortConf_Pin4 || Port_PinsSettings[i].pin_num==PortConf_Pin5 || Port_PinsSettings[i].pin_num==PortConf_Pin6 || Port_PinsSettings[i].pin_num==PortConf_Pin7)) \
          || (Port_PinsSettings[i].port_num==PortConf_PortD && (Port_PinsSettings[i].pin_num==PortConf_Pin0 || Port_PinsSettings[i].pin_num==PortConf_Pin1 || Port_PinsSettings[i].pin_num==PortConf_Pin2 || Port_PinsSettings[i].pin_num==PortConf_Pin3)) \
            || (Port_PinsSettings[i].port_num==PortConf_PortF && Port_PinsSettings[i].pin_num!=PortConf_Pin4))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_PinsSettings[i].pin_num * 4));        
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_SPI_MODULE3:
      if (Port_PinsSettings[i].port_num==PortConf_PortD && (Port_PinsSettings[i].pin_num==PortConf_Pin0 || Port_PinsSettings[i].pin_num==PortConf_Pin1 || Port_PinsSettings[i].pin_num==PortConf_Pin2 || Port_PinsSettings[i].pin_num==PortConf_Pin3))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_PinsSettings[i].pin_num * 4));
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_I2C:
      if((Port_PinsSettings[i].port_num==PortConf_PortA && (Port_PinsSettings[i].pin_num==PortConf_Pin6 || Port_PinsSettings[i].pin_num==PortConf_Pin7)) \
        || (Port_PinsSettings[i].port_num==PortConf_PortB && (Port_PinsSettings[i].pin_num==PortConf_Pin2 || Port_PinsSettings[i].pin_num==PortConf_Pin3)) \
          || (Port_PinsSettings[i].port_num==PortConf_PortD && (Port_PinsSettings[i].pin_num==PortConf_Pin0 || Port_PinsSettings[i].pin_num==PortConf_Pin1)) \
            || (Port_PinsSettings[i].port_num==PortConf_PortE && (Port_PinsSettings[i].pin_num!=PortConf_Pin4 && Port_PinsSettings[i].pin_num!=PortConf_Pin5)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_PinsSettings[i].pin_num * 4));        
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_PWM_MODULE0:
      if((Port_PinsSettings[i].port_num==PortConf_PortB && (Port_PinsSettings[i].pin_num==PortConf_Pin4 || Port_PinsSettings[i].pin_num==PortConf_Pin5 || Port_PinsSettings[i].pin_num==PortConf_Pin6 || Port_PinsSettings[i].pin_num==PortConf_Pin7)) \
        || (Port_PinsSettings[i].port_num==PortConf_PortC && (Port_PinsSettings[i].pin_num==PortConf_Pin4 || Port_PinsSettings[i].pin_num==PortConf_Pin5)) \
          || (Port_PinsSettings[i].port_num==PortConf_PortD && (Port_PinsSettings[i].pin_num==PortConf_Pin0 || Port_PinsSettings[i].pin_num==PortConf_Pin1)) \
            || (Port_PinsSettings[i].port_num==PortConf_PortE && (Port_PinsSettings[i].pin_num==PortConf_Pin4 || Port_PinsSettings[i].pin_num!=PortConf_Pin5)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_PinsSettings[i].pin_num * 4));              
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_PWM_MODULE1:
      if ((Port_PinsSettings[i].port_num==PortConf_PortA && (Port_PinsSettings[i].pin_num==PortConf_Pin6 || Port_PinsSettings[i].pin_num==PortConf_Pin7)) \
        || (Port_PinsSettings[i].port_num==PortConf_PortD && (Port_PinsSettings[i].pin_num==PortConf_Pin0 || Port_PinsSettings[i].pin_num==PortConf_Pin1)) \
          || (Port_PinsSettings[i].port_num==PortConf_PortE && (Port_PinsSettings[i].pin_num!=PortConf_Pin4 && Port_PinsSettings[i].pin_num!=PortConf_Pin5)) \
            || (Port_PinsSettings[i].port_num==PortConf_PortF && Port_PinsSettings[i].pin_num!=PortConf_Pin4))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_PinsSettings[i].pin_num * 4));                    
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;      
      
    case PORT_PIN_MODE_GPT:
      if((Port_PinsSettings[i].port_num==PortConf_PortB) \
        || (Port_PinsSettings[i].port_num==PortConf_PortC) \
          || (Port_PinsSettings[i].port_num==PortConf_PortD) \
            || (Port_PinsSettings[i].port_num==PortConf_PortF))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_PinsSettings[i].pin_num * 4));                          
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_CAN:
      if((Port_PinsSettings[i].port_num==PortConf_PortA && (Port_PinsSettings[i].pin_num==PortConf_Pin0 || Port_PinsSettings[i].pin_num==PortConf_Pin1)) \
        || (Port_PinsSettings[i].port_num==PortConf_PortB && (Port_PinsSettings[i].pin_num==PortConf_Pin4 || Port_PinsSettings[i].pin_num==PortConf_Pin5)) \
          || (Port_PinsSettings[i].port_num==PortConf_PortE && (Port_PinsSettings[i].pin_num==PortConf_Pin4 || Port_PinsSettings[i].pin_num!=PortConf_Pin5)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_PinsSettings[i].pin_num * 4));                                
      }
      else if(Port_PinsSettings[i].port_num==PortConf_PortF && (Port_PinsSettings[i].pin_num==PortConf_Pin0 || Port_PinsSettings[i].pin_num==PortConf_Pin3))      
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_PinsSettings[i].pin_num * 4));      
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_IDX:
      if ((Port_PinsSettings[i].port_num==PortConf_PortC && Port_PinsSettings[i].pin_num==PortConf_Pin4) \
        || (Port_PinsSettings[i].port_num==PortConf_PortD && Port_PinsSettings[i].pin_num==PortConf_Pin3) \
          || (Port_PinsSettings[i].port_num==PortConf_PortF && Port_PinsSettings[i].pin_num==PortConf_Pin4))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000006 << (Port_PinsSettings[i].pin_num * 4));                                      
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_MOTOR_PHASE:
      if ((Port_PinsSettings[i].port_num==PortConf_PortC && (Port_PinsSettings[i].pin_num==PortConf_Pin5 || Port_PinsSettings[i].pin_num==PortConf_Pin6)) \
        || (Port_PinsSettings[i].port_num==PortConf_PortD && (Port_PinsSettings[i].pin_num==PortConf_Pin6 || Port_PinsSettings[i].pin_num==PortConf_Pin7)) \
          || (Port_PinsSettings[i].port_num==PortConf_PortF && (Port_PinsSettings[i].pin_num==PortConf_Pin0 || Port_PinsSettings[i].pin_num==PortConf_Pin1)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000006 << (Port_PinsSettings[i].pin_num * 4));       
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_TIMER_PWM:
      if(Port_PinsSettings[i].port_num==PortConf_PortF && (Port_PinsSettings[i].pin_num==PortConf_Pin1 || Port_PinsSettings[i].pin_num==PortConf_Pin2 || Port_PinsSettings[i].pin_num==PortConf_Pin3))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x0000000e << (Port_PinsSettings[i].pin_num * 4));      
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_USB:
      if (Port_PinsSettings[i].port_num==PortConf_PortD && (Port_PinsSettings[i].pin_num==PortConf_Pin4 || Port_PinsSettings[i].pin_num==PortConf_Pin5))
      {
        /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PinsSettings[i].pin_num);
        /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PinsSettings[i].pin_num);
        /* Disable Alternative function for this pin by clearing the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Clear the PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));        
      }
      else if ((Port_PinsSettings[i].port_num==PortConf_PortC && (Port_PinsSettings[i].pin_num==PortConf_Pin6 || Port_PinsSettings[i].pin_num==PortConf_Pin7)) \
        || (Port_PinsSettings[i].port_num==PortConf_PortD && (Port_PinsSettings[i].pin_num==PortConf_Pin2 || Port_PinsSettings[i].pin_num==PortConf_Pin3)) \
          || (Port_PinsSettings[i].port_num==PortConf_PortF && Port_PinsSettings[i].pin_num==PortConf_Pin4))  
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PinsSettings[i].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_PinsSettings[i].pin_num * 4));      
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    default: /* Do nothing */ break;
    }
    
    /************************** Checking directions and internal resistor ************************/
    
    /* Making direction input */
    if (Port_PinsSettings[i].direction==PORT_PIN_IN)
    {
      /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinsSettings[i].pin_num);
      /* Checking internal resistor */
      if(Port_PinsSettings[i].resistor==PULL_UP)
      {
        /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PinsSettings[i].pin_num);
      }
      else if (Port_PinsSettings[i].resistor==PULL_DOWN)
      {
        /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PinsSettings[i].pin_num);
      }
      else
      {
        /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PinsSettings[i].pin_num);             
        /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PinsSettings[i].pin_num);
      }
      
    }
    /* Making direction Output */
    else if (Port_PinsSettings[i].direction==PORT_PIN_OUT)
    {
      /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinsSettings[i].pin_num);
      if(Port_PinsSettings[i].initial_value == STD_HIGH)
      {
        /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PinsSettings[i].pin_num);         
      }
      else
      {
        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PinsSettings[i].pin_num);        
      }
    }
    
  }
}

/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in):
*               - Port Pin ID number
*               - Port Pin direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
************************************************************************************/      

void Port_SetPinDirection(Port_PinType Pin_Num, Port_PinDirection Direction)
{
  volatile uint32 * PortGpio_Ptr = NULL_PTR;
  uint8 pin_num=0;
  boolean error = FALSE;
  
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* Check if module was initialized */
  if(Port_Status == PORT_NOT_INITIALIZED)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
    error=TRUE;
  }
  else
  {
    /* No Action Required */
  }
  /* Check if correct pin ID passed */
  if(Pin_Num>=PORT_CONFIGURED_PINS)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
    error=TRUE;
  }
  else
  {
    /* No Action Required */
  }
  /* Check if port pin configured as changeable */
  if(Port_PinsSettings[Pin_Num].port_direction_changeable == PORT_PIN_DIRECTION_UNCHANGEABLE)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
    error=TRUE;
  }
  else
  {
    /* No Action Required */
  }
#endif
  
  /* In-case there are no errors */
  if(FALSE == error)
  {
    
    /*********** Pointing towards the required PORT ************/
    /* 8 pins in PORTA */
    if(Pin_Num<=PORTA_PIN7_)
    {
      pin_num   =  Pin_Num;
      PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
    }
    /* The next 8 pins in PORTB */
    else if(Pin_Num>PORTA_PIN7_ && Pin_Num<=PORTB_PIN7_)
    {
      pin_num   =  Pin_Num-PORTB_PIN0_; /* We sub the pin number from the 8 pins in portA */
      PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
    }
    /* The next 4 pins in PORTC (JTAG pins were excluded */
    else if(Pin_Num>PORTB_PIN7_ && Pin_Num<=PORTC_PIN7_)
    {
      pin_num   =  Pin_Num - PORTC_PIN4_; /* We sub the pin number from the 8 pins in portA and the 8 pins from PortB */
      PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
    }
    /* The next 8 pins in PORTD */
    else if(Pin_Num>PORTC_PIN7_ && Pin_Num<=PORTD_PIN7_)
    {
      pin_num   =  Pin_Num - PORTD_PIN0_;
      PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */      
    }
    /* The next 6 pins in PORTE */
    else if(Pin_Num>PORTD_PIN7_ && Pin_Num<=PORTE_PIN5_)
    {
      pin_num   =  Pin_Num - PORTE_PIN0_;
      PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
    }
    /* The next 5 pins in PORTF */
    else if(Pin_Num>PORTE_PIN5_ && Pin_Num<=PORTF_PIN4_)
    {
      pin_num   =  Pin_Num - PORTF_PIN0_;
      PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
    }
    
    /**************** Changing the direction ****************/
    /* Making direction Input */
    if (Direction==PORT_PIN_IN)
    {
      /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , pin_num);      
    }
    /* Making direction Output */
    else if (Direction==PORT_PIN_OUT)
    {
      /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , pin_num); 
    }   
  }
  else
  {
    /* No Action Required */
  }
}


/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction
************************************************************************************/
void Port_RefreshPortDirection(void)
{
  volatile uint32 * PortGpio_Ptr = NULL_PTR;
  boolean error = FALSE;
  
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* Check if module was initialized */
  if(Port_Status == PORT_NOT_INITIALIZED)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIRECTION, PORT_E_UNINIT);
    error=TRUE;
  }
  else
  {
    /* No Action Required */
  }
#endif
  
  if(FALSE==error)
  {
    for(uint8 i=0;i<PORT_CONFIGURED_PINS;i++)
    {
      if(Port_PinsSettings[i].port_direction_changeable==PORT_PIN_DIRECTION_UNCHANGEABLE)
      {
        switch(Port_PinsSettings[i].port_num)
        {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
        break;
        case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
        break;
        case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
        break;
        case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
        break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
        break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
        break;
        default: /* Do Nth */ break;
        }
        switch(Port_PinsSettings[i].direction)
        {
        case PORT_PIN_IN:
          /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinsSettings[i].pin_num);
          break;
        case PORT_PIN_OUT: 
          /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinsSettings[i].pin_num);
          break;
        default: /* Do nothing */ break;
        }
      }
      else
      {
        /* No Action Required */
      }
    }
  }
  else
  {
    /* No Action Required */
  }
}

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): version info
* Return value: None
* Description: Returns the version information of this module.
************************************************************************************/
#if (PORT_VERSION_INFO_API==STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* Check if module was initialized */
  if(Port_Status == PORT_NOT_INITIALIZED)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO, PORT_E_UNINIT);
    error=TRUE;
  }
  else
  {
    /* No Action Required */
  }
  /* Check if input pointer is not Null pointer */
  if(NULL_PTR == versioninfo)
  {
    /* Report to DET  */
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                    PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
  }
  else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
  {
    /* Copy the vendor Id */
    versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
    /* Copy the module Id */
    versioninfo->moduleID = (uint16)PORT_MODULE_ID;
    /* Copy Software Major Version */
    versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
    /* Copy Software Minor Version */
    versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
    /* Copy Software Patch Version */
    versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
  }
}
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in):
*               - Port Pin number
*               - Port Pin mode
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode
************************************************************************************/
void Port_SetPinMode(Port_PinType Pin_Num, PortPinMode Pin_Mode)
{
  volatile uint32 * PortGpio_Ptr = NULL_PTR;
  uint8 port_num=0;
  uint8 pin_num=0;
  boolean error=FALSE;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* Check if module was initialized */
  if(Port_Status == PORT_NOT_INITIALIZED)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_UNINIT);
    error=TRUE;
  }
  else
  {
    /* No Action Required */
  }
  if(Pin_Num>=PORT_CONFIGURED_PINS)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_PIN);
    error=TRUE;
  }
  else
  {
    /* No Action Required */
  }
  if(Port_PinsSettings[Pin_Num].port_mode_changeable==PORT_PIN_MODE_UNCHANGEABLE)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_MODE_UNCHANGEABLE);
    error=TRUE;
  }
  else
  {
    /* No Action Required */
  }
#endif
  
  if(FALSE==error)
  {
    /*********** Pointing towards the required PORT ************/
    /* 8 pins in PORTA */
    if(Pin_Num<=PORTA_PIN7_)
    {
      port_num  =  PortConf_PortA;
      pin_num   =  Pin_Num;
      PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
    }
    /* The next 8 pins in PORTB */
    else if(Pin_Num>PORTA_PIN7_ && Pin_Num<=PORTB_PIN7_)
    {
      port_num  =  PortConf_PortB;
      pin_num   =  Pin_Num-PORTB_PIN0_; /* We sub the pin number from the 8 pins in portA */
      PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
    }
    /* The next 4 pins in PORTC (JTAG pins were excluded */
    else if(Pin_Num>PORTB_PIN7_ && Pin_Num<=PORTC_PIN7_)
    {
      port_num  =  PortConf_PortC;
      pin_num   =  Pin_Num - PORTC_PIN4_; /* We sub the pin number from the 8 pins in portA and the 8 pins from PortB */
      PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
    }
    /* The next 8 pins in PORTD */
    else if(Pin_Num>PORTC_PIN7_ && Pin_Num<=PORTD_PIN7_)
    {
      port_num  =  PortConf_PortD;
      pin_num   =  Pin_Num - PORTD_PIN0_;
      PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */      
    }
    /* The next 6 pins in PORTE */
    else if(Pin_Num>PORTD_PIN7_ && Pin_Num<=PORTE_PIN5_)
    {
      port_num  =  PortConf_PortE;
      pin_num   =  Pin_Num - PORTE_PIN0_;
      PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
    }
    /* The next 5 pins in PORTF */
    else if(Pin_Num>PORTE_PIN5_ && Pin_Num<=PORTF_PIN4_)
    {
      port_num  =  PortConf_PortF;
      pin_num   =  Pin_Num - PORTF_PIN0_;
      PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
    }
    /**************************** PORT MODES ****************************/
    
    switch(Pin_Mode)
    {
    case PORT_PIN_MODE_ADC:
      if((port_num==PortConf_PortB && (pin_num==PortConf_Pin4 || pin_num==PortConf_Pin5)) \
        || (port_num==PortConf_PortC) || (port_num==PortConf_PortE) \
          || ((port_num==PortConf_PortD) && (pin_num==PortConf_Pin0 || pin_num==PortConf_Pin1 || pin_num==PortConf_Pin2 || pin_num==PortConf_Pin3)))
      {
        /* Disable Alternative function for this pin by clearing the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Clear the PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif
      }
      break;
      
    case PORT_PIN_MODE_DIO:
      /* Disable Alternative function for this pin by clearing the corresponding bit in GPIOAFSEL register */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
      /* Clear the PMCx bits for this pin */
      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));     
      break;
      
    case PORT_PIN_MODE_UART:
      if((port_num==PortConf_PortA && (pin_num==PortConf_Pin0 || pin_num==PortConf_Pin1)) \
        || (port_num==PortConf_PortC) \
          || (port_num==PortConf_PortD && (pin_num==PortConf_Pin4 || pin_num==PortConf_Pin5 || pin_num==PortConf_Pin6 || pin_num==PortConf_Pin7)) \
            || (port_num==PortConf_PortE && (pin_num!=PortConf_Pin2 && pin_num!=PortConf_Pin3)) \
              || (port_num==PortConf_PortF && (pin_num==PortConf_Pin0 || pin_num==PortConf_Pin1)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (pin_num * 4));        
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif
      }
      break;
      
    case PORT_PIN_MODE_UART_MODULE1:
      if(port_num==PortConf_PortB && (pin_num==PortConf_Pin0 || pin_num==PortConf_Pin1))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (pin_num * 4));
      }
      else if(port_num==PortConf_PortC && (pin_num==PortConf_Pin4 || pin_num==PortConf_Pin5))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (pin_num * 4));
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif        
      }
      break;
      
    case PORT_PIN_MODE_SPI:
      if((port_num==PortConf_PortA && (pin_num==PortConf_Pin2 || pin_num==PortConf_Pin3 || pin_num==PortConf_Pin4 || pin_num==PortConf_Pin5)) \
        || (port_num==PortConf_PortB && (pin_num==PortConf_Pin4 || pin_num==PortConf_Pin5 || pin_num==PortConf_Pin6 || pin_num==PortConf_Pin7)) \
          || (port_num==PortConf_PortD && (pin_num==PortConf_Pin0 || pin_num==PortConf_Pin1 || pin_num==PortConf_Pin2 || pin_num==PortConf_Pin3)) \
            || (port_num==PortConf_PortF && pin_num!=PortConf_Pin4))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (pin_num * 4));
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif
      }
      break;
      
    case PORT_PIN_MODE_SPI_MODULE3:
      if (port_num==PortConf_PortD && (pin_num==PortConf_Pin0 || pin_num==PortConf_Pin1 || pin_num==PortConf_Pin2 || pin_num==PortConf_Pin3))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (pin_num * 4));
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif        
      }
      break;
      
    case PORT_PIN_MODE_I2C:
      if((port_num==PortConf_PortA && (pin_num==PortConf_Pin6 || pin_num==PortConf_Pin7)) \
        || (port_num==PortConf_PortB && (pin_num==PortConf_Pin2 || pin_num==PortConf_Pin3)) \
          || (port_num==PortConf_PortD && (pin_num==PortConf_Pin0 || pin_num==PortConf_Pin1)) \
            || (port_num==PortConf_PortE && (pin_num!=PortConf_Pin4 && pin_num!=PortConf_Pin5)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (pin_num * 4));        
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif
      }
      break;
      
    case PORT_PIN_MODE_PWM_MODULE0:
      if ((port_num==PortConf_PortB && (pin_num==PortConf_Pin4 || pin_num==PortConf_Pin5 || pin_num==PortConf_Pin6 || pin_num==PortConf_Pin7)) \
        || (port_num==PortConf_PortC && (pin_num==PortConf_Pin4 || pin_num==PortConf_Pin5)) \
          || (port_num==PortConf_PortD && (pin_num==PortConf_Pin0 || pin_num==PortConf_Pin1)) \
            || (port_num==PortConf_PortE && (pin_num==PortConf_Pin4 || pin_num!=PortConf_Pin5)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (pin_num * 4));              
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif        
      }
      break;
    case PORT_PIN_MODE_PWM_MODULE1: 
      if ((port_num==PortConf_PortA && (pin_num==PortConf_Pin6 || pin_num==PortConf_Pin7)) \
        || (port_num==PortConf_PortD && (pin_num==PortConf_Pin0 || pin_num==PortConf_Pin1)) \
          || (port_num==PortConf_PortE && (pin_num!=PortConf_Pin4 && pin_num!=PortConf_Pin5)) \
            || (port_num==PortConf_PortF && pin_num!=PortConf_Pin4))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (pin_num * 4));                    
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif
      }
      break;      
      
    case PORT_PIN_MODE_GPT:
      if((port_num==PortConf_PortB) \
        || (port_num==PortConf_PortC) \
          || (port_num==PortConf_PortD) \
            || (port_num==PortConf_PortF))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (pin_num * 4));                          
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif
      }
      break;
      
    case PORT_PIN_MODE_CAN:
      if((port_num==PortConf_PortA && (pin_num==PortConf_Pin0 || pin_num==PortConf_Pin1)) \
        || (port_num==PortConf_PortB && (pin_num==PortConf_Pin4 || pin_num==PortConf_Pin5)) \
          || (port_num==PortConf_PortE && (pin_num==PortConf_Pin4 || pin_num!=PortConf_Pin5)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (pin_num * 4));                                
      }
      else if(port_num==PortConf_PortF && (pin_num==PortConf_Pin0 || pin_num==PortConf_Pin3))      
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (pin_num * 4));      
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif
      }
      break;
      
    case PORT_PIN_MODE_IDX:
      if ((port_num==PortConf_PortC && pin_num==PortConf_Pin4) \
        || (port_num==PortConf_PortD && pin_num==PortConf_Pin3) \
          || (port_num==PortConf_PortF && pin_num==PortConf_Pin4))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000006 << (pin_num * 4));                                      
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif
      }
      break;
      
    case PORT_PIN_MODE_MOTOR_PHASE:
      if ((port_num==PortConf_PortC && (pin_num==PortConf_Pin5 || pin_num==PortConf_Pin6)) \
        || (port_num==PortConf_PortD && (pin_num==PortConf_Pin6 || pin_num==PortConf_Pin7)) \
          || (port_num==PortConf_PortF && (pin_num==PortConf_Pin0 || pin_num==PortConf_Pin1)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000006 << (pin_num * 4));       
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif
      }
      break;
      
    case PORT_PIN_MODE_TIMER_PWM:
      if(port_num==PortConf_PortF && (pin_num==PortConf_Pin1 || pin_num==PortConf_Pin2 || pin_num==PortConf_Pin3))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x0000000e << (pin_num * 4));      
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif
      }
      break;
      
    case PORT_PIN_MODE_USB:
      if (port_num==PortConf_PortD && (pin_num==PortConf_Pin4 || pin_num==PortConf_Pin5))
      {
        /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , pin_num);
        /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , pin_num);
        /* Disable Alternative function for this pin by clearing the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Clear the PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));        
      }
      else if ((port_num==PortConf_PortC && (pin_num==PortConf_Pin6 || pin_num==PortConf_Pin7)) \
        || (port_num==PortConf_PortD && (pin_num==PortConf_Pin2 || pin_num==PortConf_Pin3)) \
          || (port_num==PortConf_PortF && pin_num==PortConf_Pin4))  
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , pin_num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (pin_num * 4));      
      }
      else
      {
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE, PORT_E_PARAM_INVALID_MODE);
        error=TRUE;
#endif
      }
      break;
      
    default: /* Do nothing */ break;
    }
  }
  else
  {
    /* No action required */
  }
}