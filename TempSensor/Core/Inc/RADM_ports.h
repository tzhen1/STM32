//*--------------------------------------------------------------------------------------
//*                    RADM - Rotork Actuators Data acquisition Module
//*                         		(c) Synergy AST 2013
//*--------------------------------------------------------------------------------------
//* File Name           : RADM_ports.h
//* Object              : definition of MCU GPIOs header file
//* Creation            : Jiri Hofman 18/11/2013
//* Last modified       :
//*--------------------------------------------------------------------------------------
#ifndef _RADM_ports_H_
#define _RADM_ports_H_

#include "RADM_macros.h"
//*--------------------------------------------------------------------------------------
//* Definition of new types
//*--------------------------------------------------------------------------------------
//none

//*--------------------------------------------------------------------------------------
//* Directly and peripheral controlled GPIOs enumeration
//*--------------------------------------------------------------------------------------

enum MCU_GPIO
{
	LED_G = 0, 	//PD12: DIS board Green LED LD4 [1 = activated]
	LED_O, 		//PD13: DIS board Orange LED LD3 [1 = activated]
	LED_R, 		//PD14: DIS board Red LED LD5 [1 = activated]
	LED_B, 		//PD15: DIS board Blue LED LD6 [1 = activated]
//	BUTT,		//PA0: Button [1 = activated] - user button disabled

	USART2_TX,	//PA2: USART2 TX (Master command OUT)
	USART2_RX,	//PA3: USART2 RX (Master command IN)
	USART2_RTS,	//PA1: USART2 RTS (Master command IN)
	USART2_CTS,	//PA0: USART2 CTS (Master command OUT)

	USART6_TX,	//PC6: USART6 TX (ADAM command OUT)
	USART6_RX,	//PC7: USART6 RX (ADAM command IN)
	RS485_DIR1,	//PE4: RS485 DIR1 (USART6 to ADAM) [1 = RADM sends data]

	UART4_TX,	//PC10: UART4 TX (RDPs command OUT)
	UART4_RX,	//PC11: UART4 RX (RDPs command IN)
	RS485_DIR2,	//PE2:  RS485 DIR2 (UART4 to RDPs) [1 = RADM sends data]
	ALARM,		//PE6:  Alarm output [1 = activated]

	GEN_OUT,	//PB0: TIM3_CH3 (Generator PWM output)

	PLED_R,		//PE5: Panel DUAL LED red [1 = activated]
	PLED_G,		//PE3: Panel DUAL LED red [1 = activated]

	APS1,		//PD10: Actuator position switch 1 [0 = closed]
	APS2,		//PD8: 	Actuator position switch 2 [0 = closed]
	APS3,		//PE14: Actuator position switch 3 [0 = closed]
	APS4,		//PE12: Actuator position switch 4 [0 = closed]
	APS5,		//PE10: Actuator position switch 5 [0 = closed]

    CLK_SYN, 	//PC4: Synchronisation clock input [100 ms nominal]

	TEST_PIN,	//PA8: Test pin for debug purposes
	GPIO_LAST	//Last item in enum = number of GPIOs
};

extern unsigned int ActuatorPosSwitches[NR_ACT_PS];
//*--------------------------------------------------------------------------------------
//* Declaration of global variables
//*--------------------------------------------------------------------------------------
extern GPIO_TypeDef* McuGpioPerDef[GPIO_LAST];
extern uint16_t McuGpioPinDef[GPIO_LAST];

//*--------------------------------------------------------------------------------------
//* Function prototypes
//*--------------------------------------------------------------------------------------
extern void GpioInitialSetup(void);
extern void Usart2GpioSetup(void);
extern void Usart3GpioSetup(void);
extern void Uart4GpioSetup(void);
extern void Usart6GpioSetup(void);
extern void Tim3GpioSetup(void);
extern void Tim4GpioSetup(void);
extern void Spi1GpioSetup(void);
extern void Spi2GpioSetup(void);
extern void I2c1GpioSetup(void);
#endif /*_RADM_ports_H_*/
//*-----------------------------end of file RADM_ports.h---------------------------------







