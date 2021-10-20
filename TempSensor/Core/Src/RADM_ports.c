//*--------------------------------------------------------------------------------------
//*                    RADM - Rotork Actuators Data acquisition Module
//*                         		(c) Synergy AST 2013
//*--------------------------------------------------------------------------------------
//* File Name           : RADM_ports.c
//* Object              : definition of MCU GPIOs
//* Creation            : Jiri Hofman 18/11/2013
//* Last modified       : Thomas Zhen 11/10/2021
//*--------------------------------------------------------------------------------------

//*--------------------------------------------------------------------------------------
//* Include external files
//*--------------------------------------------------------------------------------------
#include "stm32f4xx.h"
#include "stm32_hal_legacy.h"
#include "stm32f4xx_hal.h"
#include "RADM_system.h"
#include "RADM_ports.h"
#include "RADM_macros.h"


//*--------------------------------------------------------------------------------------
//* External References
//*--------------------------------------------------------------------------------------
//none

//*--------------------------------------------------------------------------------------
//* Definition of global variables
//* Directly controlled GPIOs port definition
//*--------------------------------------------------------------------------------------

GPIO_TypeDef* McuGpioPerDef[GPIO_LAST] =
{
	GPIOD, 	//PD12: DIS board Green LED LD4 [1 = activated]
	GPIOD, 	//PD13: DIS board Orange LED LD3 [1 = activated]
	GPIOD, 	//PD14: DIS board Red LED LD5 [1 = activated]
	GPIOD, 	//PD15: DIS board Blue LED LD6 [1 = activated]
//	GPIOA,	//PA0: Button [1 = activated] - user button disabled

	GPIOA,	//PA2: USART2 TX (Master command OUT)
	GPIOA,	//PA3: USART2 RX (Master command IN)
	GPIOA,	//PA1: USART2 RTS (Master command IN)
	GPIOA,	//PA0: USART2 CTS (Master command OUT)

	GPIOC,	//PC6: USART6 TX (ADAM command OUT)
	GPIOC,	//PC7: USART6 RX (ADAM command IN)
	GPIOE,	//PE4: RS485 DIR1 (USART6 to ADAM) [1 = RADM sends data]

	GPIOC,	//PC10: UART4 TX (RDPs command OUT)
	GPIOC,	//PC11: UART4 RX (RDPs command IN)
	GPIOE,	//PE2:  RS485 DIR2 (UART4 to RDPs) [1 = RADM sends data]
	GPIOE,	//PE6:  Alarm output [1 = activated]

	GPIOB,	//PB0: TIM3_CH3 (Generator PWM output)

	GPIOE,	//PE5: Panel DUAL LED red [1 = activated]
	GPIOE,  //PE3: Panel DUAL LED red [1 = activated]


	GPIOD,  //PD10: Actuator position switch 1 [0 = closed]
	GPIOD,	//PD8: 	Actuator position switch 2 [0 = closed]
	GPIOE,  //PE14: Actuator position switch 3 [0 = closed]
	GPIOE,  //PE12: Actuator position switch 4 [0 = closed]
	GPIOE,  //PE10: Actuator position switch 5 [0 = closed]


	GPIOC, 	//PC4: Synchronisation clock input [100 ms nominal]

	GPIOA	//PA8: Test pin for debug purposes
};

uint16_t McuGpioPinDef[GPIO_LAST] =
{
	GPIO_PIN_12,//PD12: DIS board Green LED LD4 [1 = activated]
	GPIO_PIN_13,//PD13: DIS board Orange LED LD3 [1 = activated]
	GPIO_PIN_14,//PD14: DIS board Red LED LD5 [1 = activated]
	GPIO_PIN_15,//PD15: DIS board Blue LED LD6 [1 = activated]
//	GPIO_Pin_0,	//PA0: Button [1 = activated] - user button disabled

	GPIO_PIN_2,	//PA2: USART2 TX (Master command OUT)
	GPIO_PIN_3,	//PA3: USART2 RX (Master command IN)
	GPIO_PIN_1,	//PA1: USART2 RTS (Master command IN)
	GPIO_PIN_0,	//PA0: USART2 CTS (Master command OUT)


	GPIO_PIN_6,	//PC6: USART6 TX (ADAM command OUT)
	GPIO_PIN_7,	//PC7: USART6 RX (ADAM command IN)
	GPIO_PIN_4,	//PE4: RS485 DIR1 (USART6 to ADAM) [1 = RADM sends data]

	GPIO_PIN_10,//PC10: UART4 TX (Slave command OUT)
	GPIO_PIN_11,//PC11: UART4 RX (Slave command IN)
	GPIO_PIN_2,	//PE2:  RS485 DIR2 (UART4 to RDPs) [1 = RADM sends data]
	GPIO_PIN_6,	//PE6:  Alarm output [1 = activated]

	GPIO_PIN_0,	//PB0: TIM3_CH3 (Generator PWM output)

	GPIO_PIN_5,	//PE5: Panel DUAL LED red [1 = activated]
	GPIO_PIN_3,	//PE3: Panel DUAL LED red [1 = activated]

	GPIO_PIN_10,//PD10: Actuator position switch 1 [0 = closed]
	GPIO_PIN_8,	//PD8: 	Actuator position switch 2 [0 = closed]
	GPIO_PIN_14,//PE14: Actuator position switch 3 [0 = closed]
	GPIO_PIN_12,//PE12: Actuator position switch 4 [0 = closed]
	GPIO_PIN_10,//PE10: Actuator position switch 5 [0 = closed]

	GPIO_PIN_4,	//PC4: Synchronisation clock input [100 ms nominal]

	GPIO_PIN_8	//PA8: Test pin for debug purposes
};

unsigned int ActuatorPosSwitches[NR_ACT_PS] = {
		APS1,		//PD10: Actuator position switch 1 [0 = closed]
		APS2,		//PD8: 	Actuator position switch 2 [0 = closed]
		APS3,		//PE14: Actuator position switch 3 [0 = closed]
		APS4,		//PE12: Actuator position switch 4 [0 = closed]
		APS5};		//PE10: Actuator position switch 5 [0 = closed]

//*--------------------------------------------------------------------------------------
//* Peripheral controlled GPIOs initialisation data TODO
//*--------------------------------------------------------------------------------------


//*--------------------------------------------------------------------------------------
//* Definition of local variables
//*--------------------------------------------------------------------------------------
//none


//*--------------------------------------------------------------------------------------
//* Function Name         : GpioInitialSetup
//* Object                : Initial setup of directly controlled GPIOs
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void GpioInitialSetup(void)
{
GPIO_InitTypeDef GPIO_InitStructure;

	//GPIO setup - enable clock to particular port gates
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
//	__GPIOA_CLK_ENABLE();
//	__GPIOB_CLK_ENABLE();
//	__GPIOC_CLK_ENABLE();
//	__GPIOD_CLK_ENABLE();
//	__GPIOE_CLK_ENABLE();
//	__GPIOF_CLK_ENABLE();
//	__GPIOG_CLK_ENABLE();
//	__GPIOH_CLK_ENABLE();
//	__GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();

	//common attributes for LEDs
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;		//output mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//push-pull output type
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//No pull-up, no pull-down

	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;		//output mode
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//No pull-up, no pull-down

	//LED_G port setup
	GPIO_InitStructure.Pin = McuGpioPinDef[LED_G];
	HAL_GPIO_Init(McuGpioPerDef[LED_G], &GPIO_InitStructure);
	WriteToGpioPin(LED_G, 0);

	//LED_O port setup
	GPIO_InitStructure.Pin = McuGpioPinDef[LED_O];
	HAL_GPIO_Init(McuGpioPerDef[LED_O], &GPIO_InitStructure);
	WriteToGpioPin(LED_O, 0);

	//LED_R port setup
	GPIO_InitStructure.Pin = McuGpioPinDef[LED_R];
	HAL_GPIO_Init(McuGpioPerDef[LED_R], &GPIO_InitStructure);
	WriteToGpioPin(LED_R, 0);

	//LED_B port setup
	GPIO_InitStructure.Pin = McuGpioPinDef[LED_B];
	HAL_GPIO_Init(McuGpioPerDef[LED_B], &GPIO_InitStructure);
	WriteToGpioPin(LED_B, 0);

	//PLED_R port setup
	GPIO_InitStructure.Pin = McuGpioPinDef[PLED_R];
	HAL_GPIO_Init(McuGpioPerDef[PLED_R], &GPIO_InitStructure);
	WriteToGpioPin(PLED_R, 0);

	//PLED_G port setup
	GPIO_InitStructure.Pin = McuGpioPinDef[PLED_G];
	HAL_GPIO_Init(McuGpioPerDef[PLED_G], &GPIO_InitStructure);
	WriteToGpioPin(PLED_G, 0);

	//TEST_PIN port setup - output mode
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[TEST_PIN];//TEST_PIN port
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;		//output mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//push-pull output type
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;		//Pull-up, no pull-down
//	GPIO_Init(McuGpioPerDef[TEST_PIN], &GPIO_InitStructure);
//	WriteToGpioPin(TEST_PIN, 1);							//Initial state: logic high

GPIO_InitStructure.Pin   = McuGpioPinDef[TEST_PIN];//TEST_PIN port
GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;		//input mode
//GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
GPIO_InitStructure.Pull  = GPIO_PULLUP;		//Pull-up, no pull-down
HAL_GPIO_Init(McuGpioPerDef[TEST_PIN], &GPIO_InitStructure);


//	//BUTT port setup
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[BUTT];//BUTT port
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;		//input mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//push-pull output type - N/A
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//No pull-up, no pull-down
//	GPIO_Init(McuGpioPerDef[BUTT], &GPIO_InitStructure);

	//common attributes for actuator switches inputs
	GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;		//output mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//push-pull output type - N/A
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
#ifdef SIMULATION_MODE
	GPIO_InitStructure.Pull  = GPIO_PULLUP;		//Pull-up, no pull-down
#else
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//no pull-up, no pull-down
#endif
	//APS1 port setup
	GPIO_InitStructure.Pin = McuGpioPinDef[APS1];
	HAL_GPIO_Init(McuGpioPerDef[APS1], &GPIO_InitStructure);

	//APS2 port setup
	GPIO_InitStructure.Pin = McuGpioPinDef[APS2];
	HAL_GPIO_Init(McuGpioPerDef[APS2], &GPIO_InitStructure);

	//APS3 port setup
	GPIO_InitStructure.Pin = McuGpioPinDef[APS3];
	HAL_GPIO_Init(McuGpioPerDef[APS3], &GPIO_InitStructure);

	//APS4 port setup
	GPIO_InitStructure.Pin = McuGpioPinDef[APS4];
	HAL_GPIO_Init(McuGpioPerDef[APS4], &GPIO_InitStructure);

	//APS5 port setup
	GPIO_InitStructure.Pin = McuGpioPinDef[APS5];
	HAL_GPIO_Init(McuGpioPerDef[APS5], &GPIO_InitStructure);

	//CLK_SYN port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[CLK_SYN];//CLK_SYN port
	GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;		//input mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//no pull-up, no pull-down
	HAL_GPIO_Init(McuGpioPerDef[CLK_SYN], &GPIO_InitStructure);

	//ALARM port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[ALARM];//ALARM port
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;		//output mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//open-drain output type
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//No pull-up, no pull-down
	HAL_GPIO_Init(McuGpioPerDef[ALARM], &GPIO_InitStructure);
	WriteToGpioPin(ALARM, 0);						//Initial state: logic low - alarm muted
}
//*--------------------------------------------------------------------------------------
//* Function Name         : Usart2GpioSetup
//* Object                : Setup of USART6 controlled GPIOs
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void Usart2GpioSetup(void)
{
GPIO_InitTypeDef GPIO_InitStructure;

	//USART2 remap to pins PC6 and PC7 , use .Alternate for HAL (PA2 and PA3 it meant i think)
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

//GPIO_InitStructure.Alternate = GPIO_AF7_USART2;

	//USART2_TX port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[USART2_TX];//USART2_TX port
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;		//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//open-drain output type
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//No pull-up, no pull-down
	GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(McuGpioPerDef[USART2_TX], &GPIO_InitStructure);

	//USART2_RX port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[USART2_RX];//USART2_RX port
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;		//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//open-drain output type - N/A
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//No pull-up, no pull-down
	GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(McuGpioPerDef[USART2_RX], &GPIO_InitStructure);

	//USART2_RTS port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[USART2_RTS];//USART2_RTS port
	GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;		//output mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//open-drain output type - N/A
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//No pull-up, no pull-down
	HAL_GPIO_Init(McuGpioPerDef[USART2_RTS], &GPIO_InitStructure);

	//USART2_CTS port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[USART2_CTS];//USART2_CTS port
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;		//output mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//push-pull output type
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//No pull-up, no pull-down
	HAL_GPIO_Init(McuGpioPerDef[USART2_CTS], &GPIO_InitStructure);
	WriteToGpioPin(USART2_CTS, 1);						//initial state - logic high
}
//*--------------------------------------------------------------------------------------
//* Function Name         : Usart3GpioSetup
//* Object                : Setup of USART3 controlled GPIOs
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void Usart3GpioSetup(void)
{
//GPIO_InitTypeDef GPIO_InitStructure;

//	//USART3 remap to pin PB10 and PB11
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
//
//	//USART3_TX port setup
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[USART3_TX];//USART3_TX port
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;			//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;			//open-drain output type
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//high speed on 30 pF (80 MHz Output max speed on 15 pF)
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;		//no pull-up
//	GPIO_Init(McuGpioPerDef[USART3_TX], &GPIO_InitStructure);
//
//	//USART3_RX port setup
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[USART3_RX];//USART3_RX port
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;			//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;			//open-drain output type
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//high speed on 30 pF (80 MHz Output max speed on 15 pF)
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;		//no pull-up
//	GPIO_Init(McuGpioPerDef[USART3_RX], &GPIO_InitStructure);
}
//*--------------------------------------------------------------------------------------
//* Function Name         : Uart4GpioSetup
//* Object                : Setup of UART4 controlled GPIOs
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void Uart4GpioSetup(void)
{
GPIO_InitTypeDef GPIO_InitStructure;

	//UART4 remap to pin PC10 and PC11
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	//UART4_TX port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[UART4_TX];//UART4_TX port
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;			//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;			//open-drain output type
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;		//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;		//no pull-up
	GPIO_InitStructure.Alternate = GPIO_AF8_UART4;  /*  AF8  */
	HAL_GPIO_Init(McuGpioPerDef[UART4_TX], &GPIO_InitStructure);

	//UART4_RX port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[UART4_RX];//UART4_TX port
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;			//Alternate function Mode
//	GPIO_InitStructure.OType = GPIO_OType_OD;			//open-drain output type
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;		//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;		//no pull-up
	GPIO_InitStructure.Alternate = GPIO_AF8_UART4;
	HAL_GPIO_Init(McuGpioPerDef[UART4_RX], &GPIO_InitStructure);

	//RS485_DIR2 port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[RS485_DIR2];//RS485_DIR2 port
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;		//output mode
//	GPIO_InitStructure.OType = GPIO_OType_OD;		//open-drain output type
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//No pull-up, no pull-down
	HAL_GPIO_Init(McuGpioPerDef[RS485_DIR2], &GPIO_InitStructure);
	WriteToGpioPin(RS485_DIR2, 0);						//Initial state: logic low - RX mode
}
//*--------------------------------------------------------------------------------------
//* Function Name         : Usart6GpioSetup
//* Object                : Setup of USART6 controlled GPIOs
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void Usart6GpioSetup(void)
{
GPIO_InitTypeDef GPIO_InitStructure;

	//USART6 remap to pins PC6 and PC7
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);


#ifdef MASTER_CMD_USART2
	//USART6_TX port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[USART6_TX];//USART6_TX port
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;		//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//open-drain output type
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//No pull-up, no pull-down
	GPIO_InitStructure.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(McuGpioPerDef[USART6_TX], &GPIO_InitStructure);
#endif
#ifdef MASTER_CMD_USART6
	//USART6_TX port setup - port used master communication (debug)
	GPIO_InitStructure.Pin   = McuGpioPinDef[USART6_TX];//USART6_TX port
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;		//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//push-pull output type
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_PULLUP;		//pull-up
	GPIO_InitStructure.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(McuGpioPerDef[USART6_TX], &GPIO_InitStructure);
#endif

	//USART6_RX port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[USART6_RX];//USART6_RX port
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;		//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//open-drain output type - N/A
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//No pull-up, no pull-down
	GPIO_InitStructure.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(McuGpioPerDef[USART6_RX], &GPIO_InitStructure);

	//RS485_DIR1 port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[RS485_DIR1];//RS485_DIR1 port
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;		//output mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//open-drain output type
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//No pull-up, no pull-down
	HAL_GPIO_Init(McuGpioPerDef[RS485_DIR1], &GPIO_InitStructure);
	WriteToGpioPin(RS485_DIR1, 0);						//Initial state: logic low - RX mode
}
//*--------------------------------------------------------------------------------------
//* Function Name         : Tim3GpioSetup
//* Object                : Setup of TIM3 controlled GPIOs
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void Tim3GpioSetup(void)
{
GPIO_InitTypeDef GPIO_InitStructure;

	//USART6 remap to pins PB13 and PB15
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3); // PB0

	//GEN_OUT port setup
	GPIO_InitStructure.Pin   = McuGpioPinDef[GEN_OUT];//GEN_OUT port
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;		//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//push-pull output type
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
	GPIO_InitStructure.Pull  = GPIO_NOPULL;	//no pull-up
	GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(McuGpioPerDef[GEN_OUT], &GPIO_InitStructure);
}
//*--------------------------------------------------------------------------------------
//* Function Name         : Tim4GpioSetup
//* Object                : Setup of TIM4 controlled GPIOs
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void Tim4GpioSetup(void)
{
//GPIO_InitTypeDef GPIO_InitStructure;
//
//	//GM_PULSES port setup
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[GM_PULSES];	//GM_PULSES port
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			//Top speed, could be reduced to save power
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;		//floating input type
//	GPIO_Init(McuGpioPerDef[GM_PULSES], &GPIO_InitStructure);
}
//*--------------------------------------------------------------------------------------
//* Function Name         : Spi1GpioSetup
//* Object                : Setup of SPI1 controlled GPIOs
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void Spi1GpioSetup(void)
{
//GPIO_InitTypeDef GPIO_InitStructure;

//	//SPI1 remap to pins PA4 to PA7
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);	//SPI1_NSS
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);	//SPI1_SCK
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);	//SPI1_MISO
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);	//SPI1_MOSI
//
//	//SPI1_NSS port setup
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[SPI1_NSS];//SPI1_SCK port
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;		//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//open-drain output type - N/A
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//No pull-up, no pull-down
//	GPIO_Init(McuGpioPerDef[SPI1_NSS], &GPIO_InitStructure);
//
//	//SPI1_SCK port setup
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[SPI1_SCK];//SPI1_SCK port
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;		//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//open-drain output type - N/A
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//No pull-up, no pull-down
//	GPIO_Init(McuGpioPerDef[SPI1_SCK], &GPIO_InitStructure);
//
//	//SPI1_MISO port setup
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[SPI1_MISO];//SPI1_MISO port
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;		//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//open-drain output type - N/A
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//No pull-up, no pull-down
//	GPIO_Init(McuGpioPerDef[SPI1_MISO], &GPIO_InitStructure);
//
//	//SPI1_MOSI port setup
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[SPI1_MOSI];//SPI1_MOSI port
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;		//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//open-drain output type - N/A
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//No pull-up, no pull-down
//	GPIO_Init(McuGpioPerDef[SPI1_MOSI], &GPIO_InitStructure);
}
//*--------------------------------------------------------------------------------------
//* Function Name         : Spi2GpioSetup
//* Object                : Setup of SPI2 controlled GPIOs - DAC interface
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void Spi2GpioSetup(void)
{
//GPIO_InitTypeDef GPIO_InitStructure;

//	//SPI2 remap to pins PB13 and PB15
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);	//SPI2_SCK
////	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);	//SPI2_MISO - MISO not used for DAC
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);	//SPI2_MOSI
//
//	//SPI2_SCK port setup
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[SPI2_SCK];//SPI2_SCK port
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;		//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//push-pull output type
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//No pull-up, no pull-down
//	GPIO_Init(McuGpioPerDef[SPI2_SCK], &GPIO_InitStructure);
//
////	//SPI2_MISO port setup  - MISO not used for DAC
////	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[SPI2_MISO];//SPI2_MOSI port
////	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;		//Alternate function Mode
////	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//push-pull output type - N/A
////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
////	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//No pull-up, no pull-down
////	GPIO_Init(McuGpioPerDef[SPI2_MISO], &GPIO_InitStructure);
//
//	//SPI2_MOSI port setup
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[SPI2_MOSI];//SPI2_MOSI port
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;		//Alternate function Mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//push-pull output type
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//No pull-up, no pull-down
//	GPIO_Init(McuGpioPerDef[SPI2_MOSI], &GPIO_InitStructure);
//
//	//DAC_SYN port setup
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[DAC_SYN];//DAC_SYN port
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;		//output mode
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//push-pull output type
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//high speed on 30 pF (80 MHz Output max speed on 15 pF)
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//No pull-up, no pull-down
//	GPIO_Init(McuGpioPerDef[DAC_SYN], &GPIO_InitStructure);
//	WriteToGpioPin(DAC_SYN, 1);							//initial state - logic high

}
//*--------------------------------------------------------------------------------------
//* Function Name         : I2c1GpioSetup
//* Object                : Setup of I2C1 controlled GPIOs
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void I2c1GpioSetup(void)
{
//GPIO_InitTypeDef GPIO_InitStructure;
//
//	//I2C1 remap to PB8 and PB9
//	GPIO_PinRemapConfig(GPIO_Remap_I2C1,ENABLE);
//
//	//IR thermometer serial data clock input
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[I2C1_SCL];//I2C1_SCL port
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//Top speed, could be reduced to save power
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;		//Alternate function output Open-drain type
////GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
//	GPIO_Init(McuGpioPerDef[I2C1_SCL], &GPIO_InitStructure);
//
//	//IR thermometer serial data input/output
//	GPIO_InitStructure.GPIO_Pin   = McuGpioPinDef[I2C1_SDA];//I2C1_SDA port
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//Top speed, could be reduced to save power
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;		//Alternate function output Open-drain type
////GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
//	GPIO_Init(McuGpioPerDef[I2C1_SDA], &GPIO_InitStructure);
}
//*-------------------------------end of file RADM_ports.c---------------------------------


