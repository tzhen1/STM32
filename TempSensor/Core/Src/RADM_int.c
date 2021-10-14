//*--------------------------------------------------------------------------------------
//*                    RADM - Rotork Actuators Data acquisition Module
//*                         		(c) Synergy AST 2013
//*--------------------------------------------------------------------------------------
//* File Name           : RADM_int.c
//* Object              : interrupt related functions
//* Creation            : Jiri Hofman 18/11/2013
//* Last modified       : Thomas Zhen 11/10/2021
//*--------------------------------------------------------------------------------------

//*--------------------------------------------------------------------------------------
//* Include external files
//*--------------------------------------------------------------------------------------
//#include "misc.h"
//#include "stm32f4xx.h"
//#include "stm32f4xx_conf.h"
//#include "stm32f4xx_usart.h"
//#include "stm32f4xx_tim.h"
//#include "system_stm32f4xx.h"
#include "stm32f4xx_hal.h"
//#include "stm32f4xx_gpio.h"
//#include "stm32f4xx_rcc.h"
#include "RADM_int.h"
#include "RADM_system.h"
#include "RADM_ports.h"
#include "RADM_macros.h"
#include "RADM_global.h"

//*--------------------------------------------------------------------------------------
//* Definition of global variables
//*--------------------------------------------------------------------------------------
//none

//*--------------------------------------------------------------------------------------
//* Definition of local variables
//*--------------------------------------------------------------------------------------
//none

//*--------------------------------------------------------------------------------------
//* Function Name         : USART2_IRQHandler (defined in startup_stm32f4xx_md_vl.c)
//* Object                : Master command byte received on USART2
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------
#ifdef MASTER_CMD_USART2
void USART2_IRQHandler(void)
{
	UART_HandleTypeDef huart;
	uint8_t buffer[4];
//	CmdPacket[CmdCounter] = (unsigned char)USART_ReceiveData(USART2);
	CmdPacket[CmdCounter] = (unsigned char)HAL_UART_Receive(&huart, buffer , sizeof(buffer), HAL_MAX_DELAY);
//	ToggleGpioPin(LED_B);

	//testing of packet header
	if (((CmdCounter == 0) && (CmdPacket[0] == '!')) ||	//fixed start character
   		((CmdCounter == 1) && (CmdPacket[1] == MT )) || //Module type
   		((CmdCounter == 2) && (CmdPacket[2] == MA )) || //Module address
   		(CmdCounter > 2))
	{
		++CmdCounter;
	}
	else
	{
		CmdCounter = 0;
	}

	//packet is too long
	if(CmdCounter > CMD_PACK_MAX)
		CmdCounter = 0;

  	//testing of the end of packet
	if((CmdCounter >= CMD_PACK_MIN) && (CmdCounter == (CmdPacket[3] + 6))) //expected nr. of bytes received
	{
		CmdReceived = 1;				//CMD ready for decode
		ToggleGpioPin(LED_O);
//		USART_ITConfig(USART2, UART_IT_RXNE, DISABLE);; //cmd RX disabled until command executed
		__HAL_UART_DISABLE_IT(&huart,UART_IT_RXNE );
	}
	else
	{
		CmdReceived = 0;
	}
}
#endif
//*--------------------------------------------------------------------------------------
//* Function Name         : USART6_IRQHandler (defined in startup_stm32f4xx_md_vl.c)
//* Object                : Master command byte received on USART6
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------
#ifdef MASTER_CMD_USART6
void USART6_IRQHandler(void)
{
	CmdPacket[CmdCounter] = (unsigned char)USART_ReceiveData(USART6);
//	ToggleGpioPin(LED_B);

	//testing of packet header
	if (((CmdCounter == 0) && (CmdPacket[0] == '!')) ||	//fixed start character
   		((CmdCounter == 1) && (CmdPacket[1] == MT )) || //Module type
   		((CmdCounter == 2) && (CmdPacket[2] == MA )) || //Module address
   		(CmdCounter > 2))
	{
		++CmdCounter;
	}
	else
	{
		CmdCounter = 0;
	}

	//packet is too long
	if(CmdCounter > CMD_PACK_MAX)
		CmdCounter = 0;

  	//testing of the end of packet
	if((CmdCounter >= CMD_PACK_MIN) && (CmdCounter == (CmdPacket[3] + 6))) //expected nr. of bytes received
	{
		CmdReceived = 1;				//CMD ready for decode
		ToggleGpioPin(LED_O);
		USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);; //cmd RX disabled until command executed
	}
	else
	{
		CmdReceived = 0;
	}
}
#endif
//*--------------------------------------------------------------------------------------
//* Function Name         : TIM2_IRQHandler (defined in startup_stm32f4xx_md_vl.c)
//* Object                : Counter update event - 1ms tick
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------
//void TIM2_IRQHandler(void)
//{
//
//	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//
//
//	MsCounter++;
//	if(MsCounter >999)
//	{
//		MsCounter = 0;
//		TimeToMeasure = 1;	//Time to perform measurement
//		ToggleGpioPin(TEST_PIN);
//
////		if(ModuleMode == MM_MEAS)
////			ToggleGpioPin(LED_B);
//	}
//}
//*-------------------------------end of file RADM_int.c---------------------------------
