//*--------------------------------------------------------------------------------------
//*                    RADM - Rotork Actuators Data acquisition Module
//*                         		(c) Synergy AST 2013
//*--------------------------------------------------------------------------------------
//* File Name           : RADM_task.c
//* Object              : task manager
//* Creation            : Jiri Hofman 18/11/2013
//* Last modified       : Thomas Zhen 11/10/2021
//*--------------------------------------------------------------------------------------

//*--------------------------------------------------------------------------------------
//* Include external files
//*--------------------------------------------------------------------------------------
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32_hal_legacy.h"
#include <math.h>
#include "RADM_task.h"
#include "RADM_ports.h"
#include "RADM_system.h"
#include "RADM_macros.h"
#include "RADM_global.h"


//*--------------------------------------------------------------------------------------
//* Definition of global variables
//*--------------------------------------------------------------------------------------
//none
	UART_HandleTypeDef huart;

//*--------------------------------------------------------------------------------------
//* Definition of local variables
//*--------------------------------------------------------------------------------------
//none

//*--------------------------------------------------------------------------------------
//* Function Name         : SystemTask
//* Object                : System related regular tasks
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void SystemTask(void)
{
	//Update System LEDs
	UpdateSystemLeds();
}

//*--------------------------------------------------------------------------------------
//* Function Name         : MeasurementTask
//* Object                : Measurement related regular tasks
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void MeasurementTask(void)
{
int Status;
unsigned int AttemptLoop;

	//initialisation of measurement
	if(ModuleMode == MM_MEAS_INIT)
	{
		LastClkSync	= ReadGpioPinStatus(CLK_SYN);
		Sample = 0;
		SamplePacketCount = 0;
		ModuleMode = MM_MEAS;
	}

	//measurement
	if(ModuleMode == MM_MEAS)
	{
		//Check the rising edge of the CLK_SYN signal - time to perform measurement
		if((LastClkSync == 0) && (ReadGpioPinStatus(CLK_SYN) == 1))
		{
			ToggleGpioPin(TEST_PIN);
			ToggleGpioPin(LED_B);
			LastClkSync = ReadGpioPinStatus(CLK_SYN);
#ifndef SIMULATION_MODE //normal measurement mode
			//Measure the current - try it ADAM_ATTEMPTS times
			for(AttemptLoop = 0;AttemptLoop < ADAM_ATTEMPTS;AttemptLoop++)
			{
				Status = AdamReadAdc(&MeasuredCurrent[SamplePacketCount]);
				if(Status == 0)
					break;
			}
			if(Status)
				MeasuredCurrent[SamplePacketCount] = 1e6; //error

			//Read the top RDP - thrust
			Status = RdpReadVal(RdpAddresses[RDP_THRUST], &MeasuredThrust[SamplePacketCount]);
			if(Status)
				MeasuredThrust[SamplePacketCount] = 1e6; //error

			Sleep(50); //wait 5ms to avoid collision on the bus

			//Read the bottom RDP - torque
			Status = RdpReadVal(RdpAddresses[RDP_TORQUE], &MeasuredTorque[SamplePacketCount]);
			if(Status)
				MeasuredTorque[SamplePacketCount] = 1e6; //error
#endif
			//Read the APSs
			ActualAps[SamplePacketCount] = ReadAsp();

#ifdef SIMULATION_MODE //simulation mode
if(ReadGpioPinStatus(TEST_PIN) == 1)
	MeasuredCurrent[SamplePacketCount] = .1 * Sample;
else
	MeasuredCurrent[SamplePacketCount] = 1.23;
MeasuredThrust[SamplePacketCount] = 1e3 * Sample;
MeasuredTorque[SamplePacketCount] = 1e4 * Sample;
//ActualAps[SamplePacketCount] = 0x15;
#endif
			SampleArray[SamplePacketCount] = Sample;

			//send the results to the master
			SamplePacketCount++;
			if(SamplePacketCount > (RADM_DATA_PACK - 1))
			{
				SendMeasurementResults();
				SamplePacketCount = 0;
			}

			Sample++;
		}

		if(ReadGpioPinStatus(CLK_SYN) == 0)
			LastClkSync = 0;
	}

}
//*--------------------------------------------------------------------------------------
//* Function Name         : MasterCmdTask
//* Object                : Manages Master (PC) command tasks
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void MasterCmdTask(void)
{
unsigned char ResponsePacket[10];	//data to be sent to PC as a response
//double *PointerDouble;
//unsigned char *PointerBytes;
//double MeasTempDut;
//int Loop;

	//if command not received
	if(!CmdReceived)
		return;

	//decode CMD packet
	DecodeCmd();

	//Ping module
	if(CmdDecoded == CMD_PING)
	{
		ResponsePacket[0] = 'P';
		ResponsePacket[1] = 'O';
		ResponsePacket[2] = 'N';
		ResponsePacket[3] = 'G';
		SendResponseToMaster(ResponsePacket, 4);
		goto CLEANUP;
	}

	switch(CmdDecoded)
	{
		//*******************************************************************************
		//No CMD to be executed
		//*******************************************************************************
		case CMD_NONE:
		break;
		//*******************************************************************************
		//CMD Start measurement
		//*******************************************************************************
		case CMD_MEAS_START:
		ModuleMode = MM_MEAS_INIT;
		ResponsePacket[0] = 0; //fixed to no errors
		SendResponseToMaster(ResponsePacket, 1);
		break;
		//*******************************************************************************
		//CMD End measurement
		//*******************************************************************************
		case CMD_MEAS_END:
		ModuleMode = MM_STANDBY;
		WriteToGpioPin(LED_B, 0);
		ResponsePacket[0] = 0; //fixed to no errors
		SendResponseToMaster(ResponsePacket, 1);
		break;
		//*******************************************************************************
		//CMD Self-test
		//*******************************************************************************
		case CMD_SELF_TEST:
		RadmSelfTest(1); //RPDs tested as well
		ResponsePacket[0] = SB;
		SendResponseToMaster(ResponsePacket, 1);
		break;
		//*******************************************************************************
		//CMD RADM setup
		//*******************************************************************************
		case CMD_RADM_SETUP:
		RdpAddresses[RDP_THRUST] = CmdPacket[6];//RDP meters RS-485 addresses - thrust meter
		RdpAddresses[RDP_TORQUE] = CmdPacket[7];//RDP meters RS-485 addresses - torque meter
		WriteToGpioPin(ALARM, 0);	//reset the alarm
		ModuleMode = MM_STANDBY;	//stop the measurement
		WriteToGpioPin(LED_R, 0);
		ResponsePacket[0] = 0; //fixed to no errors
		SendResponseToMaster(ResponsePacket, 1);
		break;
		//*******************************************************************************
		//CMD alarm control
		//*******************************************************************************
		case CMD_ALARM_CTRL:
		if(CmdPacket[6] == 0x00)
		{
			WriteToGpioPin(ALARM, 0);
			WriteToGpioPin(LED_R, 0);
		}
		if(CmdPacket[6] == 0xAA)
		{
			WriteToGpioPin(ALARM, 1);
			WriteToGpioPin(LED_R, 1);
		}
		ResponsePacket[0] = 0; //fixed to no errors
		SendResponseToMaster(ResponsePacket, 1);
		break;
	}

CLEANUP:
	CmdCounter = 0;
	CmdReceived	= 0;

//if def usart 2 for ex, do this , but ifdef usart 6 from macros do usart 6 command
#ifdef MASTER_CMD_USART2
	__HAL_UART_ENABLE_IT(&huart,UART_IT_RXNE);
//	USART_ITConfig(USART2, UART_IT_RXNE, ENABLE);; //cmd RX enabled  //USART
#endif
#ifdef MASTER_CMD_USART6
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);; //cmd RX enabled
#endif

}
//*------------------------------end of file RADM_task.c---------------------------------

