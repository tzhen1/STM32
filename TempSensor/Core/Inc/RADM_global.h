//*--------------------------------------------------------------------------------------
//*                    RADM - Rotork Actuators Data acquisition Module
//*                         		(c) Synergy AST 2013
//*--------------------------------------------------------------------------------------
//* File Name           : RADM_global.h
//* Object              : global variables - header file
//* Creation            : Jiri Hofman 18/11/2013
//* Last modified       :
//*--------------------------------------------------------------------------------------
#ifndef _RADM_global_H_
#define _RADM_global_H_

//*--------------------------------------------------------------------------------------
//* Include external files
//*--------------------------------------------------------------------------------------
#include "RADM_macros.h"

//*--------------------------------------------------------------------------------------
//* Declaration of global variables
//*--------------------------------------------------------------------------------------
//system variables
extern unsigned int SysLedCounter; 				// system LED counter
extern unsigned char SB;						//Status Byte
extern unsigned char RdpAddresses[RDP_LAST];	//RDP meters RS-485 addresses

//module communication with the master
extern unsigned char MT;							//Module Type
extern unsigned char MA;							//Module Address
extern unsigned char CmdPacket[CMD_PACK_MAX];   	//command from PC data
extern unsigned char CmdCounter;					//command counter - contains number of received bytes
extern unsigned char CmdDecoded;					//command is decoded and to be executed
extern unsigned char CmdReceived;					//command is ready for decoding

//Module mode
extern unsigned int ModuleMode;						//current module mode - enum MODULE_MODES

//control and measured values
extern unsigned char ActualAps[RADM_DATA_PACK];		//actual values of actuator position switches
extern double MeasuredThrust[RADM_DATA_PACK];		//measured actuator thrust from RDP
extern double MeasuredTorque[RADM_DATA_PACK];		//measured actuator torque from RDP
extern double MeasuredCurrent[RADM_DATA_PACK];		//measured actuator current from ADAM
extern unsigned int Sample;							//sample counter - timestamp for measurement
extern unsigned int SampleArray[RADM_DATA_PACK];	//sample counter array - timestamps for measurement
extern unsigned int SamplePacketCount;				//sample packet counter - counts measurements per packet

//extern unsigned int SampleCounter;					//sample counter for synchronization
//extern unsigned int MsCounter;						//millisecond counter
//extern double MeasTempDut;							//measured DUT temperature
//extern double MeasTempAmbient;						//measured ambient temperature
//extern double SetDutTemp;							//DUT temperature set point
//extern double HeaterPower;							//Heater power to set DAC
//extern unsigned int MovingAverCount;				//Moving average counter - counts to buffer size and stays
//extern unsigned int MovingAverPointer;				//Pointer to actual item in moving average buffer
//extern double MovingAverTempDut;					//Moving average of measured DUT temperature
//extern double MovingAverTempDutBuffer[MAVG_SAMPS];  //DUT temperature moving average buffer
//extern double MovingAverTempAmbient;					//Moving average of measured ambient temperature
//extern double MovingAverTempAmbientBuffer[MAVG_SAMPS]; //Ambient temperature moving average buffer
//extern unsigned int PresetDutTempCount;					//DUT temperature preset counter - for test purposes only TODO remove

//status flags
extern unsigned int LastClkSync;					//CLK_SYNC Buffer
//extern unsigned char TimeToMeasure;					//Time to perform measurement

//state machines


#endif /*_RADM_global_H_*/
//*-----------------------------end of file RADM_global.h--------------------------------
