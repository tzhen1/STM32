//*--------------------------------------------------------------------------------------
//*                    RADM - Rotork Actuators Data acquisition Module
//*                         		(c) Synergy AST 2013
//*--------------------------------------------------------------------------------------
//* File Name           : RADM_global.c
//* Object              : global variables
//* Creation            : Jiri Hofman 18/11/2013
//* Last modified       :
//*--------------------------------------------------------------------------------------

//*--------------------------------------------------------------------------------------
//* Include external files
//*--------------------------------------------------------------------------------------
//#include "misc.h"
#include "RADM_global.h"

//*--------------------------------------------------------------------------------------
//* Definition of global variables
//*--------------------------------------------------------------------------------------
//system variables
unsigned int SysLedCounter; 				//system LED counter
unsigned char SB;							//Status Byte
unsigned char RdpAddresses[RDP_LAST];		//RDP meters RS-485 addresses


//module communication with the master
unsigned char MT;							//Module Type
unsigned char MA;							//Module Address
unsigned char CmdPacket[CMD_PACK_MAX];   	//command from PC data
unsigned char CmdCounter;					//command counter - contains number of received bytes
unsigned char CmdDecoded;					//command is decoded and to be executed
unsigned char CmdReceived;					//command is ready for decoding

//module mode
unsigned int ModuleMode;					//current module mode - enum MODULE_MODES

//control and measured values
unsigned char ActualAps[RADM_DATA_PACK];	//actual values of actuator position switches
double MeasuredThrust[RADM_DATA_PACK];		//measured actuator thrust from RDP
double MeasuredTorque[RADM_DATA_PACK];		//measured actuator torque from RDP
double MeasuredCurrent[RADM_DATA_PACK];		//measured actuator current from ADAM
unsigned int Sample;						//sample counter - timestamp for measurement
unsigned int SampleArray[RADM_DATA_PACK];	//sample counter array - timestamps for measurement

unsigned int SamplePacketCount;				//sample packet counter - counts measurements per packet

//unsigned int SampleCounter;					//sample counter for synchronisation
//unsigned int MsCounter;						//millisecond counter
//double MeasTempDut;							//measured DUT temperature
//double MeasTempAmbient;						//measured ambient temperature
//double SetDutTemp;							//DUT temperature set point
//double HeaterPower;							//Heater power to set DAC
//unsigned int MovingAverCount;				//Moving average counter - counts to buffer size and stays
//unsigned int MovingAverPointer;				//Pointer to actual item in moving average buffer
//double MovingAverTempDut;					//Moving average of measured DUT temperature
//double MovingAverTempDutBuffer[MAVG_SAMPS]; //DUT temperature moving average buffer
//double MovingAverTempAmbient;					//Moving average of measured ambient temperature
//double MovingAverTempAmbientBuffer[MAVG_SAMPS]; //Ambient temperature moving average buffer
//unsigned int PresetDutTempCount;					//DUT temperature preset counter - for test purposes only TODO remove


//status flags
unsigned int LastClkSync;					//CLK_SYNC Buffer

//state machines
//*------------------------------end of file RADM_global.c-------------------------------

