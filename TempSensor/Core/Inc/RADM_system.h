//*--------------------------------------------------------------------------------------
//*                    RADM - Rotork Actuators Data acquisition Module
//*                         		(c) Synergy AST 2013
//*--------------------------------------------------------------------------------------
//* File Name           : RADM_system.h
//* Object              : system related and supporting functions header file
//* Creation            : Jiri Hofman 18/11/2013
//* Last modified       : Thomas Zhen 11/10/2021
//*--------------------------------------------------------------------------------------
#ifndef _RADM_system_H_
#define _RADM_system_H_

//#include "misc.h"
#include "stm32f4xx_hal.h"

//*--------------------------------------------------------------------------------------
//* Definition of new types
//*--------------------------------------------------------------------------------------
//none

//*--------------------------------------------------------------------------------------
//* Macros
//*--------------------------------------------------------------------------------------
//none

//*--------------------------------------------------------------------------------------
//* Declaration of global variables
//*--------------------------------------------------------------------------------------
//none

//*--------------------------------------------------------------------------------------
//* Function prototypes
//*--------------------------------------------------------------------------------------
extern void SystemInitSetup(void);
extern void Sleep(unsigned int HundMicroSec);
extern void Sleep10us(unsigned int TenMicroSec);
extern void WdtReset(void);
extern unsigned int ReadGpioPinStatus(unsigned int PortPin);
extern void WriteToGpioPin(unsigned int PortPin, unsigned int PortPinStatus);
extern void ToggleGpioPin(unsigned int PortPin);
extern void UpdateSystemLeds(void);
extern unsigned char Get8BitCrc(unsigned char DataByte, unsigned char OldCrc);
extern void SendResponseToMaster(unsigned char *DataPacket, unsigned int Length);
extern void SendSlaveCmd(UART_HandleTypeDef USARTx, unsigned char SMT, unsigned char SMA, unsigned int Cmd, unsigned char *DataPacket, unsigned int PacketSize);
extern int ReceiveUsartPacket(UART_HandleTypeDef USARTx, unsigned char *DataPacket, unsigned int PacketLength, unsigned int TimeOut);
extern int ReceiveUsartPacketCrtCheck(UART_HandleTypeDef USARTx, unsigned char *DataPacket, unsigned int PacketLength, unsigned int TimeOut);
extern void DecodeCmd(void);
extern void SetGeneratorOut(uint16_t Prescaler, uint16_t Period, uint16_t Pulse);
extern void SetSpiChipEnable(int SpiDevice, int NewState);
extern void DebugTestFunc(void);
extern void InitUsart3(unsigned int BaudRate);
extern void InitUsart(UART_HandleTypeDef USARTx, unsigned int BaudRate);
extern void WdtReset(void);
extern unsigned char ReadAsp(void);
extern void AdamSendCommand(UART_HandleTypeDef USARTx, unsigned char *CmdPacket, unsigned int CmdSize);
extern int AdamReceiveDataCrtCheck(UART_HandleTypeDef USARTx, unsigned char *DataPacket, unsigned int PacketLength, unsigned int TimeOut);
extern void RadmSelfTest(int TestRdps);
extern void SendMeasurementResults(void);
extern int AdamReadAdc(double *MeasVolt);
extern int RdpReadVal(unsigned char RdpAddress, double *MeasVal);
extern int RdpTest(unsigned char RdpAddress);
#endif /*_RADM_system_H_*/
//*-----------------------------end of file RADM_system.h--------------------------------
