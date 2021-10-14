//*--------------------------------------------------------------------------------------
//*                    RADM - Rotork Actuators Data acquisition Module
//*                         		(c) Synergy AST 2013
//*--------------------------------------------------------------------------------------
//* File Name           : RADM_system.c
//* Object              : system related and supporting functions
//* Creation            : Jiri Hofman 18/11/2013
//* Last modified       : Thomas Zhen 11/10/2021
//*--------------------------------------------------------------------------------------

//*--------------------------------------------------------------------------------------
//* Include external files
//*--------------------------------------------------------------------------------------
//#include "misc.h"
//#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32_hal_legacy.h"
//#include "stm32f4xx_conf.h"
//#include "stm32f4xx_usart.h"
#include "stm32f4xx_hal_uart.h"
#include "system_stm32f4xx.h"
//#include "stm32f4xx_gpio.h"
//#include "stm32f4xx_rcc.h"
//#include "stm32f4xx_tim.h"
//#include "stm32f4xx_spi.h"
//#include "stm32f4xx_i2c.h"
#include "RADM_system.h"
#include "RADM_ports.h"
#include "RADM_macros.h"
#include "RADM_global.h"
#include "RADM_int.h"
#include <stdio.h>
#include <stdlib.h>

//*--------------------------------------------------------------------------------------
//* External References
//*--------------------------------------------------------------------------------------
//none
//*--------------------------------------------------------------------------------------
//* Definition of global variables
//*--------------------------------------------------------------------------------------
//none
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
UART_HandleTypeDef huart4;
TIM_HandleTypeDef htim3;
uint8_t buffer[15];
uint8_t rx_buff[15];
//*--------------------------------------------------------------------------------------
//* Definition of local variables
//*--------------------------------------------------------------------------------------
//none

//*--------------------------------------------------------------------------------------
//* Function Name         : SystemInitSetup
//* Object                : Initial setup of SFRs, global variables, peripheries etc
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void SystemInitSetup(void)
{
HAL_Init();

//NVIC_InitTypeDef NVIC_InitStruct;
//TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//SPI_InitTypeDef SPI_InitStructure;
//I2C_InitTypeDef  I2C_InitStructure;
//RCC_ClocksTypeDef RCC_ClocksNow;
RCC_ClkInitTypeDef RCC_ClocksNow = {0};
RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//USART_InitTypeDef USART_InitStructure;
//unsigned int InitTimer;
//int Status;

__HAL_RCC_PWR_CLK_ENABLE();
__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/////////////////////////////////////////////////////////////////////////////////////
	//Directly controlled GPIO setup
	/////////////////////////////////////////////////////////////////////////////////////
	GpioInitialSetup();
	WriteToGpioPin(LED_B,1);			//setup in progress

	WriteToGpioPin(LED_B,0);			//setup in progress

	//Initialise the Embedded Flash Interface, the PLL and update the SystemFrequency variable
	SystemInit();

	//System clock setup is done - read back for debug
//	RCC_GetClocksFreq(&RCC_ClocksNow); //System clock test function
	HAL_RCC_GetSysClockFreq();
	//System clock test results - external crystal  8 MHz:
	//SYSCLK = 67.199160 MHz
	//HCLK   = 67.199160 MHz
	//PCLK1  = 16.799790 MHz
	//PCKL2  = 33.599580 MHz

	/////////////////////////////////////////////////////////////////////////////////////
	//SysTick
	/////////////////////////////////////////////////////////////////////////////////////
//	RCC_HCLKConfig(RCC_SYSCLK_Div1); //setup AHB (HCLK) clock = SYSCLK
//	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); //AHB clock selected as SysTick clock source
	//Initialises the RCC Oscillators according to the specified parameters in the RCC_OscInitTypeDef structure.
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	// Initialises the CPU, AHB and APB buses clocks

	RCC_ClocksNow.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								 |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClocksNow.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClocksNow.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClocksNow.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClocksNow.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClocksNow, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}


	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk;

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //enable clock for AFIO


	/////////////////////////////////////////////////////////////////////////////////////
	//Directly controlled GPIO setup
	/////////////////////////////////////////////////////////////////////////////////////
	GpioInitialSetup();


	/////////////////////////////////////////////////////////////////////////////////////
	//USART2 (Master commands) setup
	//further changes to be done via InitUsart() function
	/////////////////////////////////////////////////////////////////////////////////////
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //RCC enable for USART2
	__USART2_CLK_ENABLE();

//	InitUsart(USART2, 115200);
//	USART_Cmd(USART2, ENABLE);

//	huart2.Instance = USART2;
//	huart2.Init.BaudRate = 115200;
//	HAL_UART_Init(&huart2);
	Usart2GpioSetup(); //initialisation of associated GPIOs


	/////////////////////////////////////////////////////////////////////////////////////
	//USART3 (DUT) setup
	//further changes to be done via InitUsart() function
	/////////////////////////////////////////////////////////////////////////////////////
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); //RCC enable for USART3
//	InitUsart(USART3, 115200);
//	USART_Cmd(USART3, ENABLE);
//	Usart3GpioSetup(); //initialisation of associated GPIOs


	/////////////////////////////////////////////////////////////////////////////////////
	//&UART4 (RDPs commands) setup
	//further changes to be done via InitUsart() function
	/////////////////////////////////////////////////////////////////////////////////////
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_&UART4,ENABLE); //RCC enable for &UART4
	__USART4_CLK_ENABLE();
//	InitUsart(&UART4, 9600);
//	InitUsart(&UART4, 57600);
//	USART_Cmd(&UART4, ENABLE);


//	huart4.Instance = &UART4;
//	huart4.Init.BaudRate = 57600;
//	HAL_UART_Init(&huart4);

	Uart4GpioSetup(); //initialisation of associated GPIOs


	/////////////////////////////////////////////////////////////////////////////////////
	//&USART6 (ADAM commands) setup
	/////////////////////////////////////////////////////////////////////////////////////
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_&USART6,ENABLE); //RCC enable for &USART6
	__USART6_CLK_ENABLE();
//	InitUsart(&USART6, 115200);
//	USART_Cmd(&USART6, ENABLE);
//	huart6.Instance = &USART6;
//	huart6.Init.BaudRate = 115200;
//	HAL_UART_Init(&huart6);


	Usart6GpioSetup(); //initialisation of associated GPIOs


	/////////////////////////////////////////////////////////////////////////////////////
	//TIM2 (1ms IRQ) setup
	/////////////////////////////////////////////////////////////////////////////////////
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //TIM2 clock enable
//
//	//TIM2 time base configuration
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_Prescaler = 0;
////	TIM_TimeBaseStructure.TIM_Period = 26880; //f = 26880000 / 26880 = 1kHz -> 1ms period
//	TIM_TimeBaseStructure.TIM_Period = 26875;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
//	TIM_ARRPreloadConfig(TIM2, ENABLE);
//	TIM_Cmd(TIM2, ENABLE); //TIM2 enable counter


	/////////////////////////////////////////////////////////////////////////////////////
	//TIM3 (Generator PWM out) setup
	/////////////////////////////////////////////////////////////////////////////////////
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //TIM3 clock enable
    __TIM3_CLK_ENABLE();
//    TIM_Cmd(TIM3, ENABLE); //TIM3 enable counter
    SetGeneratorOut(1000, 2688, 1344); //fout = 26880000 / 1000 / 2688 = 10Hz, 2688 / 1344 = 50% duty cycle
    Tim3GpioSetup();

	/////////////////////////////////////////////////////////////////////////////////////
	//TIM4 (GM pulses counting) setup
	/////////////////////////////////////////////////////////////////////////////////////
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //TIM4 clock enable
//	TIM_TIxExternalClockConfig(TIM4, TIM_TIxExternalCLK1Source_TI1, TIM_ICPolarity_Rising, 0x00);
//	TIM_Cmd(TIM4, ENABLE); //TIM4 enable counter
//    Tim4GpioSetup();

	/////////////////////////////////////////////////////////////////////////////////////
    //SPI1 (DUT to AD7794 interface test) setup
	/////////////////////////////////////////////////////////////////////////////////////
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //RCC enabled for SPI1
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
////SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
//    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
//    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
//    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; //fclk = PCLK1/16 = 13.44 MHz/16 = 0.84MHz
//    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//    SPI_InitStructure.SPI_CRCPolynomial = 7;
//    SPI_Init(SPI1, &SPI_InitStructure);
//    SPI_Cmd(SPI1, ENABLE);
//	Spi1GpioSetup();


    /////////////////////////////////////////////////////////////////////////////////////
    //SPI2 (DAC AD5060 interface) setup
	/////////////////////////////////////////////////////////////////////////////////////
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); //RCC enabled for SPI2
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
//    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; //fclk = PCLK1/32 = 13.44 MHz/32 = 0.42MHz
//    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//    SPI_InitStructure.SPI_CRCPolynomial = 7;
//    SPI_Init(SPI2, &SPI_InitStructure);
//    SPI_Cmd(SPI2, ENABLE);
//	Spi2GpioSetup();


	/////////////////////////////////////////////////////////////////////////////////////
	//I2C1 setup - used for HYT and IRT communication
	/////////////////////////////////////////////////////////////////////////////////////
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);//I2C Periph clock enable
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); // GPIO Periph clock enable
//    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//    I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
//    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//    I2C_InitStructure.I2C_ClockSpeed = 100000;
//    I2C_Init(I2C1, &I2C_InitStructure);// Apply I2C configuration after enabling it
//    I2C_Cmd(I2C1, ENABLE); //I2C Peripheral Enable
//    I2c1GpioSetup(); //initialisation of associated GPIOs

	/////////////////////////////////////////////////////////////////////////////////////
	//Interrupt system setup
	/////////////////////////////////////////////////////////////////////////////////////
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); //Configure the NVIC Preemption Priority Bits
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0); //Configure the NVIC Preemption Priority Bits

	//USART2 IRQ setup
#ifdef MASTER_CMD_USART2

    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

//	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

//	NVIC_Init(&NVIC_InitStruct);
//	USART_ITConfig(USART2, UART_IT_RXNE, ENABLE);
//	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE );
#endif

	//&USART6 IRQ setup
#ifdef MASTER_CMD_&USART6
    HAL_NVIC_SetPriority(&USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(&USART6_IRQn);

//	NVIC_InitStruct.NVIC_IRQChannel = &USART6_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStruct.NVIC_IRQCh annelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//	USART_ITConfig(&USART6, UART_IT_RXNE, ENABLE);
#endif

	//TIM2 IRQ setup
//	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);


	/////////////////////////////////////////////////////////////////////////////////////
	//Initialisation of global variables
	/////////////////////////////////////////////////////////////////////////////////////
	ModuleMode = MM_STANDBY;			//current module mode = standby
	CmdReceived = 0;					//no command is ready for decoding
	CmdCounter = 0;						//command counter - number of received bytes = 0
	CmdDecoded = CMD_NONE;				//no command is decoded and to be executed
	SB = 0;								//Status Byte - no error
	MT = 0x14;							//Module Type RADM
	MA = 0x00;							//Module Address
	Sample = 0;							//sample counter - timestamp for measurement
	SamplePacketCount = 0;				//sample packet counter - counts measurements per packet
	RdpAddresses[RDP_THRUST] = 0x00;	//RDP meters RS-485 addresses - thrust meter
	RdpAddresses[RDP_TORQUE] = 0x00;	//RDP meters RS-485 addresses - torque meter
	WriteToGpioPin(LED_B,0);			//setup done

	/////////////////////////////////////////////////////////////////////////////////////
	//welcome alarm
	/////////////////////////////////////////////////////////////////////////////////////
	WriteToGpioPin(ALARM, 1);
	Sleep(50);
	WriteToGpioPin(ALARM, 0);


	/////////////////////////////////////////////////////////////////////////////////////
	//initial self-test
	/////////////////////////////////////////////////////////////////////////////////////
	Sleep(50000); //sleep 5s for ADAM to boot-up
	RadmSelfTest(0); //RPDs not tested at this stage


	/////////////////////////////////////////////////////////////////////////////////////
	//Debug function - TODO to be commented out in the final version!
	/////////////////////////////////////////////////////////////////////////////////////
	DebugTestFunc();

}
//*--------------------------------------------------------------------------------------
//* Function Name         : DebugOnlyFunc
//* Object                : This function is to be used for initial debugging only!
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void DebugTestFunc(void)
{
////unsigned char RelayBuffer[12];
//int Loop;
//unsigned char CmdPacket[10];
//int Status;
////unsigned char DataByte;
//unsigned char AdamDataPacket[50];
//unsigned char APS;
//double MeasVolt;
//double MeasVal;

//ModuleMode = MM_MEAS_INIT;
//Sample = 0x12345678;
//MeasuredThrust = 3541423.354;
//MeasuredTorque = 53.46541;;
//MeasuredCurrent = 12.6983;
//ActualAps = 0x55;

//here
//////Status = AdamReadAdc(&MeasVolt);
//////Status++;


//test = atof("-12.3");
//test++;
//
//double *PointerDouble;
//unsigned char *PointerBytes;
//double DoubleValues[4];
//
//
////data packet
//DoubleValues[0] = MeasTempDut;
//DoubleValues[1] = MeasTempAmbient;
//DoubleValues[1] = MovingAverTempAmbient;
//DoubleValues[2] = SetDutTemp;
//DoubleValues[3] = HeaterPower;
//for(DoubleLoop = 0; DoubleLoop < 4; DoubleLoop++)
//{
//	PointerDouble = &DoubleValues[DoubleLoop];
//	PointerBytes = (unsigned char *)PointerDouble;
//	for(Loop = 0; Loop < 8; Loop++)
//	{
//		ResponsePacket[Loop + (DoubleLoop * 8)] = *PointerBytes;
//		PointerBytes++;
//	}
//}
//
//SendResponseToMaster(ResponsePacket, 32);






//int ReceiveUsartPacket(UART_HandleTypeDef USARTx, unsigned char *DataPacket, unsigned int PacketLength, unsigned int TimeOut)

////synchronised sampling CMD
//CmdPacket[0] = '#';
//CmdPacket[1] = '*';
//CmdPacket[2] = '*';
//AdamSendCommand(&USART6, CmdPacket, 3);


//while(ReadGpioPinStatus(TEST_PIN) == 1);
//while(ReadGpioPinStatus(TEST_PIN) == 0);



//while(2)
//{
//	while(ReadGpioPinStatus(CLK_SYN) == 0);
//	while(ReadGpioPinStatus(CLK_SYN) == 1);
//	ToggleGpioPin(TEST_PIN);

	//synchronised sampling CMD
//	CmdPacket[0] = '#';
//	CmdPacket[1] = '*';
//	CmdPacket[2] = '*';
//	AdamSendCommand(&USART6, CmdPacket, 3);
//
//	Sleep(800);
//

//	CmdPacket[0] = '#';
//	CmdPacket[1] = '0';
//	CmdPacket[2] = '1';
//	CmdPacket[3] = ' ';
//	CmdPacket[4] = 'S';
//	CmdPacket[5] = 'C';
//	CmdPacket[6] = 'A';
//	CmdPacket[7] = 'N';
//	CmdPacket[8] = '\r';
//	CmdPacket[9] = '\n';
//
//
//	WriteToGpioPin(RS485_DIR2,1);
//	USART_ClearFlag(&UART4, USART_FLAG_TC);
//	for(Loop = 0; Loop < 10; Loop++)
//	{
//		USART_SendData(&UART4,CmdPacket[Loop] & 0x1FF);
//		while (USART_GetFlagStatus(&UART4, USART_FLAG_TC) == RESET);
//	}
//	WriteToGpioPin(RS485_DIR2,0);
//
//	//get data
//	Status = ReceiveUsartPacket(&UART4, AdamDataPacket, 9, 10000);
//	Status = RdpReadVal(0x01, &MeasVal);
//Status = 10;
//
//Status = RdpReadVal(0x02, &MeasVal);
//Status = 10;
	//read synchronised data CMD
//	CmdPacket[0] = '#';
//	CmdPacket[1] = '0';
//	CmdPacket[2] = '1';
//	CmdPacket[3] = '4';
//	AdamSendCommand(&USART6, CmdPacket, 3);
//
//	//get data
//	ReceiveUsartPacket(&USART6, AdamDataPacket, 11, 10000);
//
//
//	USART_ClearFlag(USART2, USART_FLAG_TC);
//	for(Loop = 0; Loop < 7; Loop++)
//	{
//		USART_SendData(USART2,AdamDataPacket[Loop+1] & 0x1FF);
//		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
//	}
//
//	USART_SendData(USART2,'\n' & 0x1FF);
//	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);


//	//synchronised sampling CMD
//	CmdPacket[0] = '#';
//	CmdPacket[1] = '*';
//	CmdPacket[2] = '*';
//	AdamSendCommand(&USART6, CmdPacket, 3);

//}

//	while(2)
//	{
////		WriteToGpioPin(PLED_R, 0);
////		WriteToGpioPin(PLED_G, 1);
////		WriteToGpioPin(PLED_G, 0);
////		WriteToGpioPin(PLED_R, 1);
////		APS = ReadAsp();
////		APS = 0;
//
////		WriteToGpioPin(PLED_G, ReadGpioPinStatus(CLK_SYN));
//		WriteToGpioPin(USART2_CTS, ReadGpioPinStatus(USART2_RTS));
//		WriteToGpioPin(PLED_R, ReadGpioPinStatus(USART2_RTS));
//	}
//
//
}
//*--------------------------------------------------------------------------------------
//* Function Name         : Sleep 
//* Object                : Sleep for defined time, to be used for delay longer than 0.1ms
//*                         This delay function is based on SysTick timer - resolution is 100us
//*
//* Input Parameters      : unsigned int HundMicroSec - range 0.1ms to 4294967.296s
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void Sleep(unsigned int HundMicroSec)
{
unsigned int Loop;

	SysTick->VAL   = 0;                       /* Load the SysTick Counter Value */
	SysTick->LOAD  = 0;
	SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk; //enable timer

	for(Loop = 0; Loop < HundMicroSec; Loop++)
	{
		SysTick->LOAD  = 5376;      // set reload register for 100us: 53.76MHz/10kHz = 5376
		while(SysTick->VAL > 10);	//10 ticks to make sure MCU will catch it
		SysTick->VAL   = 0;
	}

	SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk; //disable timer
}
//*--------------------------------------------------------------------------------------
//* Function Name         : Sleep10us
//* Object                : Sleep for defined time, to be used for delay longer than 10us
//*                         This delay function is based on SysTick timer - resolution is 10us
//*
//* Input Parameters      : unsigned int TenMicroSec - range 0.01ms to 429496.7296s
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void Sleep10us(unsigned int TenMicroSec)
{
unsigned int Loop;

	SysTick->VAL   = 0;                       /* Load the SysTick Counter Value */
	SysTick->LOAD  = 0;
	SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk; //enable timer

	for(Loop = 0; Loop < TenMicroSec; Loop++)
	{
		SysTick->LOAD  = 538;      // set reload register for 100us: 53.76MHz/100kHz = 538
		while(SysTick->VAL > 10);	//10 ticks to make sure MCU will catch it
		SysTick->VAL   = 0;
	}
	SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk; //disable timer
}
//*--------------------------------------------------------------------------------------
//* Function Name         : WdtReset
//* Object                : Reset WDT TODO to be implemented
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void WdtReset(void)
{
//TODO
}
//*--------------------------------------------------------------------------------------
//* Function Name         : ReadPortStatus
//* Object                : Reads status of defined GPIO pin
//*
//* Input Parameters      : unsigned int PortPin - defines which pin is to be read
//* Output Parameters     : unsigned int - current status of PIO pin
//*--------------------------------------------------------------------------------------

unsigned int ReadGpioPinStatus(unsigned int PortPin)
{
	return HAL_GPIO_ReadPin(McuGpioPerDef[PortPin],McuGpioPinDef[PortPin]);
//    return(unsigned int)(GPIO_ReadInputDataBit(McuGpioPerDef[PortPin],McuGpioPinDef[PortPin]));
}
//*--------------------------------------------------------------------------------------
//* Function Name         : WriteToGpioPin
//* Object                : Rewrites status of defined GPIO pin
//*
//* Input Parameters      : unsigned int PortPin - defines which pin is to be modified
//*                         bool PortPinStatus - new status of PIO pin
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void WriteToGpioPin(unsigned int PortPin, unsigned int PortPinStatus)
{
    if (PortPinStatus)
//    	GPIO_WriteBit(McuGpioPerDef[PortPin],McuGpioPinDef[PortPin], Bit_SET);
    	HAL_GPIO_WritePin(McuGpioPerDef[PortPin],McuGpioPinDef[PortPin], GPIO_PIN_SET);
    else
//    	GPIO_WriteBit(McuGpioPerDef[PortPin],McuGpioPinDef[PortPin], Bit_RESET);
    HAL_GPIO_WritePin(McuGpioPerDef[PortPin],McuGpioPinDef[PortPin], GPIO_PIN_RESET);
}
//*--------------------------------------------------------------------------------------
//* Function Name         : ToggleGpioPin
//* Object                : Inverts status of defined GPIO pin
//*
//* Input Parameters      : unsigned int PortPin - defines which pin is to be negated
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void ToggleGpioPin(unsigned int PortPin)
{
////	if (GPIO_ReadOutputDataBit(McuGpioPerDef[PortPin],McuGpioPinDef[PortPin]))
	if ( HAL_GPIO_ReadPin(McuGpioPerDef[PortPin],McuGpioPinDef[PortPin]) )
		WriteToGpioPin(PortPin, 0);
    else
        WriteToGpioPin(PortPin, 1);

}
//*--------------------------------------------------------------------------------------
//* Function Name         : UpdateSystemLeds
//* Object                : Update of system LEDs: MCU alive, source positions
//*
//* Input Parameters      : none 
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void UpdateSystemLeds(void)
{
int PANEL_LED;

    //SB controls the colour of the panel LED
//	if(ModuleMode == MM_STANDBY)
//    {
//    	//RDP errors to be ignored
//		if((SB & 0x0F) == 0x00)
//			PANEL_LED = PLED_G;
//		else
//			PANEL_LED = PLED_R;
//    }
//	else
//	{
	if(SB == 0x00)
	{
		PANEL_LED = PLED_G;
		WriteToGpioPin(PLED_R, 0);
	}
	else
	{
		PANEL_LED = PLED_R;
		WriteToGpioPin(PLED_G, 0);
	}
//	}

    //blink MCU alive/error LEDs
	SysLedCounter++;
	if(SysLedCounter >= SYS_LED_CALLS)
    {
    	ToggleGpioPin(LED_G);
    	ToggleGpioPin(PANEL_LED);
    	SysLedCounter = 0;
    }

}
//*--------------------------------------------------------------------------------------
//* Function Name         : Get8BitCrc
//* Object                : Calculates 8bit CRC from original CRC and DataByte
//*
//* Input Parameters      : unsigned char DataByte - data to be CRCed
//*							unsigned char OldCrc - initial CRC
//* Output Parameters     : unsigned char CRC - checksum for DataByte
//*--------------------------------------------------------------------------------------

unsigned char Get8BitCrc(unsigned char DataByte, unsigned char OldCrc)
{
unsigned char CrcTable[256] =
{
	0x00,  0x5E,  0xBC,  0xE2,  0x61,  0x3F,  0xDD,  0x83,  0xC2,  0x9C,  0x7E,  0x20,  0xA3,  0xFD,  0x1F,  0x41,
	0x9D,  0xC3,  0x21,  0x7F,  0xFC,  0xA2,  0x40,  0x1E,  0x5F,  0x01,  0xE3,  0xBD,  0x3E,  0x60,  0x82,  0xDC,
	0x23,  0x7D,  0x9F,  0xC1,  0x42,  0x1C,  0xFE,  0xA0,  0xE1,  0xBF,  0x5D,  0x03,  0x80,  0xDE,  0x3C,  0x62,
	0xBE,  0xE0,  0x02,  0x5C,  0xDF,  0x81,  0x63,  0x3D,  0x7C,  0x22,  0xC0,  0x9E,  0x1D,  0x43,  0xA1,  0xFF,
	0x46,  0x18,  0xFA,  0xA4,  0x27,  0x79,  0x9B,  0xC5,  0x84,  0xDA,  0x38,  0x66,  0xE5,  0xBB,  0x59,  0x07,
	0xDB,  0x85,  0x67,  0x39,  0xBA,  0xE4,  0x06,  0x58,  0x19,  0x47,  0xA5,  0xFB,  0x78,  0x26,  0xC4,  0x9A,
	0x65,  0x3B,  0xD9,  0x87,  0x04,  0x5A,  0xB8,  0xE6,  0xA7,  0xF9,  0x1B,  0x45,  0xC6,  0x98,  0x7A,  0x24,
	0xF8,  0xA6,  0x44,  0x1A,  0x99,  0xC7,  0x25,  0x7B,  0x3A,  0x64,  0x86,  0xD8,  0x5B,  0x05,  0xE7,  0xB9,
	0x8C,  0xD2,  0x30,  0x6E,  0xED,  0xB3,  0x51,  0x0F,  0x4E,  0x10,  0xF2,  0xAC,  0x2F,  0x71,  0x93,  0xCD,
	0x11,  0x4F,  0xAD,  0xF3,  0x70,  0x2E,  0xCC,  0x92,  0xD3,  0x8D,  0x6F,  0x31,  0xB2,  0xEC,  0x0E,  0x50,
	0xAF,  0xF1,  0x13,  0x4D,  0xCE,  0x90,  0x72,  0x2C,  0x6D,  0x33,  0xD1,  0x8F,  0x0C,  0x52,  0xB0,  0xEE,
	0x32,  0x6C,  0x8E,  0xD0,  0x53,  0x0D,  0xEF,  0xB1,  0xF0,  0xAE,  0x4C,  0x12,  0x91,  0xCF,  0x2D,  0x73,
	0xCA,  0x94,  0x76,  0x28,  0xAB,  0xF5,  0x17,  0x49,  0x08,  0x56,  0xB4,  0xEA,  0x69,  0x37,  0xD5,  0x8B,
	0x57,  0x09,  0xEB,  0xB5,  0x36,  0x68,  0x8A,  0xD4,  0x95,  0xCB,  0x29,  0x77,  0xF4,  0xAA,  0x48,  0x16,
	0xE9,  0xB7,  0x55,  0x0B,  0x88,  0xD6,  0x34,  0x6A,  0x2B,  0x75,  0x97,  0xC9,  0x4A,  0x14,  0xF6,  0xA8,
	0x74,  0x2A,  0xC8,  0x96,  0x15,  0x4B,  0xA9,  0xF7,  0xB6,  0xE8,  0x0A,  0x54,  0xD7,  0x89,  0x6B,  0x35,
};
	return(CrcTable[(OldCrc ^ DataByte)]);
}
//*--------------------------------------------------------------------------------------
//* Function Name         : SendResponseToMaster
//* Object                : Send module response packet including CRC, no header nor end 
//*							character to be sent
//*
//* Input Parameters      : unsigned char *DataPacket - pointer to input data array
//*							unsigned int Length - total length of input data array
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void SendResponseToMaster(unsigned char *DataPacket, unsigned int Length)
{
unsigned int Loop;
unsigned char CrcByte;

#ifdef MASTER_CMD_USART2
	CrcByte = CRC_INIT;
	//send data
	__HAL_UART_CLEAR_FLAG(&huart2,UART_FLAG_TC);
//	USART_ClearITPendingBit(USART2, UART_FLAG_TC);
	for(Loop = 0; Loop < Length; Loop++)
	{
//		USART_SendData(USART2,DataPacket[Loop] & 0x1FF);
		HAL_UART_Transmit(&huart2, DataPacket[Loop] & 0x1FF, sizeof(DataPacket[Loop] & 0x1FF), HAL_MAX_DELAY);
//		while(USART_GetITStatus(USART2,UART_IT_TC) == RESET );
		while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC) == RESET);
		CrcByte = Get8BitCrc(DataPacket[Loop], CrcByte);
	}
	//send CRC
//	USART_SendData(USART2,CrcByte & 0x1FF);
	HAL_UART_Transmit(&huart2, CrcByte & 0x1FF, sizeof(CrcByte & 0x1FF), HAL_MAX_DELAY);
//	while (USART_GetFlagStatus(USART2, UART_FLAG_TC) == RESET);
	while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC) == RESET);

#endif

#ifdef MASTER_CMD_&USART6
//	CrcByte = CRC_INIT;
//	//send data
//	USART_ClearFlag(&USART6, USART_FLAG_TC);
//	for(Loop = 0; Loop < Length; Loop++)
//	{
//		USART_SendData(&USART6,DataPacket[Loop] & 0x1FF);
//		while (USART_GetFlagStatus(&USART6, USART_FLAG_TC) == RESET);
//		CrcByte = Get8BitCrc(DataPacket[Loop], CrcByte);
//	}
//	//send CRC
//	USART_SendData(&USART6,CrcByte & 0x1FF);
//	while (USART_GetFlagStatus(&USART6, USART_FLAG_TC) == RESET);
#endif
}
//*--------------------------------------------------------------------------------------
//* Function Name         : DecodeCmd
//* Object                : Decode master (PC) command
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void DecodeCmd(void)
{
unsigned char CrcByte;
unsigned char Loop;

//Command characters table - the higher position the faster response
unsigned char CmdChars[CMD_LAST - 1][2] =
{
	{'P',	'G'},	//Ping module
	{'M',	'S'},	//CMD Start measurement
	{'M',	'E'},	//CMD End measurement
	{'S',	'T'},	//CMD Self-test
	{'R',	'S'},	//CMD RADM setup
	{'A',	'C'}	//CMD alarm control
};

	//until decoded, no cmd received
	CmdDecoded = CMD_NONE;

	//double check the package length
	if(CmdCounter != (CmdPacket[3] + 6))
		return;

	//double check the package header
	if(CmdPacket[0] != '!')
		return;

	//double check the module type
	if(CmdPacket[1] != MT)
		return;

	//double check the module address
	if(CmdPacket[2] != MA)
		return;

	//compare command characters
	for(Loop = 0; Loop < (CMD_LAST - 1); Loop++)
	{
		if((CmdPacket[4] == CmdChars[Loop][0]) && (CmdPacket[5] == CmdChars[Loop][1]))
		{
			CmdDecoded = Loop + 1;
			break;
		}
	}

	//Check packet checksum
	CrcByte = CRC_INIT;
	for(Loop = 1;Loop < (CmdCounter - 2); Loop++) //header is not a part of checksum check
		CrcByte = Get8BitCrc(CmdPacket[Loop], CrcByte);

	if(CmdPacket[CmdCounter - 2] != CrcByte)
	{
		CmdDecoded = CMD_NONE;
		return;
	}

	//Packet ending char check
	if(CmdPacket[CmdCounter - 1] != '#')
	{
		CmdDecoded = CMD_NONE;
		return;
	}
}
//*--------------------------------------------------------------------------------------
//* Function Name         : SendSlaveCmd
//* Object                : Send Uart command to a slave module
//*
//* Input Parameters      : UART_HandleTypeDef USARTx -  where x can be 1, 2, 3, 4, 5 or 6 to select the USART or UART
//*							unsigned char SMT - Slave Module Type
//*							unsigned char SMA - Slave Module Address
//*							unsigned int Cmd - enum SLAVE_COMMANDS
//*							unsigned char *DataPacket - pointer to input packet of parameters
//*							unsigned int PacketSize - length of input packet of parameters
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void SendSlaveCmd(UART_HandleTypeDef USARTx, unsigned char SMT, unsigned char SMA, unsigned int Cmd, unsigned char *DataPacket, unsigned int PacketSize)
{
unsigned char CrcByte;
unsigned char Loop;
unsigned char CmdBuffer[CMD_PACK_MAX];
unsigned char CmdData[SCMD_LAST][2] =
{
	{'P',	'G'},		//Ping module
	{'W',	'R'},		//RDM Write to all relay driver data registers
	{'S',	'T'}		//RadEx SPI single byte test
};

	//Header
	CmdBuffer[0] = '!';

	//Module Type
	CmdBuffer[1] = SMT;

	//Module Address
	CmdBuffer[2] = SMA;

	//Size of CMD
	CmdBuffer[3] = 0x02 + PacketSize;

	//CMD char 1
	CmdBuffer[4] = CmdData[Cmd][0];

	//CMD char 2
	CmdBuffer[5] = CmdData[Cmd][1];

	//CMD data
	for(Loop = 0; Loop < PacketSize; Loop++)
	{
		CmdBuffer[Loop + 6] = DataPacket[Loop];
	}

	//Generate packet checksum
	CrcByte = CRC_INIT;
	for(Loop = 1;Loop < (PacketSize + 6); Loop++) //header is not a part of checksum check
		CrcByte = Get8BitCrc(CmdBuffer[Loop], CrcByte);

	CmdBuffer[PacketSize + 6] = CrcByte;

	//Package end character
	CmdBuffer[PacketSize + 7] = '#';

	//Send data
	USART_ClearFlag(USARTx, UART_FLAG_TC);
	for(Loop = 0; Loop < (PacketSize + 8); Loop++)
	{
		USART_SendData(USARTx,CmdBuffer[Loop] & 0x1FF);
//		while (USART_GetFlagStatus(USARTx, UART_FLAG_TC) == RESET);
		while(__HAL_UART_GET_FLAG(&USARTx,UART_FLAG_TC) == RESET);
	}

}
//*--------------------------------------------------------------------------------------
//* Function Name         : ReceiveUsartPacket
//* Object                : Receives USART Packet or timeout
//*
//* Input Parameters      : UART_HandleTypeDef USARTx -  where x can be 1, 2, 3, 4, 5 or 6 to select the USART or UART
//*							unsigned char *DataPacket - pointer to data packet
//*							unsigned int PacketLength - length packet to be received
//*							unsigned int TimeOut - number of 10us waiting periods to give up
//* Output Parameters     : returns status (0 - OK, other value - ERROR)
//*--------------------------------------------------------------------------------------

int ReceiveUsartPacket(UART_HandleTypeDef USARTx, unsigned char *DataPacket, unsigned int PacketLength, unsigned int TimeOut)
{
unsigned int Sleep10UsCounter;
unsigned int Loop;

	for(Loop = 0; Loop < PacketLength; Loop++)
	{
		//waiting for byte on USART or timeout
		Sleep10UsCounter  = 0;
//		while(USART_GetFlagStatus(USARTx, UART_FLAG_RXNE) == RESET)
		while(__HAL_UART_GET_FLAG(&USARTx,UART_FLAG_TC) == RESET)
		{
			Sleep10us(1);
			Sleep10UsCounter++;
			if(Sleep10UsCounter >= TimeOut)
				break;
		}

//		if(USART_GetFlagStatus(USARTx, UART_FLAG_RXNE) == SET)
		if(__HAL_UART_GET_FLAG(&USARTx,UART_FLAG_RXNE) == SET)
			DataPacket[Loop] =( unsigned char)USART_ReceiveData(USARTx); //byte received
		else
			return(-1);	//timeout error
	}

	return(0); //OK
}
//*--------------------------------------------------------------------------------------
//* Function Name         : ReceiveUsartPacketCrtCheck
//* Object                : Receives USART Packet (or timeout) and performs CRT check
//*
//* Input Parameters      : UART_HandleTypeDef USARTx -  where x can be 1, 2, 3, 4, 5 or 6 to select the USART or UART
//*							unsigned char *DataPacket - pointer to data packet
//*							unsigned int PacketLength - length packet to be received (including CRT)
//*							unsigned int TimeOut - number of 10us waiting periods to give up
//* Output Parameters     : returns status (0 - OK, other value - ERROR)
//*--------------------------------------------------------------------------------------

int ReceiveUsartPacketCrtCheck(UART_HandleTypeDef USARTx, unsigned char *DataPacket, unsigned int PacketLength, unsigned int TimeOut)
{
unsigned int Sleep10UsCounter;
unsigned int Loop;
unsigned char CrcByte;

	//receive data packet
	for(Loop = 0; Loop < PacketLength; Loop++)
	{
		//waiting for byte on USART or timeout
		Sleep10UsCounter  = 0;
//		while(USART_GetFlagStatus(&USARTx, UART_FLAG_RXNE) == RESET)
		while(__HAL_UART_GET_FLAG(&USARTx,UART_FLAG_RXNE) == RESET)
		{
			Sleep10us(1);
			Sleep10UsCounter++;
			if(Sleep10UsCounter >= TimeOut)
				break;
		}

//		if(USART_GetFlagStatus(&USARTx, UART_FLAG_RXNE) == SET)
		if(__HAL_UART_GET_FLAG(&USARTx,UART_FLAG_RXNE) == SET)
			DataPacket[Loop] =( unsigned char)USART_ReceiveData(USARTx); //byte received
		else
			return(-1);	//timeout error
	}

	//check data packet for CRT
	CrcByte = CRC_INIT;
	for(Loop = 0;Loop < (PacketLength - 1); Loop++)
		CrcByte = Get8BitCrc(DataPacket[Loop], CrcByte);

	if(DataPacket[PacketLength - 1] != CrcByte)
		return(-2);	//CRC error

	return(0); //OK
}
//*--------------------------------------------------------------------------------------
//* Function Name         : SetGeneratorOut
//* Object                : Set generator (TIM3) square output
//*
//* Input Parameters      : uint16_t Prescaler - Counter prescaler (1 = no division)
//*							uint16_t Period - pulse period (2 to 65535)
//*							uint16_t Pulse - Clock pulse width (1 to 65535)
//*							fout = CLK/Prescaler/Period
//*							pulse width = Period/Pulse
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------


void SetGeneratorOut(uint16_t Prescaler, uint16_t Period, uint16_t Pulse)
{
//TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_ClockConfigTypeDef sClockSourceConfig;
TIM_MasterConfigTypeDef TIM_TimeBaseStructure;
TIM_OC_InitTypeDef  TIM_OCInitStructure;

	//Time base configuration
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler - 1;
//	TIM_TimeBaseStructure.TIM_Period = Period - 1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	htim3.Instance = TIM3;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.Prescaler = Prescaler -1; // defined in function para
	htim3.Init.Period = Period  -1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
	Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
	Error_Handler();
	}
	TIM_TimeBaseStructure.MasterOutputTrigger = TIM_TRGO_RESET;
	TIM_TimeBaseStructure.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &TIM_TimeBaseStructure) != HAL_OK)
	{
	Error_Handler();
	}
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Init(&htim3);

	//PWM1 Mode configuration: Channel3
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse = Pulse;
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
//
//	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
//	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
	TIM_OCInitStructure.Pulse = Pulse;
	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
	TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &TIM_OCInitStructure, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //STARt PWM generation
}
//*--------------------------------------------------------------------------------------
//* Function Name         : SetSpiChipEnable
//* Object                : Enable/disable SPI slave device
//*
//* Input Parameters      : int SpiDevice - device to control (enum SPI_DEVICES)
//*							int NewState - state to be used
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

//void SetSpiChipEnable(int SpiDevice, int NewState)
//{
//
//	switch(SpiDevice)
//	{
//		case SPI_DEV_ADS1211:
//		if(NewState == 1)
//			WriteToGpioPin(ADC_CS,0); //Chip enabled
//
//		if(NewState == 0)
//			WriteToGpioPin(ADC_CS,1); //Chip enabled
//		break;
//		//*******************************************************************************
//	}
//}
//*--------------------------------------------------------------------------------------
//* Function Name         : InitUart4
//* Object                : Initial settings of USART, can be used to change BaudRate
//*							This way is used as there no way to read back USART settings
//*
//* Input Parameters      : UART_HandleTypeDef USARTx -  where x can be 1, 2, 3, 4, 5 or 6 to select the USART or UART
//*							unsigned int BaudRate - BaudRate to be selected (9600 = 9600Bd)
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void InitUsart(UART_HandleTypeDef USARTx, unsigned int BaudRate)
{
//USART_InitTypeDef USART_InitStructure;
//
//	USART_InitStructure.USART_BaudRate = BaudRate;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	USART_Init(USARTx, &USART_InitStructure);


UART_HandleTypeDef huart;

	huart.Instance = &USARTx;
	huart.Init.BaudRate = BaudRate;
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	if (HAL_UART_Init(&huart) != HAL_OK)
	{
		Error_Handler();
	}

	//clear flags
//	USART_ClearFlag(&USARTx, UART_FLAG_RXNE);
//	USART_ClearFlag(&USARTx, UART_FLAG_TXE);
//	USART_ClearFlag(&USARTx, UART_FLAG_TC);
	__HAL_UART_CLEAR_FLAG(&USARTx,UART_FLAG_RXNE);
	__HAL_UART_CLEAR_FLAG(&USARTx,UART_FLAG_TXE);
	__HAL_UART_CLEAR_FLAG(&USARTx,UART_FLAG_TC);
}
//*--------------------------------------------------------------------------------------
//* Function Name         : ReadAsp
//* Object                : Read Actuator position switch inputs
//*
//* Input Parameters      : none
//* Output Parameters     : unsigned char switches - actual status of ASPs
//*--------------------------------------------------------------------------------------

unsigned char ReadAsp(void)
{
unsigned int Loop;
unsigned char Mask;
unsigned char APS;

	Mask = 0x01;
	APS = 0x00;
	for(Loop = 0; Loop < NR_ACT_PS; Loop++)
	{
		if(ReadGpioPinStatus(ActuatorPosSwitches[Loop]) == 1)
			APS &= ~Mask;
		else
			APS |= Mask;
		Mask <<= 1;
	}
	return(APS);
}
//*--------------------------------------------------------------------------------------
//* Function Name         : AdamSendCommand
//* Object                : Send command to ADAM, including the checksum
//*
//* Input Parameters      : none

//* Input Parameters      : UART_HandleTypeDef USARTx -  where x can be 1, 2, 3, 4, 5 or 6 to select the USART or UART
//*							unsigned char *CmdPacket - pointer to command packet (excluding checksum and end char)
//*							unsigned int CmdSize - length of the command packet (excluding checksum and end char)
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void AdamSendCommand(UART_HandleTypeDef USARTx, unsigned char *CmdPacket, unsigned int CmdSize)
{
unsigned int Loop;
unsigned char CheckSum;
unsigned char CheckSumLowChar;
unsigned char CheckSumHighChar;

	//switch to TX mode
	WriteToGpioPin(RS485_DIR1, 1);

	//send the CMD packet
	CheckSum = 0;
//	USART_ClearFlag(USARTx, UART_FLAG_TC);
	__HAL_UART_CLEAR_FLAG(&USARTx,UART_FLAG_TC);
	for(Loop = 0; Loop < CmdSize; Loop++)
	{
		USART_SendData(USARTx,CmdPacket[Loop] & 0x1FF);
//		while (USART_GetFlagStatus(USARTx, UART_FLAG_TC) == RESET);
		while(__HAL_UART_GET_FLAG(&USARTx,UART_FLAG_TC) == RESET);
		CheckSum += CmdPacket[Loop];
	}

	//calculate the CheckSum
	CheckSum %= 0x100;
	CheckSumHighChar = CheckSum;
	CheckSumHighChar >>= 4;
	if(CheckSumHighChar < 10)
		CheckSumHighChar += 0x30;
	else
		CheckSumHighChar += 0x37;

	CheckSumLowChar = CheckSum;
	CheckSumLowChar &= 0x0F;
	if(CheckSumLowChar < 10)
		CheckSumLowChar += 0x30;
	else
		CheckSumLowChar += 0x37;

	//send the CheckSum
	USART_SendData(USARTx,CheckSumHighChar & 0x1FF);
	while (USART_GetFlagStatus(USARTx, UART_FLAG_TC) == RESET);
	USART_SendData(USARTx,CheckSumLowChar & 0x1FF);
	while (USART_GetFlagStatus(USARTx, UART_FLAG_TC) == RESET);

	//send the end char
	USART_SendData(USARTx,0x0D);
	while (USART_GetFlagStatus(USARTx, UART_FLAG_TC) == RESET);

	//switch back to RX mode
	WriteToGpioPin(RS485_DIR1, 0);
}
//*--------------------------------------------------------------------------------------
//* Function Name         : AdamReceiveDataCrtCheck
//* Object                : Receives USART Packet (or timeout) and performs CRT check
//*
//* Input Parameters      : UART_HandleTypeDef USARTx -  where x can be 1, 2, 3, 4, 5 or 6 to select the USART or UART
//*							unsigned char *DataPacket - pointer to data packet
//*							unsigned int PacketLength - length of packet to be received (including CRT and end char)
//*							unsigned int TimeOut - number of 10us waiting periods to give up
//* Output Parameters     : returns status (0 - OK, other value - ERROR)
//*--------------------------------------------------------------------------------------

int AdamReceiveDataCrtCheck(UART_HandleTypeDef USARTx, unsigned char *DataPacket, unsigned int PacketLength, unsigned int TimeOut)
{
unsigned int Sleep10UsCounter;
unsigned int Loop;
unsigned char CheckSum;
unsigned char CheckSumLowChar;
unsigned char CheckSumHighChar;

	//receive data packet
	CheckSum = 0;
	for(Loop = 0; Loop < PacketLength; Loop++)
	{
		//waiting for byte on USART or timeout
		Sleep10UsCounter  = 0;
//		while(USART_GetFlagStatus(USARTx, UART_FLAG_RXNE) == RESET)
		while(__HAL_UART_GET_FLAG(&USARTx,UART_FLAG_RXNE) == RESET)
		{
			Sleep10us(1);
			Sleep10UsCounter++;
			if(Sleep10UsCounter >= TimeOut)
				break;
		}

//		if(USART_GetFlagStatus(USARTx, UART_FLAG_RXNE) == SET)
		if(__HAL_UART_GET_FLAG(&USARTx,UART_FLAG_RXNE) == SET)
			DataPacket[Loop] =( unsigned char)USART_ReceiveData(USARTx); //byte received
		else
			return(-1);	//timeout error

		if(Loop < (PacketLength - 3))
			CheckSum += DataPacket[Loop];
	}

	//calculate the CheckSum
	CheckSum %= 0x100;
	CheckSumHighChar = CheckSum;
	CheckSumHighChar >>= 4;
	if(CheckSumHighChar < 10)
		CheckSumHighChar += 0x30;
	else
		CheckSumHighChar += 0x37;

	CheckSumLowChar = CheckSum;
	CheckSumLowChar &= 0x0F;
	if(CheckSumLowChar < 10)
		CheckSumLowChar += 0x30;
	else
		CheckSumLowChar += 0x37;


	//check data packet for CheckSum
	if((DataPacket[PacketLength - 3] != CheckSumHighChar)||(DataPacket[PacketLength - 2] != CheckSumLowChar))
		return(-2); //CheckSum error

	//check data packet for end char
	if(DataPacket[PacketLength - 1] != '\r')
		return(-3); //end char error

	return(0); //OK
}
//*--------------------------------------------------------------------------------------
//* Function Name         : RadmSelfTest
//* Object                : Self test of module, result stored in SB (SB_OK_ macros)
//* Input Parameters      : none

//* Input Parameters      : int TestRdps - RDP test enabled [1 = yes, 0 = no]
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void RadmSelfTest(int TestRdps)
{
unsigned int Timeout;
unsigned int FirstEdge;
unsigned char CmdPacket[10];
int Status;
unsigned char AdamDataPacket[15];
unsigned char AdamTestPacket[7] = {"!014012"};
unsigned int Loop;



	//initial SB = all OK
	SB = 0x00;

	/////////////////////////////////////////////////////////////////
	//test synchronisation clock
	/////////////////////////////////////////////////////////////////
	//1. waiting for the edge
	Timeout = 0;
	if(ReadGpioPinStatus(CLK_SYN) == 0)
	{
		FirstEdge = 0;
		while(ReadGpioPinStatus(CLK_SYN) == 0)
		{
			Sleep10us(1);
			Timeout++;

			if(Timeout > 10000)
				break;
		}
	}
	else
	{
		FirstEdge = 1;
		while(ReadGpioPinStatus(CLK_SYN) == 1)
		{
			Sleep10us(1);
			Timeout++;

			if(Timeout > 10000)
				break;
		}

	}

	//2. some edge appeared - measure the time until the another (the same) one appears
	if(Timeout < 10000)
	{
		Timeout = 0;
		if(FirstEdge == 0)
		{
			//waiting for rising edge
			while(ReadGpioPinStatus(CLK_SYN) == 1)
			{
				Sleep(1);
				Timeout++;
			}
			while(ReadGpioPinStatus(CLK_SYN) == 0)
			{
				Sleep(1);
				Timeout++;
			}
		}
		else
		{
			//waiting for falling edge
			while(ReadGpioPinStatus(CLK_SYN) == 0)
			{
				Sleep(1);
				Timeout++;
			}
			while(ReadGpioPinStatus(CLK_SYN) == 1)
			{
				Sleep(1);
				Timeout++;
			}
		}

		//check the timeout within 10% limits (nominal value is 972)
		if((Timeout > 845) && (Timeout < 1069))
			SB &= SB_OK_CLK_SYNC; //OK
		else
			SB |= ~SB_OK_CLK_SYNC; //error
	}
	else
	{
		SB |= ~SB_OK_CLK_SYNC; //error
	}


	/////////////////////////////////////////////////////////////////
	//test the presence of the ADAM module
	/////////////////////////////////////////////////////////////////
	//Read Module Name command
	CmdPacket[0] = '$';
	CmdPacket[1] = '0';
	CmdPacket[2] = '1';
	CmdPacket[3] = 'M';
	HAL_UART_Transmit(&huart6, CmdPacket, sizeof(CmdPacket), HAL_MAX_DELAY);
//	AdamSendCommand(&USART6, CmdPacket, 4);

	//get data
	Status = HAL_UART_Receive(&huart6, AdamDataPacket, sizeof(AdamDataPacket), HAL_MAX_DELAY);
//	Status = AdamReceiveDataCrtCheck(&USART6, AdamDataPacket, 10, 10000);

	//test data
	SB |= ~SB_OK_ADAM; //error
	if(Status == 0)
	{
		for(Loop = 0;Loop < 7;Loop++)
		{
			if(AdamTestPacket[Loop] !=	AdamDataPacket[Loop])
				break;
		}
		if(Loop == 7)
			SB &= SB_OK_ADAM; //OK
	}


	/////////////////////////////////////////////////////////////////
	//test the presence of the RDP meters
	/////////////////////////////////////////////////////////////////
	if(TestRdps == 1)
	{
		//test top RDP - thrust
		Status = RdpTest(RdpAddresses[RDP_THRUST]);
		if(Status)
			SB |= ~SB_OK_RDP_THRUST; //error
		else
			SB &= SB_OK_RDP_THRUST; //OK

		Sleep(10); //sleep 1ms to avoid bus collisions

		//test bottom RDP - torque
		Status = RdpTest(RdpAddresses[RDP_TORQUE]);
		if(Status)
			SB |= ~SB_OK_RDP_TORQUE; //error
		else
			SB &= SB_OK_RDP_TORQUE; //OK
	}


	SB = SB; //for debug only

}
//*--------------------------------------------------------------------------------------
//* Function Name         : SendMeasurementResults
//* Object                : Send measurements results (global variables) to the master
//* Input Parameters      : none

//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

void SendMeasurementResults(void)
{
double *PointerDouble;
unsigned char *PointerBytes;
double DoubleValues[3];
unsigned char ResponsePacket[30*RADM_DATA_PACK];
unsigned int SampleCopy;
unsigned int Loop;
unsigned int MeasLoop;
unsigned int DoubleLoop;


	//multiple measurements per packet
	for(MeasLoop = 0; MeasLoop < RADM_DATA_PACK; MeasLoop++)
	{
		//create response data packet: sample
		SampleCopy = SampleArray[MeasLoop];
		for(Loop = 4; Loop > 0; Loop--)
		{
			ResponsePacket[(Loop - 1)+(29 * MeasLoop)] = (unsigned char)(SampleCopy & 0x000000FF);
			SampleCopy >>= 8;
		}

		//create response data packet: APS
		ResponsePacket[4+(29 * MeasLoop)] = ActualAps[MeasLoop];

		//create response data packet: double values
		DoubleValues[0] = MeasuredThrust[MeasLoop];
		DoubleValues[1] = MeasuredTorque[MeasLoop];
		DoubleValues[2] = MeasuredCurrent[MeasLoop];

		for(DoubleLoop = 0; DoubleLoop < 3; DoubleLoop++)
		{
			PointerDouble = &DoubleValues[DoubleLoop];
			PointerBytes = (unsigned char *)PointerDouble;
			for(Loop = 0; Loop < 8; Loop++)
			{
				ResponsePacket[Loop + (DoubleLoop * 8) + 5 + (29 * MeasLoop)] = *PointerBytes;
				PointerBytes++;
			}
		}
	}

	SendResponseToMaster(ResponsePacket, (29*RADM_DATA_PACK));

}
//*--------------------------------------------------------------------------------------
//* Function Name         : AdamReadInput
//* Object                : Reads input value from ADAM using Analog Data In command
//*
//* Input Parameters      : double *MeasVolt - pointer to the result
//* Output Parameters     : returns status (0 - OK, other value - ERROR)
//*--------------------------------------------------------------------------------------

int AdamReadAdc(double *MeasVolt)
{
unsigned char CmdPacket[10];
int Status;
unsigned char AdamDataPacket[15];
unsigned int Loop;
char NumberStr[10];


	//Analog Data In command
	CmdPacket[0] = '#';
	CmdPacket[1] = '0';
	CmdPacket[2] = '1';
//	AdamSendCommand(&USART6, CmdPacket, 3);
	HAL_UART_Transmit(&huart6, CmdPacket, sizeof(CmdPacket), HAL_MAX_DELAY);

	//get data

//	Status = AdamReceiveDataCrtCheck(&USART6, AdamDataPacket, 11, 10000);
	Status = HAL_UART_Receive(&huart6, AdamDataPacket, sizeof(AdamDataPacket), HAL_MAX_DELAY);
	if(Status)
		return(-1);

	//data OK, time to parse the value
	for(Loop = 0; Loop < sizeof(NumberStr); Loop++)
		NumberStr[Loop] = 0x00;
	for(Loop = 0; Loop < 7; Loop++)
		NumberStr[Loop] = (char)AdamDataPacket[Loop + 1];
	*MeasVolt = atof(NumberStr);

	return(0);
}
//*--------------------------------------------------------------------------------------
//* Function Name         : RdpReadVal
//* Object                : Reads display value from RDP using SCAN command
//*
//* Input Parameters      : unsigned char RdpAddress - RS485 address
//*							double *MeasVal - pointer to the result
//* Output Parameters     : returns status (0 - OK, other value - ERROR)
//*--------------------------------------------------------------------------------------

int RdpReadVal(unsigned char RdpAddress, double *MeasVal)
{
unsigned char CmdPacket[10];
int Status;
unsigned char RdpDataPacket[10];
unsigned int Loop;
char NumberStr[10];
char AddressHighChar;
char AddressLowChar;

	//get the address characters
	AddressHighChar = RdpAddress;
	AddressHighChar >>= 4;
	if(AddressHighChar < 10)
		AddressHighChar += 0x30;
	else
		AddressHighChar += 0x37;

	AddressLowChar = RdpAddress;
	AddressLowChar &= 0x0F;
	if(AddressLowChar < 10)
		AddressLowChar += 0x30;
	else
		AddressLowChar += 0x37;

	//prepare the command string
	CmdPacket[0] = '#';
	CmdPacket[1] = AddressHighChar;
	CmdPacket[2] = AddressLowChar;
	CmdPacket[3] = ' ';
	CmdPacket[4] = 'S';
	CmdPacket[5] = 'C';
	CmdPacket[6] = 'A';
	CmdPacket[7] = 'N';
	CmdPacket[8] = '\r';
	CmdPacket[9] = '\n';

	//send the command
	WriteToGpioPin(RS485_DIR2,1);
//	USART_ClearFlag(&UART4, UART_FLAG_TC);
	__HAL_UART_CLEAR_FLAG(&huart4,UART_FLAG_TC);
	for(Loop = 0; Loop < 10; Loop++)
	{
//		USART_SendData(&UART4,CmdPacket[Loop] & 0x1FF);
		HAL_UART_Transmit(&huart4, CmdPacket[Loop] & 0x1FF , sizeof(CmdPacket[Loop] & 0x1FF), HAL_MAX_DELAY);
//		while (USART_GetFlagStatus(&UART4, UART_FLAG_TC) == RESET);
		while(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_TC) == RESET);
	}
	WriteToGpioPin(RS485_DIR2,0);

	//receive data
	Status = HAL_UART_Receive(&huart4, RdpDataPacket, sizeof(RdpDataPacket), HAL_MAX_DELAY);
//	Status = ReceiveUsartPacket(&UART4, RdpDataPacket, 9, 10000);

	if(Status)
		return(-1);

	//data OK, time to parse the value
	for(Loop = 0; Loop < sizeof(NumberStr); Loop++)
		NumberStr[Loop] = 0x00;
	for(Loop = 0; Loop < 7; Loop++)
		NumberStr[Loop] = (char)RdpDataPacket[Loop];
	*MeasVal = atof(NumberStr);

	return(0);
}
//*--------------------------------------------------------------------------------------
//* Function Name         : RdpTest
//* Object                : Test RDP presence using the sys command
//*
//* Input Parameters      : unsigned char RdpAddress - RS485 address
//* Output Parameters     : returns status (0 - OK, other value - ERROR)
//*--------------------------------------------------------------------------------------

int RdpTest(unsigned char RdpAddress)
{
unsigned char CmdPacket[10];
int Status;
unsigned char RdpDataPacket[20];
unsigned int Loop;
//char NumberStr[10];
char AddressHighChar;
char AddressLowChar;			  //01234567890123456789
unsigned char RdpTestPacket[19] = {"E725 Version 1.10\r\n"};

	//get the address characters
	AddressHighChar = RdpAddress;
	AddressHighChar >>= 4;
	if(AddressHighChar < 10)
		AddressHighChar += 0x30;
	else
		AddressHighChar += 0x37;

	AddressLowChar = RdpAddress;
	AddressLowChar &= 0x0F;
	if(AddressLowChar < 10)
		AddressLowChar += 0x30;
	else
		AddressLowChar += 0x37;

	//prepare the command string
	CmdPacket[0] = '#';
	CmdPacket[1] = AddressHighChar;
	CmdPacket[2] = AddressLowChar;
	CmdPacket[3] = ' ';
	CmdPacket[4] = 'S';
	CmdPacket[5] = 'Y';
	CmdPacket[6] = 'S';
	CmdPacket[7] = '\r';
	CmdPacket[8] = '\n';

	//send the command
	WriteToGpioPin(RS485_DIR2,1);
//	USART_ClearFlag(&UART4, UART_FLAG_TC);
	__HAL_UART_CLEAR_FLAG(&huart4,UART_FLAG_TC);
	for(Loop = 0; Loop < 9; Loop++)
	{
//		HAL_UART_Transmit(&huart, buffer , sizeof(buffer), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart4, CmdPacket[Loop] & 0x1FF , sizeof(CmdPacket[Loop] & 0x1FF), HAL_MAX_DELAY);
//		USART_SendData(&UART4,CmdPacket[Loop] & 0x1FF);
//		while (USART_GetFlagStatus(&UART4, UART_FLAG_TC) == RESET);
		while(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_TC) == RESET);
	}
	WriteToGpioPin(RS485_DIR2,0);


	//receive data
	Status = HAL_UART_Receive(&huart4, RdpDataPacket, sizeof(RdpDataPacket), HAL_MAX_DELAY);
//	Status = ReceiveUsartPacket(&UART4, RdpDataPacket, 19, 10000);
	if(Status)
		return(-1); //error
	//test data string
	for(Loop = 0;Loop < 19;Loop++)
	{
		if(RdpDataPacket[Loop] != RdpTestPacket[Loop])
			break;
	}
	if(Loop != 19)
		return(-2);	//error
	else
		return(0);	//OK
}


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

//*-----------------------------end of file RADM_system.c--------------------------------
