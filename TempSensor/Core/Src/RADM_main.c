//*--------------------------------------------------------------------------------------
//*                    RADM - Rotork Actuators Data acquisition Module
//*                         		(c) Synergy AST 2013
//*--------------------------------------------------------------------------------------
//* File Name           : RADM_main.c
//* Object              : main application
//* Creation            : Jiri Hofman 18/11/2013
//* Last modified       :
//*--------------------------------------------------------------------------------------

//*--------------------------------------------------------------------------------------
//* Include external files
//*--------------------------------------------------------------------------------------
#include "RADM_main.h"
#include "RADM_system.h"
#include "RADM_task.h"

//*--------------------------------------------------------------------------------------
//* Function Name         : Main
//* Object                : Software entry point
//*
//* Input Parameters      : none
//* Output Parameters     : none
//*--------------------------------------------------------------------------------------

int main(void)
{
	//Initial setup of system & peripherals
    SystemInitSetup();

	//Main never-ending loop
	while(2)
    {
		SystemTask();
		MeasurementTask();
		MasterCmdTask();
	}
}
//*------------------------------end of file RADM_main.c---------------------------------
