#include "stm32f4xx_hal.h"

// enable PA0 and set to analogue , put config funcs in main to initialise them (write code in main lastly)
void GPIO_Config(void);
void ADC_Config(void);
void Systick_Config(void); // for HAL_Delay

ADC_HandleTypeDef myADCtypeDef; //globally so use in all functions (e.g ADC start/stop in main)

uint32_t adcValue;

int main(void)
{
	//reset peripherals
	HAL_Init();

	//config functions in main
	GPIO_Config();
	ADC_Config();
	Systick_Config();

	while(1)
	{
		//read ADC value then add a delay (single conversion mode, read 1 value, not continuously)
		HAL_ADC_Start(&myADCtypeDef);  // change ADChandletypedef (cubemax was &hadc1), we did &myADCtypeDef
		if(HAL_ADC_PollForConversion(&myADCtypeDef, 5)== HAL_OK)
		{
			adcValue = HAL_ADC_GetValue(&myADCtypeDef); //define adcValue globally
		}

		//stop ADC, clear flags for next time
		HAL_ADC_Stop(&myADCtypeDef);
		// 100ms delay
		HAL_Delay(100);

	}

}


void GPIO_Config(void) //gpio.c hal driver tells u how to use
{
	//1. enable clock function
	__HAL_RCC_GPIOA_CLK_ENABLE(); //__HAL_RCC_GPIOx_CLK_ENABLE() , change x to A (port A)

	//3.configure GPIO_InitTypeDef typedef struct , config before INIT 2.
	GPIO_InitTypeDef myADCpin; //create struct
	myADCpin.Pin = GPIO_PIN_0;
	myADCpin.Mode = GPIO_MODE_ANALOG;
	myADCpin.Pull = GPIO_NOPULL;

	//2. INIT peri, port and GPIO_InitTypeDef structure as parameter AS pointer
	HAL_GPIO_Init(GPIOA,&myADCpin);




}

void ADC_Config(void)
{
	//enable ADC clock at ADC number 1
	__HAL_RCC_ADC1_CLK_ENABLE();

	//config
	myADCtypeDef.Instance = ADC1;
	myADCtypeDef.Init.Resolution = ADC_RESOLUTION8b;
	myADCtypeDef.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	myADCtypeDef.Init.DataAlign = ADC_DATAALIGN_RIGHT;

	//INIT ADC
	HAL_ADC_Init(&myADCtypeDef);

	//config channel
	ADC_ChannelConfTypeDef myADCchannelTypeDef; //typedef variable only used here so not global
	myADCchannelTypeDef.Channel = ADC_CHANNEL_0;
	myADCchannelTypeDef.Rank = 1; //only 1 conversion
	myADCchannelTypeDef.SamplingTime = ADC_SAMPLETIME_15CYCLES;

	// INIT channel
	HAL_ADC_ConfigChannel(&myADCtypeDef,&myADCchannelTypeDef);


}

void Systick_Config(void)
{
	//set clock source + speed
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	//sysTick interrupt priority + enable
	HAL_NVIC_SetPriority(SysTick_IRQn, 0 , 0);
	HAL_NVIC_EnableIRQ(SysTick_IRQn); // enable interrupt

}

//normally in stm32.it.interrupt driver, but if didn't include so defined here manually
//void SysTick_Handler(void)
//{
//	HAL_IncTick(); //increment tick function
//}
//

