#include "stm32f4xx_hal.h"

void GPIO_Config(void); //for led and analog input
void ADC_Config(void); // ADC
void TIM_Config(void); // timer

ADC_HandleTypeDef myADChandle;
TIM_HandleTypeDef myTimerHandle;

uint8_t adcValue; // 8 res , 8bit = optimised

int main(void)
{
	//call in config functions
	HAL_Init();
	GPIO_Config(); //for led and analog input
	ADC_Config(); // ADC
	TIM_Config();

	//start timer and adc (as interrupt)
	HAL_TIM_Base_Start(&myTimerHandle);
	HAL_ADC_Start_IT(&myADChandle); //start ADC as interrupt, define ADC interrupt conversion complt call back at end

	while(1)
	{

	}

}

void GPIO_Config(void)
{
	//enable GPIOA (ADC) + GPIOD (led) clk
	__HAL_RCC_GPIOA_CLK_ENABLE(); // analog input
	__HAL_RCC_GPIOD_CLK_ENABLE(); // led

	// ADC (port A) config struct for INIT
	GPIO_InitTypeDef myPinINIT; //typedef hold the values for both ADC + LED, reset when define pin/mode on LED

	myPinINIT.Pin = GPIO_PIN_0;  //ADC pin 0 (PA0)
	myPinINIT.Mode = GPIO_MODE_ANALOG; //analog from .h

	//INIT for port A
	HAL_GPIO_Init(GPIOA, &myPinINIT);

	//LED (port D) config struct for INIT
	myPinINIT.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_12 | GPIO_PIN_12 ;
	myPinINIT.Mode = GPIO_MODE_OUTPUT_PP;
	//INIT for port D
	HAL_GPIO_Init(GPIOD,&myPinINIT);

	//Enable systick interrupt
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

	//Enable ADC interrupt
	HAL_NVIC_SetPriority(ADC_IRQn, 0 ,0 );
	HAL_NVIC_EnableIRQ(ADC_IRQn);

}

void ADC_Config(void)
{
	//Enable ADC clk
	__HAL_RCC_ADC1_CLK_ENABLE();

	//config ADC for INIT, globally defined ADC typedef , same config seen in cubemx too
	myADChandle.Instance = ADC1; // set instance to ADC1
	myADChandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;// divide ADC mother clock (not timer trig clk)
	myADChandle.Init.ContinuousConvMode = DISABLE; // as only using external trigger mode
	myADChandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	myADChandle.Init.DiscontinuousConvMode = DISABLE;
	myADChandle.Init.DMAContinuousRequests = DISABLE;
	myADChandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	myADChandle.Init.ExternalTrigConv = ADC_EXTERNALTRIG2_T2_TRGO;
	myADChandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	myADChandle.Init.NbrOfConversion = 1;
	myADChandle.Init.Resolution = ADC_RESOLUTION8b;
	myADChandle.Init.ScanConvMode = DISABLE;

	//ADC INIT
	HAL_ADC_Init(&myADChandle);

	//ADC channel config
	ADC_ChannelConfTypeDef myChannelConfig; //global typedef struct

	myChannelConfig.Channel = ADC_CHANNEL_1;
	myChannelConfig.Offset = 0;
	myChannelConfig.Rank = 1;
	myChannelConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	HAL_ADC_ConfigChannel(&myADChandle , &myChannelConfig);

}

void TIM_Config(void)
{
	//Enable TIM2 clk
	__HAL_RCC_TIM2_CLK_ENABLE();

	//TIM config (struct) for INIT, define handletypedef globally
	myTimerHandle.Instance = TIM2; //set instance to TIM2
	myTimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; //clk division =1
	myTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	myTimerHandle.Init.Period = 16000 - 1; // -1 due to 0 being the start ,still just 16000
	myTimerHandle.Init.Prescaler = 100 - 1; // prescaler was 16000 before?

	//INIT
	HAL_TIM_Base_Init(&myTimerHandle);

	//TIM CLK source config
	TIM_ClockConfigTypeDef myClkSrcConfig; // struct
	myClkSrcConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

	HAL_TIM_ConfigClockSource(&myTimerHandle, &myClkSrcConfig); // implement the configs

	//TIM master config, define a type def
	TIM_MasterConfigTypeDef myMasterConfig;
	//configs
	myMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	myMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	//INIT master
	HAL_TIMEx_MasterConfigSynchronization(&myTimerHandle, &myMasterConfig);


}

// both links device handler to HAL library handling for SysTick interrupt and ADC interrupt
//void SysTick_Handler(void)
//{
//	HAL_IncTick();
//	HAL_SYSTICK_IRQHandler();
//}

void ADC_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&myADChandle);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */

  //read adcValue
  adcValue = HAL_ADC_GetValue(&myADChandle);
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); //show led
}
