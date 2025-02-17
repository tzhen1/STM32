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
		//1. Start ADC1
		ADC1->CR2 |= ADC_CR2_ADON; // from control register 2, ON
		ADC1->CR2 |= (1<<30); // bit 30 set ON is the software start

		//2. Stabilise the ADC clock, need ~3us for clock to stabilize when start ADC
		uint32_t counter = (ADC_STAB_DELAY_US * (SystemCoreClock/1000000U) ); // wait for 10ms (counts down)
		while(counter!= 0U)
		{
			counter--;
		}
		//3. Poll for EOC (end of conversion) - waits until status flag is triggered by EOC- look at datasheet
		uint32_t tickstart = 0U;
		tickstart = HAL_GetTick(); // gets current SysTick counter value
		while( (ADC1->SR&0x2) != 0x2 ) // adc->SR is status register
		{
			if( (HAL_GetTick() - tickstart) <10 )
			{
				break;
			}
		}
		//4. Read ADC value
		adcValue = ADC1->DR;
		//Stop ADC
		ADC1->CR2 &= ~ADC_CR2_ADON;
		//clear flags
		ADC1->SR = 0x00;
		// delay
		HAL_Delay(100);
	}

}


void GPIO_Config(void) //gpio.c hal driver tells u how to use
{
	// Register code not using HAL
	//1. enable port A clock
	RCC->AHB1ENR |=0x01; // write 1 to turn on clock register
	//2. set mode to analog (0x03)
	GPIOA->MODER |= 0x03;
	//3. remove pull up / pull down
	GPIOA->PUPDR &= ~(0x00000003); //datasheet set register to 1,1 for analo (11 = 3 in decimal)

}

void ADC_Config(void)
{
	//1. enable clock
	__HAL_RCC_ADC1_CLK_ENABLE(); //__HAL_RCC_ADC_CLK_ENABLE() , number 1

	// ADC basic configuration
	ADC1->CR1 = (0x2 << 24); // control register 1, set resolution 8bit + channel 0 (see datasheet) at bit 24
	ADC1->CR2 = (0x1 << 10); // EOC (end of conversion flag) is bit 10, set to 1 so happens after end of conversion
	ADC1->SMPR2 = 1; //sample time to 15 clock cycles , sample time register 2
	ADC1->SQR1 = (0x0 << 20); //regular sequence register, datasheet : 1 conversion in bit 20
	ADC1->SQR3 = 0x00; //squencier to convert channel 0 first



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

//normally in stm32.it.interrupt driver, but we didn't include so defined here manually
//void SysTick_Handler(void)
//{
//	HAL_IncTick(); //increment tick function
//}
//

