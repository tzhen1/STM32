#include "stm32f4xx_hal.h"

void ConfigLEDpins(void);
void msDelay(uint32_t msTime);

int main(void)
{
	//1. configure LEDS
	ConfigLEDpins();
	//2. Delay function

	while(1)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
		msDelay(1000); // 1sec delay

	}
}

//check hal gpio lib
void ConfigLEDpins(void)
{
	//enable GPIO clock
	__HAL_RCC_GPIOD_CLK_ENABLE();

	//LED Config
	GPIO_InitTypeDef myLEDconfig;
	myLEDconfig.Mode = GPIO_MODE_OUTPUT_PP; //output mode (push/pull)
	myLEDconfig.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;

	//INIT
	HAL_GPIO_Init(GPIOD, &myLEDconfig); //config as a pointer


}

void msDelay(uint32_t msTime)
{
	for(uint32_t i=0 ; i<msTime*4000 ; i++); // *4000 would be a proper delay, each for loop takes 4 clock cycles
	//clk cycle of stm32 is 16MHz (16/4 = 4MHz). 4MHz /1000 (which is miliseconds) = 4000

}

