#include "stm32f4xx_hal.h"

void ConfigLEDpins(void);
void msDelay(uint32_t msTime); //delay func

int main(void)
{
	//1. configure LEDS
	ConfigLEDpins();

	while(1)
	{
		//output LED lights (toggles ON/OFF due to XOR with prev state)  if 0(OFF) XOR 1 = 1 (ON). If 1 XOR 1 = 0 (OFF)
		GPIOD->ODR ^= (0xFUL << 12); //ouput data reg, look at document, 0xF = 1111, UL = unsigned long hexa
		msDelay(1000);

	}
}

//check hal gpio lib
void ConfigLEDpins(void)
{
	/* AHB1 is the clock register*/
	RCC-> AHB1ENR |= (1<<3) ; //on data sheet enable port D, write 1 to bit 3

	//set mode as output for GPIOD pins 12 to 15,
	GPIOD->MODER &= ~(0xFF<<12*2); // first set the pins 12-15 to 0 (input - rest state)
	GPIOD->MODER |= (0x55<<12*2); //0x55 (hex) = 01 in binary which is general purpose output mode, shift to bit 12 (pin 12)

}

void msDelay(uint32_t msTime)
{
	for(uint32_t i=0 ; i<msTime*4000 ; i++); // *4000 would be a proper delay, each for loop takes 4 clock cycles
	//clk cycle of stm32 is 16MHz (16/4 = 4MHz). 4MHz /1000 (which is miliseconds) = 4000

}

