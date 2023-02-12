/*
 * 002led_buttontoggle.c
 *
 *  Created on: Nov 7, 2022
 *      Author: Jason
 */

#include <stdint.h>
#include <malloc.h>
#include "stm32f407xx.h"


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i ++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioButton;

	//populate GPIO Handle struct with pin specific parameters for LED control
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE); //enable GPIOD peripheral clock
	GPIO_Init(&GpioLed);  //initialize/configure GPIO pin for LED control

	//populate GPIO Handle struct with pin specific parameters for button control
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE); //enable GPIOD peripheral clock
	GPIO_Init(&GpioButton);  //initialize/configure GPIO pin for Button control


    /* Loop forever */
	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == GPIO_PIN_SET)
		{
			delay();  //to pass button bounce
			GPIO_ToggleOutputPin(GPIOD , GPIO_PIN_NO_12);
		}


	}
}

