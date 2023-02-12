/*
 * 004led_ExternalButton.c
 *
 *  Created on: Nov 27, 2022
 *      Author: Jason
 *
 *      using external button via pin PB12 to toggle EXTernal LED on pin PA8
 */


#include <stdint.h>
#include <malloc.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#define HIGH 	1
#define LOW 	0
#define BTN_PRESSED	LOW



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



	//populate GPIO Handle struct with pin specific parameters for external button with external PU
	GpioButton.pGPIOx = GPIOB;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE); //enable GPIOD peripheral clock
	GPIO_Init(&GpioButton);  //initialize/configure GPIO pin for Button control

	//populate GPIO Handle struct with pin specific parameters for external LED control
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE); //enable GPIOD peripheral clock
	GPIO_Init(&GpioLed);  //initialize/configure GPIO pin for LED control


    /* Loop forever */
	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED)
		{
			delay();  //to pass button bounce
			GPIO_ToggleOutputPin(GPIOA , GPIO_PIN_NO_8);
		}


	}
}

