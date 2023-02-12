/*
 * 003led_toggleinterrupt.c
 *
 *  Created on: Nov 9, 2022
 *      Author: Jason
 *
 *      using external button via pin PB12 to trigger GPIO interrupt and toggle EXTernal LED on pin PA8
 *      With this code, button needs external 22K pull up resistor
 *      LED needs a current limiting (series) resistor). No PUPD
 */
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void EXTI15_10_IRQHandler(void);

void delay();

int main(void)
{
	//push button is on GPIOB12
	//LED is on GPIOA8
	GPIO_Handle_t GPIOButton, GPIOLed;
	memset(&GPIOButton, 0, sizeof(GPIOButton));
	memset(&GPIOLed, 0, sizeof(GPIOButton));

	GPIOButton.pGPIOx = GPIOB;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);  //for button
	GPIO_PeriClockControl(GPIOA, ENABLE);  //for LED

	GPIO_Init(&GPIOButton);
	GPIO_Init(&GPIOLed);

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 13); //optional

	while(1)
	{

	}

	return 0;

}

//Push button is on PB12, which goes through EXTI12
void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_ToggleOutputPin(GPIOA, 8);  //toggle LED
	GPIO_IRQHandling(GPIO_PIN_NO_12);  //call ISR API to clear EXTI Pending Register for the GPIO PIn that triggered interrupt

}

void delay()
{
	for (uint32_t i = 0; i < 50000; i++);
}
