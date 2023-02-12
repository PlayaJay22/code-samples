/*
 * 015USART_senddata_testing.c
 *
 *  Created on: Nov 26, 2022
 *      Author: Jason
 */

#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_usart_driver.h"

void USART2_GPIOInit(void);
void USART2_Init();
void GPIO_ButtonInit();
void GPIO_LEDInit();
void delay(uint32_t count);

USART_Handle_t usart2_handle;

int main(void)
{

	uint8_t msg[] = "Sending from STM32.\n\r";
	//Init GPIO pins for USART functionality
	USART2_GPIOInit();

	//init USART2 peripheral
	USART2_Init();

	//Init GPIO pins for push button
	GPIO_ButtonInit();

	//init an LED for monitoring
	GPIO_LEDInit();

	//enable USART2 peripheral
	USART_PeripheralControl(USART2,ENABLE);

	//wait for button press, then send data
	while(1) //loop button presses
	{
		// wait for button press
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		delay(500000); //remove button bounce

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);  //turn on LED to visually see sending
		delay(500000*2);


		//send data
		USART_SendData(&usart2_handle, msg, (uint32_t)strlen((char*)msg));

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);  //turn LED off again
	}

	return 0;
}

void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode =7;

	//USART2 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	//USART2 RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);


}

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}

void GPIO_ButtonInit()
{
	GPIO_Handle_t GpioButton;
	//populate GPIO Handle struct with pin specific parameters for button control
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioButton);  //initialize/configure GPIO pin for Button control
}

void GPIO_LEDInit()
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioLed);  //initialize/configure GPIO pin for LED control
}

void delay(uint32_t count)
{
	for(uint32_t i = 0; i < count; i ++);
}

