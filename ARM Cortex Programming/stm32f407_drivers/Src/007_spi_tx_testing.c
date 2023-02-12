/*
 * 006_spi_tx_testing.c
 *
 *  Created on: Nov 12, 2022
 *      Author: Jason
 *
 *      A basic program to test transmitting data on MOSI pin for SPI2
 *      slave is Arduino
 *      We wait for button press on PA0
 *
 *      for SPI2, we can use following pins:
 *      PB12 --> SPI2_NSS
 *      PB13 --> SPI2_SCLK
 *      PB14 --> SPI2_MISO
 *      PB15 --> SPI2_MOSI
 *      pin alternate function mode: 5
 *
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

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

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPI2Pins;

	SPI2Pins.pGPIOx = GPIOB;

	SPI2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI2Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPI2Pins);

	//MOSI
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPI2Pins);

	//NSS
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI2Pins);



	//MISO
	//SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPI2Pins);


}

void SPI2_Inits()
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;  //for hardware slave management
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;  //16/8 MHz

	SPI_Init(&SPI2Handle);
}

void delay(uint32_t count)
{
	for(uint32_t i = 0; i < count; i ++);
}

int main (void)
{
	//init periperhal clocks
	//init 4 GPIO pins into AF5
	//init SPI periperal
	//send data

	char user_data[] = "I am a winner. There is so much study left to do but I'm totally smashing it and getting better";

	//Initialize ports and SPI2
	SPI2_GPIOInits();
	SPI2_Inits();
	GPIO_ButtonInit();
	GPIO_LEDInit();

	//SSOE = 1 so that when peripheral is enabled, NSS pin is
	//automatically pulled low to "signal" the slave.
	//NSS PIN high/low is then managed by hardware
	SPI_SSOEConfig(SPI2, ENABLE);



	while(1)
	{
				// wait for button press
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		delay(500000); //remove button bounce

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);  //turn on LED to visually see sending
		delay(500000*2);

		//Enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

			//send data buffer length
		uint8_t datalen = strlen(user_data);
		SPI_SendData(SPI2, &datalen, 1);
			//send data buffer
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		//wait for SPI to complete
		while ( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//Disable SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);  //turn LED off again
	}




	return 0;
}
