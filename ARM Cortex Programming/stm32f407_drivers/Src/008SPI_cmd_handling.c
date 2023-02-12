/*
 * 006SPI_cmd_handling.c
 *
 *  Created on: Nov 13, 2022
 *      Author: Jason
 *
 *       * A program to test sending CMDs to a slave, and receive replies
 *       Slave will be Arduino
 *
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

//Command codes. must match the slave codes
#define COMMAND_LED_CTRL	0x50
#define COMMAND_SENSOR_READ	0x51
#define COMMAND_LED_READ	0x52
#define COMMAND_PRINT		0x53
#define COMMAND_ID_READ		0x54

#define LED_ON		1
#define LED_OFF		0

//Arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

//arduino LED external
#define LED_PIN			9


void SPI2_GPIOInits(void);
void SPI2_Inits();
void GPIO_ButtonInit();
void GPIO_LEDInit();
void delay(uint32_t count);
uint8_t SPI_VerifyResponse(uint8_t ackbyte);

void cmd_led_on(void);
void cmd_led_off(void);
void cmd_print_message(void);

int main (void)
{


	//Initialize ports and SPI2
	SPI2_GPIOInits();
	SPI2_Inits();
	GPIO_ButtonInit();
	GPIO_LEDInit();

	//SSOE = 1 so that when peripheral is enabled, NSS pin is
	//automatically pulled low to "signal" the slave.
	//NSS PIN high/low is then managed by hardware
	SPI_SSOEConfig(SPI2, ENABLE);

			//TODO:
		//1 . CMD_LED_CtRL <pin no>	<value>  //turn LED ON or OFF
		/// send command code (this results in dummy receive value)
		//  receive back ACk or NACK  (use a dummy write buffer)
		//  clear RX buffer with a "Read", otherwise RX buffer wont allow new data to come
		//  If ACK, send command parameters

	while(1)
	{
				//Enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

				// wait for button press
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		delay(500000); //remove button bounce
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);  //turn on LED to visually see sending
		delay(500000*2);

		cmd_led_on();
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);  //turn LED off again

				// wait for button press
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		delay(500000); //remove button bounce
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);  //turn on LED to visually see sending
		delay(500000*2);

		cmd_led_off();
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);  //turn LED off again

			// wait for button press
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		delay(500000); //remove button bounce
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);  //turn on LED to visually see sending
		delay(500000*2);

		cmd_print_message();
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);  //turn LED off again


		//wait for SPI to complete
		while ( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//Disable SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}

/******************* Functions **************************/

//1. CMD_LED_CTRL <pin no(1)>	<value(1)>
void cmd_led_on(void)
{
		uint8_t dummy_write = 0xFF;
		uint8_t dummy_read = 0xFF;
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		SPI_SendData(SPI2, &commandcode, 1);
		SPI_ReceiveData(SPI2, &dummy_read,  1);  //dummy read to clear RXNE and receive register

		//now send a dummy byte in order to fetch response from slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte,  1);

		if(SPI_VerifyResponse(ackbyte))
		{ //send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
		}
}

//1. CMD_LED_CTRL <pin no(1)>	<value(1)>
void cmd_led_off(void)
{
		uint8_t dummy_write = 0xFF;
		uint8_t dummy_read = 0xFF;
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		SPI_SendData(SPI2, &commandcode, 1);
		SPI_ReceiveData(SPI2, &dummy_read,  1);  //dummy read to clear RXNE and receive register

		//now send a dummy byte in order to fetch response from slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte,  1);

		if(SPI_VerifyResponse(ackbyte))
		{ //send arguments
			args[0] = LED_PIN;
			args[1] = LED_OFF;
			SPI_SendData(SPI2, args, 2);
		}
}

//CMD_PRINT	<len>	<message>
void cmd_print_message(void)
{
	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read = 0xFF;
	uint8_t commandcode = COMMAND_PRINT;
	uint8_t ackbyte;
	char message[] = "Sending this message to Arduino";

	SPI_SendData(SPI2, &commandcode, 1);
	SPI_ReceiveData(SPI2, &dummy_read,  1);  //dummy read to clear RXNE and receive register

	//now send a dummy byte in order to fetch response from slave
	SPI_SendData(SPI2, &dummy_write, 1);
	SPI_ReceiveData(SPI2, &ackbyte,  1);

	if(SPI_VerifyResponse(ackbyte))
	{
			//send data buffer length
		uint8_t datalen = strlen(message);
		SPI_SendData(SPI2, &datalen, 1);
			//send data buffer
		SPI_SendData(SPI2, (uint8_t*)message, strlen(message));
	}
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
		return 1;
	else
		return 0;
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

	//MISO
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPI2Pins);

	//NSS
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI2Pins);



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
