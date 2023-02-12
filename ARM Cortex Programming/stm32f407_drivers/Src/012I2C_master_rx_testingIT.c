/*
 * 012I2C_master_rx_testingIT.c
 *
 *  Created on: Nov 22, 2022
 *      Author: Jason
 *
 *
 *      for I2C1 we use
 *      I2C1 SCL => PB6
 *      I2C1 SDA => PB7
 */

#include <stdio.h>
#include "stm32f407xx.h"
#include "stm32f407xx_I2C_driver.h"
#include "stm32f407xx_gpio_driver.h"

void I2C1_GPIOInits(void);
void I2C1_Inits(I2C_Handle_t* I2C1Handle);
void GPIO_ButtonInit();
void GPIO_LEDInit();
void delay(uint32_t count);

#define MY_ADDR 				0x61
#define SLAVE_ADDR 				0x68  //make sure using Arduino serial monitor tool
#define CODE_FOR_DATA_LENGTH 	0x51
#define CODE_FOR_DATA_CONTENT 	0x52

//global variables
I2C_Handle_t I2C1Handle;  //must be global otherwise IRQHandler cannot access it
uint8_t rxComplt = RESET;  //the InterruptCallback will set this when receiving is complete

int main(void)
{
	//variables
	uint8_t pRxBuffer[32] = {0};
	uint8_t pTxBuffer = 0;  //we are only sending a single byte each time
	uint8_t Len = 0;


	//Init GPIO pins for I2C functionality
	I2C1_GPIOInits();

	//init I2C1 peripheral
	I2C1_Inits(&I2C1Handle);

	//Init GPIO pins for push button
	GPIO_ButtonInit();

	//init an LED for monitoring
	GPIO_LEDInit();

	//enable I2 interrupts
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_PeripheralControl(I2C1Handle.pI2Cx, ENABLE);  //Turn on I2C peripheral by setting PE=1

	//ACK enable can only be done AFTER PE=1
	I2C_ManageAcking(I2C1Handle.pI2Cx, I2C_ACK_ENABLE);

	//wait for button press, then send data
	while(1) //loop button presses
	{
		// wait for button press
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		delay(500000); //remove button bounce

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);  //turn on LED to visually see sending
		delay(500000*2);

			//first send a request for the length of data
			pTxBuffer = CODE_FOR_DATA_LENGTH;
			while(I2C_MasterSendDataIT(&I2C1Handle, &pTxBuffer, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);  //send data

			//now receive the length
			while(I2C_MasterReceiveDataIT(&I2C1Handle, &Len, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);  //receive data

			//send another request for the actual data
			pTxBuffer = CODE_FOR_DATA_CONTENT;
			while(I2C_MasterSendDataIT(&I2C1Handle, &pTxBuffer, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);  //send data

			//receive data
			rxComplt = RESET;
			while(I2C_MasterReceiveDataIT(&I2C1Handle, pRxBuffer, Len, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY);  //receive data

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);  //turn LED off again

		//display data received, but wait first for receive to complete
		while(rxComplt != SET);
		pRxBuffer[Len+1] = '\0'; //add null char to string
		printf("Data received from I2C slave: %s\n", pRxBuffer);
		rxComplt = RESET;
	}

	I2C_PeripheralControl(I2C1Handle.pI2Cx, DISABLE);  //Turn off I2C peripheral



	return 0;
}

/******************* Functions **************************/

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2C1Pins;

	I2C1Pins.pGPIOx = GPIOB;

	I2C1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C1Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;  //I2C mode
	I2C1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2C1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  //irrelevant

	//SCL PB6
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2C1Pins);

	//SDA PB7
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2C1Pins);
}

void I2C1_Inits(I2C_Handle_t* I2C1Handle)
{
	I2C1Handle->pI2Cx = I2C1;
	I2C1Handle->I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle->I2C_Config.I2C_DeviceAddress = MY_ADDR; //arbitrary
	I2C1Handle->I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;  //not relevant since we using SM
	I2C1Handle->I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(I2C1Handle);
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

void I2C1_EV_IRQHandler()
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler()
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t* pHandle, uint8_t AppEv)
{
	if(AppEv == I2C_EV_TX_CMPLT)
	{
		printf("Tx complete\n");
	}
	else if(AppEv == I2C_EV_RX_CMPLT)
	{
		printf("Rx complete\n");
		rxComplt = SET;
	}
	else if(AppEv == I2C_ERROR_AF)
	{
		//this happens if slave is not present, or not responding.
		//then close communication
		printf("Erro: ACK failure\n");
		I2C_CloseSendData(pHandle);
		//now send STOP onto bus
		I2C_StopCondition(pHandle);

		//hang. so we can troubleshoot or just quit
		while(1);
	}
}
