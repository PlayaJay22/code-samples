/*
 * 013I2C_slave_tx_testing.c
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
void I2C1_Inits();
void GPIO_ButtonInit();
void GPIO_LEDInit();
void delay(uint32_t count);


#define SLAVE_ADDR 				0x69  //make sure using Arduino serial monitor tool
#define MY_ADDR 				SLAVE_ADDR
#define CODE_FOR_DATA_LENGTH 	0x51
#define CODE_FOR_DATA_CONTENT 	0x52

//global variables
I2C_Handle_t I2C1Handle;  //must be global otherwise IRQHandler cannot access it
uint8_t pTxBuffer[32] = "STM32 slave mode testing\n";

int main(void)
{
	//variables


	//Init GPIO pins for I2C functionality
	I2C1_GPIOInits();

	//init I2C1 peripheral
	I2C1_Inits();

	//enable I2C interrupts
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	I2C_PeripheralControl(I2C1Handle.pI2Cx, ENABLE);  //Turn on I2C peripheral by setting PE=1

	//ACK enable can only be done AFTER PE=1
	I2C_ManageAcking(I2C1Handle.pI2Cx, I2C_ACK_ENABLE);

	while(1)
	{
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

void I2C1_Inits()
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR; //arbitrary
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;  //not relevant since we using SM
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
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
	static uint8_t commandCode = 0;
	static uint8_t Cnt = 0;

	if(AppEv == I2C_EV_DATA_REQ)
	{
		if(commandCode == 0x51)
		{
			//send string length information to master
			I2C_SlaveSendData(pHandle->pI2Cx, strlen((char*)pHandle->pTxBuffer));
		} else if (commandCode == 0x52)
		{
			//send actual date
			I2C_SlaveSendData(pHandle->pI2Cx, pTxBuffer[Cnt++]);
		}

	} else if(AppEv == I2C_EV_DATA_RCV)
	{
		//data received by slave
		commandCode = I2C_SlaveReceiveData(pHandle->pI2Cx);

	} else if(AppEv == I2C_ERROR_AF)
	{
		//happens when master is done receiving
		//this happens when master fails to ACK. Which means it no longer wants more data
		commandCode = 0xff; //reset value
		Cnt = 0; //reset count

	} else if(AppEv == I2C_EV_STOP)
	{
		//only happens when slave is receiving data
		//happens when master has ended the session
	}
}
