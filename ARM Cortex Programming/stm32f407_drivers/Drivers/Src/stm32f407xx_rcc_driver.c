/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Nov 26, 2022
 *      Author: Jason
 */
#include "stm32f407xx_rcc_driver.h"

uint32_t RCC_GetAPB1PCLKValue()
{
	uint8_t	SystemClockSwitch; //holds the system clock source
	uint32_t ClockSpeed;  //holds the system clock speed
	uint8_t temp;
	uint16_t AHBp; //AHB Prescaler value
	uint16_t APB1p; //APB1 prescaler value
	uint16_t HPREValues[] = {2,4,8,16,32,64,128,256,512};
	uint16_t PPPRE1Values[] = {2,4,8,16};

	SystemClockSwitch = (RCC->CFGR >> 2) & 0x3; //move SWS1 and SWS0 bits to LSB position
	if(SystemClockSwitch == 0) //HSI
		ClockSpeed = (uint32_t)16000000;
	else if(SystemClockSwitch == 1) //HSE
		ClockSpeed = (uint32_t)8000000;

	temp = (RCC->CFGR >> 4) & 0xF; //get bits 7:4
	if (temp < 8)  //system clock not divided
		AHBp = 1;
	else
		AHBp = HPREValues[temp-8];

	temp = (RCC->CFGR >> 10) & 0x7; //get bits 12:10
	if(temp < 4)
		APB1p = 1;
	else
		APB1p = PPPRE1Values[temp-4];

	return (ClockSpeed / AHBp) / APB1p;
}

uint32_t RCC_GetAPB2PCLKValue()
{
	uint8_t	SystemClockSwitch; //holds the system clock source
	uint32_t ClockSpeed;  //holds the system clock speed
	uint8_t temp;
	uint16_t AHBp; //AHB Prescaler value
	uint16_t APB2p; //APB1 prescaler value
	uint16_t HPREValues[] = {2,4,8,16,32,64,128,256,512};
	uint16_t PPPRE1Values[] = {2,4,8,16};

	SystemClockSwitch = (RCC->CFGR >> 2) & 0x3; //move SWS1 and SWS0 bits to LSB position
	if(SystemClockSwitch == 0) //HSI
		ClockSpeed = (uint32_t)16000000;
	else if(SystemClockSwitch == 1) //HSE
		ClockSpeed = (uint32_t)8000000;

	temp = (RCC->CFGR >> 4) & 0xF; //get bits 7:4
	if (temp < 8)  //system clock not divided
		AHBp = 1;
	else
		AHBp = HPREValues[temp-8];

	temp = (RCC->CFGR >> 13) & 0x7; //get bits 15:13
	if(temp < 4)
		APB2p = 1;
	else
		APB2p = PPPRE1Values[temp-4];

	return (ClockSpeed / AHBp) / APB2p;
}
