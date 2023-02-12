/*
 * stm32f407_gpio_driver.c
 *
 *  Created on: Nov 5, 2022
 *      Author: Jason
 */


/************************************************************
 * @fn			- Function Name
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */

/************************************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for given GPIO clock
 *
 * @param[in]	- base address of the gpio
 * @param[in]	- ENABLE or DISABLE macros
 * @param[in]	-
 *
 * @return		-
 *
 * @note
 */
#include "stm32f407xx_gpio_driver.h"


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		switch ((uint32_t)pGPIOx) { //Test pGPIOx to see which peripheral it's pointing to
		case (uint32_t)GPIOA:
					GPIOA_PCLK_EN();
					break;
		case (uint32_t)GPIOB:
					GPIOB_PCLK_EN();
					break;
		case (uint32_t)GPIOC:
					GPIOC_PCLK_EN();
					break;
		case (uint32_t)GPIOD:
					GPIOD_PCLK_EN();
					break;
		case (uint32_t)GPIOE:
					GPIOE_PCLK_EN();
					break;
		case (uint32_t)GPIOF:
					GPIOF_PCLK_EN();
					break;
		case (uint32_t)GPIOG:
					GPIOG_PCLK_EN();
					break;
		case (uint32_t)GPIOH:
					GPIOH_PCLK_EN();
					break;
		case (uint32_t)GPIOI:
					GPIOI_PCLK_EN();
					break;

		}
	} else  //disable peripheral
	{
		switch ((uint32_t)pGPIOx) { //Test pGPIOx to see which peripheral it's pointing to
		case (uint32_t)GPIOA:
					GPIOA_PCLK_DI();
					break;
		case (uint32_t)GPIOB:
					GPIOB_PCLK_DI();
					break;
		case (uint32_t)GPIOC:
					GPIOC_PCLK_DI();
					break;
		case (uint32_t)GPIOD:
					GPIOD_PCLK_DI();
					break;
		case (uint32_t)GPIOE:
					GPIOE_PCLK_DI();
					break;
		case (uint32_t)GPIOF:
					GPIOF_PCLK_DI();
					break;
		case (uint32_t)GPIOG:
					GPIOG_PCLK_DI();
					break;
		case (uint32_t)GPIOH:
					GPIOH_PCLK_DI();
					break;
		case (uint32_t)GPIOI:
					GPIOI_PCLK_DI();
					break;

		}
	}
}
//Init and DeInit
/************************************************************
 * @fn			- GPIO_Init
 *
 * @brief		- Initialise the specific GPIO pin with the parameters given in pGPIOHandle
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 * 		// steps to configure GPIO interrupts
		//1. pin input mode
		//2. Configure edge trigger (via EXTICFG register)
		//3. Enable interrupt delivery from peripheral to processor (EXTI)
		//4. configure IRQ priority
		//5. Enable interrupt reception for IRQ on NVIC side
		//6. Implement handler
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	uint32_t temp = 0;
	//mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) //non interrupt mode
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	} else  //interrupt mode
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
		{
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);  //the actual value to write into the register, based on the GPIO port we are using
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (portcode << (temp2 * 4));

		//3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
	//speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//PUPD
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//output type
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x1 << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//alternate functionality (optional)
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;  //temp1 indexes the correct array member in the struct
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;  //temp2 indexes the correct register position
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/************************************************************
 * @fn			- Function Name
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	switch ((uint32_t)pGPIOx) { //Test pGPIOx to see which peripheral it's pointing to
		case (uint32_t)GPIOA:
					GPIOA_REG_RESET();
					break;
		case (uint32_t)GPIOB:
					GPIOB_REG_RESET();
					break;
		case (uint32_t)GPIOC:
					GPIOC_REG_RESET();
					break;
		case (uint32_t)GPIOD:
					GPIOD_REG_RESET();
					break;
		case (uint32_t)GPIOE:
					GPIOE_REG_RESET();
					break;
		case (uint32_t)GPIOF:
					GPIOF_REG_RESET();
					break;
		case (uint32_t)GPIOG:
					GPIOG_REG_RESET();
					break;
		case (uint32_t)GPIOH:
					GPIOH_REG_RESET();
					break;
		case (uint32_t)GPIOI:
					GPIOI_REG_RESET();
					break;
	}
}
//Read and Write

/************************************************************
 * @fn			- Function Name
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t temp;
	temp = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
	return temp;
}

/************************************************************
 * @fn			- Function Name
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t temp;
	temp = (uint16_t)pGPIOx->IDR;
	return temp;
}

/************************************************************
 * @fn			- Function Name
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET) //write 1 to corresponding output pin
		pGPIOx->ODR |= (1 << PinNumber);
	else
		pGPIOx->ODR &= ~(1 << PinNumber);
}

/************************************************************
 * @fn			- Function Name
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/************************************************************
 * @fn			- Function Name
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);

}

/************************************************************
 * @fn			- Function Name
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
//Interrupt enable and handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
			*NVIC_ISER0 |= (1 << IRQNumber);
		else if((IRQNumber > 31) && (IRQNumber <= 63))
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		else if((IRQNumber > 63) && (IRQNumber <= 95))
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
	} else
	{
		if(IRQNumber <= 31)
			*NVIC_ICER0 |= (1 << IRQNumber);
		else if((IRQNumber > 31) && (IRQNumber <= 63))
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		else if((IRQNumber > 63) && (IRQNumber <= 995))
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;  //find the right register
	uint8_t iprx_section = IRQNumber % 4;  //find the right section in the register
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); //to cater for the fact some LSB bits are not implemented, and we need to shift into MSB section
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount );  //write the priority into the correct register and section
}

/************************************************************
 * @fn			- Function Name
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @note		-
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI PR register (pending register)
	if(EXTI->PR & (1 << PinNumber))
		EXTI->PR |= (1 << PinNumber);
}


