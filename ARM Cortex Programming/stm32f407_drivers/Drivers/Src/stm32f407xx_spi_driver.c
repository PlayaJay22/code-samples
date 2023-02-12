/*
 * stm32f407_spi_driver.c
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

#include "stm32f407xx_spi_driver.h"

//helper functions for this .c file only
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);

// Clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	//first test for which SPI peripheral we want to enable/disable clock, based on the address we receive

	if (EnorDi == ENABLE)
	{
		switch((uint32_t)pSPIx)
		{
			case (uint32_t)SPI1:
				SPI1_PCLK_EN();
				break;
			case (uint32_t)SPI2:
				SPI2_PCLK_EN();
				break;
			case (uint32_t)SPI3:
				SPI3_PCLK_EN();
				break;
		}
	} else
	{
		switch((uint32_t)pSPIx)
		{
			case SPI1_BASEADDR:
				SPI1_PCLK_DI();
				break;
			case SPI2_BASEADDR:
				SPI2_PCLK_DI();
				break;
			case SPI3_BASEADDR:
				SPI3_PCLK_DI();
				break;
		}
	}
}

// init and De-init
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	uint32_t tempreg = 0; //store all parameters into a single variable, then write variable to config register

	//1. Master or slave
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Bus type
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)  //Full-duplex
	{
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg &= ~(1 << SPI_CR1_RXONLY);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)  //Half-duplex
	{
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) //SIMPLEX_RXONLY
	{
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Clock speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Data frame size
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Polarity
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. phase
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7. SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;
	//what about the SSM/SSI bit?
	//I Implemented it anyway

	pSPIHandle->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	//todo
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	else
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	else
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	else
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
		return FLAG_SET;

	return FLAG_RESET;
}

//Data send and receive
//this is a blocking function since it blocks all other CPU operations until send is complete
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t Len)
{
	 //We need to write the data to the Data register SPI_DR. This register is then auto pushed to the TX buffer

	if(Len == 0)
		return; //nothing to send

	while (Len > 0) //there is still more to send
	{
		//while( !(pSPIx->SR & (1 << 1)) );  //hang while TX buffer is not empty
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF)) )  //16-bit buffer
		{
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		} else if ( !(pSPIx->CR1 & (1 << SPI_CR1_DFF)) )  //8-bit buffer
		{
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t Len)
{
	if(Len == 0)
		return; //nothing to receive

	while (Len > 0) //there is still more to receive
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		if ( (pSPIx->CR1 & (1 << SPI_CR1_DFF)) )  //16-bit buffer
		{
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		} else if ( !(pSPIx->CR1 & (1 << SPI_CR1_DFF)) )  //8-bit buffer
		{
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}

	}

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t* pTxBuffer, uint32_t Len)
{

	uint8_t state = pSPIHandle->TxState;

	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		//save TX buffer address and Len into global variable (for ISR to access)
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//Mark SPI state as busy (to prevent conflicts)
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//Enable TXEIE to trigger interrupt when TXE flag is set in SR
		//essentially, we are allowing SPI2 to interrupt as soon as TX buffer is empty
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//Data transmission is handled by ISR
	}

	return state;


}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t* pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(pSPIHandle->RxState != SPI_BUSY_IN_RX)
		{
			//save RX buffer address and Len into global variable (for ISR to access)
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = Len;

			//Mark SPI state as busy (to prevent conflicts)
			pSPIHandle->RxState = SPI_BUSY_IN_RX;

			//Enable RXNEIE to trigger interrupt when RXNE flag is set in SR
			//essentially, we are allowing SPI2 to interrupt as soon as RX buffer contains data
			pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

			//Data transmission is handled by ISR
		}

	return state;
}


//IRQ configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;  //find the right register
	uint8_t iprx_section = IRQNumber % 4;  //find the right section in the register
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); //to cater for the fact some LSB bits are not implemented, and we need to shift into MSB section
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount );  //write the priority into the correct register and section
}


void SPI_IRQHandling(SPI_Handle_t* pHandle)
{
	//this function should be called for EVERY byte that needs to be sent/received by SPI

	//1. check what caused interrupt. TXE, RXNE, FAULT
	// RXNE is SR bit 0
	// TXE is SR bit 1
	// fault is SR bits 4,5,6  (we will only check for overrun error
	//2. call corresponding code to handle it

	uint8_t temp1,temp2;

	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		spi_txe_interrupt_handle(pHandle);
	}

	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		spi_rxne_interrupt_handle(pHandle);
	}

	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		spi_ovr_interrupt_handle(pHandle);
	}

}

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR =   *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR =   *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}


static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}

static void spi_ovr_interrupt_handle(SPI_Handle_t *pHandle)
{
	//clear OVR flag
	//inform application
	if(pHandle->TxState != SPI_BUSY_IN_TX)
	{
		SPI_ClearOVRFlag(pHandle->pSPIx);  //only clear if doing RX
	}
	SPI_ApplicationEventCallback(pHandle, SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp; //to eliminate compiler warning

}

void SPI_CloseTransmission(SPI_Handle_t* pHandle)
{
	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);  //disable transmit interrupt
	pHandle->TxLen = 0;
	pHandle->pTxBuffer = NULL;
	pHandle->TxState = SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t* pHandle)
{
	pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);  //disable receive interrupt
	pHandle->RxLen = 0;
	pHandle->pRxBuffer = NULL;
	pHandle->RxState = SPI_READY;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t* pHandle, uint8_t AppEv)
{

}
