/*
 * stm32f407xx_I2C_driver.c
 *
 *  Created on: Nov 18, 2022
 *      Author: Jason
 */

#include "stm32f407xx_i2c_driver.h"

/***********************helper functions ************/
static uint32_t RCC_GetPCLK1Value();
static void I2C_StartCondition(I2C_Handle_t *pI2CHandle);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle );


static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle );
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle );

static uint32_t RCC_GetPCLK1Value()
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

static void I2C_StartCondition(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}



static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}


static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}


}


void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}


static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{

	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;

	}

}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	}


	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

			//read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}

// Clock setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	//first test for which peripheral we want to enable/disable clock, based on the address we receive

	if (EnorDi == ENABLE)
	{
		switch((uint32_t)pI2Cx)
		{
			case (uint32_t)I2C1:
				I2C1_PCLK_EN();
				break;
			case (uint32_t)I2C2:
				I2C2_PCLK_EN();
				break;
			case (uint32_t)I2C3:
				I2C3_PCLK_EN();
				break;
		}
	} else
	{
		switch((uint32_t)pI2Cx)
		{
			case I2C1_BASEADDR:
				I2C1_PCLK_DI();
				break;
			case I2C2_BASEADDR:
				I2C2_PCLK_DI();
				break;
			case I2C3_BASEADDR:
				I2C3_PCLK_DI();
				break;
		}
	}
}

void I2C_ManageAcking(I2C_RegDef_t* pI2CReg, uint8_t EnorDi)
{
	//NOTE: writing 1 to this bit will automatically be cleared to "0" if PE (peripheral Enable) is still "0"
	//Writing to the ACK bit is pointless unless PE is 1
	if(EnorDi == ENABLE)
		 pI2CReg->CR1 |= (1 << I2C_CR1_ACK);
	else
		pI2CReg->CR1 &= ~(1 << I2C_CR1_ACK);
}

// init and De-init
void I2C_Init(I2C_Handle_t *pI2CHandle)
{

	//enable I2C peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t temp;



	//clock frequency FREQ field in CR2
	temp = 0;
	temp |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = temp & 0x3F; //write FREQ value to register

	//own address (for slave mode)
	temp = 0;
	temp |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	temp |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = temp;

	//CCR clock value
	uint16_t ccrValue = 0;
	temp = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//std mode
		temp |= (0 << I2C_CCR_FS); //0 for SM
		ccrValue = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed); //see reference manual formula

	} else
	{
		//fast mode
		temp |= (1 << I2C_CCR_FS); //0 for FM
		temp |= pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY;

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)  //DUTY = 0
		{
			ccrValue = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed); //see reference manual formula
		} else //DUTY = 1
		{
			ccrValue = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed); //see reference manual formula
		}
	}
	temp |= ccrValue & 0xFFF;  //combine all the bits into a single variable
	pI2CHandle->pI2Cx->CCR = temp; //write values to register


	//rise time for pins
	temp = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)  //Standard mode
	{
		temp = (RCC_GetPCLK1Value() / 1000000U) +1;
	} else  //fast mode
	{
		temp = (RCC_GetPCLK1Value() *300 / 1000000U) +1;
	}
	pI2CHandle->pI2Cx->TRISE = temp & 0x3F;

}
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	//TODO
	//see RCC reset registers
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
		return FLAG_SET;

	return FLAG_RESET;
}



void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Initiate START condition
	I2C_StartCondition(pI2CHandle);

	//2. wait for START generation to complete, by checking SB flag in SR1
	while (! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) ) );

	//3. send address of slave, as well as r/w bit set to 0
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm Address phase complete by checking ADDR flag in SR1
	while (! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) ) );
	I2C_ClearADDRFlag(pI2CHandle);

	//5. Send data
	while (Len > 0) //keep sending data
	{
		while (! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) ) );  //wait for data register to become empty
		pI2CHandle->pI2Cx->DR = *pTxBuffer;  //move byte into Data register
		pTxBuffer++; //move to next byte
		Len--;
	}

	//6. STOP condition
	while (! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) ) ); //wait for TXE = 1
	while (! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF) ) ); //wait for BTF = 1

	if(Sr == I2C_DISABLE_SR)  //if Sr ==1 then we want a repeated start. Therefore, don't STOP
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	//1. Generate the START condition
	I2C_StartCondition(pI2CHandle);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while (! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) ) );

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while (! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) ) );


	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while (! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) ) );

		//generate STOP condition
		if(Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;  //move DR into buffer
	}


    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while (! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) ) );

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				if(Sr == I2C_DISABLE_SR)
					 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;  //move DR into buffer

			//increment the buffer address
			pRxBuffer++; //move to next byte
		}

	}
	//re-enable ACKing (to be ready for next receive session)
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
}

//all we do here is enable the interrupts and trigger START condition
//the ISR does the actual transfer
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_StartCondition(pI2CHandle);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_StartCondition(pI2CHandle);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);


	}

	return busystate;
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	else
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
}



//IRQ configuration and ISR handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;  //find the right register
	uint8_t iprx_section = IRQNumber % 4;  //find the right section in the register
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); //to cater for the fact some LSB bits are not implemented, and we need to shift into MSB section
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount );  //write the priority into the correct register and section
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t) pI2C->DR;
}

//The EV interrupt could happen for many reasons
//We need to decode why interrupt happened, and take according actions
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	temp1   = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;
	temp2   = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN) ;

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set .
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE) )
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0 )
				{
					//1. generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);

				}
			}

		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			;
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode
	if(temp1 && temp3)
	{
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}


	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		    }
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//check device mode .
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//The device is master

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}

		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2C1x, uint8_t EnorDi)
 {
	 if(EnorDi == ENABLE)
	 {
			pI2C1x->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
			pI2C1x->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
			pI2C1x->CR2 |= ( 1 << I2C_CR2_ITERREN);
	 }else
	 {
			pI2C1x->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
			pI2C1x->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
			pI2C1x->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	 }

 }

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
	    I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);

	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}


// Application callback
//the callback is meant to be implemented by user application
//therefore we need to give a "weak" implementation that should be overridden by main.c
void I2C_ApplicationEventCallback(I2C_Handle_t* pHandle, uint8_t AppEv);

