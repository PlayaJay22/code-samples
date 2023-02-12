/*
 * stm32f407xx_I2C_driver.h
 *
 *  Created on: Nov 18, 2022
 *      Author: Jason
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint32_t	I2C_SCLSpeed;
	uint8_t		I2C_DeviceAddress;
	uint8_t		I2C_ACKControl;
	uint16_t	I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t  	I2C_Config;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxRxState;  //READY, IN_RX, IN_TX
	uint8_t			DevAddr;
	uint32_t		RxSize;		//constant value throughout entire Rx session. Helps to know the total Rx size for proper ISR
	uint8_t			Sr;			//Repeat start or not
}I2C_Handle_t;

// I2C application states for interrupt method
#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2

//@I2C_SCLSpeed
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM2K	200000
#define I2C_SCL_SPEED_FM4K	400000

//@I2C_ACKControl
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

//@I2C_FMDutyCycle
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

//Status register flags
//SPI status flags macros
#define	I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define	I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)
#define	I2C_FLAG_BTF		(1 << I2C_SR1_BTF)
#define	I2C_FLAG_ADD10		(1 << I2C_SR1_ADD10)
#define	I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)
#define	I2C_FLAG_RXNE		(1 << I2C_SR1_RXNE)
#define	I2C_FLAG_TXE		(1 << I2C_SR1_TXE)
#define	I2C_FLAG_BERR		(1 << I2C_SR1_BERR)
#define	I2C_FLAG_ARLO		(1 << I2C_SR1_ARLO)
#define	I2C_FLAG_AF			(1 << I2C_SR1_AF)
#define	I2C_FLAG_OVR		(1 << I2C_SR1_OVR)
#define	I2C_FLAG_TIMEOUT	(1 << I2C_SR1_TIMEOUT)

//macros for repeated start condition
#define I2C_DISABLE_SR 		RESET
#define I2C_ENABLE_SR		SET

//I2C appliction event macros (for callbacks)
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9


/*********** Function prototypes ************/

// Clock setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// init and De-init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

//peripheral control ENABLE/DISABLE
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

//Data send and receive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

//Data send and receive slave
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2C1x, uint8_t EnorDi);

//Data send and receive using interrupts
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

//for Interrupt style sending/receiving
void I2C_CloseReceiveData(I2C_Handle_t* pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t* pI2CHandle);

void I2C_StopCondition(I2C_Handle_t *pI2CHandle);


void I2C_ManageAcking(I2C_RegDef_t* pI2CReg, uint8_t EnorDi);

//IRQ configuration and ISR handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_IRQHandling(I2C_Handle_t* pHandle);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

// Application callback
//the callback is meant to be implemented by user application
//therefore we need to give a "weak" implementation that should be overridden by main.c
void I2C_ApplicationEventCallback(I2C_Handle_t* pHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
