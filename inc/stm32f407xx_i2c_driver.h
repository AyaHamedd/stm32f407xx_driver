/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Jun 1, 2020
 *      Author: ayaha
 */


#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"
/*
 * Configuration structure for I2Cx peripheral
 */

typedef struct
{
	uint32_t I2C_SCLSpeed;					/* I2C Speed 	   ,Possible Values : @I2C_SCLSpeed    		 */
	uint8_t  I2C_DeviceAddress;				/* Device Address							 		   		 */
	uint8_t  I2C_ACKControl;				/* ACK bit Control ,Possible Values : @I2C_AckControl 		 */
	uint16_t I2C_FMDutyCycle;				/* Fast mode Duty Cycle , Possible Values : @I2C_FMDutyCycle */
}I2C_Config_t;




/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t 	 *pTxBuffer;
	uint8_t 	 *pRxBuffer;
	uint32_t	 TxLen;
	uint32_t	 RxLen;
	uint8_t		 TxRxState;
	uint8_t 	 DevAddr;
	uint32_t	 RxSize;
	uint8_t		 Sr;
}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM4K		400000
#define	I2C_SCL_SPEED_FM2K		200000



/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0


/*
 * @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2			  0
#define I2C_FM_DUTY_16_9		1


/*
 * I2C application states
 */
#define I2C_READY				    0
#define I2C_BUSY_IN_RX			1
#define	I2C_BUSY_IN_TX			2
/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_SB				(1 << I2C_SR1_SB)				  /* Start bit							*/
#define I2C_FLAG_TxE			(1 << I2C_SR1_TxE)				/*Data register empty (transmitters)	*/
#define I2C_FLAG_RxNE			(1 << I2C_SR1_RxNE)				/*Data register not empty (receivers)	*/
#define I2C_FLAG_ADDR			(1 << I2C_SR1_ADDR)				/* Address sent 						*/
#define I2C_FLAG_AF				(1 << I2C_SR1_AF)
#define I2C_FLAG_BTF			(1 << I2C_SR1_BTF)				/* Byte transfer finished				*/
#define I2C_FLAG_STOPF			(1 << I2C_SR1_STOPF)		/*Stop detection (slave mode)			*/
#define I2C_FLAG_BERR			(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO			(1 << I2C_SR1_ARLO)
#define I2C_FLAG_OVR			(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT		(1 << I2C_SR1_TIMEOUT)


/*
 * I2C Application Events
 */
#define I2C_EVENT_TX_CMPLT		0
#define I2C_EVENT_RX_CMPLT		1
#define I2C_EVENT_STOP			  2
#define I2C_ERROR_BERR  		  3
#define I2C_ERROR_ARLO  		  4
#define I2C_ERROR_AF    	  	5
#define I2C_ERROR_OVR   	  	6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EVENT_DATA_REQ		8
#define I2C_EVENT_DATA_RCV		9

/*
 * Repeated Start possible Configurations
 */
#define I2C_DISABLE_SR			0
#define I2C_ENABLE_SR		  	1




/******************************************************************************************************
 * 												APIs supported by this driver
 * 							For more information about the APIs check the function definitions
 ******************************************************************************************************/

/*
 * Peripheral Clock Setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx , uint8_t EnOrDi);

/*
 * Init and De-init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DInit(I2C_RegDef_t *pI2Cx);


/*
 * Data Send and Recieve
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle , uint8_t *pTxBuffer , uint32_t Length ,uint8_t SlaveAddr, uint8_t Sr );
void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle , uint8_t *RxBuffer , uint8_t Length , uint8_t SlaveAddr , uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx , uint8_t data );
uint8_t I2C_SlaveRecieveData(I2C_RegDef_t *pI2Cx );


/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnORDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber ,uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control I2Cs
 */
uint8_t I2C_GetFlagStatus ( I2C_RegDef_t *I2C , uint32_t FlagName);
void I2C_PeriphControl(I2C_RegDef_t *pI2Cx , uint8_t EnOrDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi);

/*
 * Close Communication functions
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseRecieveData(I2C_Handle_t *pI2CHandle);

/*
 * Application Callback ( Implemented by application )
 */
void I2C_ApplicationEventCallback ( I2C_Handle_t * pI2CHandle,uint8_t AppEvent);

void I2C_GenerateStop(I2C_RegDef_t *pI2Cx);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2C,uint8_t EnOrDi);
#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
