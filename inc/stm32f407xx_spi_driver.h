/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: May 27, 2020
 *      Author: Aya Hamed
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"


/*
 * This is a Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode; 		 /* Device Mode		: possible values from @SPI_DEVICE_MODE */
	uint8_t SPI_BusConfig;			 /* Bus Configuration: possible values from @SPI_BUS_CONFIG	*/
	uint8_t SPI_SclkSpeed;			 /* Clock Speed 		: possible values from @SPI_SCLK_SPEED	*/
	uint8_t SPI_DFF;				     /* Data Frame Format: possible values from @SPI_DATA_FRAME	*/
	uint8_t SPI_CPOL;				     /* Clock Polarity	: possible values from @SPI_CLK_POLARITY*/
	uint8_t SPI_CPHA; 				   /* Clock Phase 		: possible values from @SPI_CLK_PHASE	*/
	uint8_t SPI_SSM;				     /* Software Slave management:			   @SPI_SSM			*/
}SPI_BusConfig_t;


/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t 		*pSPIx ;		  /* This holds the base address of the SPI */
	SPI_BusConfig_t 	SPI_Config; /* This holds SPI Configuration Settings  */
	uint8_t 			*pTxBuffer; 	  /* To Store the application Tx Buffer Address */
	uint8_t 			*pRxBuffer; 	  /* To Store the application Rx Buffer Address */
	uint32_t 			TxLen; 			    /* To store Tx Length */
	uint32_t 			RxLen; 			    /* To store Rx Length */
	uint8_t 			TxState; 		    /* To store Tx State */
	uint8_t 			RxState; 		    /* To store Rx state */
}SPI_Handle_t;



/*
 * @SPI_DEVICE_MODE
 */
#define SPI_DEVICE_MODE_MASTER		1		/* SPI Device master mode 	*/
#define SPI_DEVICE_MODE_SLAVE			0		/* SPI Device in slave mode */


/*
 * @SPI_BUS_CONFIG
 */
#define SPI_BUS_CONFIG_FD					    1		/* Full duplex (Transmit and receive) 	*/
#define SPI_BUS_CONFIG_HD					    2		/* Half duplex (Transmit and receive)	*/
#define SPI_BUS_CONFIG_SIMPLEX_RX			3		/* Simplex 	   (Receive Only)			*/



/*
 * @SPI_SCLK_SPEED
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7



/*
 * @SPI_DATA_FRAME
 */
#define SPI_DFF_8bit						0		/* 8-bit data frame format			*/
#define SPI_DFF_16bit						1		/* 16-bit data frame format			*/



/*
 * @SPI_CLK_POLARIT
 */
#define SPI_CPOL_LOW						0		/* Clock to 0 when idle */
#define SPI_CPOL_HIGH						1		/* Clock to 1 when idle */




/*
 * @SPI_CLK_PHASE
 */
#define SPI_CPHASE_LOW						0		/* The first clock transition is the first data capture edge */
#define SPI_CPHASE_HIGH						1		/* The second clock transition is the first data capture edge*/




/*
 * @SPI_SSM
 */
#define SPI_SSM_DI					0		/* Software slave management disabled */
#define SPI_SSM_EN					1		/* Software slave management enabled  */

/*
 * SPI Related Status Flag definitions
 */
#define SPI_FLAG_TXE				(1<< SPI_SR_TXE)				/* Transmit buffer empty	*/
#define SPI_FLAG_RXNE				(1<< SPI_SR_RXNE)				/* Receive buffer not empty */
#define SPI_FLAG_BSY				(1<< SPI_SR_BSY)				/* Busy flag				*/
#define SPI_FLAG_FRE				(1<< SPI_SR_FRE)				/* Frame format error		*/
#define SPI_FLAG_OVR				(1<< SPI_SR_OVR)				/* 	Overrun flag			*/
#define SPI_FLAG_MODF				(1<< SPI_SR_MODF)				/* 	Mode fault				*/
#define SPI_FLAG_CRCERR				(1<< SPI_SR_CRCERR)				/* 	CRC error flag			*/
#define SPI_FLAG_UDR				(1<< SPI_SR_UDR)				/* 	Underrun flag			*/
#define SPI_FLAG_CHSIDE				(1<< SPI_SR_CHSIDE)				/*  Channel side			*/


/*
 * Possible SPI Application States
 */

#define SPI_STATE_READY				0
#define SPI_STATE_BUSY_RX			1
#define SPI_STATE_BUSY_TX			2

/*
 * Possible SPI application events
 */
#define SPI_EVENT_TX_CMPLT 			1
#define SPI_EVENT_RX_CMPLT 			2
#define SPI_EVENT_OVR_ERR			3

/******************************************************************************************************
 * 												APIs supported by this driver
 * 							For more information about the APIs check the function definitions
 ******************************************************************************************************/

/*
 * Peripheral Clock Setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnOrDi);

/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DInit(SPI_RegDef_t *pSPIx);


/*
 * Data Send and Recieve
 */
void SPI_SendData(SPI_RegDef_t *pSPIx , uint8_t *pTxBuffer , uint32_t Length);
void SPI_RecieveData(SPI_RegDef_t *pSPIx , uint8_t *pRxBuffer , uint32_t Length);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle , uint8_t *pTxBuffer , uint32_t Length);
uint8_t SPI_RecieveDataIT(SPI_Handle_t *Handle , uint8_t *pRxBuffer , uint32_t Length);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnORDi);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber ,uint32_t IRQPriority);


/*
 * Other Peripheral Control SPIs
 */
uint8_t SPI_GetFlagStatus ( SPI_RegDef_t *pSPIx , uint32_t FlagName);
void SPI_PeriphControl(SPI_RegDef_t *pSPIx , uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx , uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx , uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback ( Implemented by application )
 */
void SPI_ApplicationEventCallback ( SPI_Handle_t * pSPIHandle,uint8_t AppEvent);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
