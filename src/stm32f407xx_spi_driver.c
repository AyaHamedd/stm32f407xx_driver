/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: May 27, 2020
 *      Author: Aya Hamed
 */


#include "stm32f407xx_spi_driver.h"

/*
 * Private Helper Functions
 */
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral Clock Setup
 */
/********************************************************************************************
 * @fn				- SPI_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for given SPIS port
 *
 * @param			- *pSPIx : SPI port Base Address
 * @param			- EnOrDi  : ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnOrDi)
{
	if( EnOrDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}

			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}

			else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
		}

		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}

			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}

			else if (pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
		}
}

/********************************************************************************************
 * @fn				- SPI_PeriphControl
 *
 * @brief			- This function enables or disables SPI Peripheral
 *
 * @param			- *pSPIx  : SPI bus Base Address
 * @param			- EnOrDi  : ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 */
void SPI_PeriphControl(SPI_RegDef_t *pSPIx , uint8_t EnOrDi)
{
	if ( EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*
 * Init and De-init SPI bus peripheral
 */
/********************************************************************************************
 * @fn				- SPI_Init
 *
 * @brief			- Initialize SPI bus and SPI bus Configuration Settings
 *
 * @param			- *pSPIHandle : Handle structure for SPI
 *
 * @return			- none
 *
 * @Note			- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Enable SPI Peripheral Clock
	SPI_PeriClockControl(pSPIHandle->pSPIx , ENABLE);


	//Configure SPI control register 1
	uint32_t temp = 0 ;

	// 1. Configure Device Mode
	temp |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR) ;

	//2.Configure the bus configuration
	if ( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//Set BIDIMODE bit in CR1 : Bidirectional data mode enable
		temp |= (1 << SPI_CR1_BIDI_MODE );
	}
	else if ( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//Clear BIDIMODE bit in CR1 : Bidirectional data mode disable
		temp &= ~(1 << SPI_CR1_BIDI_MODE );
	}
	else if ( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
	{
		//Clear BIDIMODE bit in CR1 : Bidirectional data mode disable
		temp &= ~(1 << SPI_CR1_BIDI_MODE );
		//Set RXONLY bit in CR1 : Receive only
		temp |= (1 << SPI_CR1_RX_ONLY );
	}

	// 3. Configure the SPI Serial Clock Speed (Baud Rate)
	temp |= ( pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	//4. Configure the Data Frame Format
	temp |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	//5. Configure the Clock Polarity
	temp |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the Clock Phase
	temp |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA ;

	//7. Configure The Software Slave management
	temp |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	//8. Load the Configuration to the register
	pSPIHandle->pSPIx->SPI_CR1 = temp ;
}


/********************************************************************************************
 * @fn				- SPI_DInit
 *
 * @brief			- De-initializes the SPIx peripheral registers to their default reset values.
 *
 * @param			- *pSPIx : SPI port Base Address
 *
 * @return			- none
 *
 * @Note			- none
 */
void SPI_DInit(SPI_RegDef_t *pSPIx)
{
	if ( pSPIx == SPI1 )
	{
		SPI1_REG_RESET();
	}

	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}

	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}


/********************************************************************************************
 * @fn				- SPI_SSIConfig
 *
 * @brief			- Configuration of the SSI pin in CR1
 *
 * @param			- *pSPIx 		: SPI bus Base Address
 * @param			- EnOrDi	 	: Enable or Disable
 *
 * @return			- none
 *
 * @Note			- none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx , uint8_t EnOrDi)
{
	if ( EnOrDi)
	{
		pSPIx->SPI_CR1 |= (1<< SPI_CR1_SSI);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1<< SPI_CR1_SSI);
	}
}


/********************************************************************************************
 * @fn				- SPI_SSOEConfig
 *
 * @brief			- Configuration of the SSOE pin in CR1
 *
 * @param			- *pSPIx 		: SPI bus Base Address
 * @param			- EnOrDi	 	: Enable or Disable
 *
 * @return			- none
 *
 * @Note			- none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx , uint8_t EnOrDi)
{
	if ( EnOrDi)
	{
		pSPIx->SPI_CR1 |= (1<< SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1<< SPI_CR2_SSOE);
	}
}



/*
 * Data Send and Receive
 */
/********************************************************************************************
 * @fn				- SPI_GetFlagStatus
 *
 * @brief			- Get Flag Status from SPI Status Register
 *
 * @param			- *pSPIx 		: SPI bus Base Address
 * @param			- FlagName  	: Flag Name
 *
 * @return			- Flag Status
 *
 * @Note			- none
 */
 uint8_t SPI_GetFlagStatus ( SPI_RegDef_t *pSPIx , uint32_t FlagName)
 {
	 if( pSPIx->SPI_SR && FlagName)
	 {
		 return FLAG_SET;
	 }
	 return FLAG_RESET;
 }


/********************************************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			- Send data in blocking mode.
 *
 * @param			- *pSPIx 		: SPI bus Base Address
 * @param			- *pTxBuffer  	: pointer to data buffer
 * @param			- Length  		: amount of data to be sent
 *
 * @return			- none
 *
 * @Note			- none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx , uint8_t *pTxBuffer , uint32_t Length)
{
	while (Length)
	{
		//1. Wait until TXE is set
		while ( SPI_GetFlagStatus(pSPIx,SPI_FLAG_TXE));

		//2.Check the DFF bit in CR1
		if ( (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit Data Frame
			//1. Load the data into the Data Register DT
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);

			--Length;
			--Length;

			//2. Increment Tx Buffer
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bit Data Frame
			//1. Load the data into the Data Register DT
			pSPIx->SPI_DR = *pTxBuffer;
			Length--;
			//2. Increment Tx Buffer
			pTxBuffer++;
		}
	}
}


/********************************************************************************************
 * @fn				- SPI_RecieveData
 *
 * @brief			- Receive data in blocking mode.
 *
 * @param			- *pSPIx 		: SPI bus Base Address
 * @param			- *pRxBuffer  	: Pointer to data buffer
 * @param			- Length  		: Amount of data received
 *
 * @return			- none
 *
 * @Note			- none
 */
void SPI_RecieveData(SPI_RegDef_t *pSPIx , uint8_t *pRxBuffer , uint32_t Length)
{
	while (Length)
		{
			//1. Wait until RXNE is set ( Receive buffer full )
			while ( ! SPI_GetFlagStatus(pSPIx,SPI_FLAG_RXNE));

			//2.Check the DFF bit in CR1
			if ( (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)))
			{
				//16 bit Data Frame
				//1. Load the data from the Data Register DT
				*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR ;

				--Length;
				--Length;

				//2. Increment Rx Buffer
				(uint16_t*)pRxBuffer++;
			}
			else
			{
				//8 bit Data Frame
				//1. Load the data into the Data Register DT
				*(pRxBuffer) = pSPIx->SPI_DR ;;
				Length--;
				//2. Increment Rx Buffer
				pRxBuffer++;
			}
		}

}



/********************************************************************************************
 * @fn				- SPI_SendDataIT
 *
 * @brief			- Send Data in interrupt ( non-blocking ) mode
 *
 * @param			- *pSPIHandle   : Handle structure for a SPI
 * @param			- *pTxBuffer    : Pointer to data buffer
 * @param			- Length		: Data length
 *
 * @return			- SPI State
 *
 * @Note			- none
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle , uint8_t *pTxBuffer , uint32_t Length)
{

	uint8_t state = pSPIHandle->TxState ;

	if(state != SPI_STATE_BUSY_TX )
	{
		//1. Save the Tx buffer address and Length information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen 	  = Length	 ;

		//2. Mark the SPI state as busy in transmission so that no other code can take over the same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_STATE_BUSY_TX ;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_TXEIE );

		//4. Data Transmission will be handled by the ISR code

	}
	return state;
}

uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle , uint8_t *pRxBuffer , uint32_t Length)
{
	uint8_t state = pSPIHandle->RxState ;

		if(state != SPI_STATE_BUSY_RX )
		{
			//1. Save the Tx buffer address and Length information in some global variables
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen 	  = Length	 ;

			//2. Mark the SPI state as busy in transmission so that no other code can take over the same SPI peripheral until transmission is over
			pSPIHandle->RxState = SPI_STATE_BUSY_RX ;

			//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
			pSPIHandle->pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_RXNEIE );

			//4. Data Transmission will be handled by the ISR code

		}
		return state;
}

/*
 * IRQ Configuration and ISR Handling
 */
/********************************************************************************************
 * @fn				- SPI_IRQInterruptConfig
 *
 * @brief			- Configures IRQ number of the SPI bus, enable or disable the interrupt
 *
 * @param			- IRQNumber  : Number of the interrupt request
 * @param			- EnORDi     : ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnORDi)
{
	// Enable or Disable Interrupt
		if (EnORDi == ENABLE)
		{
			//Enable Interrupt using "Interrupt Set-enable Registers ISER "
			if ( IRQNumber <= 31)
			{
				//Program ISER0 Register
				*NVIC_ISER0 |= (1 << IRQNumber);
			}
			else if (IRQNumber >31 && IRQNumber < 64)
			{
				//Program ISER1 Register
				*NVIC_ISER1 |= (1 <<  (IRQNumber%32) );
			}
			else if (IRQNumber >= 64 && IRQNumber < 96)
			{
				//Program ISER2 Register
				*NVIC_ISER2 |= (1 <<  (IRQNumber%64) );
			}
		}

		else if (EnORDi == DISABLE)
		{
			//Disable Interrupt using "Interrupt Clear-enable Registers ICER "
			if ( IRQNumber <= 31)
			{
				//Program ICER0 Register
				*NVIC_ICER0 |= (1 << IRQNumber);
			}
			else if (IRQNumber >31 && IRQNumber < 64)
			{
				//Program ICER1 Register
				*NVIC_ICER1 |= (1 <<  (IRQNumber%32) );
			}
			else if (IRQNumber >= 64 && IRQNumber < 96)
			{
				//Program ICER2 Register
				*NVIC_ICER2 |= (1 <<  (IRQNumber%64) );
			}
		}

}



/********************************************************************************************
 * @fn				- SPI_IRQHandling
 *
 * @brief			- When an interrupt occurs , this function should be called to proccess the interrupt
 * 					  This function handles EXTI interrupt request.
 *
 * @param			- *pHandle : Handle structure for a SPI
 *
 * @return			- none
 *
 * @Note			- none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t state1 , state2 ; /* state1 checks for event flag and state2 checks for interrupt enable bit */

	//First lets check for TXE
	state1 = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_TXE);
	//Check for TXIE if set or not ( if interrupt is enabled for Tx
	state2 = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_CR2_TXEIE);

	if ( state1 && state2)
	{
		// Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//check for RXNE
	state1 = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_RXNE);
	//Check for RXNEIE if set or not ( if interrupt is enabled for Tx
	state2 = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_CR2_RXNEIE);

	if ( state1 && state2)
	{
		// Handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//check for OVR overrun flag
	state1 = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_OVR);
	//Check for ERRIE ( error interrupt enable
	state2 = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_CR2_ERRIE);

	if ( state1 && state2)
	{
		// Handle RXNE
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}

}



/********************************************************************************************
 * @fn				- SPI_IRQPriorityConfig
 *
 * @brief			- Configure the priority of the interrupt
 *
 * @param			- IRQNumber  : Number of the interrupt request
 * @param			- IRQPriority: Interrupt Priority
 *
 * @return			- None
 *
 * @Note			- None
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. Find the number of the IPRx Register
	uint8_t iprx = IRQNumber / 4;

	//2.Find the bit field
	uint8_t iprx_section = IRQNumber % 4;

	//3.Calculate the position of the priority,knowing that each register contains 4 IRQs priorities of 8-bit length
	//  only 4 bits of interrupt priority are used "High field"
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + (iprx)) |= ( IRQPriority << shift_amount);

	//the base address of NVIC is 4 bytes so it will increment (iprx * 4)

}

/*
 * Private Helper Functions Implementations
 */

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	//1.Check the Data Frame Format DFF bit in CR1
	if ( (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit Data Frame
		//2. Load the data into the Data Register DT
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		//3. Increment Tx Buffer
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}

	else
	{
		//8 bit Data Frame
		//2. Load the data into the Data Register DT
		pSPIHandle->pSPIx->SPI_DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		//3. Increment Tx Buffer
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		// TxLen is zero , so close the SPI transmission and inform the application that Tx is over
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback ( pSPIHandle,SPI_EVENT_TX_CMPLT);


	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	//1.Check the Data Frame Format DFF bit in CR1
	if ( (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit Data Frame
		//2. Load the data into the Data Register DT
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR ;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		//3. Increment Rx Buffer
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}

	else
	{
		//8 bit Data Frame
		//2. Load the data into the Data Register DT
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR ;
		pSPIHandle->RxLen--;
		//3. Increment Rx Buffer
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		// RxLen is zero , so close the SPI transmission and inform the application that Rx is over
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback ( pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1. First Clear the OVR flag
	// Only if the Tx is not busy since the Data may be required by the application
	if ( pSPIHandle->TxState != SPI_STATE_BUSY_TX)
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}
	//2. Inform the application
	SPI_ApplicationEventCallback ( pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp; /* Remove warning of unused variable */
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	// This prevents interrupts from setting TXE flag
	pSPIHandle->pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_TXEIE);
	// Reset Data buffer
	pSPIHandle->pTxBuffer = NULL ;
	// Reset Tx length
	pSPIHandle->TxLen = 0 ;
	// Set SPI State to READY
	pSPIHandle->TxState = SPI_STATE_READY;

}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	// This prevents interrupts from setting RXNEIE flag
	pSPIHandle->pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	// Reset Data buffer+
	pSPIHandle->pRxBuffer = NULL ;
	// Reset Rx length
	pSPIHandle->RxLen = 0 ;
	// Set SPI State to READY
	pSPIHandle->RxState = SPI_STATE_READY;

}


__weak void SPI_ApplicationEventCallback ( SPI_Handle_t * pSPIHandle,uint8_t AppEvent)
{
	//This is a weak implementation. The application may override this function
}

