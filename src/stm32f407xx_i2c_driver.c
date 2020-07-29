/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jun 1, 2020
 *      Author: ayaha
 */

#include "stm32f407xx_i2c_driver.h"



/*
 * Private Helper functions
 */
static void I2C_GenerateStart(I2C_RegDef_t *pI2Cx);
static void I2C_SendAddressWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveADDR);
static void I2C_SendAddressRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveADDR);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);



static void I2C_GenerateStart(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_START);
}

void I2C_GenerateStop(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_STOP);
}

static void I2C_SendAddressWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveADDR)
{
	// Shift to leave 1 bit space for R/W
	SlaveADDR = SlaveADDR << 1 ;
	// Clear R/W to write data
	SlaveADDR &= ~(1);
	pI2Cx->I2C_DR = SlaveADDR;
}

static void I2C_SendAddressRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveADDR)
{
	// Shift to leave 1 bit space for R/W
	SlaveADDR = SlaveADDR << 1 ;
	// Clear R/W to write data
	SlaveADDR |= 1;
	pI2Cx->I2C_DR = SlaveADDR;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//Check for the device mode
	if ( pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_MSL))
	{
		//Master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//First Disable the ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx , DISABLE);

				//Clear the ADDR flag ( Read SR1 , read SR2)
				dummy_read= pI2CHandle->pI2Cx->I2C_SR1 ;
				dummy_read= pI2CHandle->pI2Cx->I2C_SR2 ;
				(void)dummy_read;
			}
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Clear the ADDR flag ( Read SR1 , read SR2)
			dummy_read= pI2CHandle->pI2Cx->I2C_SR1 ;
			dummy_read= pI2CHandle->pI2Cx->I2C_SR2 ;
			(void)dummy_read;
		}

	}
	else
	{
		//Slave mode
		//Clear the ADDR flag ( Read SR1 , read SR2)
		dummy_read= pI2CHandle->pI2Cx->I2C_SR1 ;
		dummy_read= pI2CHandle->pI2Cx->I2C_SR2 ;
		(void)dummy_read;
	}
}


/********************************************************************************************
 * @fn				- I2C_ManageAcking
 *
 * @brief			- This function enables or disables ACK bit
 *
 * @param			- *pI2CHandle : I2C Handle Structure
 * @param			- EnOrDi  : ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx ,uint8_t EnOrDi)
{
	if ( EnOrDi == ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);

	}
	else if (EnOrDi == DISABLE)
	{
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
	}
}



/*
 * Peripheral Clock Setup
 */
/********************************************************************************************
 * @fn				- I2C_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for given I2CS port
 *
 * @param			- *pI2Cx : I2C port Base Address
 * @param			- EnOrDi  : ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx , uint8_t EnOrDi)
{
	if( EnOrDi == ENABLE)
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_EN();
			}

			else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_EN();
			}

			else if (pI2Cx == I2C3)
			{
				I2C3_PCLK_EN();
			}
		}

		else
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}

			else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}

			else if (pI2Cx == I2C3)
			{
				I2C3_PCLK_DI();
			}
		}
}



/********************************************************************************************
 * @fn				- I2C_PeriphControl
 *
 * @brief			- This function enables or disables I2C Peripheral
 *
 * @param			- *pI2Cx  : SPI bus Base Address
 * @param			- EnOrDi  : ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 */
void I2C_PeriphControl(I2C_RegDef_t *pI2Cx , uint8_t EnOrDi)
{
	if ( EnOrDi == ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/*
 * Init and De-init I2C bus peripheral
 */
/********************************************************************************************
 * @fn				- I2C_Init
 *
 * @brief			- Initialize I2C bus and I2C bus Configuration Settings
 *
 * @param			- *pI2CHandle : Handle structure for I2C
 *
 * @return			- none
 *
 * @Note			- none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{

	uint32_t tempReg = 0;

	//Enable Clock for I2C
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//1. Configure the ACK bit
	tempReg |= ( pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->I2C_CR1 = tempReg;


	//2. Configure the FREQ field of CR2
	tempReg=0;
	// Divide by 1,000,000 to convert to Mhz
	tempReg |= RCC_GetPCLK1Value() / 1000000U ;
	//Mask cause we only want first 5 bits for safety purpose
	pI2CHandle->pI2Cx->I2C_CR2 = tempReg & 0x3F ;



	//3. Configure Slave Address
	tempReg=0;
	tempReg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD7 ;
	// The 14th bit should be kept 1 by the SW based on the datasheet
	tempReg |= ( 1 << 14);
	pI2CHandle->pI2Cx->I2C_OAR1 = tempReg;


	//4. Configure CCR
	tempReg=0;
	uint16_t CCRvalue = 0;
	if ( pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM )
	{
		// Mode is Standard mode , Tlow = Thigh
		CCRvalue = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempReg |= (CCRvalue & 0xFFF);
	}
	else
	{
		// Mode is Fast mode
		tempReg |= (1 << I2C_CCR_FS);
		//Program Duty Cycle
		tempReg |= ( pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY );

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			CCRvalue = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9)
		{
			CCRvalue = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}

		tempReg |= ( CCRvalue & 0xFFF);
	}


pI2CHandle->pI2Cx->I2C_CCR |= ( tempReg << I2C_CCR_CCR ) ;

//TRise Configuration

if ( pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM )
{
	// Mode is Standard mode , Tlow = Thigh
	// This calculation is according to the reference manual
	tempReg = ( RCC_GetPCLK1Value() / 1000000U ) + 1 ;
}
else{
	//Mode is fast mode
	tempReg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1 ;
}
	pI2CHandle->pI2Cx->I2C_TRISE |= tempReg & 0x3F;
}


/********************************************************************************************
 * @fn				- I2C_DInit
 *
 * @brief			- De-initializes the I2Cx peripheral registers to their default reset values.
 *
 * @param			- *pI2Cx : I2C port Base Address
 *
 * @return			- none
 *
 * @Note			- none
 */
void I2C_DInit(I2C_RegDef_t *pI2Cx)
{
	if ( pI2Cx == I2C1 )
	{
		I2C1_REG_RESET();
	}

	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}

	else if (pI2Cx == I2C3)
	{
		SPI3_REG_RESET();
	}
}


/*
 * IRQ Configuration and ISR Handling
 */
/********************************************************************************************
 * @fn				- I2C_IRQInterruptConfig
 *
 * @brief			- Configures IRQ number of the I2C bus, enable or disable the interrupt
 *
 * @param			- IRQNumber  : Number of the interrupt request
 * @param			- EnORDi     : ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnORDi)
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
 * @fn				- I2C_IRQPriorityConfig
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
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


/********************************************************************************************
 * @fn				- I2C_GetFlagStatus
 *
 * @brief			- Get Flag Status from I2C Status Register
 *
 * @param			- *pI2Cx 		: SPI bus Base Address
 * @param			- FlagName  	: Flag Name
 *
 * @return			- Flag Status
 *
 * @Note			- none
 */
 uint8_t I2C_GetFlagStatus ( I2C_RegDef_t *pI2Cx , uint32_t FlagName)
 {
	 if( pI2Cx->I2C_SR1 & FlagName)
	 {
		 return FLAG_SET;
	 }
	 return FLAG_RESET;
 }

 /********************************************************************************************
  * @fn				- I2C_MasterSendData
  *
  * @brief			- Master Sends Data
  *
  * @param			- *pI2CHandle 	: Handle Structurfor I2C
  * @param			- *pTxBuffer  	: pointer to data buffer
  * @param			- Length  		: amount of data to be sent
  * @param			- SlaveAddr  	: Slave address
  *
  * @return			- none
  *
  * @Note			- none
  */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle , uint8_t *pTxBuffer , uint32_t Length , uint8_t SlaveAddr , uint8_t Sr )
{
	//1. Generate the START condition
	I2C_GenerateStart(pI2CHandle->pI2Cx);

	//2. Confirm start generation completed by checking SB flag in SR1
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	//3. Send the address of the slave with r/w bit set to write=0 , total 8 bits
	I2C_SendAddressWrite(pI2CHandle->pI2Cx,SlaveAddr );

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	//5.Clear ADDR flag according to the sequence
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send the data until length is 0
	while ( Length > 0 )
	{
		//While till TxE is Set ( Transmitter buffer empty)
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_TxE));
		pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
		pTxBuffer++;
		--Length;
	}


	//7. when length becomes zero wait for TXE=1 and BFT=1 before generating the STOP condition
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_TxE));
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition
	// Generate only if Repeated start is disabled
	if ( Sr == I2C_DISABLE_SR)
		I2C_GenerateStop(pI2CHandle->pI2Cx);
}

void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle , uint8_t *RxBuffer , uint8_t Length , uint8_t SlaveAddr , uint8_t Sr)
{
	//1. Generate the START condition
	I2C_GenerateStart(pI2CHandle->pI2Cx);

	//2. Confirm start generation completed by checking SB flag in SR1
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	//3. Send the address of the slave with r/w bit set to write=0 , total 8 bits
	I2C_SendAddressRead(pI2CHandle->pI2Cx,SlaveAddr );

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	//Procedure to  read only 1 byte from slave
	if(Length == 1)
	{
		//In case a single byte has to be received, the Acknowledge disable is made (before ADDR flag is cleared)
		//the STOP condition generation is made after
		I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until RXNE becomes 1
		while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));

		//Generate STOP condition
		I2C_GenerateStop(pI2CHandle->pI2Cx);

		//Read data in to buffer
		*RxBuffer = pI2CHandle->pI2Cx->I2C_DR;
	}

	//Procedure to  read more than one byte from slave
	else if (Length > 1)
	{
		//clear ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Read data until Length becomes zero
		for ( uint32_t i = Length ; i > 0 ; i-- )
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RxNE) );

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable ACKing
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//generate STOP condition if Repeated start is disabled
				if ( Sr == I2C_DISABLE_SR)
					I2C_GenerateStop(pI2CHandle->pI2Cx);

			}

			//read the data from data register in to buffer
			*RxBuffer = pI2CHandle->pI2Cx->I2C_DR;

			//increment the buffer address
			RxBuffer++;

		}

	}

	//Re-enable the ACK if it was enabled before the last block
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}


/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
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
		I2C_GenerateStart(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

/*********************************************************************
* @fn      		  - I2C_MasterReceiveDataIT
*
* @brief             -
*
* @param[in]         -
* @param[in]         -
* @param[in]         -
*
* @return            -
*
* @Note              -
*/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception(doesn't decrement)
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStart(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}



/*********************************************************************
* @fn      		  - I2C_MasterHandleTXEInterrupt
*
* @brief             -
*
* @param[in]         -*pI2CHandle

*
* @return            - none
*
* @Note              - none
*/
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//Check if TXE is also set
	if ( pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE) )
	{
		//BTF and TXE are both set then we should close the transmission

		if( pI2CHandle->TxLen == 0)
		{
			//1. Generate STOP condition if repeated start is disabled
			if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStop(pI2CHandle->pI2Cx);

			//2. RESET all member elements of the handle structure
			I2C_CloseSendData(pI2CHandle);

			//3.Notify the application about transmission complete
			I2C_ApplicationEventCallback ( pI2CHandle,I2C_EVENT_TX_CMPLT);
		}
	}
}



/*********************************************************************
* @fn      		  - I2C_MasterHandleTXEInterrupt
*
* @brief             -
*
* @param[in]         -*pI2CHandle

*
* @return            - none
*
* @Note              - none
*/
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
	{
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->RxLen--;
	}
	else if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxSize == 2) //if last 2 bytes are remaining
		{
			//Clear the ACKING
			I2C_ManageAcking(pI2CHandle->pI2Cx , DISABLE);
		}
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	else if (pI2CHandle->RxSize == 0)
	{
		//Close the I2C data reception and notify the application

		//1. Generate STOP condition
		if(pI2CHandle->Sr == DISABLE)
			I2C_GenerateStop(pI2CHandle->pI2Cx);

		//2.Close the I2C Rx
		I2C_CloseRecieveData(pI2CHandle);

		//3.Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_RX_CMPLT);
	}
}


/*********************************************************************
* @fn      		  - I2C_MasterHandleTXEInterrupt
*
* @brief             -
*
* @param[in]         -*pI2CHandle

*
* @return            - none
*
* @Note              - none
*/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	//Interrupt  handling for both master and slave mode of a device

	uint32_t state1 , state2 ,state3 ;

	state1 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN);
	state2 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN);

	//1. Handle for interrupt generated by SB event, Note : SB only applicable in Master mode
	state3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB);
	if (state1 && state3)
	{
		//Interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero

		//Send Slave address
		if ( pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Write in case application is busy in transmission
			I2C_SendAddressWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			//Read if application state is busy in receive
			I2C_SendAddressRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}

		//I2C_ClearSBFlag(pI2CHandle->pI2Cx);
	}

	//2.Handle for interrupt generated by ADDR event
	// When master mode :Address is sent, When Slave mode : Address matched with own address

	state3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR);
	if (state1 && state3)
	{
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}


	//3.Handle for interrupt generated by BFT( Byte Transfer Finished) event
	state3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF);
	if ( state1 && state3)
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_MasterHandleTXEInterrupt(pI2CHandle);
		}
	}

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode
	state3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF);
	if ( state1 && state3)
	{
		//STOPF flag is set
		//Clear the STOPF by reading SR1 then writing to CR1
		pI2CHandle->pI2Cx->I2C_CR1 |=0x000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_STOP);
	}

	//5. Handle For interrupt generated by TXE event
	state3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE);
	if ( state1 && state2 && state3)
	{

		//ONLY IF THE DEVICE IS MASTER
		if ( pI2CHandle->pI2Cx->I2C_SR2 & (1<< I2C_SR2_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				if(pI2CHandle->TxLen > 0 )
				{
					//1. Load the data in to DR
					pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);

					//2.Decrement the TxLen
					pI2CHandle->TxLen -- ;

					//3.Increment the buffer address
					pI2CHandle->pTxBuffer ++ ;
				}
			}
		}
		else
		{
			//SLAVE mode
			// Request for Data from Master IF the slave is in transmitter mode ( Based of R/W bit )
			if ( pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_TRA))
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_DATA_REQ);
		}
	}


	//6. Handle For interrupt generated by RXNE event
	state3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RxNE);
	if ( state1 && state2 && state3)
	{
		//MASTER MODE
		if ( pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
			//We have to do data reception if device is in busy in RX
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		//SLAVE MODE
		else
		{
			// Receiving Data from Master IF the slave is in receiver mode ( Based of R/W bit )
			if ( ! ( pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_TRA)))
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_DATA_RCV);

		}

	}

}





void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN and ITEVTEN Control
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}




void I2C_CloseRecieveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN and ITEVTEN Control
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
}


/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - none

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);

	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);


		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);

	}

}


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx , uint8_t data )
{
	pI2Cx->I2C_DR = data;
}
uint8_t I2C_SlaveRecieveData(I2C_RegDef_t *pI2Cx )
{
	return (uint8_t)(pI2Cx->I2C_DR);
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2C,uint8_t EnOrDi)
{
	if ( EnOrDi == ENABLE)
	{
		pI2C->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2C->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2C->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2C->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2C->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2C->I2C_CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}
