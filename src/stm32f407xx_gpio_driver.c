/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: May 22, 2020
 *      Author: Aya Hamed
 */


#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */
/********************************************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for given GPIO port
 *
 * @param			- *pGPIOx : GPIO port Base Address
 * @param			- EnOrDi  : ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnOrDi)
{
	if( EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}

		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}

		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}

		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}

		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}

		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}

		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}

		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}

		else if (pGPIOx == GPIOI )
		{
			GPIOI_PCLK_EN();
		}

	}

	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}

		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}

		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}

		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}

		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}

		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}

		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}

		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}

		else if ( pGPIOx == GPIOI )
		{
			GPIOI_PCLK_DI();
		}
	}
}




/*
 * Initialization and De-Initialization
 */
/********************************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- Initialize GPIO port and GPIO Pin Configuration Settings
 *
 * @param			- *pGPIOHandle : Handle structure for a GPIO pin
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx , ENABLE);
	uint32_t temp = 0 ; /* This variable will contain the computed value to be loaded into the register */

	// 1. Configure the mode of GPIO pin
	if( pGPIOHandle-> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG )
	{
		//The non interrupt mode
		temp =( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); /* Clearing the two bit fields of the required pin */
		pGPIOHandle->pGPIOx->MODER |= temp;
	}

	else
	{
		//Interrupt Mode
		if ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_FT)
		{
			//1.Configure Falling trigger selection register FTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear corresponding RTSR bit
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RT)
		{
			//1.Configure Rising trigger selection register RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear corresponding FTSR bit
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RFT )
		{
			//1.Configure Both Rising and Falling trigger selection registers FTSR and RTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2.Configure the GPIO port selection in SYSCFG_EXTICR
		uint32_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		uint32_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4 ;
		uint8_t  portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (portcode << (temp2*4) );

		//3.Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// 2. Configure the speed
	temp =( pGPIOHandle-> GPIO_PinConfig.GPIO_PinSpeed << ( 2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;


	temp = 0;

	// 3. Configure the Pull up / Pull down settings
	temp = ( pGPIOHandle-> GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// 4. Configure the output type
	temp = ( pGPIOHandle-> GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER |= temp;


	temp = 0;

	// 5. Configure the alternative functionality
	if ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1 , temp2; /* temp1 : determines whether the pin resides in AFR[0] or AFR[1]
		                         * temp2 : determines the bit number where the value is loaded */

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8 ;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8 ;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4 * temp2 ));
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2 ) );
	}

}



/********************************************************************************************
 * @fn				- GPIO_DInit
 *
 * @brief			- De-initializes the GPIOx peripheral registers to their default reset values.
 *
 * @param  			- *pGPIOx : GPIO port Base Address
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_DInit(GPIO_RegDef_t *pGPIOx)
{
	if ( pGPIOx == GPIOA )
	{
		GPIOA_REG_RESET();
	}

	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}

	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}

	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}

	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}

	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}

	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}

	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

	else if ( pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}


}





/*
 * Data read and write
 */
/********************************************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- Reads the specified input port pin.
 *
 * @param  			- *pGPIOx   : GPIO Port Base address
 * @param			- PinNumber : GPIO pin number
 *
 * @return			- Input value
 *
 * @Note			- None
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber)
{
uint8_t value = 0;
value = (uint8_t)(pGPIOx->IDR >> PinNumber)  & 0x00000001 ;
return ( value );
}



/********************************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- Read value from input port
 *
 * @param  			- *pGPIOx   : GPIO Port Base address
 *
 * @return			- Input value
 *
 * @Note			- None
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value = 0;
	value = (uint16_t)(pGPIOx->IDR);
	return ( value );
}


/********************************************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			-Sets or clears the selected data port bit
 *
 * @param  			- *pGPIOx   : GPIO Port Base address
 * @param  			- PinNumber : GPIO pin number
 * @param  			- Value     : Output Value
 *
 * @return 			- None
 *
 * @Note			- None
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber, uint8_t Value)
{
	if ( Value == SET )
	{
		pGPIOx->ODR |= (1 << PinNumber );
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber );
	}
}


/********************************************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- Write a value to an output port
 *
 * @param  			- *pGPIOx : GPIO Port Base address
 * @param			- Value   : Output value
 *
 * @return			- None
 *
 * @Note			- None
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx , uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/********************************************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- Toggles the specified GPIO pin
 *
 * @param  			- *pGPIOx : GPIO Port Base address
 * @param			- PinNumber : GPIO pin number
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber);
}






/*
 * IRQ Configuration and ISR Handling
 */
/********************************************************************************************
 * @fn				- GPIO_IRQConfig
 *
 * @brief			- Configures IRQ number of the GPIO pin , enable or disable the interrupt ,and sets the priority
 *
 * @param			- IRQNumber  : Number of the interrupt request
 * @param			- EnORDi     : ENABLE or DISABLE macros
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnORDi)
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
 * @fn				- GPIO_IRQPriorityConfig
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. Find the number of the IPRx Register
	uint8_t iprx = IRQNumber / 4;

	//2.Find the bit field
	uint8_t iprx_section = IRQNumber % 4;

	//3.Calculate the position of the priority,knowing that each register contains 4 IRQs priorities of 8-bit length + only 4 bits of interrupt priority are used "High field"
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + (iprx)) |= ( IRQPriority << shift_amount);
	//the base address of NVIC is 4 bytes so it will increment (iprx * 4)

}



/********************************************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			- When an interrupt occurs , this function should be called to proccess the interrupt
 * 					  This function handles EXTI interrupt request.
 *
 * @param			- PinNumber : GPIO pin number
 *
 * @return			- None
 *
 * @Note			- None
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI (Pending register bit) corresponding to the pin number
	if( EXTI->PR  & (1 << PinNumber) )
	{
		//Clear : This bit is cleared by programming it to ‘1’
		EXTI->PR |= (1 << PinNumber );
	}
}
