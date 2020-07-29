/*
 * stm32f407xx.h
 *
 *  Created on: May 20, 2020
 *      Author: Aya Hamed
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>


#define __weak __attribute__((weak))
/**************************************************** Processor Specific Details  ****************************************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register addresses " Interrupt Set-Enable Registers "
 */

#define NVIC_ISER0		 			((volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1		 			((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2		 			((volatile uint32_t*)0xE000E108)


/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses " Interrupt Clear-Enable Registers "
 */
#define NVIC_ICER0		 			((volatile uint32_t*)0XE000E180 )
#define NVIC_ICER1		 			((volatile uint32_t*)0xE000E104 )
#define NVIC_ICER2		 			((volatile uint32_t*)0xE000E108 )

/*
 * ARM Cortex Mx Processor NVIC IPRx register addresses " Interrupt Priority Registers "
 */
#define NVIC_IPR_BASEADDR		 	((volatile uint32_t*)0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED		4



/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR			0x08000000U				/* Flash Memory Base Address */
#define SRAM1_BASEADDR			0x20000000U				/* SRAM1  Base  Address      */
#define SRAM2_BASEADDR			0x2001C000U				/* SRAM2  Base Address       */
#define ROM						0x1FFF0000U				/* System Memory Base Address*/
#define SRAM					SRAM1_BASEADDR			/* SRAM  Base Address        */



/*
 * AHBx and APBx Bus Peripheral Base addresses
 */
#define PERIPH_BASE				0x40000000U				/* Peripherals Base Address     */
#define APB1PERIPH_BASE			PERIPH_BASE				/* APB1 Bus periph Base Address */
#define APB2PERIPH_BASE			0x40010000U				/* APB2 Bus periph Base Address */
#define AHB1PERIPH_BASE			0x40020000U				/* AHB1 Bus periph Base Address */
#define AHB2PERIPH_BASE			0x50000000U				/* AHB2 Bus periph Base Address */





/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASE + 0x0000)		/* GPIO port A Base Address */
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE + 0X0400)		/* GPIO port B Base Address */
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE + 0x0800)		/* GPIO port C Base Address */
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE + 0x0C00)		/* GPIO port D Base Address */
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE + 0x1000)		/* GPIO port E Base Address */
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE + 0x1400)		/* GPIO port F Base Address */
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE + 0x1800)		/* GPIO port G Base Address */
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE + 0x1C00)		/* GPIO port H Base Address */
#define GPIOI_BASEADDR			(AHB1PERIPH_BASE + 0x2000)		/* GPIO port I Base Address */

#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x3800)		/* RCC Base Address 		*/

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400 )		/* I2C 1 Base Address */
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800 )		/* I2C 2 Base Address */
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0x5C00)		/* I2C 3 Base Address */

#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)		/* SPI 2 Base Address */
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)		/* SPI 3 Base Address */

#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)		/* USART 2 Base Address */
#define USART3_BASEADDR			(APB1PERIPH_BASE + 0x4800)		/* USART 3 Base Address */

#define USART4_BASEADDR			(APB1PERIPH_BASE + 0x4C00)		/* UART 4 Base Address */
#define USART5_BASEADDR			(APB1PERIPH_BASE + 0x5000)		/* UART 5 Base Address */






/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)		/* SPI 1 Base Address   */

#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x1000)		/* USART 1 Base Address */
#define USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400)		/* USART 6 Base Address */

#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)		/* External Interrupt Base Address  */
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0x3800)		/* System config controller Address */




/************************ peripheral register definition structure **********************/
/*
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
volatile uint32_t MODER;			/* mode register						     */
volatile uint32_t OTYPER;			/* output type register			     		 */
volatile uint32_t OSPEEDR;			/* output speed register			     	 */
volatile uint32_t PUPDR;			/* pull-up/pull-down register			     */
volatile uint32_t IDR;				/* input data register			     		 */
volatile uint32_t ODR;				/* output data register			     		 */
volatile uint32_t BSRR;				/* bit set/reset register			         */
volatile uint32_t LCKR;				/* configuration lock register			     */
volatile uint32_t AFR[2];			/* AFR[0] : alternate function low register							         * AFR[1] : alternate function high register */
}GPIO_RegDef_t ;


/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
volatile uint32_t RCC_CR;			/* clock control register 									*/
volatile uint32_t RCC_PLLCFGR;		/* PLL configuration register 								*/
volatile uint32_t RCC_CFGR;			/* clock configuration register 							*/
volatile uint32_t RCC_CIR;			/* clock interrupt register     							*/
volatile uint32_t RCC_AHB1RSTR;		/* AHB1 peripheral reset register     						*/
volatile uint32_t RCC_AHB2RSTR;		/* AHB2 peripheral reset register     						*/
volatile uint32_t RCC_AHB3RSTR;		/* AHB3 peripheral reset register     						*/
volatile uint32_t RESERVED0;		/* RESERVED     											*/
volatile uint32_t RCC_APB1RSTR;		/* APB1 peripheral reset register     						*/
volatile uint32_t RCC_APB2RSTR;		/* APB2 peripheral reset register     						*/
volatile uint32_t RESERVED1[2];		/* RESERVED     											*/
volatile uint32_t RCC_AHB1ENR;		/* AHB1 peripheral clock enable register     				*/
volatile uint32_t RCC_AHB2ENR;		/* AHB2 peripheral clock enable register     				*/
volatile uint32_t RCC_AHB3ENR;		/* AHB3 peripheral clock enable register     				*/
volatile uint32_t RESERVED2;		/* RESERVED     											*/
volatile uint32_t RCC_APB1ENR;		/* APB1 peripheral clock enable register     				*/
volatile uint32_t RCC_APB2ENR;		/* APB2 peripheral clock enable register     				*/
volatile uint32_t RESERVED3[2];		/* RESERVED     											*/
volatile uint32_t RCC_AHB1LPENR;	/* AHB1 peripheral clock enable in low power mode register	*/
volatile uint32_t RCC_AHB2LPENR;	/* AHB2 peripheral clock enable in low power mode register	*/
volatile uint32_t RCC_AHB3LPENR;	/* AHB3 peripheral clock enable in low power mode register	*/
volatile uint32_t RESERVED4;		/* RESERVED     											*/
volatile uint32_t RCC_APB1LPENR;	/* APB1 peripheral clock enable in low power mode register  */
volatile uint32_t RCC_APB2LPENR;	/* APB2 peripheral clock enable in low power mode register  */
volatile uint32_t RESERVED5[2];		/* RESERVED     											*/
volatile uint32_t RCC_BDCR;			/* Backup domain control register    						*/
volatile uint32_t RCC_CSR;			/* clock control & status register     						*/
volatile uint32_t RESERVED6[2];		/* RESERVED     											*/
volatile uint32_t RCC_SSCGR;		/* spread spectrum clock generation register     			*/
volatile uint32_t RCC_PLLI2SCFGR;	/* PLLI2S configuration register     						*/
}RCC_RegDef_t;


/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
volatile uint32_t IMR;				/* Interrupt mask register			  */
volatile uint32_t EMR;				/* Event mask register				  */
volatile uint32_t RTSR;				/* Rising trigger selection register  */
volatile uint32_t FTSR;				/* Falling trigger selection register */
volatile uint32_t SWIER;			/* Software interrupt event register  */
volatile uint32_t PR;				/* Pending register (EXTI_PR)		  */
}EXTI_RegDef_t ;


/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
volatile uint32_t MEMRMP;			/* memory remap register					   */
volatile uint32_t PMC;				/* peripheral mode configuration register	   */
volatile uint32_t EXTICR[4];		/* external interrupt configuration registers  */
volatile uint32_t RESERVED0[2];		/* reserved									   */
volatile uint32_t CMPCR;			/* Compensation cell control register		   */
}SYSCFG_RegDef_t ;



/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
volatile uint32_t SPI_CR1;				/* Control register 1				*/
volatile uint32_t SPI_CR2;				/* Control register 1				*/
volatile uint32_t SPI_SR;				/* Status register   				*/
volatile uint32_t SPI_DR;				/* Data register  					*/
volatile uint32_t SPI_CRCPR;			/* CRC polynomial register			*/
volatile uint32_t SPI_RXCRCR;			/* RX CRC register					*/
volatile uint32_t SPI_TXCRCR;			/* TX CRC register					*/
volatile uint32_t SPI_I2SCFGR;			/* SPI_I2S configuration register	*/
volatile uint32_t SPI_I2SPR;			/* SPI_I2S prescaler register		*/
}SPI_RegDef_t ;


/*
 * Peripheral register definition structure for I2C
 */
typedef struct
{
volatile uint32_t I2C_CR1;				/* Control register 1				*/
volatile uint32_t I2C_CR2;				/* Control register 2				*/
volatile uint32_t I2C_OAR1;				/* Own address register 1   		*/
volatile uint32_t I2C_OAR2;				/* Own address register 2  			*/
volatile uint32_t I2C_DR;				/* Data Register					*/
volatile uint32_t I2C_SR1;				/* Status register 1				*/
volatile uint32_t I2C_SR2;				/* Status register 2				*/
volatile uint32_t I2C_CCR;				/* Clock control register			*/
volatile uint32_t I2C_TRISE;			/*  TRISE register					*/
volatile uint32_t I2C_FLTR;				/* FLTR register					*/
}I2C_RegDef_t ;

/*
 * Peripheral register definition structure for USART
 */
typedef struct
{
volatile uint32_t USART_SR;				/* Status register					*/
volatile uint32_t USART_DR;				/* Data register					*/
volatile uint32_t USART_BRR;			/* Baud rate register   			*/
volatile uint32_t USART_CR1;			/* Control register 1 				*/
volatile uint32_t USART_CR2;			/* Control register 2				*/
volatile uint32_t USART_CR3;			/* Control register 3				*/
volatile uint32_t USART_GTPR;			/* Guard time and prescaler register*/
}USART_RegDef_t ;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1					((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1					((USART_RegDef_t*)USART1_BASEADDR)
#define USART2					((USART_RegDef_t*)USART2_BASEADDR)
#define USART3					((USART_RegDef_t*)USART3_BASEADDR)
#define USART4					((USART_RegDef_t*)USART4_BASEADDR)
#define USART5					((USART_RegDef_t*)USART5_BASEADDR)
#define USART6					((USART_RegDef_t*)USART6_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN() ( RCC->RCC_AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN() ( RCC->RCC_AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN() ( RCC->RCC_AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN() ( RCC->RCC_AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN() ( RCC->RCC_AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN() ( RCC->RCC_AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN() ( RCC->RCC_AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN() ( RCC->RCC_AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN() ( RCC->RCC_AHB1ENR |= (1 << 8) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() ( RCC->RCC_APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN() ( RCC->RCC_APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN() ( RCC->RCC_APB1ENR |= (1 << 23) )



/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() ( RCC->RCC_APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN() ( RCC->RCC_APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN() ( RCC->RCC_APB1ENR |= (1 << 15) )




/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() ( RCC->RCC_APB2ENR |= (1 << 4)  )
#define USART2_PCLK_EN() ( RCC->RCC_APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN() ( RCC->RCC_APB1ENR |= (1 << 18) )
#define USART4_PCLK_EN() ( RCC->RCC_APB1ENR |= (1 << 19) )
#define USART5_PCLK_EN() ( RCC->RCC_APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN() ( RCC->RCC_APB2ENR |= (1 << 5)  )





/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() ( RCC->RCC_APB2ENR |= (1 << 14) )


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI() ( RCC->RCC_AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI() ( RCC->RCC_AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI() ( RCC->RCC_AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI() ( RCC->RCC_AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI() ( RCC->RCC_AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI() ( RCC->RCC_AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI() ( RCC->RCC_AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI() ( RCC->RCC_AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI() ( RCC->RCC_AHB1ENR &= ~(1 << 8) )



/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI() ( RCC->RCC_APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI() ( RCC->RCC_APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI() ( RCC->RCC_APB1ENR &= ~(1 << 23) )


/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() ( RCC->RCC_APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI() ( RCC->RCC_APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI() ( RCC->RCC_APB1ENR &= ~(1 << 15) )


/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI() 		( RCC->RCC_APB2ENR &= ~(1 << 4)  )
#define USART2_PCLK_DI() 		( RCC->RCC_APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI() 		( RCC->RCC_APB1ENR &= ~(1 << 18) )
#define USART4_PCLK_DI() 		( RCC->RCC_APB1ENR &= ~(1 << 19) )
#define USART5_PCLK_DI() 		( RCC->RCC_APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_V()  		( RCC->RCC_APB2ENR &= ~(1 << 5)  )


/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() ( RCC->RCC_APB2ENR &= ~(1 << 14) )




/*
 * Macros to reset GPIOx peripherals .By setting the AHB1 RESET register then clearing it , otherwise it would be always in reset state
 */
#define GPIOA_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (1 << 0)); 	(RCC->RCC_AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (1 << 1)); 	(RCC->RCC_AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (1 << 2)); 	(RCC->RCC_AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (1 << 3)); 	(RCC->RCC_AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (1 << 4)); 	(RCC->RCC_AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (1 << 5)); 	(RCC->RCC_AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (1 << 6)); 	(RCC->RCC_AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (1 << 7)); 	(RCC->RCC_AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (1 << 8)); 	(RCC->RCC_AHB1RSTR &= ~(1 << 8)); } while(0)



/*
 * Macros to reset SPIx peripherals.
 */
#define SPI1_REG_RESET()		do{ (RCC->RCC_APB2RSTR |= (1 << 12)); 	(RCC->RCC_APB2RSTR &= ~(1 << 12)); } while(0)
#define SPI2_REG_RESET()		do{ (RCC->RCC_APB1RSTR |= (1 << 14)); 	(RCC->RCC_APB1RSTR &= ~(1 << 14)); } while(0)
#define SPI3_REG_RESET()		do{ (RCC->RCC_APB1RSTR |= (1 << 15)); 	(RCC->RCC_APB1RSTR &= ~(1 << 15)); } while(0)




/*
 * Macros to reset I2Cx peripherals.
 */
#define I2C1_REG_RESET()		do{ (RCC->RCC_APB1RSTR |= (1 << 21)); 	(RCC->RCC_APB1RSTR &= ~(1 << 21)); } while(0)
#define I2C2_REG_RESET()		do{ (RCC->RCC_APB1RSTR |= (1 << 22)); 	(RCC->RCC_APB1RSTR &= ~(1 << 22)); } while(0)
#define I2C3_REG_RESET()		do{ (RCC->RCC_APB1RSTR |= (1 << 23)); 	(RCC->RCC_APB1RSTR &= ~(1 << 23)); } while(0)


/*
 * Macros to reset USARTx peripherals.
 */
#define USART1_REG_RESET()		do{ (RCC->RCC_APB2RSTR |= (1 << 4)); 	(RCC->RCC_APB2RSTR &= ~(1 << 4)); } while(0)
#define USART2_REG_RESET()		do{ (RCC->RCC_APB1RSTR |= (1 << 17)); 	(RCC->RCC_APB1RSTR &= ~(1 << 22)); } while(0)
#define USART3_REG_RESET()		do{ (RCC->RCC_APB1RSTR |= (1 << 18)); 	(RCC->RCC_APB1RSTR &= ~(1 << 23)); } while(0)
#define USART4_REG_RESET()		do{ (RCC->RCC_APB1RSTR |= (1 << 19)); 	(RCC->RCC_APB1RSTR &= ~(1 << 21)); } while(0)
#define USART5_REG_RESET()		do{ (RCC->RCC_APB1RSTR |= (1 << 20)); 	(RCC->RCC_APB1RSTR &= ~(1 << 22)); } while(0)
#define USART6_REG_RESET()		do{ (RCC->RCC_APB2RSTR |= (1 << 5)); 	(RCC->RCC_APB2RSTR &= ~(1 << 5)); } while(0)




#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									 (x == GPIOB) ? 1 :\
									 (x == GPIOC) ? 2 :\
									 (x == GPIOD) ? 3 :\
									 (x == GPIOE) ? 4 :\
									 (x == GPIOF) ? 5 :\
									 (x == GPIOG) ? 6 :\
									 (x == GPIOH) ? 7 :\
									 (x == GPIOH) ? 8 :0)


/*
 * IRQ(Interrupt Request) Numbers
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73

#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_USART4		52
#define IRQ_NO_USART5		53
#define IRQ_NO_USART6		71

/*
 * NVIC IRQ possible priority levels macros
 */
#define NVIC_IRQ_PRIO0		0
#define NVIC_IRQ_PRIO1		1
#define NVIC_IRQ_PRIO2		2
#define NVIC_IRQ_PRIO3		3
#define NVIC_IRQ_PRIO4		4
#define NVIC_IRQ_PRIO5		5
#define NVIC_IRQ_PRIO6		6
#define NVIC_IRQ_PRIO7		7
#define NVIC_IRQ_PRIO8		8
#define NVIC_IRQ_PRIO9		9
#define NVIC_IRQ_PRIO10		10
#define NVIC_IRQ_PRIO11		11
#define NVIC_IRQ_PRIO12		12
#define NVIC_IRQ_PRIO13		13
#define NVIC_IRQ_PRIO14		14
#define NVIC_IRQ_PRIO15		15

/*****************************************************************************************************************************
 *  Bit Position Definitions of SPI peripheral
 ****************************************************************************************************************************/


/*
 * Bit Position Definitions of SPI CR1 Register
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB_FIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRC_NEXT	12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDI_OE		14
#define SPI_CR1_BIDI_MODE	15

/*
 * Bit Position Definitions of SPI CR2 Register
 */
#define SPI_CR2_RXDMAEN		0				/* Rx buffer DMA enable					*/
#define SPI_CR2_TXDMAEN		1				/* Tx buffer DMA enable					*/
#define SPI_CR2_SSOE		2				/* SS output enable						*/
#define SPI_CR2_FRF			4				/* Frame format							*/
#define SPI_CR2_ERRIE		5				/* Error interrupt enable				*/
#define SPI_CR2_RXNEIE		6				/* RX buffer not empty interrupt enable	*/
#define SPI_CR2_TXEIE		7				/* Tx buffer empty interrupt enable 	*/


/*
 * Bit Position Definitions of SPI SR Register
 */
#define SPI_SR_RXNE			0				/* Receive buffer not empty	*/
#define SPI_SR_TXE			1				/* Transmit buffer empty	*/
#define SPI_SR_CHSIDE		2				/* Channel side				*/
#define SPI_SR_UDR			3				/* Underrun flag			*/
#define SPI_SR_CRCERR		4				/* CRC error flag			*/
#define SPI_SR_MODF			5				/* Mode fault				*/
#define SPI_SR_OVR			6				/* Overrun flag 			*/
#define SPI_SR_BSY			7				/* Busy flag				*/
#define SPI_SR_FRE			8				/* Frame format error		*/


/*****************************************************************************************************************************
 *  Bit Position Definitions of I2C peripheral
 ****************************************************************************************************************************/


/*
 * Bit Position Definitions of I2C CR1 Register
 */


#define I2C_CR1_PE			0				/*  Peripheral enable 		*/
#define I2C_CR1_SMBU_S		1				/* SMBus mode 				*/
#define I2C_CR1_SMB_TYPE	3				/* SMBus type 				*/
#define I2C_CR1_ENARP		4				/* ARP enable				*/
#define I2C_CR1_ENPEC		5				/* PEC enable				*/
#define I2C_CR1_ENGC		6				/* General call enable		*/
#define I2C_CR1_NO_STRETCH	7				/* Clock stretching disable */
#define I2C_CR1_START		8				/* Start generation 		*/
#define I2C_CR1_STOP		9				/* Stop generation			*/
#define I2C_CR1_ACK			10				/* Acknowledge enable		*/
#define I2C_CR1_POS			11				/* Acknowledge/PEC Position	*/
#define I2C_CR1_PEC			12				/* Packet error checking	*/
#define I2C_CR1_ALERT		13				/* SMBus alert 				*/
#define I2C_CR1_SWRST		15				/* Software reset			*/

/*
 * Bit Position Definitions of I2C CR2 Register
 */
#define I2C_CR2_FREQ		0				/* Peripheral clock frequency	*/
#define I2C_CR2_ITERREN		8				/* Error interrupt enable 		*/
#define I2C_CR2_ITEVTEN		9				/* Event interrupt enable		*/
#define I2C_CR2_ITBUFEN 	10				/* Buffer interrupt enable		*/
#define I2C_CR2_DMAEN		11				/* DMA requests enable			*/
#define I2C_CR2_LAST		12				/* DMA last transfer 			*/



/*
 * Bit Position Definitions of I2C SR1 Register
 */
#define I2C_SR1_SB	 		0				/* Start bit				*/
#define I2C_SR1_ADDR		1				/* Address sent				*/
#define I2C_SR1_BTF 		2				/* Byte transfer finished	*/
#define I2C_SR1_ADD10		3				/* 10-bit header sent		*/
#define I2C_SR1_STOPF		4				/* Stop detection			*/
#define I2C_SR1_RxNE		6				/* Data register not empty	*/
#define I2C_SR1_TxE			7				/* Data register empty		*/
#define I2C_SR1_BERR		8				/* Bus error				*/
#define I2C_SR1_ARLO		9				/* Arbitration lost			*/
#define I2C_SR1_AF			10				/* Acknowledge failure		*/
#define I2C_SR1_OVR			11				/* Overrun/Underrun			*/
#define I2C_SR1_PECERR 		12				/* PEC Error in reception	*/
#define I2C_SR1_TIMEOUT 	14				/* Timeout or Tlow error	*/
#define I2C_SR1_SMBALERT 	15				/* SMBus alert				*/


/*
 * Bit Position Definitions of I2C SR2 Register
 */
#define I2C_SR2_MSL			0				/* Master/slave						*/
#define I2C_SR2_BUSY		1				/* Bus busy							*/
#define I2C_SR2_TRA			2				/* Transmitter/receiver 			*/
#define I2C_SR2_GENCALL		4				/* General call address				*/
#define I2C_SR2_SMBDEFAULT	5				/* SMBus device default address		*/
#define I2C_SR2_SMBHOST		6				/* SMBus host header				*/
#define I2C_SR2_DUALF 		7				/* Dual flag						*/
#define I2C_SR2_PEC			8				/* Packet error checking register	*/

/*
 * Bit Position Definitions of I2C CCR Register
 */
#define I2C_CCR_CCR			0				/*  Clock control register in Fm/Sm mode	*/
#define I2C_CCR_DUTY		14				/*  Fm mode duty cycle 						*/
#define I2C_CCR_FS			15				/* 	I2C master mode selection				*/


/*
 * Bit Position Definitions of I2C OAR1 Register
 */
#define I2C_OAR1_ADD10		0				/*  Interface address in case 10 bit	*/
#define I2C_OAR1_ADD7		1				/*  Interface address in case 7 bit		*/
#define I2C_OAR1_ADDMODE 	15				/* 	Addressing mode						*/

/*
 * Bit Position Definitions of I2C OAR2 Register
 */
#define I2C_OAR2_ENDUAL		0				/*  Dual addressing mode enable		*/
#define I2C_OAR2_ADD2		1				/*  Interface address	*/




/*****************************************************************************************************************************
 *  Bit Position Definitions of USART peripheral
 ****************************************************************************************************************************/
/*
 * Bit Position Definitions of USART SR Register
 */
#define USART_SR_PE					0 /*  Parity error					*/
#define USART_SR_FE					1 /*  Framing error					*/
#define USART_SR_NF					2 /* Noise detected flag			*/
#define USART_SR_ORE				3 /* Overrun error					*/
#define USART_SR_IDLE				4 /* IDLE line detected				*/
#define USART_SR_RXNE				5 /* Read data register not empty	*/
#define USART_SR_TC					6 /* Transmission complete			*/
#define USART_SR_TXE				7 /* Transmit data register empty	*/
#define USART_SR_LBD				8 /* LIN break detection flag 		*/
#define USART_SR_CTS				9 /* CTS flag 						*/


/*
 * Bit Position Definitions of USART_BRR Register
 */
#define USART_BRR_DIV_Fraction		0 /*  fraction of USARTDIV			*/
#define USART_BRR_DIV_Mantissa		4 /* mantissa of USARTDIV			*/

/*
 * Bit Position Definitions of USART_CR1 Register
 */
#define USART_CR1_SBK		0		/* Send break								*/
#define USART_CR1_RWU		1		/* Receiver wakeup							*/
#define USART_CR1_RE		2		/* Receiver enable							*/
#define USART_CR1_TE		3 		/* Transmitter enable						*/
#define USART_CR1_IDLEIE	4 		/* IDLE interrupt enable					*/
#define USART_CR1_RXNEIE	5 		/* RXNE interrupt enable					*/
#define USART_CR1_TCIE		6 		/* Transmission complete interrupt enable	*/
#define USART_CR1_TXEIE		7 		/* TXE interrupt enable						*/
#define USART_CR1_PEIE		8 		/* PE interrupt enable						*/
#define USART_CR1_PS		9 		/* Parity selection							*/
#define USART_CR1_PCE		10 		/* Parity control enable					*/
#define USART_CR1_WAKE		11		/* Wakeup method							*/
#define USART_CR1_M			12 		/* Word length								*/
#define USART_CR1_UE		13 		/* USART enable								*/
#define USART_CR1_OVER8		15 		/* Oversampling mode						*/


/*
 * Bit Position Definitions of USART_CR2 Register
 */
#define USART_CR2_ADD		0		/* Address of the USART node			*/
#define USART_CR2_LBDL		5 		/* lin break detection length			*/
#define USART_CR2_LBDIE		6 		/* LIN break detection interrupt enable	*/
#define USART_CR2_LBCL		8 		/* Last bit clock pulse					*/
#define USART_CR2_CPHA 		9 		/* Clock phase							*/
#define USART_CR2_CPOL		10 		/* Clock polarity						*/
#define USART_CR2_CLKEN		11		/* Clock enable							*/
#define USART_CR2_STOP		12 		/*  STOP bits							*/
#define USART_CR2_LINEN		14 		/*  LIN mode enable						*/



/*
 * Bit Position Definitions of USART_CR2 Register
 */
#define USART_CR3_EIE		0		/* Error interrupt enable		*/
#define USART_CR3_IREN		1 		/* IrDA mode enable				*/
#define USART_CR3_IRLP		2 		/* IrDA low-power				*/
#define USART_CR3_HDSEL		3 		/* Half-duplex selection		*/
#define USART_CR3_NACK 		4 		/* Smartcard NACK enable		*/
#define USART_CR3_SCEN		5 		/* Smartcard mode enable		*/
#define USART_CR3_DMAR		6		/* DMA enable receiver			*/
#define USART_CR3_DMAT		7 		/* DMA enable transmitter		*/
#define USART_CR3_RTSE		8 		/* RTS enable					*/
#define USART_CR3_CTSE		9 		/* CTS enable					*/
#define USART_CR3_CTSIE		10		/* CTS interrupt enable			*/
#define USART_CR3_ONEBIT	11 		/* One sample bit method enable	*/


/*
 * Bit Position Definitions of USART_GTPR Register
 */
#define USART_GTPR_PSC				0		/* Prescaler value		*/
#define USART_GTPR_GT 	 			8		/* Guard time value		*/

/**************************************************** Some Generic Macros ****************************************************/

#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_SET		SET
#define FLAG_RESET		RESET

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
