/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: May 22, 2020
 *      Author: Aya Hamed
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber; 		/* Pin Number		: possible values from @GPIO_PIN_NUMBERS*/
	uint8_t GPIO_PinMode;			/* Pin Mode	 		: possible values from @GPIO_PIN_MODES	*/
	uint8_t GPIO_PinSpeed;			/* Pin Speed 		: possible values from @GPIO_PIN_SPEED	*/
	uint8_t GPIO_PinPuPdControl;	/* Pull-up/Pull-down: possible values from @GPIO_PIN_PUPD	*/
	uint8_t GPIO_PinOPType;			/* Pin output Type	: possible values from @GPIO_OP_TYPE	*/
	uint8_t GPIO_PinAltFunMode; 	/* Alternative Function Mode*/
}GPIO_PinConfig_t;


/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx ;				/* This holds the base address of the GPIO port */
	GPIO_PinConfig_t GPIO_PinConfig; 	/* This holds GPIO Pin Configuration Settings   */
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15



/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes for " GPIO port mode register"
 */
#define	GPIO_MODE_IN 		0		/* Input mode						*/
#define	GPIO_MODE_OUT 		1		/* Output mode						*/
#define	GPIO_MODE_ALTFN		2		/* Alternate function mode			*/
#define	GPIO_MODE_ANALOG	3		/* Analog mode 						*/
#define	GPIO_MODE_IN_FT		4		/* Input Falling Edge Trigger		*/
#define	GPIO_MODE_IN_RT		5		/* Input Rising Edge Trigger 		*/
#define	GPIO_MODE_IN_RFT	6		/* Input Falling Rising Edge Trigger*/


/*
 * GPIO pin possible output types for " GPIO port output type register"
 * @GPIO_OP_TYPE
 */
#define GPIO_OP_TYPE_PP		0		/* Output push pull		*/
#define GPIO_OP_TYPE_OD 	1		/* Output open drain	*/

/*
 * GPIO pin possible output speeds for " GPIO port output speed register"
 * @GPIO_PIN_SPEED
 */
#define GPIO_SPEED_LOW		0		/* Low output speed	    */
#define GPIO_SPEED_MED		1		/* Medium output speed  */
#define GPIO_SPEED_FAST		2		/* Fast output speed	*/
#define GPIO_SPEED_HIGH		3		/* High speed output	*/


/*
 * GPIO pin pull up and pull down configuration macros for " GPIO port pull-up/pull-down register"
 * @GPIO_PIN_PUPD
 */
#define GPIO_NO_PUPD		0		/* No pull-up, pull-down */
#define GPIO_PIN_PU			1		/* Pull up				 */
#define	GPIO_PIN_PD			2		/* Pull down			 */

/*******************************************************************************************
* 								APIs supported by this driver
* 			For more information about the APIs check the function definitions
*******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnOrDi);





/*
 * Initialization and De-Initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DInit(GPIO_RegDef_t *pGPIOx);




/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx , uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber);





/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnORDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber ,uint32_t IRQPriority);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
