/*
 * stm32f446xx_gpio.c
 *
 *  Created on: Sep 15, 2024
 *      Author: ivy
 */



#include "stm32f446xx_gpio_driver.h"
/*
 * Peripheral Clock Setup
 */


/*****************************************************
 * 	@fn					- GPIO_PeriClockControl
 *
 * 	@brief 				- This function enables or disables peripheral clock for the given GPIO port
 *
 * 	@param[in]			- base address of the gpio peripheral
 * 	@param[in]			- ENABLE or DISABLE macros
 *
 * 	@return				- none
 *
 * 	@Note				- none
 */

void GPIO_PeriClockControl(GPIO_RegDef_T *pGPIOx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_PCLK_EN();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_PCLK_EN();
			}else if (pGPIOx == GPIOF)
			{
				GPIOF_PCLK_EN();
			}else if (pGPIOx == GPIOG)
			{
				GPIOG_PCLK_EN();
			}else if (pGPIOx == GPIOH)
			{
				GPIOH_PCLK_EN();
			}
		}
		else
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DI();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DI();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DI();
			}else if (pGPIOx == GPIOF)
			{
				GPIOF_PCLK_DI();
			}else if (pGPIOx == GPIOG)
			{
				GPIOG_PCLK_DI();
			}else if (pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DI();
			}
		}

}


/*
 * Initialize GPIO
 */
void GPIO_Init(GPIO_Handle_T *pGPIOHandle)
{
	uint32_t temp = 0;
	//1. configure mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting

	}else
	{

	}
	temp= 0;


	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ); //clearinG
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;

	temp = 0;

	//3. configure pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ); //clearinG
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. configure the optype

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ); //clearinG
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function registers.
		uint8_t temp1, temp2;


		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << ( 4 * temp2) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << ( 4 * temp2 ) );
	}
}


/* De initialize to set GPIO to initial state
 *
 */
void GPIO_DeInt(GPIO_RegDef_T *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_T *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_T *pGPIOx)
{
	uint8_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}
void Gpio_WriteToOutputPin(GPIO_RegDef_T *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_T *pGPIOx, uint8_t Value)
{
	pGPIOx->ODR  = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_T *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR  ^= ( 1 << PinNumber);
}

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI){

}
void GPIO_IRQHandling(uint8_t PinNumber){

}


#include "stm32f446xx_gpio_driver.h"
