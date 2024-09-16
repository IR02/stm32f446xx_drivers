/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Sep 15, 2024
 *      Author: ivy
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"


/*
 * This is a Configuration structure for GPIO pin
 */
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConfig_T;

/**
 * This is a handle structure for GPIO pin
 */

typedef struct{

	GPIO_RegDef_T *pGPIOx;/*!< This holds the base address of the GPIO PORT to which the pin belongs >*/
	GPIO_PinConfig_T GPIO_PinConfig; /*! < This holds GPIO pin configuration settings >*/
}GPIO_Handle_T;



/***************************************************************************************************************
 * 									APIs supported by this driver
 * 				For more information bout the APIs check the function definitions
 */


/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_T *pGPIOx, uint8_t ENorDI);

/*
 * Initialize GPIO
 */
void GPIO_Init(GPIO_Handle_T *pGPIOHandle);


/* De initialize to set GPIO to initial state
 *
 */
void GPIO_DeInt(GPIO_RegDef_T *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_T *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_T *pGPIOx);
void Gpio_WriteToOutputPin(GPIO_RegDef_T *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_T *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_T *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
