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
void GPIO_PeriClockControl(void);

/*
 * Init and De-init
 */
void GPIO_Init(void);
void GPIO_DeInt(void);

/*
 * Data read and write
 */
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void Gpio_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);

/*
 * IRQ confirgation and ISR handling
 */
void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
