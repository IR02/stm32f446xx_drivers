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
	uint8_t GPIO_PinMode;			/*!< possible values values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values values from @@GPIO_PIN_SPEED >*/
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

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 * GPIO Pin possible modes
 * @GPIO_PIN_MODES
 */

#define GPIO_MODE_IN				1
#define GPIO_MODE_OUT				0
#define GPIO_MODE_ALTFN				2
#define GPIO_MODE_ANALOG			3

//interrupt modes
#define GPIO_MODE_IT_FT				4 	//input falling edge trigger
#define GPIO_MODE_IT_RT				5	//input rising edge trigger
#define GPIO_Mode_IT_RFT			6	//rising falling edge trigger

/*
 * GPIO Pin possible output types
 *
 */

#define GPIO_OP_TYPE_PP 			0
#define GPIO_OP_TYPE_OD				1

/*
 * GPIO pin possible output speeds
 * @GPIO_PIN_SPEED
 */
#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_FAST				2
#define GPOI_SPEED_HIGH				3

/*
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   				0
#define GPIO_PIN_PU					1
#define GPIO_PIN_PD					2

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
