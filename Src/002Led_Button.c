/*
 * 002Led_Button.c
 *
 *  Created on: Sep 18, 2024
 *      Author: Ivy
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

#define HIGH 0
#define BTN_PRESSED HIGH
void delay(void)
{
	for(uint32_t i = 0; i< 500000/2 ; i++); // // Increase the loop count for a visible delay
}

int main(void)
{
	GPIO_Handle_T GpioLed,GpioBtn;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);


	//Button gpio

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1)
	{
		//		 Check if button is pressed
		if(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13 ) == BTN_PRESSED)
			{
				// Button pressed, turn LED on
				delay();
				GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
			}

	}

}
