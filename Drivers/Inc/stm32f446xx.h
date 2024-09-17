/*
 * stm43f446xx.h
 *
 *  Created on: Sep 14, 2024
 *      Author: ivy
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>


#define FLASH_BASEADDR				0x08000000U			/*!< Flash memory is used for storing program code and constants.	*/
#define SRAM1_BASEADDR				0x20000000U			/*!< SRAM1 is used for volatile data storage during program execution. 112KB	*/
#define SRAM2_BASEADDR				0x2001C000U			/*!< SRAM2 is also used for volatile data storage. It starts immediately after SRAM1	*/
#define ROM							0x1FFF0000U			/*!< System memory (ROM) contains system functions such as bootloading routines	*/
#define SRAM						SRAM1_BASEADDR		/*!< This macro provides a convenient alias for SRAM1_BASEADDR	*/

#define PERIPH_BASE					0x40000000U			/*!< Base address of Peripheral registers. */
#define APB1PERIPH_BASEADDR			PERIPH_BASE			/*!< Base address of APB1 peripheral registers. It uses PERIPH_BASE for consistency and ease of modification. */
#define APB2PERIPH_BASEADDR			0x40010000U			/*!< Base address of APB2 peripheral registers. */

#define AHB1PERIPH_BASEADDR			0x40020000U			/*!< Base address of AHB1 peripheral registers. */
#define AHB2PERIPH_BASEADDR			0x50000000U			/*!< Base address of AHB2 peripheral registers. */
#define AHB3PERIPH_BASEADDR			0x60000000U			/*!< Base address of AHB3 peripheral registers. */

#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)
/**
 * Base addresses of peripherals which are hanging on AHB1 bus.
 *
 * The base address for GPIOX is calculated by adding the offset
 * to the base address of AHB1 bus. The offset syntax is for ease
 * of reference.
 *
 */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)


/**
 * Base addresses of peripherals which are hanging on APB1 bus.
 * I2C, SPI, UART, USART only
 *
 */

#define	I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define	I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define	I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADD				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)

/**
 * Base addresses of peripherals which are hanging on APB2 bus.
 * SPI, USART, EXTI, SYSCFG only
 *
 */

#define SPI1_BASE					(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASE					(APB2PERIPH_BASE + 0x3400)

#define USART6_BASE					(APB2PERIPH_BASE + 0x1400)
#define USART1_BASE					(APB2PERIPH_BASE + 0x1000)

#define EXTI_BASE					(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASE					(APB2PERIPH_BASE + 0x3800)




/******************************Peripheral Register Definition Structure******************************/

/**
 * Note	:	Registers of a peripheral are specific to MCU
 * e.g	:	Number of Registers of I2C peripheral of STM32F4x family of MCU may differ compared to number
 * of registers compared to number of registers of I2C of STM32F0x family
 *
 * Each register is of length 4bytes, 32bits. Using struct will automatically offset to the register address
 */

typedef struct{
	 volatile uint32_t MODER;		/*!< GPIO port mode register. >*/
	 volatile uint32_t OTYPER;		/*!< GPIO port output type register. >*/
	 volatile uint32_t OSPEEDER;	/*!< GPIO port output speed register. >*/
	 volatile uint32_t PUPDR;		/*!< GPIO port pull-up/pull-down register. >*/
	 volatile uint32_t IDR;			/*!< GPIO port input data register.	>*/
	 volatile uint32_t ODR;			/*!< GPIO port output data register. >*/
	 volatile uint32_t BSRR;		/*!< GPIO port bit set/reset register.	>*/
	 volatile uint32_t LCKR;		/*!< GPIO port configuration lock register.	>*/
	 volatile uint32_t AFR[2];		/*!< GPIO alternate function low and high register. AFR[0] LOW, AFR[1] HIGH. >*/

}GPIO_RegDef_T;


/******************************RCC Register Definition Structure******************************/
typedef struct{

	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
			 uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
			 uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	 	 	 uint32_t RESERVED3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
			 uint32_t RESERVED4;
			 uint32_t RESERVED5;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
			 uint32_t RESERVED6;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	         uint32_t RESERVED7;
	         uint32_t RESERVED8;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
			 uint32_t RESERVED9;
		     uint32_t RESERVED10;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;
}RCC_RegDef_T;

/**
 * Peripheral definitions (Peripheral base address typecasted to xxx_RegDef_t)
 */

#define GPIOA			((GPIO_RegDef_T*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_T*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_T*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_T*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_T*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_T*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_T*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_T*)GPIOH_BASEADDR)


#define RCC 			((RCC_RegDef_T*)RCC_BASEADDR)



/**
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 0) )		/*!< Enable GPIOA CLOCK. */
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 1) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 2) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 3) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 4) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 5) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 6) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 7) )


/**
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 21) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 22) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 23) )


/**
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 12) )
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 13) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 14) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 15) )
/**
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 4) )
#define USART2_PCLK_EN()	( RCC->APB1ENR |= ( 1 << 17) )
#define USART3_PCLK_EN()	( RCC->APB1ENR |= ( 1 << 18) )
#define USART4_PCLK_EN()	( RCC->APB1ENR |= ( 1 << 19) )
#define USART5_PCLK_EN()	( RCC->APB1ENR |= ( 1 << 20) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 6) )

/**
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 14) )











/**
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 0) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 1) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 2) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 3) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 4) )
#define GPIOF_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 5) )
#define GPIOG_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 6) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 7) )




/**
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 21) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 22) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 23) )

/**
 * Clock Disable Macros for SPIx peripherals
 */


#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 12) )
#define SPI4_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 13) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 14) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 15) )

/**
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 4) )
#define USART2_PCLK_DI()	( RCC->APB1ENR &= ~( 1 << 17) )
#define USART3_PCLK_DI()	( RCC->APB1ENR &= ~( 1 << 18) )
#define USART4_PCLK_DI()	( RCC->APB1ENR &= ~( 1 << 19) )
#define USART5_PCLK_DI()	( RCC->APB1ENR &= ~( 1 << 20) )
#define USART6_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 6) )


/**
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~( 1 << 14) )

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

// some generic macros
#define ENABLE 			1
#define DISABLE 		1
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET	RESET



#endif /* INC_STM32F446XX_H_ */
