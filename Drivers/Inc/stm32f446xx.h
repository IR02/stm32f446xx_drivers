/*
 * stm43f446xx.h
 *
 *  Created on: Sep 14, 2024
 *      Author: ivy
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

/*
 * Memory Mapping of the Buses, peripherals hangs on certain buses refer to the register boundary address table
 *
 */

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
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)

/**
 * Base addresses of peripherals which are hanging on APB2 bus.
 * SPI, USART, EXTI, SYSCFG only
 *
 */

#define SPI1_BASE					(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASE					(APB2PERIPH_BASE + 0x3400)

#define USART6_BASEADDR				(APB2PERIPH_BASE + 0x1400)
#define USART1_BASEADDR				(APB2PERIPH_BASE + 0x1000)

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

/*
 * Structure defining the layout of GPIO registers for a microcontroller.
 *
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

typedef struct {
    volatile uint32_t SR;      /*!< USART Status Register.
                                  This register contains status flags indicating the current state of the USART peripheral.
                                  - Bit 0: TXE (Transmit Data Register Empty)
                                  - Bit 1: RXNE (Read Data Register Not Empty)
                                  - Bit 2: TC (Transmission Complete)
                                  - Bit 3: IDLE (Idle Line Detected)
                                  - Bit 4: ORE (Overrun Error)
                                  - Bit 5: NE (Noise Error Flag)
                                  - Bit 6: FE (Framing Error) */

    volatile uint32_t DR;      /*!< USART Data Register.
                                  This register is used for data transmission and reception.
                                  - Writing to this register sends data.
                                  - Reading from this register retrieves received data. */

    volatile uint32_t BRR;     /*!< USART Baud Rate Register.
                                  This register configures the baud rate for USART communication.
                                  The value in this register is derived from the system clock and the desired baud rate. */

    volatile uint32_t CR1;     /*!< USART Control Register 1.
                                  This register configures the main settings for the USART peripheral, including:
                                  - Word length, stop bits, parity control, and mode (transmitter/receiver).
                                  - Control settings for interrupts. */

    volatile uint32_t CR2;     /*!< USART Control Register 2.
                                  This register contains additional configuration options for the USART, such as:
                                  - Stop bits, clock polarity, clock phase, and synchronous mode settings. */

    volatile uint32_t CR3;     /*!< USART Control Register 3.
                                  This register allows configuration of advanced features, including:
                                  - Flow control, error management, and configuration for smartcard mode. */

    volatile uint32_t GTPR;    /*!< USART Guard Time and Prescaler Register.
                                  This register is used to set the guard time and prescaler for the USART.
                                  It is primarily used in smartcard mode to control the timing between characters. */

} USART_RegDef_T; /*!< Structure defining the layout of USART registers for a microcontroller. */


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
 * USART & UART Peripheral definitions
 * Peripheral base address typecasted to USARTx_RegDef_T
 *
 * !< Macro to access USARTx registers.
 *    This macro defines a pointer to the USART_RegDef_T structure
 *    at the memory address specified by USARTx_BASEADDR.
 *    It allows easy manipulation of USARTx control, status, and data registers
 *    using the structured representation defined by USART_RegDef_T.
 *    Example usage:
 *    	- USARTx->SR  // Access the Status Register
 *   	- USARTx->DR  // Access the Data Register
 *        This improves code readability and maintainability when interacting
 *        with USARTx hardware registers.
 *
 */

#define USART1 			(USART_RegDef_T*)USART1_BASEADDR
#define USART2			(USART_RegDef_T*)USART2_BASEADDR
#define USART3			(USART_RegDef_T*)USART3_BASEADDR
#define	UART4			(USART_RegDef_T*)UART4_BASEADDR
#define	UART5			(USART_RegDef_T*)UART5_BASEADDR
#define	USART6			(USART_RegDef_T*)USART6_BASEADDR


/**
 * Peripheral definitions (Peripheral base address typecasted to GPIOx_RegDef_T)
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
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()       ( RCC->AHB1ENR &= ~( 1 << 0) )     /*!< Disable GPIOA CLOCK. */
#define GPIOB_PCLK_DI()       ( RCC->AHB1ENR &= ~( 1 << 1) )     /*!< Disable GPIOB CLOCK. */
#define GPIOC_PCLK_DI()       ( RCC->AHB1ENR &= ~( 1 << 2) )     /*!< Disable GPIOC CLOCK. */
#define GPIOD_PCLK_DI()       ( RCC->AHB1ENR &= ~( 1 << 3) )     /*!< Disable GPIOD CLOCK. */
#define GPIOE_PCLK_DI()       ( RCC->AHB1ENR &= ~( 1 << 4) )     /*!< Disable GPIOE CLOCK. */
#define GPIOF_PCLK_DI()       ( RCC->AHB1ENR &= ~( 1 << 5) )     /*!< Disable GPIOF CLOCK. */
#define GPIOG_PCLK_DI()       ( RCC->AHB1ENR &= ~( 1 << 6) )     /*!< Disable GPIOG CLOCK. */
#define GPIOH_PCLK_DI()       ( RCC->AHB1ENR &= ~( 1 << 7) )     /*!< Disable GPIOH CLOCK. */


/**
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 21) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 22) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 23) )

/**
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()       ( RCC->APB1ENR &= ~( 1 << 21) )      /*!< Disable I2C1 CLOCK. */
#define I2C2_PCLK_DI()       ( RCC->APB1ENR &= ~( 1 << 22) )      /*!< Disable I2C2 CLOCK. */
#define I2C3_PCLK_DI()       ( RCC->APB1ENR &= ~( 1 << 23) )      /*!< Disable I2C3 CLOCK. */


/**
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 12) )
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 13) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 14) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 15) )


#define SPI1_PCLK_DI()       ( RCC->APB2ENR &= ~( 1 << 12) )      /*!< Disable SPI1 CLOCK. */
#define SPI4_PCLK_DI()       ( RCC->APB2ENR &= ~( 1 << 13) )      /*!< Disable SPI4 CLOCK. */
#define SPI2_PCLK_DI()       ( RCC->APB1ENR &= ~( 1 << 14) )      /*!< Disable SPI2 CLOCK. */
#define SPI3_PCLK_DI()       ( RCC->APB1ENR &= ~( 1 << 15) )      /*!< Disable SPI3 CLOCK. */

/**
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 4) )
#define USART2_PCLK_EN()	( RCC->APB1ENR |= ( 1 << 17) )
#define USART3_PCLK_EN()	( RCC->APB1ENR |= ( 1 << 18) )
#define UART4_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 19) )
#define UART5_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 20) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= ( 1 << 5) )

/**
 * Clock Disable Macros for USARTX and USART peripherals
 */
#define USART1_PCLK_DI()      ( RCC->APB2ENR &= ~( 1 << 4) )      /*!< Disable USART1 CLOCK. */
#define USART2_PCLK_DI()      ( RCC->APB1ENR &= ~( 1 << 17) )     /*!< Disable USART2 CLOCK. */
#define USART3_PCLK_DI()      ( RCC->APB1ENR &= ~( 1 << 18) )     /*!< Disable USART3 CLOCK. */
#define UART4_PCLK_DI()       ( RCC->APB1ENR &= ~( 1 << 19) )     /*!< Disable UART4 CLOCK. */
#define UART5_PCLK_DI()       ( RCC->APB1ENR &= ~( 1 << 20) )     /*!< Disable UART5 CLOCK. */
#define USART6_PCLK_DI()      ( RCC->APB2ENR &= ~( 1 << 5) )      /*!< Disable USART6 CLOCK. */


/**
 *@brief USART Control Register 1 (USART_CR1) Bit Definitions
 *
 * Bit definitions for configuring USART_CR1:
 *
 * @note Proper configuration is essential for reliable communication.
 */
#define USART_CR1_SBK					0  /**< Send Break: Generates a break character. */
#define USART_CR1_RWU 					1  /**< Receiver Wake Up: Enables the wake-up from mute mode. */
#define USART_CR1_RE  					2  /**< Receiver Enable: Enables the USART receiver. */
#define USART_CR1_TE 					3  /**< Transmitter Enable: Enables the USART transmitter. */
#define USART_CR1_IDLEIE 				4  /**< Idle Interrupt Enable: Enables interrupt on idle detection. */
#define USART_CR1_RXNEIE  				5  /**< Receive Not Empty Interrupt Enable: Interrupt on data reception. */
#define USART_CR1_TCIE					6  /**< Transmission Complete Interrupt Enable: Interrupt when transmission is complete. */
#define USART_CR1_TXEIE					7  /**< TXE Interrupt Enable: Enables interrupt when the transmit buffer is empty. */
#define USART_CR1_PEIE 					8  /**< Parity Error Interrupt Enable: Enables interrupt on parity error detection. */
#define USART_CR1_PS 					9  /**< Parity Selection: Selects odd or even parity. */
#define USART_CR1_PCE 					10 /**< Parity Control Enable: Enables parity checking. */
#define USART_CR1_WAKE  				11 /**< Wake Up Method: Selects wake-up method (address or idle). */
#define USART_CR1_M 					12 /**< Word Length: Configures the number of data bits (8 or 9). */
#define USART_CR1_UE 					13 /**< USART Enable: Activates the USART peripheral. */
#define USART_CR1_OVER8  				15 /**< Oversampling Mode: Selects 8 or 16 times oversampling. */


/**
 *@brief USART Control Register 2 (USART_CR2) Bit Definitions
 *
 * Bit definitions for configuring USART_CR2:
 *
 * @note Proper configuration is essential for reliable communication.
 */

#define USART_CR2_ADD   				0  /**< USART address for multi-processor mode. */
#define USART_CR2_LBDL   				5  /**< LIN Break Detection Length: 0 for 10 bits, 1 for 11 bits. */
#define USART_CR2_LBDIE  				6  /**< LIN Break Detection Interrupt Enable. */
#define USART_CR2_LBCL   				8  /**< Last Bit Clock Pulse: Output during the last bit in synchronous mode. */
#define USART_CR2_CPHA   				9  /**< Clock Phase: Selects data sampling edge. */
#define USART_CR2_CPOL   				10 /**< Clock Polarity: Idle state of the clock line (high or low). */
#define USART_CR2_CLKEN					11 /**< Clock Enable: Activates clock output in synchronous mode. */
#define USART_CR2_STOP   				12 /**< Stop Bit Selection: Configures number of stop bits (1, 1.5, or 2). */
#define USART_CR2_LINEN   				14 /**< LIN Mode Enable: Activates LIN protocol features. */

/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0   /*!< Error Interrupt Enable: Enables USART error interrupts (framing, noise, overrun). */
#define USART_CR3_IREN   				1   /*!< IrDA Mode Enable: Activates IrDA communication mode. */
#define USART_CR3_IRLP  				2   /*!< IrDA Low-Power: Enables low-power mode for IrDA. */
#define USART_CR3_HDSEL   				3   /*!< Half-Duplex Selection: Configures USART for half-duplex operation. */
#define USART_CR3_NACK   				4   /*!< NACK Transmission Enable: Allows transmission of a NACK signal. */
#define USART_CR3_SCEN   				5   /*!< Smart Card Mode Enable: Activates smart card communication mode. */
#define USART_CR3_DMAR  				6   /*!< DMA Enable Receiver: Enables DMA for receiving data. */
#define USART_CR3_DMAT   				7   /*!< DMA Enable Transmitter: Enables DMA for transmitting data. */
#define USART_CR3_RTSE   				8   /*!< RTS Enable: Enables RTS (Request to Send) flow control. */
#define USART_CR3_CTSE   				9   /*!< CTS Enable: Enables CTS (Clear to Send) flow control. */
#define USART_CR3_CTSIE   				10  /*!< CTS Interrupt Enable: Enables interrupt on CTS change. */
#define USART_CR3_ONEBIT   				11  /*!< One Bit Method: Enables one sample bit method for sampling. */

#define USART_SR_PE        				0   /*!< Parity Error: Indicates a parity error in the received data. */
#define USART_SR_FE        				1   /*!< Framing Error: Indicates a framing error in the received data. */
#define USART_SR_NE        				2   /*!< Noise Error: Indicates noise detected in the received data. */
#define USART_SR_ORE       				3   /*!< Overrun Error: Indicates an overrun error in the receiver. */
#define USART_SR_IDLE       			4   /*!< Idle Line Detected: Indicates that the idle line has been detected. */
#define USART_SR_RXNE        			5   /*!< Read Data Register Not Empty: Indicates that data is available in the receive register. */
#define USART_SR_TC        				6   /*!< Transmission Complete: Indicates that transmission is complete. */
#define USART_SR_TXE        			7   /*!< Transmit Data Register Empty: Indicates that the transmit register is empty and ready for new data. */
#define USART_SR_LBD        			8   /*!< LIN Break Detection: Indicates that a break has been detected in LIN mode. */
#define USART_SR_CTS        			9   /*!< Clear to Send: Indicates the state of the CTS signal for flow control. */

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


#include "stm32f446xx_gpio_driver.h"
#include "stm32f407xx_usart_driver.h"

#endif /* INC_STM32F446XX_H_ */
