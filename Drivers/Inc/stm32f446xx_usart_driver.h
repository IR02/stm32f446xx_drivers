/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: Sep 24, 2024
 *      Author: Ivy
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include "stm32f446xx.h"

/*
 *	@USART_Mode
 *	Possible options for USART_Mode
 */

#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX    2

/*
 *	@USART_Baud
 *	Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/*
 *	@USART_ParityControl
 *	Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE  0

/*
 *	@USART_WordLength
 *	Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *	@USART_NoOfStopBits
 *	Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3



/*
 * Configuration structure for USARTx peripheral
 */

typedef struct{
	 uint32_t USART_BaudRate;             /*!< Baud rate for the USART communication */
	 uint8_t USART_WordLength;            /*!< Possible values from @USART_WORD_LENGTH */
	 uint8_t USART_StopBits;              /*!< Possible values from @USART_STOP_BITS */
	 uint8_t USART_Parity;                /*!< Possible values from @USART_PARITY */
	 uint8_t USART_Mode;                  /*!< Possible values from @USART_MODE (e.g., transmit, receive, or both) */
	 uint8_t USART_HWFlowControl;         /*!< Possible values from @USART_HW_FLOW_CONTROL */

}USART_Config_T;


/*
 * Handle structure for USARTx peripheral
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;      // Pointer to the USART register definition structure
	USART_Config_t  USART_Config; // Configuration settings for the USART peripheral
} USART_Handle_T;



/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/


#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
