//
// Created by marijn on 3/20/23.
//

#ifndef STM32F_CMSIS_USART_H
#define STM32F_CMSIS_USART_H
#include "main.h"
#include "gpio.h"
#include "sys.h"


/* data layout:
 * struct:
 *		dev_id						: 8;  // MSB
 *		misc						: 4;  // (reserved)
 *		alternate_function_number	: 4;
 *		port_number					: 4;
 *		pin_number					: 4;  // LSB
 * */
typedef enum {
	USART_PIN_DISABLE =	0x000000,
	// USART1
	USART1_CLK_A8 =		0x640708,
	USART1_TX_A9 =		0x640709,
	USART1_RX_A10 =		0x64070A,
	USART1_CTS_A11 =	0x64070B,
	USART1_RTS_A12 =	0x64070C,
	USART1_TX_A15 =		0x64070F,
	USART1_RX_B3 =		0x640713,
	USART1_TX_B6 =		0x640716,
	USART1_RX_B7 =		0x640717,
	// USART2
	USART2_CTS_A0 =		0x510700,
	USART2_RTS_A1 =		0x510701,
	USART2_TX_A2 =		0x510702,
	USART2_RX_A3 =		0x510703,
	USART2_CLK_A4 =		0x510704,
	USART2_CTS_D3 =		0x510733,
	USART2_RTS_D4 =		0x510734,
	USART2_TX_D5 =		0x510735,
	USART2_RX_D6 =		0x510736,
	USART2_CLK_D7 =		0x510737,
	// USART6
	USART6_TX_A11 =		0x65080B,
	USART6_RX_A12 =		0x65080C,
	USART6_TX_C6 =		0x650826,
	USART6_RX_C7 =		0x650827,
	USART6_CLK_C8 =		0x650828
} USART_GPIO_TypeDef;
typedef enum {
	USART_OVERSAMPLING_16 =	0,
	USART_OVERSAMPLING_8 =	1,
} USART_oversampling_TypeDef;


/*!< misc */
dev_id_t USART_to_id(USART_TypeDef* usart);
USART_TypeDef* id_to_USART(dev_id_t id);
/*!< init / enable / disable */
void disable_USART(USART_TypeDef* usart);
void fconfig_UART(USART_TypeDef* uart, uint32_t baud, USART_GPIO_TypeDef tx, USART_GPIO_TypeDef rx, USART_oversampling_TypeDef oversampling);
void config_UART(USART_TypeDef* uart, uint32_t baud, USART_GPIO_TypeDef tx, USART_GPIO_TypeDef rx);
/*!< input / output */
uint8_t USART_transmit(USART_TypeDef* usart, uint8_t byte, uint32_t timeout);
uint8_t USART_receive(USART_TypeDef* usart, uint8_t* byte, uint32_t timeout);
uint32_t USART_write(USART_TypeDef* usart, const uint8_t* buffer, uint32_t size, uint32_t timeout);
uint32_t USART_read(USART_TypeDef* usart, uint8_t* buffer, uint32_t size, uint32_t timeout);
uint8_t USART_print(USART_TypeDef* usart, char* str, uint32_t timeout);

#endif //STM32F_CMSIS_USART_H
