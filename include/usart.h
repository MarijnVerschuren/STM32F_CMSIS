//
// Created by marijn on 3/20/23.
//

#ifndef STM32F_CMSIS_USART_H
#define STM32F_CMSIS_USART_H
#include "main.h"
#include "gpio.h"
#include "base.h"
#include "sys.h"


/* data layout:
 * struct:
 *		dev_id						: 16;  // MSB
 *			num							: 5;	// MSB
 *			clock						: 5;
 *			sub							: 6;	// LSB	// (reserved)
 *		alternate_function_number	: 4;
 *		port_number					: 4;
 *		pin_number					: 4;
 *		misc						: 4;  // LSB
 * */
typedef enum {
	USART_PIN_DISABLE =	0x00000000,
	// USART1
	USART1_CLK_A8 =		0x64007080,
	USART1_TX_A9 =		0x64007090,
	USART1_RX_A10 =		0x640070a0,
	USART1_CTS_A11 =	0x640070b0,
	USART1_RTS_A12 =	0x640070c0,
	USART1_TX_A15 =		0x640070f0,
	USART1_RX_B3 =		0x64007130,
	USART1_TX_B6 =		0x64007160,
	USART1_RX_B7 =		0x64007170,
	// USART2
	USART2_CTS_A0 =		0x51007000,
	USART2_RTS_A1 =		0x51007010,
	USART2_TX_A2 =		0x51007020,
	USART2_RX_A3 =		0x51007030,
	USART2_CLK_A4 =		0x51007040,
	USART2_CTS_D3 =		0x51007330,
	USART2_RTS_D4 =		0x51007340,
	USART2_TX_D5 =		0x51007350,
	USART2_RX_D6 =		0x51007360,
	USART2_CLK_D7 =		0x51007370,
	// USART6
	USART6_TX_A11 =		0x650080b0,
	USART6_RX_A12 =		0x650080c0,
	USART6_TX_C6 =		0x65008260,
	USART6_RX_C7 =		0x65008270,
	USART6_CLK_C8 =		0x65008280
} USART_GPIO_t;
typedef enum {
	USART_OVERSAMPLING_16 =	0,
	USART_OVERSAMPLING_8 =	1,
} USART_oversampling_t;


/*!< init / enable / disable */
void disable_USART(USART_TypeDef* usart);
void fconfig_UART(USART_GPIO_t tx, USART_GPIO_t rx, uint32_t baud, USART_oversampling_t oversampling);
void config_UART(USART_GPIO_t tx, USART_GPIO_t rx, uint32_t baud);
/*!< irq */
void start_USART_read_irq(USART_TypeDef* usart, io_buffer_t* buffer, uint8_t fifo);
void stop_USART_read_irq(USART_TypeDef* usart);
// void start_USART_transmit_irq(USART_TypeDef* usart, uint8_t* buffer, uint32_t size, uint8_t fifo, uint32_t hold_off);
void disable_USART_irq(USART_TypeDef* usart);
/*!< input / output */
uint32_t USART_write(USART_TypeDef* usart, const uint8_t* buffer, uint32_t size, uint32_t timeout);
uint32_t USART_read(USART_TypeDef* usart, uint8_t* buffer, uint32_t size, uint32_t timeout);
uint8_t USART_print(USART_TypeDef* usart, char* str, uint32_t timeout);

#endif //STM32F_CMSIS_USART_H
