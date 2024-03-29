//
// Created by marijn on 4/2/23.
//

#ifndef STM32F_CMSIS_BASE_H
#define STM32F_CMSIS_BASE_H
#include "main.h"


// device id
typedef enum {
	DEV_CLOCK_ID_AHB1 = 0,
	DEV_CLOCK_ID_AHB2 = 1,
	DEV_CLOCK_ID_APB1 = 2,
	DEV_CLOCK_ID_APB2 = 3,
	DEV_CLOCK_INVALID = 4,
} dev_clock_id_t;

typedef struct {
	uint16_t	num: 5;		// 1024
	uint16_t	clk: 5;		// dev_clock_id_t
	uint16_t	sub: 6;		// sub device info
} dev_id_t;  // 16 bit

typedef struct {
	dev_id_t	id;
	uint8_t		alt		: 4;
	uint8_t		port	: 4;
	uint8_t		num		: 4;
	uint8_t		_		: 4;
} dev_pin_t;  // 32 bit

typedef struct {
	volatile void*		ptr;
	uint32_t			size;
	volatile uint32_t	i;	// write
	volatile uint32_t	o;	// read
} io_buffer_t;


/*!< device */
void* id_to_dev(dev_id_t id);
/*!< buffer */
io_buffer_t* new_buffer(uint32_t size);
void free_buffer(io_buffer_t* buf);
/*!< string */
uint32_t strlen(const char* str);


#endif //STM32F_CMSIS_BASE_H
