//
// Created by marijn on 4/2/23.
//
#include "base.h"


/*!<
 * device
 * */
void* id_to_dev(dev_id_t id) {
	if (id.clk == DEV_CLOCK_INVALID) { return NULL; }
	if (id.clk == DEV_CLOCK_ID_AHB2) { return (void*)((id.num << 10) + AHB2PERIPH_BASE); }
	if (id.clk == DEV_CLOCK_ID_AHB1) { return (void*)((id.num << 10) + AHB1PERIPH_BASE); }
	if (id.clk == DEV_CLOCK_ID_APB2) { return (void*)((id.num << 10) + APB2PERIPH_BASE); }
	return (void*)((id.num << 10) + APB1PERIPH_BASE);
}


/*!<
 * buffer
 * */
io_buffer_t* new_buffer(uint32_t size) {
	io_buffer_t* buf = malloc(sizeof(io_buffer_t));
	if (!buf) { return NULL; }		// struct allocation error
	buf->ptr = malloc(size);
	if (!buf->ptr) { return NULL; }	// buffer allocation error
	buf->size = size;
	buf->i = 0;
	buf->o = 0;
	return buf;
}
void free_buffer(io_buffer_t* buf) { free(buf->ptr); free(buf); }


/*!<
 * string
 * */
uint32_t strlen(const char* str) {
	register uint32_t len = 0;
	while (*str++) { len++; }
	return len;
}
