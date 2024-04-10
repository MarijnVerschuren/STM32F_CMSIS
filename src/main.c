#include "main.h"
#include "gpio.h"	// all pins are defined here
#include "exti.h"
#include "tim.h"	// all timers and delays are defined here
#include "sys.h"
#include "usart.h"
#include "pwm.h"
#include "crc.h"
#include "i2c.h"
#include "rng.h"
#include "encoder.h"
#include "watchdog.h"
#include "usb/usb.h"
#include "usb/hid.h"


#if defined(STM32F4xx)
#define LED_GPIO_PORT GPIOC
#define LED_PIN 13
#define BTN_GPIO_PORT GPIOA
#define BTN_PIN 0


uint8_t HID_buffer[8] = {0, 0, 0x4, 0, 0, 0, 0, 0};
volatile uint8_t GO = 0;

extern void TIM1_UP_TIM10_IRQHandler(void) {
	TIM10->SR &= ~TIM_SR_UIF;
	//GPIO_toggle(LED_GPIO_PORT, LED_PIN);
}
extern void EXTI0_IRQHandler(void) {
	EXTI->PR = EXTI_PR_PR0;
	GO = 1;
}

int main(void) {
	// sys_clock: 25Mhz / 25 * 192 / 2 = 96Mhz
	// usb_clock: 25Mhz / 25 * 192 / 4 = 48Mhz
	set_SYS_PLL_config(25, 192, PLL_P_DIV2, 4, PLL_SRC_HSE);
	set_SYS_CLOCK_config(SYS_CLK_SRC_PLL, AHB_CLK_NO_DIV, APBx_CLK_DIV2, APBx_CLK_NO_DIV, 0);
	set_SYS_FLASH_config(FLASH_LATENCY4, 1, 1, 1);  // latency is set automatically (when need be)
	set_SYS_tick_config(1, 1, NULL);
	sys_clock_init();

	// GPIO input / output
	config_GPIO(LED_GPIO_PORT, LED_PIN, GPIO_output, GPIO_no_pull, GPIO_push_pull);
	config_GPIO(BTN_GPIO_PORT, BTN_PIN, GPIO_input, GPIO_pull_up, GPIO_push_pull);
	GPIO_write(LED_GPIO_PORT, LED_PIN, 1);  // led is active low

	// EXTI
	config_EXTI(BTN_PIN, BTN_GPIO_PORT, 1, 0);
	start_EXTI(BTN_PIN);

	// initialize CRC
	/*enable_CRC();  // polynomial: 0x4C11DB7
	*/

	// UART input
	/*io_buffer_t* uart_buf = new_buffer(1024);
	if (!uart_buf) { return -1; }  // allocation error
	config_UART(USART1_TX_A9, USART1_RX_A10, 115200);
	start_USART_read_irq(USART1, uart_buf, 1);
	*/

	// UART buffer polling interrupt
	/*config_TIM(TIM10, 1000, 20000);  // 5 Hz
	start_TIM_update_irq(TIM10);  // TIM1_UP_TIM10_IRQHandler
	start_TIM(TIM10);
	*/

	// PWM output
	/*config_PWM(TIM4_CH4_B9, 100, 20000); TIM4->CCR4 = 550;
	 */

	// RNG generator
	/*start_RNG();
	 */

	// I2C
	/*config_I2C(I2C1_SCL_B6, I2C1_SDA_B7, 0);
	 */

	// S0S90 external counter
	/*config_encoder_S0S90(TIM2_CH1_A0, TIM2_CH2_A1);
	start_encoder_S0S90(TIM2);
	*/

	// watchdog
	/*config_watchdog(0, 0xfff);  // 512 ms
	start_watchdog();
	*/

	// USB
	USB_device_init(USB_OTG_FS);
USB_OFF:
	GPIO_write(LED_GPIO_PORT, LED_PIN, 1);
	while (USB_handle.dev_state != USBD_STATE_CONFIGURED);
	GPIO_write(LED_GPIO_PORT, LED_PIN, 0);

	uint8_t code[6] = {4, 4, 4, 4, 3, 3};
	uint8_t i;
	uint8_t delay = 18;  // min: 18
	// main loop
	for(;;) {
		if (USB_handle.dev_state != USBD_STATE_CONFIGURED) { goto USB_OFF; }
		if (!GO) { continue; }
		for (i = 0; i < 6; i++) {
			HID_buffer[2] = code[i] + 0x1E;
			send_HID_report(&USB_handle, HID_buffer, 8);
			delay_ms(delay);
			HID_buffer[2] = 0;
			send_HID_report(&USB_handle, HID_buffer, 8);
			delay_ms(delay);
		}
		HID_buffer[2] = 0x28;
		send_HID_report(&USB_handle, HID_buffer, 8);
		delay_ms(delay);
		HID_buffer[2] = 0;
		send_HID_report(&USB_handle, HID_buffer, 8);
		delay_ms(delay);

		for (i = 0; i < 6; i++) {
			code[i] = (code[i] + 1) % 10;
			if (code[i]) { break; }
		}

		GO = 0;
	}
}
#elif defined(STM32F3xx)
int main(void) {}
#endif