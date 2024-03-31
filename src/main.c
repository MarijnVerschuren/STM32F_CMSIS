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


#if defined(STM32F4xx)
#define LED_GPIO_PORT GPIOC
#define LED_PIN 13
#define BTN_GPIO_PORT GPIOA
#define BTN_PIN 0


extern void TIM1_UP_TIM10_IRQHandler(void) {
	TIM10->SR &= ~TIM_SR_UIF;
	//GPIO_toggle(LED_GPIO_PORT, LED_PIN);
}
extern void EXTI0_IRQHandler(void) {
	EXTI->PR = EXTI_PR_PR0;
	//GPIO_toggle(LED_GPIO_PORT, LED_PIN);
}

int main(void) {
	// sys_clock: 25Mhz / 25 * 192 / 2 = 96Mhz
	// usb_clock: 25Mhz / 25 * 192 / 4 = 48Mhz
	SYS_CLK_Config_t* sys_config = new_SYS_CLK_config();
	set_SYS_PLL_config(sys_config, 25, 192, PLL_P_DIV2, 4, PLL_SRC_HSE);
	set_SYS_CLOCK_config(sys_config, SYS_CLK_SRC_PLL, AHB_CLK_NO_DIV, APBx_CLK_DIV2, APBx_CLK_NO_DIV, 0);
	set_SYS_FLASH_config(sys_config, FLASH_LATENCY4, 1, 1, 1);  // latency is set automatically (when need be)
	set_SYS_tick_config(sys_config, 1, 1, NULL);
	sys_clock_init(sys_config);
	free(sys_config);

	// GPIO input / output
	config_GPIO(LED_GPIO_PORT, LED_PIN, GPIO_output, GPIO_no_pull, GPIO_push_pull);
	config_GPIO(BTN_GPIO_PORT, BTN_PIN, GPIO_input, GPIO_pull_up, GPIO_push_pull);
	GPIO_write(LED_GPIO_PORT, LED_PIN, 1);  // led is active low

	// EXTI
	config_EXTI(BTN_PIN, BTN_GPIO_PORT, 1, 1);
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
	config_USB_FS_device(USB_DP_A12, USB_DM_A11);
	USB_OTG_DeviceTypeDef		*device =	(void*)((uint32_t)USB_OTG_FS + 0x800);
	USB_OTG_INEndpointTypeDef		*in =	(void*)((uint32_t)USB_OTG_FS + 0x900);
	*((uint16_t*)&USB_OTG_FS->GCCFG) = 0x21;
	USB_OTG_FS->CID = 0x4D2E562EUL; // set CID to "M.V." for fun :)

	/*

	USBD_DescriptorsTypeDef FS_Desc =
	{
	  USBD_FS_DeviceDescriptor
	, USBD_FS_LangIDStrDescriptor
	, USBD_FS_ManufacturerStrDescriptor
	, USBD_FS_ProductStrDescriptor
	, USBD_FS_SerialStrDescriptor
	, USBD_FS_ConfigStrDescriptor
	, USBD_FS_InterfaceStrDescriptor
	};
	 */

	/* DEV MM
	DCFG  0x03002000      0x00022000

	DSTS  0x07000000      0x03000000
		- 0x04000000
	 */
	/* IEP[0] MM
	DIEPCTL		0x00800080      0x00800280	->	0x00000200
		+ bit9?
	DIEPINT		0xC0200000      0xD0200000  ->	0x10000000
		+ bit28?
	DIEPTSIZ	0x12000800      0x09000800	->	-0x12000000, +0x09000000
		- bit28?
		- bit25?
		+ bit23?
		+ PKTCNT_H (bit 20)
	 */

	USB_handle_t* handle = fconfig_USB_handle(USB_CLASS_HID_KEYBOARD, 1U, 0U, HID_KEYBOARD_DESCRIPTOR_SIZE);
	// config interfaces  TODO: redo structure!!!!! ( hide handle :(( )

	(void)write_descriptor(
	write_HID_descriptor(
	write_descriptor(
	write_descriptor(
		handle->class->descriptor,
		USB_config_descriptor_type,
		0x22U, 0x01U, 0x01U, 0x00U,
		USB_bus_powered, 0x32U
	),
		USB_interface_descriptor_type,
		0x00U, 0x00U, 0x01U, 0x03U, 0x01U,
		0x01U,			// interface protocol
		0x00U
	),
		0x0111U,
		0x00,
		HID_KEYBOARD_REPORT_DESCRIPTOR_SIZE
	),
		USB_endpoint_descriptor_type,
		0x08U, 0x81U, EP_TYPE_INTERRUPT, 0x0AU
	);

	(void)write_descriptor(
		handle->descriptor->device,
		USB_device_descriptor_type,
		0x2000U,		// USB 2.0
		0x03U,			// HID TODO: define
		0x00U,			// no subclasss
		0x01U,			// device protocol set to the same as inteface TODO: valid??
		64U,			// EP0_MPS TODO: define
		0x0000,			// vendor ID
		0x0000,			// product ID
		0x2000,			// device version (USB 2.0??) TODO: valid?
		0x1U,			// manufacturer string index
		0x2U,			// product string index
		0x3U,			// serial string index
		0x1U			// config count
	);

	handle->descriptor->lang_ID_string = create_string_descriptor("NL");
	handle->descriptor->manufacturer_string = create_string_descriptor("Marijn");
	handle->descriptor->product_string = create_string_descriptor("Keyboard");
	handle->descriptor->serial_string = create_string_descriptor("fb49484a-ce2a-466e-aded-073dab3a483b");
	handle->descriptor->configuration_string = create_string_descriptor("");
	handle->descriptor->interface_string = create_string_descriptor("");

	start_USB();

	// TODO: IEP/OEP0 configured in USBD_LL_Reset!!!!!!!!!!!!!!
	// main loop
	for(;;) {
		//reset_watchdog();
	}
}
#elif defined(STM32F3xx)
int main(void) {}
#endif