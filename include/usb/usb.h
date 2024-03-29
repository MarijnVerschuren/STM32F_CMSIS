//
// Created by marijn on 7/19/23.
//

#ifndef STM32H_CMSIS_USB_F
#define STM32H_CMSIS_USB_F
#include "main.h"
#include "sys.h"
#include "nvic.h"
#include "gpio.h"
#include "usb/usb_handle.h"


/*!<
 * definitions
 * */
#define USB_OTG_ENDPOINT_COUNT		4U
#define USB_OTG_MAX_INTERFACE_COUNT	15U
#define USB_OTG_MAX_CONFIG_COUNT	1U


/*!<
 * types
 * */
typedef enum {
	USB_CLK_SRC_DISABLED =	 	 0b00,	//R
	USB_CLK_SRC_PLL1_Q =	 	 0b01,
	USB_CLK_SRC_PLL3_Q =	 	 0b10,
	USB_CLK_SRC_HSI48 =		 	 0b11
}	USB_CLK_SRC_t;
typedef enum {
	USB_PIN_DISABLE =		 	0x00000000,
	USB_VBUS_A9 =		 	 	0x7200A090,
	USB_DM_A11 =	 			0x7200A0B0,
	USB_DP_A12 =	 	 		0x7200A0C0
}	USB_GPIO_t;
typedef enum {
	USB_CLASS_COMPOSITE =		 0x0U,
	USB_CLASS_HID_KEYBOARD =	 0x1U,
	USB_CLASS_HID_MOUSE =		 0x2U,
	USB_CLASS_HID_CUSTOM =		 0x4U,
	USB_CLASS_MSC =				 0x8U
} USB_class_type_t;

typedef enum {
	USB_device_descriptor_type =			 	0x1U,
	USB_config_descriptor_type =			 	0x2U,
	USB_string_descriptor_type =			 	0x3U,
	USB_interface_descriptor_type =			 	0x4U,
	USB_endpoint_descriptor_type =			 	0x5U,
	USB_qualifier_descriptor_type =			 	0x6U,
	USB_other_speed_descriptor_type =		 	0x7U,
	//USB_interface_power_descriptor_type =		0x8U,
	//USB_OTG_descriptor_type =					0x9U,
	//USB_debug_descriptor_type =				0xAU,
	USB_IAD_descriptor_type =				 	0xBU,
	USB_BOS_descriptor_type =				 	0xFU
} USB_descriptor_type_t;
typedef enum {
	USB_device_descriptor_size =		 	 	0x12U,
	USB_config_descriptor_size =		 	 	0x09U,
	USB_interface_descriptor_size =		 	 	0x09U,
	USB_endpoint_descriptor_size =		 	 	0x07U,
	USB_qualifier_descriptor_size =		 	 	0x0AU,
	USB_other_speed_descriptor_size =	 	 	0x09U,
	//USB_interface_power_descriptor_size =	 	0x02U,
	//USB_OTG_descriptor_size =				 	0x03U
} USB_descriptor_size_t;
typedef enum {
	USB_language_ID_string_descriptor_type =	0x0U,
	USB_manufacturer_string_descriptor_type =	0x1U,
	USB_product_string_descriptor_type =		0x2U,
	USB_serial_string_descriptor_type =			0x3U,
	USB_config_string_descriptor_type =			0x4U,
	USB_interface_string_descriptor_type =		0x5U,
} USB_string_descriptor_type_t;
typedef enum {
	USB_language_ID_descriptor_size =			0x04U,
	USB_max_string_descriptor_size =	 	 	0x200U
} USB_string_descriptor_size_t;

typedef enum {
	USB_bus_powered =	0xA0U,
	USB_self_powered =	0xE0U
} USB_power_type_t;

#define HID_DESCRIPTOR_TYPE						0x21U
#define HID_REPORT_DESCRIPTOR_TYPE				0x22U

#define HID_DESCRIPTOR_SIZE						0x9U

/*!< keyboard */
#define HID_KEYBOARD_MPS						0x08U
#define HID_KEYBOARD_INTERVAL					0x0AU

// TODO: always the case??
#define HID_KEYBOARD_DESCRIPTOR_SIZE			0x22U	/* 34 */
#define HID_KEYBOARD_REPORT_DESCRIPTOR_SIZE		0xBBU	/* 187 */




/*!<
 * variables
 * */
extern uint32_t USB_kernel_frequency;


/*!<
 * init
 * */
void fconfig_USB_FS_device(USB_GPIO_t dp, USB_GPIO_t dn, uint32_t RX_FIFO_size);
void config_USB_FS_device(USB_GPIO_t dp, USB_GPIO_t dn);

USB_handle_t* fconfig_USB_handle(USB_class_type_t class_type, uint8_t iep_num, uint8_t oep_num, uint32_t descriptor_size);

/*!< descriptors */
void* create_descriptor(uint32_t size);
void* create_string_descriptor(void* str);  // TODO: w_string?
void* write_descriptor(void* ptr, USB_descriptor_type_t type, ...);
void* write_HID_descriptor(void* ptr, uint16_t release_number, uint8_t country_code, uint16_t HID_report_size);

void config_USB_RX_FIFO(uint32_t size);
void config_USB_TX_FIFO(uint8_t ep, uint32_t size);

void start_USB();
void stop_USB();

// TODO: deinit func

#endif // STM32H_CMSIS_USB_F
