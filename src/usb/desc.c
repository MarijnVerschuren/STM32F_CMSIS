//
// Created by marijn on 4/2/24.
//
#include "usb/usb.h"


/*!<
 * definitions
 * */
#define SERIAL_STRING_DESCRIPTOR_SIZE	0x1AU
#define DEVICE_DESCRIPTOR_SIZE			0x12U
#define LANG_ID_STRING_DESCRIPTOR_SIZE	0x04U

#define  USBD_IDX_LANGID_STR                            0x00U
#define  USBD_IDX_MFC_STR                               0x01U
#define  USBD_IDX_PRODUCT_STR                           0x02U
#define  USBD_IDX_SERIAL_STR                            0x03U
#define  USBD_IDX_CONFIG_STR                            0x04U
#define  USBD_IDX_INTERFACE_STR                         0x05U


/*!<
 * descriptors TODO: elsewhere?
 * */
__ALIGN_BEGIN uint8_t device_descriptor[DEVICE_DESCRIPTOR_SIZE] __ALIGN_END = {
	0x12,                       /*bLength */
	USB_DEVICE_DESCRIPTOR,       /*bDescriptorType*/
	0x00,                       /*bcdUSB */
	0x02,
	0x00,                       /*bDeviceClass*/
	0x00,                       /*bDeviceSubClass*/
	0x00,                       /*bDeviceProtocol*/
	USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
	0x00,        			   	/*idVendor low*/
	0x00,         				/*idVendor hi*/
	0x00,        				/*idProduct low*/
	0x00,        				/*idProduct hi*/
	0x00,                       /*bcdDevice rel. 2.00*/
	0x02,
	USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
	USBD_IDX_PRODUCT_STR,       /*Index of product string*/
	USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
	USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};
__ALIGN_BEGIN uint8_t lang_ID_descriptor[LANG_ID_STRING_DESCRIPTOR_SIZE] __ALIGN_END = {
	LANG_ID_STRING_DESCRIPTOR_SIZE,
	USB_STRING_DESCRIPTOR,
	0x09U,
	0x04U
};
__ALIGN_BEGIN uint8_t manufacturer_string_descriptor[0xE] __ALIGN_END = {
		0xE,
		USB_STRING_DESCRIPTOR,
		'M', 0x00,
		'A', 0x00,
		'R', 0x00,
		'I', 0x00,
		'J', 0x00,
		'N', 0x00,
};
__ALIGN_BEGIN uint8_t product_string_descriptor[0x16] __ALIGN_END = {
		0x16,
		USB_STRING_DESCRIPTOR,
		'M', 0x00,
		'A', 0x00,
		'R', 0x00,
		'I', 0x00,
		'J', 0x00,
		'N', 0x00,
		' ', 0x00,
		'H', 0x00,
		'I', 0x00,
		'D', 0x00,
};
__ALIGN_BEGIN uint8_t serial_string_descriptor[SERIAL_STRING_DESCRIPTOR_SIZE] __ALIGN_END = {
		SERIAL_STRING_DESCRIPTOR_SIZE,
		USB_STRING_DESCRIPTOR,
		'?', 0x00,
		'?', 0x00,
		'?', 0x00,
		'?', 0x00,
		'?', 0x00,
		'?', 0x00,
		'?', 0x00,
		'?', 0x00,
		'?', 0x00,
		'?', 0x00,
		'?', 0x00,
		'?', 0x00
};
__ALIGN_BEGIN uint8_t configuration_string_descriptor[0x16] __ALIGN_END = {
		0x16,
		USB_STRING_DESCRIPTOR,
		'H', 0x00,
		'I', 0x00,
		'D', 0x00,
		' ', 0x00,
		'C', 0x00,
		'O', 0x00,
		'N', 0x00,
		'F', 0x00,
		'I', 0x00,
		'G', 0x00,
};
__ALIGN_BEGIN uint8_t interface_string_descriptor[0x1C] __ALIGN_END = {
		0x1C,
		USB_STRING_DESCRIPTOR,
		'H', 0x00,
		'I', 0x00,
		'D', 0x00,
		' ', 0x00,
		'I', 0x00,
		'N', 0x00,
		'T', 0x00,
		'E', 0x00,
		'R', 0x00,
		'F', 0x00,
		'A', 0x00,
		'C', 0x00,
		'E', 0x00,
};


USBD_DescriptorsTypeDef FS_Desc = {
	device_descriptor,
	DEVICE_DESCRIPTOR_SIZE,
	lang_ID_descriptor,
	LANG_ID_STRING_DESCRIPTOR_SIZE,
	manufacturer_string_descriptor,
	sizeof(manufacturer_string_descriptor),
	product_string_descriptor,
	sizeof(product_string_descriptor),
	serial_string_descriptor,
	SERIAL_STRING_DESCRIPTOR_SIZE,
	configuration_string_descriptor,
	sizeof(configuration_string_descriptor),
	interface_string_descriptor,
	sizeof(interface_string_descriptor),
};