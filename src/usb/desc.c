//
// Created by marijn on 4/2/24.
//
#include "usb/usb.h"


/*!<
 * definitions
 * */
#define SERIAL_STRING_DESCRIPTOR_SIZE	0x1AU
#define MAX_STRING_DESCRIPTOR_SIZE		0x30U
#define DEVICE_DESCRIPTOR_SIZE			0x12U


/*!<
 * handle and struct init
 * */
uint8_t* get_device_descriptor(uint16_t* length);
uint8_t* get_lang_ID_string_descriptor(uint16_t* length);
uint8_t* get_manufacturer_string_descriptor(uint16_t* length);
uint8_t* get_product_string_descriptor(uint16_t* length);
uint8_t* get_serial_string_descriptor(uint16_t* length);
uint8_t* get_config_string_descriptor(uint16_t* length);
uint8_t* get_interface_string_descriptor(uint16_t* length);
USBD_DescriptorsTypeDef FS_Desc = {
	get_device_descriptor,
	get_lang_ID_string_descriptor,
	get_manufacturer_string_descriptor,
	get_product_string_descriptor,
	get_serial_string_descriptor,
	get_config_string_descriptor,
	get_interface_string_descriptor
};


/*!<
 * descriptors TODO: elsewhere?
 * */
__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[DEVICE_DESCRIPTOR_SIZE] __ALIGN_END = {
	0x12,                       /*bLength */
	USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
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
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
	USB_LEN_LANGID_STR_DESC,
	USB_DESC_TYPE_STRING,
	0x09U,
	0x04U
};
__ALIGN_BEGIN uint8_t USBD_StringSerial[SERIAL_STRING_DESCRIPTOR_SIZE] __ALIGN_END = {
		SERIAL_STRING_DESCRIPTOR_SIZE,
		USB_DESC_TYPE_STRING,
};
__ALIGN_BEGIN uint8_t USBD_StrDesc[MAX_STRING_DESCRIPTOR_SIZE] __ALIGN_END;


/*!<
 * functions
 * */
void set_string_descriptor(void* descriptor, uint8_t* unicode, uint16_t* len) {
	uint8_t index = 0U;
	*len = (strlen(descriptor) * 2U) + 2U;
	unicode[index++] = *(uint8_t *)len;
	unicode[index++] = USB_DESC_TYPE_STRING;
	uint8_t* src = descriptor;
	while (*src) {
		unicode[index++] = *src++;
		unicode[index++] = 0U;
	}
}
uint8_t* get_device_descriptor(uint16_t *length) {
	*length = sizeof(USBD_FS_DeviceDesc);
	return USBD_FS_DeviceDesc;
}
uint8_t* get_lang_ID_string_descriptor(uint16_t *length) {
	*length = sizeof(USBD_LangIDDesc);
	return USBD_LangIDDesc;
}
uint8_t* get_manufacturer_string_descriptor(uint16_t *length) {
	set_string_descriptor("MARIJN", USBD_StrDesc, length);
	return USBD_StrDesc;
}
uint8_t* get_product_string_descriptor(uint16_t *length) {
	set_string_descriptor("MARIJN HID", USBD_StrDesc, length);
	return USBD_StrDesc;
}
uint8_t* get_serial_string_descriptor(uint16_t *length) {
	*length = SERIAL_STRING_DESCRIPTOR_SIZE;
	uint8_t* dst = USBD_StringSerial + 2;
	uint8_t* src = (uint8_t*)UID;
	for (uint8_t i = 0; i < 12; i++) {
		*dst++ = *src++;
		*dst++ = 0x00;
	} return (uint8_t*)USBD_StringSerial;
}
uint8_t* get_config_string_descriptor(uint16_t *length) {
	set_string_descriptor("HID Config", USBD_StrDesc, length);
	return USBD_StrDesc;
}
uint8_t* get_interface_string_descriptor(uint16_t *length) {
	set_string_descriptor("HID Interface", USBD_StrDesc, length);
	return USBD_StrDesc;
}