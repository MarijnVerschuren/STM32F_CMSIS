//
// Created by marijn on 4/2/24.
//
#include "usb/usb.h"



// L1 ========================================= /
#define USBD_VID     0
#define USBD_LANGID_STRING     1033
#define USBD_MANUFACTURER_STRING     "MARIJN"
#define USBD_PID_FS     0
#define USBD_PRODUCT_STRING_FS     "MARIJN HID"
#define USBD_CONFIGURATION_STRING_FS     "HID Config"
#define USBD_INTERFACE_STRING_FS     "HID Interface"

#define  USB_SIZ_STRING_SERIAL       0x1A
#define USBD_MAX_STR_DESC_SIZ     512U

#define         DEVICE_ID1          (UID_BASE)
#define         DEVICE_ID2          (UID_BASE + 0x4)
#define         DEVICE_ID3          (UID_BASE + 0x8)

uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);


// L0 ========================================= /
USBD_DescriptorsTypeDef FS_Desc = {
		USBD_FS_DeviceDescriptor
		, USBD_FS_LangIDStrDescriptor
		, USBD_FS_ManufacturerStrDescriptor
		, USBD_FS_ProductStrDescriptor
		, USBD_FS_SerialStrDescriptor
		, USBD_FS_ConfigStrDescriptor
		, USBD_FS_InterfaceStrDescriptor
};


// L1 ========================================= /
__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
	0x12,                       /*bLength */
	USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
	0x00,                       /*bcdUSB */
	0x02,
	0x00,                       /*bDeviceClass*/
	0x00,                       /*bDeviceSubClass*/
	0x00,                       /*bDeviceProtocol*/
	USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
	0x00,           /*idVendor low*/
	0x00,           /*idVendor hi*/
	0x00,        /*idProduct low*/
	0x00,        /*idProduct hi*/
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
	LOBYTE(USBD_LANGID_STRING),
	HIBYTE(USBD_LANGID_STRING)
};
__ALIGN_BEGIN uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
		USB_SIZ_STRING_SERIAL,
		USB_DESC_TYPE_STRING,
};
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;


// L3 ========================================= /
static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len) {
	uint8_t idx = 0;

	for (idx = 0; idx < len; idx++) {
		if (((value >> 28)) < 0xA) {
			pbuf[2 * idx] = (value >> 28) + '0';
		}
		else {
			pbuf[2 * idx] = (value >> 28) + 'A' - 10;
		}
		value = value << 4;
		pbuf[2 * idx + 1] = 0;
	}
}
static uint8_t USBD_GetLen(uint8_t *buf) {
	uint8_t  len = 0U;
	uint8_t *pbuff = buf;

	while (*pbuff != (uint8_t)'\0') {
		len++;
		pbuff++;
	}

	return len;
}


// L2 ========================================= /
void USBD_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len) {
	uint8_t idx = 0U;
	uint8_t *pdesc;

	if (desc == NULL) {
		return;
	}

	pdesc = desc;
	*len = ((uint16_t)USBD_GetLen(pdesc) * 2U) + 2U;
	unicode[idx] = *(uint8_t *)len;
	idx++;
	unicode[idx] = USB_DESC_TYPE_STRING;
	idx++;

	while (*pdesc != (uint8_t)'\0') {
		unicode[idx] = *pdesc;
		pdesc++;
		idx++;

		unicode[idx] = 0U;
		idx++;
	}
}
static void Get_SerialNum(void) {
	uint32_t deviceserial0;
	uint32_t deviceserial1;
	uint32_t deviceserial2;

	deviceserial0 = *(uint32_t *) DEVICE_ID1;
	deviceserial1 = *(uint32_t *) DEVICE_ID2;
	deviceserial2 = *(uint32_t *) DEVICE_ID3;

	deviceserial0 += deviceserial2;

	if (deviceserial0 != 0) {
		IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
		IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
	}
}


// L1 ========================================= /
uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	UNUSED(speed);
	*length = sizeof(USBD_FS_DeviceDesc);
	return USBD_FS_DeviceDesc;
}
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	UNUSED(speed);
	*length = sizeof(USBD_LangIDDesc);
	return USBD_LangIDDesc;
}
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	UNUSED(speed);
	USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
	return USBD_StrDesc;
}
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	UNUSED(speed);
	*length = USB_SIZ_STRING_SERIAL;
	/* Update the serial number string descriptor with the data from the unique
   * ID */
	Get_SerialNum();
	return (uint8_t *) USBD_StringSerial;
}
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
	return USBD_StrDesc;
}
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
	return USBD_StrDesc;
}