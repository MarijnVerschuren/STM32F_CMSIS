//
// Created by marijn on 4/2/24.
//

#ifndef STM32F_CMSIS_USB_HAL_CPY_HID_H
#define STM32F_CMSIS_USB_HAL_CPY_HID_H


// L1 ========================================= /
#define USBD_HID_REQ_SET_PROTOCOL                       0x0BU
#define USBD_HID_REQ_GET_PROTOCOL                       0x03U
#define USBD_HID_REQ_SET_IDLE                           0x0AU
#define USBD_HID_REQ_GET_IDLE                           0x02U


typedef enum {
	USBD_HID_IDLE = 0,
	USBD_HID_BUSY,
} USBD_HID_StateTypeDef;

typedef struct {
	uint32_t Protocol;
	uint32_t IdleState;
	uint32_t AltSetting;
	USBD_HID_StateTypeDef state;
} USBD_HID_HandleTypeDef;

typedef struct {
	uint8_t           bLength;
	uint8_t           bDescriptorType;
	uint16_t          bcdHID;
	uint8_t           bCountryCode;
	uint8_t           bNumDescriptors;
	uint8_t           bHIDDescriptorType;
	uint16_t          wItemLength;
} __PACKED USBD_HIDDescTypeDef;

typedef struct {
	uint8_t   bLength;
	uint8_t   bDescriptorType;
	uint8_t   bEndpointAddress;
	uint8_t   bmAttributes;
	uint16_t  wMaxPacketSize;
	uint8_t   bInterval;
} __PACKED USBD_EpDescTypeDef;


// L2 ========================================= /
typedef  struct {
	uint8_t  bLength;
	uint8_t  bDescriptorType;
	uint8_t  bDescriptorSubType;
} USBD_DescHeaderTypeDef;

typedef struct {
	uint8_t   bLength;
	uint8_t   bDescriptorType;
	uint16_t  wTotalLength;
	uint8_t   bNumInterfaces;
	uint8_t   bConfigurationValue;
	uint8_t   iConfiguration;
	uint8_t   bmAttributes;
	uint8_t   bMaxPower;
} __PACKED USBD_ConfigDescTypeDef;


//
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);


#endif // STM32F_CMSIS_USB_HAL_CPY_HID_H
