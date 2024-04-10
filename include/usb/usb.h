//
// Created by marijn on 7/19/23.
//

#ifndef STM32H_CMSIS_USB_F
#define STM32H_CMSIS_USB_F
#include "main.h"
#include "sys.h"
#include "nvic.h"
#include "gpio.h"


// macros and other hot fixes for BS
#define __ALIGN_BEGIN
#define __ALIGN_END

/*!<
 * definitions
 * */
// L1 ========================================= /  // TODO: typedef enum
#define USBD_MAX_SUPPORTED_CLASS                       1U
#define USBD_MAX_NUM_CONFIGURATION     1U
#define USBD_MAX_NUM_INTERFACES     1U

#define USBD_STATE_DEFAULT                              0x01U
#define USBD_STATE_ADDRESSED                            0x02U
#define USBD_STATE_CONFIGURED                           0x03U
#define USBD_STATE_SUSPENDED                            0x04U

#define USB_OTG_MODE_DEVICE                    0U

#define USB_MAX_EP0_SIZE                                64U
#define HID_FS_BINTERVAL     0xAU


#define USBD_MAX_POWER                                  0x32U /* 100 mA */


// L3 ========================================= /

#define USBD_EP0_IDLE                                   0x00U
#define USBD_EP0_SETUP                                  0x01U
#define USBD_EP0_DATA_IN                                0x02U
#define USBD_EP0_DATA_OUT                               0x03U
#define USBD_EP0_STATUS_IN                              0x04U
#define USBD_EP0_STATUS_OUT                             0x05U
#define USBD_EP0_STALL                                  0x06U


// L4 ========================================= /
#define USB_FEATURE_EP_HALT                             0x00U
#define USB_FEATURE_REMOTE_WAKEUP                       0x01U
#define USB_FEATURE_TEST_MODE                           0x02U


/*!<
 * enum types
 * */
typedef enum {
   USB_DEVICE_DESCRIPTOR =			 	0x1U,
   USB_CONFIG_DESCRIPTOR =			 	0x2U,
   USB_STRING_DESCRIPTOR =			 	0x3U,
   USB_INTERFACE_DESCRIPTOR =			0x4U,
   USB_ENDPOINT_DESCRIPTOR =			0x5U,
   USB_QUALIFIER_DESCRIPTOR =			0x6U,
   USB_OTHER_SPEED_DESCRIPTOR =			0x7U,
   USB_IAD_DESCRIPTOR =					0xBU,
   USB_BOS_DESCRIPTOR =					0xFU
} USB_descriptor_type_t;
typedef enum {
	USB_LANGUAGE_ID_STRING_DESCRIPTOR =	0X0U,
	USB_MANUFACTURER_STRING_DESCRIPTOR=	0X1U,
	USB_PRODUCT_STRING_DESCRIPTOR =		0X2U,
	USB_SERIAL_STRING_DESCRIPTOR =		0X3U,
	USB_CONFIG_STRING_DESCRIPTOR =		0X4U,
	USB_INTERFACE_STRING_DESCRIPTOR =	0X5U,
} USB_string_descriptor_type_t;

typedef enum {
	STANDARD_REQUEST =		0b00,
	CLASS_REQUEST =			0b01,
	VENDOR_REQUEST =		0b10
}	SETUP_request_type_t;
typedef enum {
	RECIPIANT_DEVICE =		0b00000,
	RECIPIANT_INTERFACE =	0b00001,
	RECIPIANT_ENDPOINT =	0b00010
}	SETUP_recipiant_t;
typedef enum {
	GET_STATUS =			0x00,
	CLEAR_FEATURE =			0x01,
	SET_FEATURE =			0x03,
	SET_ADDRESS =			0x05,
	GET_DESCRIPTOR =		0x06,
	SET_DESCRIPTOR =		0x07,
	GET_CONFIGURATION =		0x08,
	SET_CONFIGURATION =		0x09,
	GET_INTERFACE =			0x0A,
	SET_INTERFACE =			0x0B
}	SETUP_command_t;

typedef enum {
	EP_TYPE_CTRL =			0b00U,
	EP_TYPE_ISOC =			0b01U,
	EP_TYPE_BULK =			0b10U,
	EP_TYPE_INTR =			0b11U
}	EP_type_t;


/*!<
 * struct types
 * */
typedef __PACKED_STRUCT {
	uint32_t EPNUM		: 4;	// endpoint number
	uint32_t BCNT		: 11;	// byte count
	uint32_t DPID		: 2;	// data PID
	uint32_t PKTSTS		: 4;	// packet status
	uint32_t _			: 11;
}	GRXSTS_t;

typedef __PACKED_STRUCT {
	SETUP_recipiant_t		    recipiant	: 5;		// |
	SETUP_request_type_t	    type		: 2;		// | bmRequest
	uint8_t					    direction	: 1;		// |  // TODO: MSB?!!!!!!!!!!!!!!!!!!
	SETUP_command_t			    command;				// bRequest
	uint16_t				    value;					// wValue
	uint16_t				    index;					// wIndex
	uint16_t				    length;					// wLength
}	setup_header_t;


// L1 ========================================= /
typedef struct {
	uint8_t* device_descriptor;
	uint16_t device_descriptor_size;
	uint8_t* language_ID_string_descriptor;
	uint16_t language_ID_string_descriptor_size;
	uint8_t* manufacturer_string_descriptor;
	uint16_t manufacturer_string_descriptor_size;
	uint8_t* product_string_descriptor;
	uint16_t product_string_descriptor_size;
	uint8_t* serial_string_descriptor;
	uint16_t serial_string_descriptor_size;
	uint8_t* configuration_string_descriptor;
	uint16_t configuration_string_descriptor_size;
	uint8_t* interface_string_descriptor;
	uint16_t interface_string_descriptor_size;
} USBD_DescriptorsTypeDef;

typedef struct {
	uint8_t	(*config)			(void* handle, uint8_t cfgidx);
	void	(*DeInit)			(void* handle, uint8_t cfgidx);

	void	(*setup)			(void* handle, setup_header_t* req);
	void	(*EP0_TxSent)		(void* handle);
	void	(*EP0_RxReady)		(void* handle);

	void	(*DataIn)			(void* handle, uint8_t epnum);
	void	(*DataOut)			(void* handle, uint8_t epnum);
	void	(*SOF)				(void* handle);
	void	(*IsoINIncomplete)	(void* handle, uint8_t epnum);
	void	(*IsoOUTIncomplete)	(void* handle, uint8_t epnum);

	uint8_t* configuration_descriptor;
	uint16_t configuration_descriptor_size;
} USBD_ClassTypeDef;

// L2 ========================================= /
typedef struct {
	uint8_t dev_endpoints			: 4;
	uint8_t Sof_enable				: 1;
	uint8_t low_power_enable		: 1;
	uint8_t battery_charging_enable	: 1;
	uint8_t vbus_sensing_enable		: 1;
} USB_config_t;

typedef struct {
	uint16_t	mps;
	uint8_t*	buffer;
	uint16_t	size;
	uint16_t	count;
	uint16_t	status;
	EP_type_t	type				: 2;
	uint8_t		is_used				: 1;
	uint8_t		is_stall			: 1;
	uint8_t		is_iso_incomplete	: 1;
} USB_EPTypeDef;

typedef struct {
	USB_OTG_GlobalTypeDef*		instance;
	USB_config_t				config;
	__IO uint8_t				address;
	USB_EPTypeDef				IN_ep[16];
	USB_EPTypeDef				OUT_ep[16];
	uint32_t					setup[12];
	uint32_t					frame_number;

	uint32_t					dev_config;
	uint32_t					dev_default_config;
	uint32_t					dev_config_status;
	__IO uint32_t				ep0_state;
	uint32_t					ep0_data_len;
	__IO uint8_t				dev_state;
	__IO uint8_t				dev_old_state;
	uint32_t					dev_remote_wakeup;

	setup_header_t				header;
	USBD_DescriptorsTypeDef*	desc;
	USBD_ClassTypeDef*			class;
} USB_handle_t;


/*!<
 * variables
 * */
extern USBD_DescriptorsTypeDef FS_Desc;  // TODO: init
USB_handle_t USB_handle;


/*!<
 * init
 * */
void USB_device_init(USB_OTG_GlobalTypeDef*	usb);


#endif // STM32H_CMSIS_USB_F
