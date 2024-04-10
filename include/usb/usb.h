//
// Created by marijn on 7/19/23.
//

#ifndef STM32H_CMSIS_USB_F
#define STM32H_CMSIS_USB_F
#include "main.h"
#include "sys.h"
#include "nvic.h"
#include "gpio.h"


/*!<
 * definitions
 * */
#define MAX_CONFIGURATION_COUNT     		1U
#define MAX_INTERFACE_COUNT     			1U
#define EP0_MPS								0x40U
#define MAX_POWER							0x32U /* 100 mA */


/*!<
 * enum types
 * */
typedef enum {
	STANDARD_REQUEST =						0b00,
	CLASS_REQUEST =							0b01,
	VENDOR_REQUEST =						0b10
}	SETUP_request_type_t;
typedef enum {
	RECIPIANT_DEVICE =						0b00000,
	RECIPIANT_INTERFACE =					0b00001,
	RECIPIANT_ENDPOINT =					0b00010
}	SETUP_recipiant_t;
typedef enum {
	GET_STATUS =							0x00,
	CLEAR_FEATURE =							0x01,
	SET_FEATURE =							0x03,
	SET_ADDRESS =							0x05,
	GET_DESCRIPTOR =						0x06,
	SET_DESCRIPTOR =						0x07,
	GET_CONFIGURATION =						0x08,
	SET_CONFIGURATION =						0x09,
	GET_INTERFACE =							0x0A,
	SET_INTERFACE =							0x0B
}	SETUP_command_t;

typedef enum {
	USB_DEVICE_DESCRIPTOR =					0x1U,
	USB_CONFIG_DESCRIPTOR =					0x2U,
	USB_STRING_DESCRIPTOR =					0x3U,
	USB_INTERFACE_DESCRIPTOR =				0x4U,
	USB_ENDPOINT_DESCRIPTOR =				0x5U,
	USB_QUALIFIER_DESCRIPTOR =				0x6U,
	USB_OTHER_SPEED_DESCRIPTOR =			0x7U,
	USB_IAD_DESCRIPTOR =					0xBU,
	USB_BOS_DESCRIPTOR =					0xFU
}	USB_descriptor_type_t;
typedef enum {
	USB_LANGUAGE_ID_STRING_DESCRIPTOR =		0x0U,
	USB_MANUFACTURER_STRING_DESCRIPTOR =	0x1U,
	USB_PRODUCT_STRING_DESCRIPTOR =			0x2U,
	USB_SERIAL_STRING_DESCRIPTOR =			0x3U,
	USB_CONFIG_STRING_DESCRIPTOR =			0x4U,
	USB_INTERFACE_STRING_DESCRIPTOR =		0x5U,
}	USB_string_descriptor_type_t;

typedef enum {
	USB_FEATURE_EP_HALT	=					0x0U,
	USB_FEATURE_REMOTE_WAKEUP =				0x1U,
	USB_FEATURE_TEST_MODE =					0x2U
}	USB_feature_t;

typedef enum {
	EP_TYPE_CTRL =							0b00U,
	EP_TYPE_ISOC =							0b01U,
	EP_TYPE_BULK =							0b10U,
	EP_TYPE_INTR =							0b11U
}	EP_type_t;
typedef enum {
	EP0_IDLE =								0x0U,
	EP0_SETUP =								0x1U,
	EP0_DATA_IN =							0x2U,
	EP0_DATA_OUT =							0x3U,
	EP0_STATUS_IN =							0x4U,
	EP0_STATUS_OUT =						0x5U,
	EP0_STALL =								0x6U,
}	EP0_status_t;

typedef enum {
	DEV_STATE_DEFAULT =						0x0U,
	DEV_STATE_ADDRESSED =					0x1U,
	DEV_STATE_CONFIGURED =					0x2U,
	DEV_STATE_SUSPENDED =					0x3U
}	DEV_status_t;


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
} descriptor_handle_t;

typedef struct {
	uint8_t	(*init)					(void* handle, uint8_t cfgidx);
	void	(*deinit)				(void* handle, uint8_t cfgidx);

	void	(*setup)				(void* handle, setup_header_t* req);
	void	(*IEP0_complete)		(void* handle);
	void	(*OEP0_ready)			(void* handle);

	void	(*data_IN)				(void* handle, uint8_t epnum);
	void	(*data_OUT)				(void* handle, uint8_t epnum);
	void	(*SOF)					(void* handle);
	void	(*ISO_IN_incomplete)	(void* handle, uint8_t epnum);
	void	(*ISO_OUT_incomplete)	(void* handle, uint8_t epnum);

	uint8_t* configuration_descriptor;
	uint16_t configuration_descriptor_size;
} class_handle_t;

typedef struct {
	uint8_t dev_endpoints			: 4;
	uint8_t SOF_enable				: 1;
	uint8_t low_power_enable		: 1;
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
} EP_handle_t;

typedef struct {
	USB_OTG_GlobalTypeDef*		instance;
	uint32_t					setup[12];
	uint32_t					frame_number;

	EP_handle_t					IN_ep[16];
	EP_handle_t					OUT_ep[16];

	descriptor_handle_t*		desc;
	class_handle_t*				class;

	setup_header_t				header;
	uint16_t					dev_config_status;
	uint8_t						dev_default_config;
	uint8_t						dev_config;
	__IO uint8_t				address;
	USB_config_t				config;

	__IO DEV_status_t			dev_state		: 2;
	__IO DEV_status_t			dev_old_state	: 2;
	__IO EP0_status_t			ep0_state		: 3;
	uint8_t						remote_wakeup	: 1;
} USB_handle_t;


/*!<
 * variables
 * */
extern descriptor_handle_t FS_Desc;  // TODO: init
USB_handle_t USB_handle;


/*!<
 * init
 * */
void config_USB(
	USB_OTG_GlobalTypeDef* usb, class_handle_t* class, descriptor_handle_t* desc,
	uint8_t enable_SOF, uint8_t enable_low_power
);
void start_USB(USB_OTG_GlobalTypeDef* usb);

#endif // STM32H_CMSIS_USB_F
