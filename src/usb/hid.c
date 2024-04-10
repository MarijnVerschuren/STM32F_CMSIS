//
// Created by marijn on 4/2/24.
//
#include "usb/usb.h"
#include "usb/hid.h"


/*!<
 * defines
 * */
#define HID_IEP							0x01U
#define HID_MPS							0x04U

#define HID_CONFIG_DESCRIPTOR_SIZE		34U
#define HID_DESCRIPTOR_SIZE				9U
#define HID_DESCRIPTOR_TYPE				0x21U
#define HID_REPORT_DESCRIPTOR_TYPE		0x22U
#define HID_REPORT_DESCRIPTOR_SIZE		63U


/*!<
 * types
 * */
typedef enum {
	USBD_HID_IDLE = 0,
	USBD_HID_BUSY,
} HID_state_t;

typedef enum {
	HID_SET_PROTOCOL =	0x0BU,
	HID_GET_PROTOCOL =	0x03U,
	HID_SET_IDLE =		0x0AU,
	HID_GET_IDLE =		0x02U
} HID_command_t;

typedef struct {
	uint8_t		protocol;
	uint8_t		idle;
	uint8_t		alt_setting;
	HID_state_t	state;
} __PACKED HID_handle_t;


/*!<
 * descriptors TODO: elsewhere?
 * */
__ALIGN_BEGIN uint8_t USBD_HID_CfgDesc[HID_CONFIG_DESCRIPTOR_SIZE] __ALIGN_END = {
	0x09,                                               /* bLength: Configuration Descriptor size */
	USB_CONFIG_DESCRIPTOR,                        /* bDescriptorType: Configuration */
	HID_CONFIG_DESCRIPTOR_SIZE,                         /* wTotalLength: Bytes returned */
	0x00,
	0x01,                                               /* bNumInterfaces: 1 interface */
	0x01,                                               /* bConfigurationValue: Configuration value */
	0x00,                                               /* iConfiguration: Index of string descriptor describing the configuration */
	0xA0,                                               /* bmAttributes: Bus Powered according to user configuration */
	USBD_MAX_POWER,                                     /* MaxPower (mA) */
	/************** Descriptor of Joystick Mouse interface ****************/
	0x09,                                               /* bLength: Interface Descriptor size */
	USB_INTERFACE_DESCRIPTOR,                            /* bDescriptorType: Interface descriptor type */
	0x00,                                               /* bInterfaceNumber: Number of Interface */
	0x00,                                               /* bAlternateSetting: Alternate setting */
	0x01,                                               /* bNumEndpoints */
	0x03,                                               /* bInterfaceClass: HID */
	0x01,                                               /* bInterfaceSubClass : 1=BOOT, 0=no boot */
	0x01,                                               /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
	0,                                                  /* iInterface: Index of string descriptor */
	/******************** Descriptor of Joystick Mouse HID ********************/
	0x09,                                               /* bLength: HID Descriptor size */
	HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
	0x11,                                               /* bcdHID: HID Class Spec release number */
	0x01,
	0x00,                                               /* bCountryCode: Hardware target country */
	0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
	0x22,                                               /* bDescriptorType */
	HID_REPORT_DESCRIPTOR_SIZE,                         /* wItemLength: Total length of Report descriptor */
	0x00,
	/******************** Descriptor of Mouse endpoint ********************/
	0x07,                                               /* bLength: Endpoint Descriptor size */
	USB_ENDPOINT_DESCRIPTOR,                             /* bDescriptorType:*/
	HID_IEP | 0x80,                       		        /* bEndpointAddress: Endpoint Address (IN) */
	0x03,                                               /* bmAttributes: Interrupt endpoint */
	HID_MPS,                               		        /* wMaxPacketSize: 4 Bytes max */
	0x00,
	HID_FS_BINTERVAL,                                   /* bInterval: Polling Interval */
};
// NOTE: this is actually a keyboard descriptor!!!!
__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[HID_REPORT_DESCRIPTOR_SIZE] __ALIGN_END = {
	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	0x09, 0x06,                    // USAGE (Keyboard)
	0xa1, 0x01,                    // COLLECTION (Application)
	0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
	0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
	0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
	0x75, 0x01,                    //   REPORT_SIZE (1)
	0x95, 0x08,                    //   REPORT_COUNT (8)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
	0x95, 0x05,                    //   REPORT_COUNT (5)
	0x75, 0x01,                    //   REPORT_SIZE (1)
	0x05, 0x08,                    //   USAGE_PAGE (LEDs)
	0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
	0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
	0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x75, 0x03,                    //   REPORT_SIZE (3)
	0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
	0x95, 0x06,                    //   REPORT_COUNT (6)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
	0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
	0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event in322222dicated))
	0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
	0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
	0xc0                           // END_COLLECTION
};
__ALIGN_BEGIN static uint8_t USBD_HID_Desc[HID_DESCRIPTOR_SIZE] __ALIGN_END = {
	0x09,                                               /* bLength: HID Descriptor size */
	HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
	0x11,                                               /* bcdHID: HID Class Spec release number */
	0x01,
	0x00,                                               /* bCountryCode: Hardware target country */
	0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
	0x22,                                               /* bDescriptorType */
	HID_REPORT_DESCRIPTOR_SIZE,                         /* wItemLength: Total length of Report descriptor */
	0x00,
};


/*!<
 * handle and class definition
 * */
HID_handle_t HID_handle;
static uint8_t HID_init(void* handle, uint8_t cfgidx);
static void HID_deinit(void* handle, uint8_t cfgidx);
static void HID_setup(void* handle, setup_header_t* req);
static void HID_in_transfer(void* handle, uint8_t epnum);
USBD_ClassTypeDef USBD_HID = {
		HID_init,
		HID_deinit,
		HID_setup,
		NULL,
		NULL,
		HID_in_transfer,
		NULL,
		NULL,
		NULL,
		NULL,
		USBD_HID_CfgDesc,
		HID_CONFIG_DESCRIPTOR_SIZE
};


/*!<
 * imported functions
 * */
extern void IN_transfer(USB_handle_t *handle, uint8_t ep_num, void* buffer, uint32_t size);
extern void IEP0_transfer(USB_handle_t* handle, uint8_t *buffer, uint32_t size);
extern void open_IEP(USB_handle_t *handle, uint8_t ep_num, uint16_t ep_mps, uint8_t ep_type);
extern void open_OEP(USB_handle_t *handle, uint8_t ep_num, uint16_t ep_mps, uint8_t ep_type);
extern void close_IEP(USB_handle_t *handle, uint8_t ep_num);
extern void stall_IEP(USB_handle_t *handle, uint8_t ep_num);
extern void stall_OEP(USB_handle_t *handle, uint8_t ep_num);
extern void stall_EP(USB_handle_t* handle, uint8_t ep_num);


/*!<
 * class functions
 * */
static uint8_t HID_init(void* handle, uint8_t config_index) {
	(void)config_index;
	open_IEP(handle, HID_IEP, HID_MPS, USBD_EP_TYPE_INTR);
	((USB_handle_t*)handle)->IN_ep[HID_IEP].is_used = 1U;
	HID_handle.state = USBD_HID_IDLE;
	return 0;
}
static void HID_deinit(void* handle, uint8_t config_index) {
	(void)config_index;
	close_IEP(handle, HID_IEP);
	((USB_handle_t*)handle)->IN_ep[HID_IEP].is_used = 0U;
}
static void HID_setup(void* handle, setup_header_t* request) {
	uint16_t	size;
	uint8_t*	buffer;
	uint16_t	status = 0U;

	switch (request->type) {
	case CLASS_REQUEST :
		switch ((HID_command_t)request->command) {
		case HID_SET_PROTOCOL:
			HID_handle.protocol = request->value & 0xFFU;
			return;
		case HID_GET_PROTOCOL:
			return IEP0_transfer(handle, (uint8_t *)&HID_handle.protocol, 1U);
		case HID_SET_IDLE:
			HID_handle.idle = (request->value >> 8) & 0xFFU;
			return;
		case HID_GET_IDLE:
			return IEP0_transfer(handle, (uint8_t *)&HID_handle.idle, 1U);
		default: break;
		} break;
	case STANDARD_REQUEST:
		switch (request->command) {
		case GET_STATUS:
			if (((USB_handle_t*)handle)->dev_state == USBD_STATE_CONFIGURED) {
				return IEP0_transfer(handle, (uint8_t *)&status, 2U);
			} break;
		case GET_DESCRIPTOR:
			if (((request->value >> 8) & 0xFFU) == HID_REPORT_DESCRIPTOR_TYPE) {
				size = HID_REPORT_DESCRIPTOR_SIZE > request->length? request->length : HID_REPORT_DESCRIPTOR_SIZE;
				buffer = HID_MOUSE_ReportDesc;
			} else if (((request->value >> 8) & 0xFFU) == HID_DESCRIPTOR_TYPE) {
				size = HID_DESCRIPTOR_SIZE > request->length? request->length : HID_DESCRIPTOR_SIZE;
				buffer = USBD_HID_Desc;
			} else { break; }
			return IEP0_transfer(handle, buffer, size);
		case GET_INTERFACE:
			if (((USB_handle_t*)handle)->dev_state == USBD_STATE_CONFIGURED) {
				return IEP0_transfer(handle, (uint8_t *)&HID_handle.alt_setting, 1U);
			} break;
		case SET_INTERFACE:
			if (((USB_handle_t*)handle)->dev_state == USBD_STATE_CONFIGURED) {
				HID_handle.alt_setting = request->value & 0xFFU;
				return;
			} break;
		case CLEAR_FEATURE:	return;
		default: break;
		} break;
	default: break;
	}
	stall_EP(handle, 0x0U);
}
static void HID_in_transfer(void* handle, uint8_t epnum) {
	(void)handle; (void)epnum; HID_handle.state = USBD_HID_IDLE;
}
static uint8_t* get_HID_config_descriptor(uint16_t* length) {
	*length = (uint16_t)sizeof(USBD_HID_CfgDesc);
	return USBD_HID_CfgDesc;
}


/*!<
 * usage
 * */
void send_HID_report(USB_handle_t* handle, uint8_t* report, uint16_t len) {
	if (handle->dev_state != USBD_STATE_CONFIGURED || HID_handle.state != USBD_HID_IDLE) { return; }
	HID_handle.state = USBD_HID_BUSY; IN_transfer(handle, HID_IEP, report, len);
}