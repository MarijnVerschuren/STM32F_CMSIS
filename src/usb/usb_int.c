//
// Created by marijn on 4/2/24.
//
#include "usb/usb.h"


/*!<
 * defines
 * */
#define inline __attribute__((always_inline))


/*!<
 * external functions
 * */
extern void flush_RX_FIFO(USB_OTG_GlobalTypeDef* usb);
extern void flush_TX_FIFO(USB_OTG_GlobalTypeDef* usb, uint8_t ep);
extern void flush_TX_FIFOS(USB_OTG_GlobalTypeDef* usb);


/*!<
 * IO
 * */
static inline void USB_write_packet(const USB_OTG_GlobalTypeDef *usb, uint8_t *src, uint8_t ep_num, uint16_t len) {
	uint32_t*	FIFO = (uint32_t*)((uint32_t)usb + (0x1000UL * (ep_num + 1U)));
	uint32_t	word_count;

	word_count = ((uint32_t)len + 3U) / 4U;
	for (uint32_t i = 0U; i < word_count; i++) {
		*FIFO = __UNALIGNED_UINT32_READ(src);
		src++; src++; src++; src++;
	}
}
void IN_transfer(USB_handle_t* handle, uint8_t ep_num, void* buffer, uint32_t size) {
	ep_num &= 0xFU;
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->IN_ep[ep_num];

	ep->xfer_buff =		buffer;
	ep->xfer_len =		size;
	ep->xfer_count =	0U;
	ep->is_in =			1U;
	ep->num =			ep_num;

	if (!ep->xfer_len) {
		in->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ);
		in->DIEPTSIZ |= 0x1UL << USB_OTG_DIEPTSIZ_PKTCNT_Pos;
	} else {
		in->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
		in->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
		if (!ep_num) {
			if (ep->xfer_len > ep->maxpacket) { ep->xfer_len = ep->maxpacket; }
			in->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19));
		} else {
			in->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket) << 19));
		}
		in->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & ep->xfer_len);
		if (ep->type == EP_TYPE_ISOC) {
			in->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_MULCNT);
			in->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_MULCNT & (1U << 29));
		}
	}

	in->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
	if (ep->type != EP_TYPE_ISOC && ep->xfer_len) {
		device->DIEPEMPMSK |= (0x01UL << ep->num);
	}
	else {
		if ((device->DSTS & (1U << 8)) == 0U)	{ in->DIEPCTL |= USB_OTG_DIEPCTL_SODDFRM; }
		else									{ in->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM; }
		USB_write_packet(usb, ep->xfer_buff, ep->num, (uint16_t)ep->xfer_len);
	}

}
void OUT_transfer(USB_handle_t* handle, uint8_t ep_num, void* buffer, uint32_t size) {
	ep_num &= 0xFU;
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->OUT_ep[ep_num];

	ep->xfer_buff =		buffer;
	ep->xfer_len =		size;
	ep->xfer_count =	0U;
	ep->is_in =			0U;
	ep->num =			ep_num;

	out->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ | USB_OTG_DOEPTSIZ_PKTCNT);
	if (!ep_num) {
		if (ep->xfer_len) { ep->xfer_len = ep->maxpacket; }
		ep->xfer_size = ep->maxpacket;
		out->DOEPTSIZ |= (
				(USB_OTG_DOEPTSIZ_XFRSIZ & ep->xfer_size)	|
				(USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19))
		);
	} else {
		if (!ep->xfer_len) {
			out->DOEPTSIZ |= (
					(USB_OTG_DOEPTSIZ_XFRSIZ & ep->maxpacket)	|
					(USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19))
			);
		} else {
			uint16_t pktcnt = (uint16_t)((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket);
			ep->xfer_size = ep->maxpacket * pktcnt;
			out->DOEPTSIZ |= USB_OTG_DOEPTSIZ_PKTCNT & ((uint32_t)pktcnt << 19);
			out->DOEPTSIZ |= USB_OTG_DOEPTSIZ_XFRSIZ & ep->xfer_size;
		}
	}
	if (ep->type == EP_TYPE_ISOC) {
		if ((device->DSTS & (1U << 8)) == 0U)	{ out->DOEPCTL |= USB_OTG_DOEPCTL_SODDFRM; }
		else									{ out->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM; }
	}
	out->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
}
void IEP0_transfer(USB_handle_t* handle, void* buffer, uint32_t size) {
	handle->ep0_state = USBD_EP0_DATA_IN;
	handle->ep_in[0].total_length = size;
	handle->ep_in[0].rem_length = size;
	IN_transfer(handle, 0x00U, buffer, size);
}
void start_OEP0(USB_OTG_GlobalTypeDef* usb) {
	USB_OTG_OUTEndpointTypeDef*	out = (void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE);
	out->DOEPTSIZ = (
			(USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19)) |
			(3U * 8U) |
			USB_OTG_DOEPTSIZ_STUPCNT
	);
}


/*!<
 * ep init
 * */
void open_IEP(USB_handle_t *handle, uint8_t ep_num, uint16_t ep_mps, uint8_t ep_type) {
	ep_num &= 0xF;
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->IN_ep[ep_num];

	ep->is_in = 1U;
	ep->num = ep_num;
	ep->maxpacket = ep_mps;
	ep->type = ep_type;
	ep->tx_fifo_num = ep->num;

	if (ep_type == EP_TYPE_BULK) { ep->data_pid_start = 0U; }
	device->DAINTMSK |= (0x01UL << ep->num);
	if (in->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) { return; }
	in->DIEPCTL |= (
		(ep->maxpacket & USB_OTG_DIEPCTL_MPSIZ)		|
		((uint32_t)ep->type << 18) | (ep_num << 22)	|
		USB_OTG_DIEPCTL_SD0PID_SEVNFRM				|
		USB_OTG_DIEPCTL_USBAEP
	);
}
void open_OEP(USB_handle_t *handle, uint8_t ep_num, uint16_t ep_mps, uint8_t ep_type) {
	ep_num &= 0xF;
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->OUT_ep[ep_num];

	ep->is_in = 1U;
	ep->num = ep_num;
	ep->maxpacket = ep_mps;
	ep->type = ep_type;

	if (ep_type == EP_TYPE_BULK) { ep->data_pid_start = 0U; }
	device->DAINTMSK |= (0x10UL << ep->num);
	if (out->DOEPCTL & USB_OTG_DOEPCTL_USBAEP) { return; }
	out->DOEPCTL |= (
		(ep->maxpacket & USB_OTG_DOEPCTL_MPSIZ)	|
		((uint32_t)ep->type << 18)				|
		USB_OTG_DIEPCTL_SD0PID_SEVNFRM			|
		USB_OTG_DOEPCTL_USBAEP
	);
}
void close_IEP(USB_handle_t *handle, uint8_t ep_num) {
	ep_num &= 0xF;
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->IN_ep[ep_num];

	ep->is_in = 1U;
	ep->num = ep_num;

	if (in->DIEPCTL & USB_OTG_DIEPCTL_EPENA) { in->DIEPCTL |= (USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_EPDIS); }
	device->DEACHMSK &= ~(0x01UL << ep->num);
	device->DAINTMSK &= ~(0x01UL << ep->num);
	in->DIEPCTL &= ~(
		USB_OTG_DIEPCTL_USBAEP |
		USB_OTG_DIEPCTL_MPSIZ |
		USB_OTG_DIEPCTL_TXFNUM |
		USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
		USB_OTG_DIEPCTL_EPTYP
	);
}
void close_OEP(USB_handle_t *handle, uint8_t ep_num) {
	ep_num &= 0xF;
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->OUT_ep[ep_num];

	ep->is_in = 0U;
	ep->num = ep_num;
	if (out->DOEPCTL & USB_OTG_DOEPCTL_EPENA) { out->DOEPCTL |= (USB_OTG_DOEPCTL_SNAK | USB_OTG_DOEPCTL_EPDIS); }
	device->DEACHMSK &= ~(0x10UL << ep->num);
	device->DAINTMSK &= ~(0x10UL << ep->num);
	out->DOEPCTL &= ~(
		USB_OTG_DOEPCTL_USBAEP |
		USB_OTG_DOEPCTL_MPSIZ |
		USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
		USB_OTG_DOEPCTL_EPTYP
	);
}


/*!<
 * stall/un-stall
 * */
void stall_IEP(USB_handle_t* handle, uint8_t ep_num) {
	ep_num &= 0xFU;  // TODO: needed?
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->IN_ep[ep_num];

	ep->is_in =		1U;
	ep->is_stall =	1U;
	ep->num =		ep_num;

	if (!(in->DIEPCTL & USB_OTG_DIEPCTL_EPENA) && ep_num) {
		in->DIEPCTL &= ~(USB_OTG_DIEPCTL_EPDIS);
	} in->DIEPCTL |= USB_OTG_DIEPCTL_STALL;

	if (!ep_num) { start_OEP0(usb); }
}
void stall_OEP(USB_handle_t* handle, uint8_t ep_num) {
	ep_num &= 0xFU;  // TODO: needed?
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->OUT_ep[ep_num];

	ep->is_in =		0U;
	ep->is_stall =	1U;
	ep->num =		ep_num;

	if (!(out->DOEPCTL & USB_OTG_DOEPCTL_EPENA) && ep_num) {
		out->DOEPCTL &= ~(USB_OTG_DOEPCTL_EPDIS);
	} out->DOEPCTL |= USB_OTG_DOEPCTL_STALL;

	if (!ep_num) { start_OEP0(usb); }
}
void stall_EP(USB_handle_t* handle, uint8_t ep_num) {
	stall_IEP(handle, ep_num); stall_OEP(handle, ep_num);
}
void unstall_IEP(USB_handle_t* handle, uint8_t ep_num) {
	ep_num &= 0xFU;  // TODO: needed?
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->IN_ep[ep_num];

	ep->is_stall = 0U;
	ep->num = ep_num;

	in->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
	if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK)) {
		in->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
	}
}
void unstall_OEP(USB_handle_t* handle, uint8_t ep_num) {
	ep_num &= 0xFU;  // TODO: needed?
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->OUT_ep[ep_num];

	ep->is_stall = 0U;
	ep->num = ep_num;

	out->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
	if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK)) {
		out->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
	}
}


/*!<
 * request handling
 * */
static inline void get_device_status(USB_handle_t* handle) {
	if (
		handle->dev_state == USBD_STATE_SUSPENDED ||
		handle->header.length != 0x2U
	) { return stall_EP(handle, 0x0U); }
	handle->dev_config_status = 0U;
	if (handle->dev_remote_wakeup) { handle->dev_config_status |= 0x02U; }  // enable remote wakeup
	IEP0_transfer(handle, &handle->dev_config_status, 2U);
}
static inline void clear_device_feature(USB_handle_t* handle) {
	if (
		handle->dev_state == USBD_STATE_SUSPENDED
	) { return stall_EP(handle, 0x0U); }
	if (handle->header.value == USB_FEATURE_REMOTE_WAKEUP) {
		handle->dev_remote_wakeup = 0U;
		handle->ep0_state = USBD_EP0_STATUS_IN;
		IN_transfer(handle, 0x00U, NULL, 0U);
	}
}
static inline void set_device_feature(USB_handle_t* handle) {
	if (handle->header.value == USB_FEATURE_REMOTE_WAKEUP) {
		handle->dev_remote_wakeup = 1U;
		handle->ep0_state = USBD_EP0_STATUS_IN;
		return IN_transfer(handle, 0x00U, NULL, 0U);
	}
	if (handle->header.value == USB_FEATURE_TEST_MODE) {
		handle->ep0_state = USBD_EP0_STATUS_IN;
		return IN_transfer(handle, 0x00U, NULL, 0U);
	}
	stall_EP(handle, 0x0U);
}
static inline void set_device_address(USB_handle_t* handle) {
	USB_OTG_GlobalTypeDef* 	usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*	device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	uint8_t					device_address;
	if (
		!handle->header.index &&
		!handle->header.length &&
		(handle->header.value < 128U) &&
		handle->dev_state != USBD_STATE_CONFIGURED
	) {
		device_address = (uint8_t)(handle->header.value) & 0x7FU;
		handle->USB_Address = device_address;
		device->DCFG &= ~(USB_OTG_DCFG_DAD);
		device->DCFG |= device_address << USB_OTG_DCFG_DAD_Pos;

		handle->ep0_state = USBD_EP0_STATUS_IN;
		IN_transfer(handle, 0x00U, NULL, 0U);
		if (device_address)	{ handle->dev_state = USBD_STATE_ADDRESSED; }
		else				{ handle->dev_state = USBD_STATE_DEFAULT; }
		return;
	}
	stall_EP(handle, 0x0U);
}
static inline void get_device_descriptor(USB_handle_t* handle) {
	uint16_t	size = 0U;
	uint8_t*	buffer = NULL;

	switch (handle->header.value >> 8) {
	case USB_DEVICE_DESCRIPTOR:
		buffer = handle->pDesc->GetDeviceDescriptor(&size); break;
	case USB_CONFIG_DESCRIPTOR:
		buffer = (uint8_t *)handle->pClass->GetFSConfigDescriptor(&size);
		buffer[1] = USB_CONFIG_DESCRIPTOR; break;
	case USB_STRING_DESCRIPTOR:
		switch (handle->header.value & 0xFFU) {
		case USB_LANGUAGE_ID_STRING_DESCRIPTOR:
			if (handle->pDesc->GetLangIDStrDescriptor) {
				buffer = handle->pDesc->GetLangIDStrDescriptor(&size); break;
			} return stall_EP(handle, 0x0U);
		case USB_MANUFACTURER_STRING_DESCRIPTOR:
			if (handle->pDesc->GetManufacturerStrDescriptor) {
				buffer = handle->pDesc->GetManufacturerStrDescriptor(&size); break;
			} return stall_EP(handle, 0x0U);
		case USB_PRODUCT_STRING_DESCRIPTOR:
			if (handle->pDesc->GetProductStrDescriptor) {
				buffer = handle->pDesc->GetProductStrDescriptor(&size); break;
			} return stall_EP(handle, 0x0U);
		case USB_SERIAL_STRING_DESCRIPTOR:
			if (handle->pDesc->GetSerialStrDescriptor) {
				buffer = handle->pDesc->GetSerialStrDescriptor(&size); break;
			} return stall_EP(handle, 0x0U);
		case USB_CONFIG_STRING_DESCRIPTOR:
			if (handle->pDesc->GetConfigurationStrDescriptor) {
				buffer = handle->pDesc->GetConfigurationStrDescriptor(&size); break;
			} return stall_EP(handle, 0x0U);
		case USB_INTERFACE_STRING_DESCRIPTOR:
			if (handle->pDesc->GetInterfaceStrDescriptor) {
				buffer = handle->pDesc->GetInterfaceStrDescriptor(&size); break;
			} return stall_EP(handle, 0x0U);
		default:									return stall_EP(handle, 0x0U);
		} break;
	case USB_QUALIFIER_DESCRIPTOR:					return stall_EP(handle, 0x0U);
	case USB_OTHER_SPEED_DESCRIPTOR:				return stall_EP(handle, 0x0U);
	default:										return stall_EP(handle, 0x0U);
	}

	if (handle->header.length) {
		if (size) {
			if (size > handle->header.length) { size = handle->header.length; }
			return IEP0_transfer(handle, buffer, size);
		} return stall_EP(handle, 0x0U);
	}
	handle->ep0_state = USBD_EP0_STATUS_IN;
	IN_transfer(handle, 0x00U, NULL, 0U);
}
static inline void get_device_configuration(USB_handle_t* handle) {
	if (
		handle->header.length != 0xFFFFU ||
		handle->dev_state == USBD_STATE_SUSPENDED
	) { return stall_EP(handle, 0x0U); }

	switch (handle->dev_state) {
	case USBD_STATE_DEFAULT:
	case USBD_STATE_ADDRESSED:
		handle->dev_default_config = 0U;
		return IEP0_transfer(handle, &handle->dev_default_config, 1U);
	case USBD_STATE_CONFIGURED:
		return IEP0_transfer(handle, &handle->dev_config, 1U);
	}
}
static inline void set_device_configuration(USB_handle_t* handle) {
	uint8_t config = handle->header.value & 0xFFU;
	if (config > USBD_MAX_NUM_CONFIGURATION) { return stall_EP(handle, 0x0U); }

	switch (handle->dev_state) {
	case USBD_STATE_ADDRESSED:
		if (!config) {
			handle->ep0_state = USBD_EP0_STATUS_IN;
			return IN_transfer(handle, 0x00U, NULL, 0U);
		}
		handle->dev_config = config;
		if (handle->pClass->Init(handle, config)) {
			handle->dev_state = USBD_STATE_ADDRESSED;
			break;
		}
		handle->ep0_state = USBD_EP0_STATUS_IN;
		handle->dev_state = USBD_STATE_CONFIGURED;
		return IN_transfer(handle, 0x00U, NULL, 0U);
	case USBD_STATE_CONFIGURED:
		if (!config) {
			handle->dev_state = USBD_STATE_ADDRESSED;
			handle->dev_config = config;
			handle->pClass->DeInit(handle, config);
			handle->ep0_state = USBD_EP0_STATUS_IN;
			return IN_transfer(handle, 0x00U, NULL, 0U);
		} else if (config != handle->dev_config) {
			handle->pClass->DeInit(handle, (uint8_t)handle->dev_config);
			handle->dev_config = config;
			if (handle->pClass->Init(handle, config)){
				handle->pClass->DeInit(handle, (uint8_t)handle->dev_config);
				handle->dev_state = USBD_STATE_ADDRESSED;
				break;
			}
		}
		handle->ep0_state = USBD_EP0_STATUS_IN;
		return IN_transfer(handle, 0x00U, NULL, 0U);
	default: handle->pClass->DeInit(handle, config); break;
	}
	stall_EP(handle, 0x0U);
}
static inline void get_dendpoint_status(USB_handle_t* handle, uint8_t ep_num) {
	USBD_EndpointTypeDef*	pep;

	switch (handle->dev_state) {
	case USBD_STATE_ADDRESSED:
		if ((ep_num != 0x00U) && (ep_num != 0x80U)) { break; }
		pep = ((ep_num & 0x80U) == 0x80U) ? &handle->ep_in[ep_num & 0x7FU] : &handle->ep_out[ep_num & 0x7FU];
		pep->status = 0x0000U;
		return IEP0_transfer(handle, &pep->status, 2U);
	case USBD_STATE_CONFIGURED:
		if (
			(ep_num & 0x80U && !handle->ep_in[ep_num & 0xFU].is_used) ||
			!handle->ep_out[ep_num & 0xFU].is_used
		) { break; }
		pep = (ep_num & 0x80U) ? &handle->ep_in[ep_num & 0x7FU] : &handle->ep_out[ep_num & 0x7FU];
		pep->status = 0x0000U;
		if ((ep_num != 0x00U) && (ep_num != 0x80U) &&
			((ep_num & 0x80U && handle->IN_ep[ep_num & 0x7FU].is_stall) ||
			handle->OUT_ep[ep_num & 0x7FU].is_stall)
		) { pep->status = 0x0001U; }
		return IEP0_transfer(handle, &pep->status, 2U);
	default: break;
	}
	stall_EP(handle, 0x0U);
}
static inline void clear_endpoint_feature(USB_handle_t* handle, uint8_t ep_num) {
	switch (handle->dev_state) {
	case USBD_STATE_ADDRESSED:
		if ((ep_num != 0x00U) && (ep_num != 0x80U)) {
			if (ep_num & 0x80)	{ stall_IEP(handle, ep_num & 0xFU); }
			else				{ stall_OEP(handle, ep_num & 0xFU); }
		} return stall_EP(handle, 0x0U);
	case USBD_STATE_CONFIGURED:
		if (handle->header.value == USB_FEATURE_EP_HALT) {
			if (ep_num == 0x80U)		{ unstall_IEP(handle, 0U); }
			else if (ep_num == 0x00U)	{ unstall_OEP(handle, 0U); }
			handle->ep0_state = USBD_EP0_STATUS_IN;
			IN_transfer(handle, 0x00U, NULL, 0U);
			if (handle->pClass->Setup != NULL) {
				(void)(handle->pClass->Setup(handle, (void*)&handle->header));
			}
		}
		return;
	default: break;
	}
	stall_EP(handle, 0x0U);
}
static inline void set_endpoin_feature(USB_handle_t* handle, uint8_t ep_num) {
	switch (handle->dev_state) {
	case USBD_STATE_ADDRESSED:
		if ((ep_num != 0x00U) && (ep_num != 0x80U)) {
			if (ep_num & 0x80)	{ stall_IEP(handle, ep_num & 0xFU); }
			else				{ stall_OEP(handle, ep_num & 0xFU); }
			stall_IEP(handle, 0x0U);
		}
		else {
			stall_EP(handle, 0x0U);
		}
		return;
	case USBD_STATE_CONFIGURED:
		if (handle->header.value == USB_FEATURE_EP_HALT) {
			if ((ep_num != 0x00U) && (ep_num != 0x80U) && !handle->header.length) {
				if (ep_num & 0x80)	{ stall_IEP(handle, ep_num & 0xFU); }
				else				{ stall_OEP(handle, ep_num & 0xFU); }
			}
		}
		handle->ep0_state = USBD_EP0_STATUS_IN;
		IN_transfer(handle, 0x00U, NULL, 0U);
		return;
	default: break;
	}
	stall_EP(handle, 0x0U);
}

static inline void device_setup_request(USB_handle_t* handle) {
	switch (handle->header.type) {
	case STANDARD_REQUEST:
		switch (handle->header.command) {
		case GET_STATUS:		return get_device_status(handle);
		case CLEAR_FEATURE:		return clear_device_feature(handle);
		case SET_FEATURE:		return set_device_feature(handle);
		case SET_ADDRESS:		return set_device_address(handle);
		case GET_DESCRIPTOR:	return get_device_descriptor(handle);
		case SET_DESCRIPTOR:	break;
		case GET_CONFIGURATION:	return get_device_configuration(handle);
		case SET_CONFIGURATION:	return set_device_configuration(handle);
		default:				break;
		} break;
	default: break;
	}
	stall_EP(handle, 0x0U);
}
static inline void interface_setup_request(USB_handle_t* handle) {
	if (
		handle->dev_state == USBD_STATE_SUSPENDED ||
		!handle->dev_state || handle->header.type > VENDOR_REQUEST ||
		(handle->header.index & 0xFFU) > USBD_MAX_NUM_INTERFACES
	) { return stall_EP(handle, 0x0U); }
	if (handle->pClass->Setup != NULL) {
		(void)(handle->pClass->Setup(handle, (void*)&handle->header));
	}
	if (!handle->header.length) {
		handle->ep0_state = USBD_EP0_STATUS_IN;
		IN_transfer(handle, 0x00U, NULL, 0U);
	}
}
static inline void endpoint_setup_request(USB_handle_t* handle) {
	uint8_t ep_num = handle->header.index & 0xFFU;

	switch (handle->header.type) {
	case STANDARD_REQUEST:
		switch (handle->header.command) {
		case SET_FEATURE:	return set_endpoin_feature(handle, ep_num);
		case CLEAR_FEATURE:	return clear_endpoint_feature(handle, ep_num);
		case GET_STATUS:	return get_dendpoint_status(handle, ep_num);
		default: break;
		} break;
	case CLASS_REQUEST:
	case VENDOR_REQUEST:
		if (handle->pClass->Setup) {
			(void)handle->pClass->Setup(handle, (void*)&handle->header);
		}
		return;
	default: break;
	}
	stall_EP(handle, 0x0U);
}

static inline void USB_OTG_IRQ(USB_handle_t* handle, USB_OTG_GlobalTypeDef* usb) {
	// TODO: redo handle and make it possible to pass usb ptr
	uint32_t tmp = usb->GOTGINT;
	if (tmp & USB_OTG_GOTGINT_SEDET) {
		handle->dev_state = USBD_STATE_DEFAULT;
		if (handle->pClass == NULL) { return; }
		handle->pClass->DeInit(handle, (uint8_t)handle->dev_config);
	}
	usb->GOTGINT |= tmp;
}
static inline void USB_SOF_IRQ(USB_handle_t* handle, USB_OTG_GlobalTypeDef* usb ) {
	// TODO: redo handle and make it possible to pass usb ptr
	if (handle->dev_state == USBD_STATE_CONFIGURED) {
		if (handle->pClass == NULL)		{ return; }
		if (handle->pClass->SOF == NULL)	{ return; }
		(void)handle->pClass->SOF(handle);
	}
	usb->GINTSTS |= USB_OTG_GINTSTS_SOF;
}
static inline void USB_receive_packet_IRQ(USB_handle_t* handle) {
	// TODO: redo handle and make it possible to pass usb ptr
	USB_OTG_GlobalTypeDef*	usb = handle->Instance;
	usb->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
	uint32_t				tmp = usb->GRXSTSP;
	GRXSTS_t				status = *((GRXSTS_t*)&tmp);
	USB_EPTypeDef*			ep = &handle->OUT_ep[status.EPNUM];  // TODO: redo handle
	volatile uint32_t*		FIFO = (void*)(((uint32_t)usb) + USB_OTG_FIFO_BASE);

	switch (status.PKTSTS) {
	case 0b0010:  // DATA packet
		if (!status.BCNT) { break; }
		const uint32_t	words = status.BCNT >> 2U;
		const uint8_t	bytes = status.BCNT & 0b11UL;
		for (uint32_t i = 0UL; i < words; i++) {
			__UNALIGNED_UINT32_WRITE(ep->xfer_buff, *FIFO);
			// 4x inc is faster than an iadd due to pipelining
			ep->xfer_buff++; ep->xfer_buff++;
			ep->xfer_buff++; ep->xfer_buff++;
		} if (bytes) {
			uint32_t tmp; __UNALIGNED_UINT32_WRITE(&tmp, *FIFO);
			for (uint8_t i = 0; i < bytes; i++) {
				*(uint8_t*)ep->xfer_buff++ = (uint8_t)(tmp >> (8U * i));
			}
		} ep->xfer_count += status.BCNT;
		break;
	case 0b0110:  // SETUP packet
		__UNALIGNED_UINT32_WRITE(&handle->Setup[0], *FIFO);
		__UNALIGNED_UINT32_WRITE(&handle->Setup[1], *FIFO);
		ep->xfer_count += 8;
		break;
	default: break;
	}

	usb->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
}
static inline void USB_global_NAK_OUT_IRQ(USB_handle_t* handle) {
	// TODO: redo handle and make it possible to pass usb ptr
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE);
	USB_EPTypeDef*				ep;

	usb->GINTMSK &= ~USB_OTG_GINTMSK_GONAKEFFM;
	for (uint8_t ep_num = 1U; ep_num < handle->Init.dev_endpoints; ep_num++) {
		ep = &handle->OUT_ep[ep_num];
		if (ep->is_iso_incomplete == 1U) { continue; }
		if (out[ep_num].DOEPCTL & USB_OTG_DOEPCTL_EPENA) {
			out[ep_num].DOEPCTL |= (USB_OTG_DOEPCTL_SNAK | USB_OTG_DOEPCTL_EPDIS);
			while (out[ep_num].DOEPCTL & USB_OTG_DOEPCTL_EPENA);
		}
	}
}
static inline void USB_suspend_IRQ(USB_handle_t* handle) {
	// TODO: redo handle and make it possible to pass usb ptr
	USB_OTG_GlobalTypeDef* 	usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*	device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	__IO uint32_t*			PCGCCTL =	(void*)(((uint32_t)usb) + USB_OTG_PCGCCTL_BASE);

	if ((device->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS) {
		if (handle->dev_state != USBD_STATE_SUSPENDED) {
			handle->dev_old_state = handle->dev_state;
		}
		handle->dev_state = USBD_STATE_SUSPENDED;
		*PCGCCTL |= USB_OTG_PCGCCTL_STOPCLK;
		if (handle->Init.low_power_enable) {
			SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
		}
	}
	usb->GINTSTS |= USB_OTG_GINTSTS_USBSUSP;
}
static inline void USB_reset_IRQ(USB_handle_t* handle) {
	// TODO: redo handle and make it possible to pass usb ptr
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE);

	device->DCTL &= ~USB_OTG_DCTL_RWUSIG;
	flush_TX_FIFOS(handle->Instance);
	for (uint8_t i = 0U; i < handle->Init.dev_endpoints; i++) {
		in[i].DIEPINT =		0xFB7FU;
		in[i].DIEPCTL &=	~USB_OTG_DIEPCTL_STALL;
		out[i].DOEPINT =	0xFB7FU;
		out[i].DOEPCTL &=	~USB_OTG_DOEPCTL_STALL;
		out[i].DOEPCTL |=	USB_OTG_DOEPCTL_SNAK;
	}
	device->DAINTMSK |= 0x10001U;
	device->DOEPMSK |= (
		USB_OTG_DOEPMSK_STUPM		|
		USB_OTG_DOEPMSK_XFRCM		|
		USB_OTG_DOEPMSK_EPDM		|
		USB_OTG_DOEPMSK_OTEPSPRM	|
		USB_OTG_DOEPMSK_NAKM
   );
	device->DIEPMSK |= (
		USB_OTG_DIEPMSK_TOM			|
		USB_OTG_DIEPMSK_XFRCM		|
		USB_OTG_DIEPMSK_EPDM
	);

	device->DCFG &= ~USB_OTG_DCFG_DAD;
	start_OEP0(usb);
	usb->GINTSTS |= USB_OTG_GINTSTS_USBRST;
}
static inline void USB_enumeration_done_IRQ(USB_handle_t* handle) {
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE);

	in[0].DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
	device->DCTL |= USB_OTG_DCTL_CGINAK;

	USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;  // reset turnaround time
	if		(PLLQ_clock_frequency < 15000000UL)	{ USB_OTG_FS->GUSBCFG |= 0xFU << USB_OTG_GUSBCFG_TRDT_Pos; }
	else if	(PLLQ_clock_frequency < 16000000UL)	{ USB_OTG_FS->GUSBCFG |= 0xEU << USB_OTG_GUSBCFG_TRDT_Pos; }
	else if	(PLLQ_clock_frequency < 17200000UL)	{ USB_OTG_FS->GUSBCFG |= 0xDU << USB_OTG_GUSBCFG_TRDT_Pos; }
	else if	(PLLQ_clock_frequency < 18500000UL)	{ USB_OTG_FS->GUSBCFG |= 0xCU << USB_OTG_GUSBCFG_TRDT_Pos; }
	else if	(PLLQ_clock_frequency < 20000000UL)	{ USB_OTG_FS->GUSBCFG |= 0xBU << USB_OTG_GUSBCFG_TRDT_Pos; }
	else if	(PLLQ_clock_frequency < 21800000UL)	{ USB_OTG_FS->GUSBCFG |= 0xAU << USB_OTG_GUSBCFG_TRDT_Pos; }
	else if	(PLLQ_clock_frequency < 24000000UL)	{ USB_OTG_FS->GUSBCFG |= 0x9U << USB_OTG_GUSBCFG_TRDT_Pos; }
	else if	(PLLQ_clock_frequency < 27700000UL)	{ USB_OTG_FS->GUSBCFG |= 0x8U << USB_OTG_GUSBCFG_TRDT_Pos; }
	else if	(PLLQ_clock_frequency < 32000000UL)	{ USB_OTG_FS->GUSBCFG |= 0x7U << USB_OTG_GUSBCFG_TRDT_Pos; }
	else										{ USB_OTG_FS->GUSBCFG |= 0x6U << USB_OTG_GUSBCFG_TRDT_Pos; }

	handle->dev_state = USBD_STATE_DEFAULT;
	handle->ep0_state = USBD_EP0_IDLE;
	handle->dev_config = 0U;
	handle->dev_remote_wakeup = 0U;
	if (handle->pClass != NULL) {
		if (handle->pClass->DeInit != NULL) {
			handle->pClass->DeInit(handle, (uint8_t)handle->dev_config);
		}
	}

	open_OEP(handle, 0x00U, USB_MAX_EP0_SIZE, USBD_EP_TYPE_CTRL);
	handle->ep_out[0].is_used = 1U;
	handle->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;

	open_IEP(handle, 0x00U, USB_MAX_EP0_SIZE, USBD_EP_TYPE_CTRL);
	handle->ep_in[0].is_used = 1U;
	handle->ep_in[0].maxpacket = USB_MAX_EP0_SIZE;

	usb->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE;
}
static inline void IEP_transfer_complete_IRQ(USB_handle_t* handle, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USBD_EndpointTypeDef*		ep;
	uint8_t*					data = handle->IN_ep[ep_num].xfer_buff;

	device->DIEPEMPMSK &= ~(0x1UL << ep_num);
	in->DIEPINT |= USB_OTG_DIEPINT_XFRC;

	if (!ep_num) {
		ep = &handle->ep_in[0];
		if (handle->ep0_state != USBD_EP0_DATA_IN) { return; }
		if (ep->rem_length > ep->maxpacket) {
			ep->rem_length -= ep->maxpacket;
			IN_transfer(handle, 0x00U, data, ep->rem_length);
			OUT_transfer(handle, 0x00U, NULL, 0U);
		} else if (
			(ep->maxpacket == ep->rem_length) &&
			(ep->total_length >= ep->maxpacket) &&
			(ep->total_length < handle->ep0_data_len)
		) {
			IN_transfer(handle, 0x00U, NULL, 0U);
			handle->ep0_data_len = 0U;
			OUT_transfer(handle, 0x00U, NULL, 0U);
		} else {
			if (handle->dev_state == USBD_STATE_CONFIGURED && handle->pClass->EP0_TxSent != NULL) {
				handle->pClass->EP0_TxSent(handle);
			}
			stall_IEP(handle, 0x0U);
			handle->ep0_state = USBD_EP0_STATUS_OUT;
			OUT_transfer(handle, 0x00U, NULL, 0U);
		}
		return;
	}
	if (handle->dev_state == USBD_STATE_CONFIGURED && handle->pClass->DataIn != NULL) {
		(void)handle->pClass->DataIn(handle, ep_num);
	}
}
static inline void IEP_disabled_IRQ(USB_handle_t* handle, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->IN_ep[ep_num];

	flush_TX_FIFO(handle->Instance, ep_num);
	if (ep->is_iso_incomplete == 1U) {
		ep->is_iso_incomplete = 0U;
		if (
			handle->pClass &&
			handle->pClass->IsoINIncomplete &&
			handle->dev_state == USBD_STATE_CONFIGURED
		) {
			(void)handle->pClass->IsoINIncomplete(handle, ep_num);
		}
	}
	in->DIEPINT |= USB_OTG_DIEPINT_EPDISD;
}
static inline void IEP_FIFO_empty_IRQ(USB_handle_t* handle, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->IN_ep[ep_num];
	uint32_t len;

	while (ep->xfer_count < ep->xfer_len && ep->xfer_len) {
		len = ep->xfer_len - ep->xfer_count;
		if (len > ep->maxpacket) { len = ep->maxpacket; }
		if ((in->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) < ((len + 3U) >> 2U)) { break; }
		USB_write_packet(usb, ep->xfer_buff, (uint8_t)ep_num, (uint16_t)len);
		ep->xfer_buff  += len;
		ep->xfer_count += len;
	}
	// disable interrupt if transfer done
	if (ep->xfer_len <= ep->xfer_count) { device->DIEPEMPMSK &= ~((uint32_t)(0x1UL << ep_num)); }
}
static inline void OEP_transfer_complete(USB_handle_t* handle, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->OUT_ep[ep_num];
	out->DOEPINT |= USB_OTG_DOEPINT_XFRC;

	if (!(ep_num | ep->xfer_len)) { start_OEP0(usb); }

	if (!ep_num && handle->ep0_state == USBD_EP0_DATA_OUT) {
		USBD_EndpointTypeDef*	USBD_ep = &handle->ep_out[0];
		if (USBD_ep->rem_length > USBD_ep->maxpacket) {
			USBD_ep->rem_length -= USBD_ep->maxpacket;
			OUT_transfer(handle, 0x00U, ep->xfer_buff, USBD_ep->rem_length > USBD_ep->maxpacket ? USBD_ep->maxpacket : USBD_ep->rem_length);
			return;
		}
		if (handle->dev_state == USBD_STATE_CONFIGURED && handle->pClass->EP0_RxReady) {
			handle->pClass->EP0_RxReady(handle);
		}
		handle->ep0_state = USBD_EP0_STATUS_IN;
		IN_transfer(handle, 0x00U, NULL, 0U);
	} else if (handle->dev_state == USBD_STATE_CONFIGURED && handle->pClass->DataOut) {
		(void)handle->pClass->DataOut(handle, ep_num);
	}
}
static inline void OEP_disabled(USB_handle_t* handle, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_EPTypeDef*				ep =		&handle->OUT_ep[ep_num];

	if (usb->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) { device->DCTL |= USB_OTG_DCTL_CGONAK; }
	if (ep->is_iso_incomplete == 1U) {
		ep->is_iso_incomplete = 0U;
		if (
			handle->pClass &&
			handle->dev_state == USBD_STATE_CONFIGURED &&
			handle->pClass->IsoOUTIncomplete
		) {
			(void)handle->pClass->IsoOUTIncomplete(handle, ep_num);
		}
	}
	out->DOEPINT |= USB_OTG_DOEPINT_EPDISD;
}
static inline void OEP_setup_done(USB_handle_t* handle, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));

	out->DOEPINT |= USB_OTG_DOEPINT_STUP;

	uint8_t* data =		handle->Setup;
	uint8_t* header =	(uint8_t*)&handle->header;
	*header++ =	*data++;
	*header++ =	*data++;
	*header++ =	*data++;
	*header++ =	*data++;
	*header++ =	*data++;
	*header++ =	*data++;
	*header++ =	*data++;
	*header =	*data;

	handle->ep0_state = USBD_EP0_SETUP;
	handle->ep0_data_len = handle->header.length;
	switch (handle->header.recipiant) {
	case RECIPIANT_DEVICE:		return device_setup_request(handle);
	case RECIPIANT_INTERFACE:	return interface_setup_request(handle);
	case RECIPIANT_ENDPOINT:	return endpoint_setup_request(handle);
	default: break;
	}
	if (handle->header.direction)	{ stall_IEP(handle, 0x0U); }
	else						{ stall_OEP(handle, 0x0U); }
}
static inline void USB_incomplete_ISO_IN_IRQ(USB_handle_t* handle) {
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE);
	USB_EPTypeDef*				ep;

	for (uint8_t ep_num = 1U; ep_num < handle->Init.dev_endpoints; ep_num++) {
		ep = &handle->IN_ep[ep_num];
		if (ep->type != EP_TYPE_ISOC || !(in[ep_num].DIEPCTL & USB_OTG_DIEPCTL_EPENA)) { continue; }
		ep->is_iso_incomplete = 1U;
		if (in[ep_num].DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
			in[ep_num].DIEPCTL |= (USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_EPDIS);
			while (in[ep_num].DIEPCTL & USB_OTG_DIEPCTL_EPENA);
		}
	}
	usb->GINTSTS |= USB_OTG_GINTSTS_IISOIXFR;
}
static inline void USB_incomplete_ISO_OUT_IRQ(USB_handle_t* handle) {
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE);
	uint32_t					tmp;

	for (uint8_t ep_num = 1U; ep_num < handle->Init.dev_endpoints; ep_num++) {
		tmp = out[ep_num].DOEPCTL;
		if ((handle->OUT_ep[ep_num].type == EP_TYPE_ISOC) && tmp & USB_OTG_DOEPCTL_EPENA &&
			(tmp & (0x1U << 16)) == (handle->FrameNumber & 0x1U)) {
			handle->OUT_ep[ep_num].is_iso_incomplete = 1U;
			usb->GINTMSK |= USB_OTG_GINTMSK_GONAKEFFM;
			if ((usb->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) == 0U) {
				device->DCTL |= USB_OTG_DCTL_SGONAK;
				break;
			}
		}
	}
	usb->GINTSTS |= USB_OTG_GINTSTS_PXFR_INCOMPISOOUT;
}
static inline void USB_connection_IRQ(USB_handle_t* handle) {
	USB_OTG_GlobalTypeDef*	usb =		handle->Instance;
	// TODO!!!
	usb->GINTSTS |= USB_OTG_GINTSTS_SRQINT;
}
static inline void USB_wake_up_IRQ(USB_handle_t* handle) {
	USB_OTG_GlobalTypeDef*	usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*	device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	device->DCTL &= ~USB_OTG_DCTL_RWUSIG;
	if (handle->dev_state == USBD_STATE_SUSPENDED) {
		handle->dev_state = handle->dev_old_state;
	}
	usb->GINTSTS |= USB_OTG_GINTSTS_WKUINT;
}


/*!<
 * handlers
 * */
static inline void IEP_common_handler(USB_handle_t* handle, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	uint32_t					ep_int =	(in->DIEPINT & device->DIEPMSK) | ((device->DIEPEMPMSK >> ep_num) & 0x1U) << 7;

	/* transfer complete interrupt */
	if (ep_int & USB_OTG_DIEPINT_XFRC)			{ IEP_transfer_complete_IRQ(handle, ep_num); }
	/* endpoint disabled interrupt */
	if (ep_int & USB_OTG_DIEPINT_EPDISD)		{ IEP_disabled_IRQ(handle, ep_num); }
	/* AHB error interrupt */
	if (ep_int & USB_OTG_DIEPINT_AHBERR)		{ in->DIEPINT |= USB_OTG_DIEPINT_AHBERR;}
	/* timeout condition interrupt */
	if (ep_int & USB_OTG_DIEPINT_TOC)			{ in->DIEPINT |= USB_OTG_DIEPINT_TOC; }
	/* IN token received when TX FIFO is empty interrupt */
	if (ep_int & USB_OTG_DIEPINT_ITTXFE)		{ in->DIEPINT |= USB_OTG_DIEPINT_ITTXFE; }
	/* IN token recieved with EP mismatch interrupt */
	if (ep_int & USB_OTG_DIEPINT_INEPNM)		{ in->DIEPINT |= USB_OTG_DIEPINT_INEPNM; }
	/* IN enpoint NAK effective interrupt */
	if (ep_int & USB_OTG_DIEPINT_INEPNE)		{ in->DIEPINT |= USB_OTG_DIEPINT_INEPNE; }
	/* TX FIFO empty interrupt */
	if (ep_int & USB_OTG_DIEPINT_TXFE)			{ IEP_FIFO_empty_IRQ(handle, ep_num); }
	/* TX FIFO underrun interrupt */
	if (ep_int & USB_OTG_DIEPINT_TXFIFOUDRN)	{ in->DIEPINT |= USB_OTG_DIEPINT_TXFIFOUDRN; }
	/* buffer not available interrupt */
	if (ep_int & USB_OTG_DIEPINT_BNA)			{ in->DIEPINT |= USB_OTG_DIEPINT_BNA; }
	/* packet dropped interrupt */
	if (ep_int & USB_OTG_DIEPINT_PKTDRPSTS)		{ in->DIEPINT |= USB_OTG_DIEPINT_PKTDRPSTS; }
	/* NAK interrupt */
	if (ep_int & USB_OTG_DIEPINT_PKTDRPSTS)		{ in->DIEPINT |= USB_OTG_DIEPINT_NAK; }
}
static inline void OEP_common_handler(USB_handle_t* handle, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		handle->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	uint32_t					ep_int =	out->DOEPINT & device->DOEPMSK;

	/* transfer complete interrupt */
	if (ep_int & USB_OTG_DOEPINT_XFRC)					{ OEP_transfer_complete(handle, ep_num); }
	/* endpoint disabled interrupt */
	if (ep_int & USB_OTG_DOEPINT_EPDISD)				{ OEP_disabled(handle, ep_num); }
	/* AHB error interrupt */
	if (ep_int & USB_OTG_DOEPINT_AHBERR)				{ out->DOEPINT |= USB_OTG_DOEPINT_AHBERR; }
	/* SETUP phase done */
	if (ep_int & USB_OTG_DOEPINT_STUP)					{ OEP_setup_done(handle, ep_num); }
	/* OUT token received when endpoint disabled interrupt */
	if (ep_int & USB_OTG_DOEPINT_OTEPDIS)				{ out->DOEPINT |= USB_OTG_DOEPINT_OTEPDIS; }
	/* status phase received for control write */
	if (ep_int & USB_OTG_DOEPINT_OTEPSPR)				{ out->DOEPINT |= USB_OTG_DOEPINT_OTEPSPR; }
	/* back to back setup packet recived interrupt */
	if (ep_int & USB_OTG_DOEPINT_B2BSTUP)				{ out->DOEPINT |= USB_OTG_DOEPINT_B2BSTUP; }
	/* OUT packet error interrupt */
	if (ep_int & USB_OTG_DOEPINT_OUTPKTERR)				{ out->DOEPINT |= USB_OTG_DOEPINT_OUTPKTERR; }
	/* buffer not available interrupt */
	if (ep_int & USB_OTG_DIEPINT_BNA)					{ out->DOEPINT |= USB_OTG_DIEPINT_BNA; }
	/* NAK interrupt */
	if (ep_int & USB_OTG_DOEPINT_NAK)					{ out->DOEPINT |= USB_OTG_DOEPINT_NAK; }
	/* NYET interrupt */
	if (ep_int & USB_OTG_DOEPINT_NYET)					{ out->DOEPINT |= USB_OTG_DOEPINT_NYET;	}
	/* setup packet received interrupt */
	if (ep_int & USB_OTG_DOEPINT_STPKTRX)				{ out->DOEPINT |= USB_OTG_DOEPINT_STPKTRX;	}
}
static inline void USB_common_handler(USB_handle_t* handle) {
	USB_OTG_GlobalTypeDef*	usb = handle->Instance;
	if (((usb->GINTSTS) & 0x1U) != USB_OTG_MODE_DEVICE) { return; }
	const uint32_t			irqs = usb->GINTSTS & usb->GINTMSK;
	if (!irqs)				{ return; }
	USB_OTG_DeviceTypeDef*	device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	uint8_t					ep_num;
	uint16_t				ep_gint;

	/* store current frame number */
	handle->FrameNumber = (device->DSTS & USB_OTG_DSTS_FNSOF_Msk) >> USB_OTG_DSTS_FNSOF_Pos;

	/* mode mismatch interrupt */
	if (irqs & USB_OTG_GINTSTS_OTGINT)					{ usb->GINTSTS |= USB_OTG_GINTSTS_MMIS; }
	/* OTG interrupt */
	if (irqs & USB_OTG_GINTSTS_OTGINT)					{ USB_OTG_IRQ(handle, usb); }
	/* start of frame interrupt */
	if (irqs & USB_OTG_GINTSTS_SOF)						{ USB_SOF_IRQ(handle, usb); }
	/* receive packet interrupt */
	if (irqs & USB_OTG_GINTSTS_RXFLVL)					{ USB_receive_packet_IRQ(handle); }
	/* global OUT NAK effective interrupt */
	if (irqs & USB_OTG_GINTSTS_BOUTNAKEFF)				{ USB_global_NAK_OUT_IRQ(handle); }
	/* suspend interrupt */
	if (irqs & USB_OTG_GINTSTS_USBSUSP)					{ USB_suspend_IRQ(handle); }
	/* reset interrupt */
	if (irqs & USB_OTG_GINTSTS_USBRST)					{ USB_reset_IRQ(handle); }
	/* enumeration done interrupt */
	if (irqs & USB_OTG_GINTSTS_ENUMDNE)					{ USB_enumeration_done_IRQ(handle); }
	/* IN endpoint interrupts */
	if (irqs & USB_OTG_GINTSTS_IEPINT) {
		ep_gint = device->DAINT & device->DAINTMSK & 0xFFFFU;
		ep_num = 0U;
		while (ep_gint) {
			if (ep_gint & 0b1UL)						{ IEP_common_handler(handle, ep_num); }
			ep_num++; ep_gint >>= 1U;
		}
	}
	/* OUT endpoint interrupts */
	if (irqs & USB_OTG_GINTSTS_OEPINT) {
		ep_gint = (device->DAINT & device->DAINTMSK) >> 0x10UL;
		ep_num = 0U;
		while (ep_gint) {
			if (ep_gint & 0x1U)							{ OEP_common_handler(handle, ep_num); }
			ep_num++; ep_gint >>= 1U;
		}
	}
	/* incomplete isochronous IN interrupt */
	if (irqs & USB_OTG_GINTSTS_IISOIXFR)				{ USB_incomplete_ISO_IN_IRQ(handle); }
	/* incomplete isochronous OUT interrupt */
	if (irqs & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)		{ USB_incomplete_ISO_OUT_IRQ(handle); }
	/* connection event interrupt */
	if (irqs & USB_OTG_GINTSTS_SRQINT)					{ USB_connection_IRQ(handle); }
	/* wake-up interrupt */
	if (irqs & USB_OTG_GINTSTS_WKUINT)					{ USB_wake_up_IRQ(handle); }
}
static inline void USB_wakeup_handler(USB_handle_t* handle) {
	USB_OTG_GlobalTypeDef*	usb =		handle->Instance;
	__IO uint32_t*			PCGCCTL =	(void*)(((uint32_t)usb) + USB_OTG_PCGCCTL_BASE);
	if (handle->Init.low_power_enable) {
		SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
		sys_clock_init();
	}
	*PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK);
	EXTI->PR |= EXTI_PR_PR18;  // USB FS EXTI line TODO: modular
}


/*!<
 * interrupts
 * */
void OTG_FS_IRQHandler(void)		{ USB_common_handler(&USB_handle); }
void OTG_FS_WKUP_IRQHandler(void)	{ USB_wakeup_handler(&USB_handle); }