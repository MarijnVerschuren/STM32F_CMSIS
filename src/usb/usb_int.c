//
// Created by marijn on 4/2/24.
//
#include "usb/usb.h"
//#include "usb/hid.h"


#define inline __attribute__((always_inline))


// L1 ========================================= /  // TODO: typedef enum
#define STS_GOUT_NAK                           1U
#define STS_DATA_UPDT                          2U
#define STS_XFER_COMP                          3U
#define STS_SETUP_COMP                         4U
#define STS_SETUP_UPDT                         6U


// external functions
extern void flush_RX_FIFO(USB_OTG_GlobalTypeDef* usb);
extern void flush_TX_FIFO(USB_OTG_GlobalTypeDef* usb, uint8_t ep);
extern void flush_TX_FIFOS(USB_OTG_GlobalTypeDef* usb);


// IO
static inline void USB_write_packet(const USB_OTG_GlobalTypeDef *usb, uint8_t *src, uint8_t ep_num, uint16_t len) {
	uint32_t*	FIFO = (uint32_t*)((uint32_t)usb + (0x1000UL * (ep_num + 1U)));
	uint32_t	word_count;

	word_count = ((uint32_t)len + 3U) / 4U;
	for (uint32_t i = 0U; i < word_count; i++) {
		*FIFO = __UNALIGNED_UINT32_READ(src);
		src++; src++; src++; src++;
	}
}
void IN_transfer(PCD_HandleTypeDef* hpcd, uint8_t ep_num, void* buffer, uint32_t size) {
	ep_num &= 0xFU;
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	PCD_EPTypeDef*				ep =		&hpcd->IN_ep[ep_num];

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
void OUT_transfer(PCD_HandleTypeDef* hpcd, uint8_t ep_num, void* buffer, uint32_t size) {
	ep_num &= 0xFU;
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	PCD_EPTypeDef*				ep =		&hpcd->OUT_ep[ep_num];

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
void IEP0_transfer(USBD_HandleTypeDef* handle, void* buffer, uint32_t size) {
	handle->ep0_state = USBD_EP0_DATA_IN;
	handle->ep_in[0].total_length = size;
	handle->ep_in[0].rem_length = size;
	IN_transfer(handle->pData, 0x00U, buffer, size);
}
void start_OEP0(USB_OTG_GlobalTypeDef* usb) {
	USB_OTG_OUTEndpointTypeDef*	out = (void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE);
	out->DOEPTSIZ = (
			(USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19)) |
			(3U * 8U) |
			USB_OTG_DOEPTSIZ_STUPCNT
	);
}


// EP init
void open_IEP(PCD_HandleTypeDef *hpcd, uint8_t ep_num, uint16_t ep_mps, uint8_t ep_type) {
	ep_num &= 0xF;
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	PCD_EPTypeDef*				ep =		&hpcd->IN_ep[ep_num];

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
void open_OEP(PCD_HandleTypeDef *hpcd, uint8_t ep_num, uint16_t ep_mps, uint8_t ep_type) {
	ep_num &= 0xF;
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	PCD_EPTypeDef*				ep =		&hpcd->OUT_ep[ep_num];

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
void close_IEP(PCD_HandleTypeDef *hpcd, uint8_t ep_num) {
	ep_num &= 0xF;
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	PCD_EPTypeDef*				ep =		&hpcd->IN_ep[ep_num];

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
void close_OEP(PCD_HandleTypeDef *hpcd, uint8_t ep_num) {
	ep_num &= 0xF;
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	PCD_EPTypeDef*				ep =		&hpcd->OUT_ep[ep_num];

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


// stall/un-stall
void stall_IEP(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	ep_num &= 0xFU;  // TODO: needed?
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	PCD_EPTypeDef*				ep =		&hpcd->IN_ep[ep_num];

	ep->is_in =		1U;
	ep->is_stall =	1U;
	ep->num =		ep_num;

	if (!(in->DIEPCTL & USB_OTG_DIEPCTL_EPENA) && ep_num) {
		in->DIEPCTL &= ~(USB_OTG_DIEPCTL_EPDIS);
	} in->DIEPCTL |= USB_OTG_DIEPCTL_STALL;

	if (!ep_num) { start_OEP0(usb); }
}
void stall_OEP(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	ep_num &= 0xFU;  // TODO: needed?
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	PCD_EPTypeDef*				ep =		&hpcd->OUT_ep[ep_num];

	ep->is_in =		0U;
	ep->is_stall =	1U;
	ep->num =		ep_num;

	if (!(out->DOEPCTL & USB_OTG_DOEPCTL_EPENA) && ep_num) {
		out->DOEPCTL &= ~(USB_OTG_DOEPCTL_EPDIS);
	} out->DOEPCTL |= USB_OTG_DOEPCTL_STALL;

	if (!ep_num) { start_OEP0(usb); }
}
void stall_EP(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	stall_IEP(hpcd, ep_num); stall_OEP(hpcd, ep_num);
}
void unstall_IEP(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	ep_num &= 0xFU;  // TODO: needed?
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	PCD_EPTypeDef*				ep =		&hpcd->IN_ep[ep_num];

	ep->is_stall = 0U;
	ep->num = ep_num;

	in->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
	if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK)) {
		in->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
	}
}
void unstall_OEP(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	ep_num &= 0xFU;  // TODO: needed?
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	PCD_EPTypeDef*				ep =		&hpcd->OUT_ep[ep_num];

	ep->is_stall = 0U;
	ep->num = ep_num;

	out->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
	if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK)) {
		out->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
	}
}


// requests
static inline void get_device_status(USBD_HandleTypeDef* pdev) {
	if (
		pdev->dev_state == USBD_STATE_SUSPENDED ||
		pdev->header.length != 0x2U
	) { return stall_EP(pdev->pData, 0x0U); }
	pdev->dev_config_status = 0U;
	if (pdev->dev_remote_wakeup) { pdev->dev_config_status |= USB_CONFIG_REMOTE_WAKEUP; }
	IEP0_transfer(pdev, &pdev->dev_config_status, 2U);
}
static inline void clear_device_feature(USBD_HandleTypeDef* pdev) {
	if (
		pdev->dev_state == USBD_STATE_SUSPENDED
	) { return stall_EP(pdev->pData, 0x0U); }
	if (pdev->header.value == USB_FEATURE_REMOTE_WAKEUP) {
		pdev->dev_remote_wakeup = 0U;
		pdev->ep0_state = USBD_EP0_STATUS_IN;
		IN_transfer(pdev->pData, 0x00U, NULL, 0U);
	}
}
static inline void set_device_feature(USBD_HandleTypeDef* pdev) {
	if (pdev->header.value == USB_FEATURE_REMOTE_WAKEUP) {
		pdev->dev_remote_wakeup = 1U;
		pdev->ep0_state = USBD_EP0_STATUS_IN;
		return IN_transfer(pdev->pData, 0x00U, NULL, 0U);
	}
	if (pdev->header.value == USB_FEATURE_TEST_MODE) {
		pdev->dev_test_mode = (uint8_t)(pdev->header.index >> 8);
		pdev->ep0_state = USBD_EP0_STATUS_IN;
		return IN_transfer(pdev->pData, 0x00U, NULL, 0U);
	}
	stall_EP(pdev->pData, 0x0U);
}
static inline void set_device_address(USBD_HandleTypeDef* pdev) {
	PCD_HandleTypeDef*		hpcd = 		pdev->pData;
	USB_OTG_GlobalTypeDef* 	usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*	device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	uint8_t					device_address;
	if (
		!pdev->header.index &&
		!pdev->header.length &&
		(pdev->header.value < 128U) &&
		pdev->dev_state != USBD_STATE_CONFIGURED
	) {
		device_address = (uint8_t)(pdev->header.value) & 0x7FU;
		pdev->dev_address = hpcd->USB_Address = device_address;
		device->DCFG &= ~(USB_OTG_DCFG_DAD);
		device->DCFG |= device_address << USB_OTG_DCFG_DAD_Pos;

		pdev->ep0_state = USBD_EP0_STATUS_IN;
		IN_transfer(pdev->pData, 0x00U, NULL, 0U);
		if (device_address)	{ pdev->dev_state = USBD_STATE_ADDRESSED; }
		else				{ pdev->dev_state = USBD_STATE_DEFAULT; }
		return;
	}
	stall_EP(pdev->pData, 0x0U);
}
static inline void get_device_descriptor(USBD_HandleTypeDef* pdev) {
	uint16_t	size = 0U;
	uint8_t*	buffer = NULL;

	switch (pdev->header.value >> 8) {
	case USB_DESC_TYPE_DEVICE:
		buffer = pdev->pDesc->GetDeviceDescriptor(pdev->dev_speed, &size); break;
	case USB_DESC_TYPE_CONFIGURATION:
		buffer = (uint8_t *)pdev->pClass[0]->GetFSConfigDescriptor(&size);
		buffer[1] = USB_DESC_TYPE_CONFIGURATION; break;
	case USB_DESC_TYPE_STRING:
		switch (pdev->header.value & 0xFFU) {
		case USBD_IDX_LANGID_STR:
			if (pdev->pDesc->GetLangIDStrDescriptor) {
				buffer = pdev->pDesc->GetLangIDStrDescriptor(pdev->dev_speed, &size); break;
			} return stall_EP(pdev->pData, 0x0U);
		case USBD_IDX_MFC_STR:
			if (pdev->pDesc->GetManufacturerStrDescriptor) {
				buffer = pdev->pDesc->GetManufacturerStrDescriptor(pdev->dev_speed, &size); break;
			} return stall_EP(pdev->pData, 0x0U);
		case USBD_IDX_PRODUCT_STR:
			if (pdev->pDesc->GetProductStrDescriptor) {
				buffer = pdev->pDesc->GetProductStrDescriptor(pdev->dev_speed, &size); break;
			} return stall_EP(pdev->pData, 0x0U);
		case USBD_IDX_SERIAL_STR:
			if (pdev->pDesc->GetSerialStrDescriptor) {
				buffer = pdev->pDesc->GetSerialStrDescriptor(pdev->dev_speed, &size); break;
			} return stall_EP(pdev->pData, 0x0U);
		case USBD_IDX_CONFIG_STR:
			if (pdev->pDesc->GetConfigurationStrDescriptor) {
				buffer = pdev->pDesc->GetConfigurationStrDescriptor(pdev->dev_speed, &size); break;
			} return stall_EP(pdev->pData, 0x0U);
		case USBD_IDX_INTERFACE_STR:
			if (pdev->pDesc->GetInterfaceStrDescriptor) {
				buffer = pdev->pDesc->GetInterfaceStrDescriptor(pdev->dev_speed, &size); break;
			} return stall_EP(pdev->pData, 0x0U);
		default:									return stall_EP(pdev->pData, 0x0U);
		} break;
	case USB_DESC_TYPE_DEVICE_QUALIFIER:			return stall_EP(pdev->pData, 0x0U);
	case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:	return stall_EP(pdev->pData, 0x0U);
	default:										return stall_EP(pdev->pData, 0x0U);
	}

	if (pdev->header.length) {
		if (size) {
			size = MIN(size, pdev->header.length);
			return IEP0_transfer(pdev, buffer, size);
		} return stall_EP(pdev->pData, 0x0U);
	}
	pdev->ep0_state = USBD_EP0_STATUS_IN;
	IN_transfer(pdev->pData, 0x00U, NULL, 0U);
}
static inline void get_device_configuration(USBD_HandleTypeDef* pdev) {
	if (
		pdev->header.length != 0xFFFFU ||
		pdev->dev_state == USBD_STATE_SUSPENDED
	) { return stall_EP(pdev->pData, 0x0U); }

	switch (pdev->dev_state) {
	case USBD_STATE_DEFAULT:
	case USBD_STATE_ADDRESSED:
		pdev->dev_default_config = 0U;
		return IEP0_transfer(pdev, &pdev->dev_default_config, 1U);
	case USBD_STATE_CONFIGURED:
		return IEP0_transfer(pdev, &pdev->dev_config, 1U);
	}
}
static inline void set_device_configuration(USBD_HandleTypeDef* pdev) {
	uint8_t config = pdev->header.value & 0xFFU;
	if (config > USBD_MAX_NUM_CONFIGURATION) { return stall_EP(pdev->pData, 0x0U); }

	switch (pdev->dev_state) {
	case USBD_STATE_ADDRESSED:
		if (!config) {
			pdev->ep0_state = USBD_EP0_STATUS_IN;
			return IN_transfer(pdev->pData, 0x00U, NULL, 0U);
		}
		pdev->dev_config = config;
		if (pdev->pClass[0]->Init(pdev, config) != USBD_OK) {
			pdev->dev_state = USBD_STATE_ADDRESSED;
			break;
		}
		pdev->ep0_state = USBD_EP0_STATUS_IN;
		pdev->dev_state = USBD_STATE_CONFIGURED;
		return IN_transfer(pdev->pData, 0x00U, NULL, 0U);
	case USBD_STATE_CONFIGURED:
		if (!config) {
			pdev->dev_state = USBD_STATE_ADDRESSED;
			pdev->dev_config = config;
			pdev->pClass[0]->DeInit(pdev, config);
			pdev->ep0_state = USBD_EP0_STATUS_IN;
			return IN_transfer(pdev->pData, 0x00U, NULL, 0U);
		} else if (config != pdev->dev_config) {
			pdev->pClass[0]->DeInit(pdev, (uint8_t)pdev->dev_config);
			pdev->dev_config = config;
			if (pdev->pClass[0]->Init(pdev, config) != USBD_OK){
				pdev->pClass[0]->DeInit(pdev, (uint8_t)pdev->dev_config);
				pdev->dev_state = USBD_STATE_ADDRESSED;
				break;
			}
		}
		pdev->ep0_state = USBD_EP0_STATUS_IN;
		return IN_transfer(pdev->pData, 0x00U, NULL, 0U);
	default: pdev->pClass[0]->DeInit(pdev, config); break;
	}
	stall_EP(pdev->pData, 0x0U);
}
static inline void get_dendpoint_status(USBD_HandleTypeDef* pdev, uint8_t ep_num) {
	PCD_HandleTypeDef*		hpcd = (PCD_HandleTypeDef*) pdev->pData;
	USBD_EndpointTypeDef*	pep;

	switch (pdev->dev_state) {
	case USBD_STATE_ADDRESSED:
		if ((ep_num != 0x00U) && (ep_num != 0x80U)) { break; }
		pep = ((ep_num & 0x80U) == 0x80U) ? &pdev->ep_in[ep_num & 0x7FU] : &pdev->ep_out[ep_num & 0x7FU];
		pep->status = 0x0000U;
		return IEP0_transfer(pdev, &pep->status, 2U);
	case USBD_STATE_CONFIGURED:
		if (
			(ep_num & 0x80U && !pdev->ep_in[ep_num & 0xFU].is_used) ||
			!pdev->ep_out[ep_num & 0xFU].is_used
		) { break; }
		pep = (ep_num & 0x80U) ? &pdev->ep_in[ep_num & 0x7FU] : &pdev->ep_out[ep_num & 0x7FU];
		pep->status = 0x0000U;
		if ((ep_num != 0x00U) && (ep_num != 0x80U) &&
			((ep_num & 0x80U && hpcd->IN_ep[ep_num & 0x7FU].is_stall) ||
			hpcd->OUT_ep[ep_num & 0x7FU].is_stall)
		) { pep->status = 0x0001U; }
		return IEP0_transfer(pdev, &pep->status, 2U);
	default: break;
	}
	stall_EP(pdev->pData, 0x0U);
}
static inline void clear_endpoint_feature(USBD_HandleTypeDef* pdev, uint8_t ep_num) {
	switch (pdev->dev_state) {
	case USBD_STATE_ADDRESSED:
		if ((ep_num != 0x00U) && (ep_num != 0x80U)) {
			if (ep_num & 0x80)	{ stall_IEP(pdev->pData, ep_num & 0xFU); }
			else				{ stall_OEP(pdev->pData, ep_num & 0xFU); }
		} return stall_EP(pdev->pData, 0x0U);
	case USBD_STATE_CONFIGURED:
		if (pdev->header.value == USB_FEATURE_EP_HALT) {
			if (ep_num == 0x80U)		{ unstall_IEP(pdev->pData, 0U); }
			else if (ep_num == 0x00U)	{ unstall_OEP(pdev->pData, 0U); }
			pdev->ep0_state = USBD_EP0_STATUS_IN;
			IN_transfer(pdev->pData, 0x00U, NULL, 0U);
			pdev->classId = 0;
			if (pdev->pClass[0]->Setup != NULL) {
				(void)(pdev->pClass[0]->Setup(pdev, (void*)&pdev->header));
			}
		}
		return;
	default: break;
	}
	stall_EP(pdev->pData, 0x0U);
}
static inline void set_endpoin_feature(USBD_HandleTypeDef* pdev, uint8_t ep_num) {
	switch (pdev->dev_state) {
	case USBD_STATE_ADDRESSED:
		if ((ep_num != 0x00U) && (ep_num != 0x80U)) {
			if (ep_num & 0x80)	{ stall_IEP(pdev->pData, ep_num & 0xFU); }
			else				{ stall_OEP(pdev->pData, ep_num & 0xFU); }
			stall_IEP(pdev->pData, 0x0U);
		}
		else {
			stall_EP(pdev->pData, 0x0U);
		}
		return;
	case USBD_STATE_CONFIGURED:
		if (pdev->header.value == USB_FEATURE_EP_HALT) {
			if ((ep_num != 0x00U) && (ep_num != 0x80U) && !pdev->header.length) {
				if (ep_num & 0x80)	{ stall_IEP(pdev->pData, ep_num & 0xFU); }
				else				{ stall_OEP(pdev->pData, ep_num & 0xFU); }
			}
		}
		pdev->ep0_state = USBD_EP0_STATUS_IN;
		IN_transfer(pdev->pData, 0x00U, NULL, 0U);
		return;
	default: break;
	}
	stall_EP(pdev->pData, 0x0U);
}

static inline void device_setup_request(USBD_HandleTypeDef* pdev) {
	switch (pdev->header.type) {
	case STANDARD_REQUEST:
		switch (pdev->header.command) {
		case GET_STATUS:		return get_device_status(pdev);
		case CLEAR_FEATURE:		return clear_device_feature(pdev);
		case SET_FEATURE:		return set_device_feature(pdev);
		case SET_ADDRESS:		return set_device_address(pdev);
		case GET_DESCRIPTOR:	return get_device_descriptor(pdev);
		case SET_DESCRIPTOR:	break;
		case GET_CONFIGURATION:	return get_device_configuration(pdev);
		case SET_CONFIGURATION:	return set_device_configuration(pdev);
		default:				break;
		} break;
	default: break;
	}
	stall_EP(pdev->pData, 0x0U);
}
static inline void interface_setup_request(USBD_HandleTypeDef* pdev) {
	if (
		pdev->dev_state == USBD_STATE_SUSPENDED ||
		!pdev->dev_state || pdev->header.type > VENDOR_REQUEST ||
		(pdev->header.index & 0xFFU) > USBD_MAX_NUM_INTERFACES
	) { return stall_EP(pdev->pData, 0x0U); }
	if (pdev->pClass[0]->Setup != NULL) {
		pdev->classId = 0;
		(void)(pdev->pClass[0]->Setup(pdev, (void*)&pdev->header));
	}
	if (!pdev->header.length) {
		pdev->ep0_state = USBD_EP0_STATUS_IN;
		IN_transfer(pdev->pData, 0x00U, NULL, 0U);
	}
}
static inline void endpoint_setup_request(USBD_HandleTypeDef* pdev) {
	uint8_t ep_num = pdev->header.index & 0xFFU;

	switch (pdev->header.type) {
	case USB_REQ_TYPE_STANDARD:
		switch (pdev->header.command) {
		case USB_REQ_SET_FEATURE:	return set_endpoin_feature(pdev, ep_num);
		case USB_REQ_CLEAR_FEATURE:	return clear_endpoint_feature(pdev, ep_num);
		case USB_REQ_GET_STATUS:	return get_dendpoint_status(pdev, ep_num);
		default: break;
		} break;
	case USB_REQ_TYPE_CLASS:
	case USB_REQ_TYPE_VENDOR:
		pdev->classId = 0;
		if (pdev->pClass[0]->Setup) {
			(void)pdev->pClass[0]->Setup(pdev, (void*)&pdev->header);
		}
		return;
	default: break;
	}
	stall_EP(pdev->pData, 0x0U);
}

static inline void USB_OTG_IRQ(USBD_HandleTypeDef* pdev, USB_OTG_GlobalTypeDef* usb) {
	// TODO: redo handle and make it possible to pass usb ptr
	uint32_t tmp = usb->GOTGINT;
	if (tmp & USB_OTG_GOTGINT_SEDET) {
		pdev->dev_state = USBD_STATE_DEFAULT;
		if (pdev->pClass[0] == NULL) { return; }
		pdev->pClass[0]->DeInit(pdev, (uint8_t)pdev->dev_config);
	}
	usb->GOTGINT |= tmp;
}
static inline void USB_SOF_IRQ(USBD_HandleTypeDef* pdev, USB_OTG_GlobalTypeDef* usb ) {
	// TODO: redo handle and make it possible to pass usb ptr
	if (pdev->dev_state == USBD_STATE_CONFIGURED) {
		if (pdev->pClass[0] == NULL)		{ return; }
		if (pdev->pClass[0]->SOF == NULL)	{ return; }
		(void)pdev->pClass[0]->SOF(pdev);
	}
	usb->GINTSTS |= USB_OTG_GINTSTS_SOF;
}
static inline void USB_receive_packet_IRQ(PCD_HandleTypeDef* hpcd) {
	// TODO: redo handle and make it possible to pass usb ptr
	USB_OTG_GlobalTypeDef*	usb = hpcd->Instance;
	usb->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
	uint32_t				tmp = usb->GRXSTSP;
	GRXSTS_t				status = *((GRXSTS_t*)&tmp);
	USB_OTG_EPTypeDef*		ep = &hpcd->OUT_ep[status.EPNUM];  // TODO: redo handle
	volatile uint32_t*		FIFO = (void*)(((uint32_t)usb) + USB_OTG_FIFO_BASE);

	switch (status.PKTSTS) {
	case STS_DATA_UPDT:
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
	case STS_SETUP_UPDT:
		__UNALIGNED_UINT32_WRITE(&hpcd->Setup[0], *FIFO);
		__UNALIGNED_UINT32_WRITE(&hpcd->Setup[1], *FIFO);
		ep->xfer_count += 8;
		break;
	default: break;
	}

	usb->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
}
static inline void USB_global_NAK_OUT_IRQ(PCD_HandleTypeDef* hpcd) {
	// TODO: redo handle and make it possible to pass usb ptr
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE);
	PCD_EPTypeDef*				ep;

	usb->GINTMSK &= ~USB_OTG_GINTMSK_GONAKEFFM;
	for (uint8_t ep_num = 1U; ep_num < hpcd->Init.dev_endpoints; ep_num++) {
		ep = &hpcd->OUT_ep[ep_num];
		if (ep->is_iso_incomplete == 1U) { continue; }
		if (out[ep_num].DOEPCTL & USB_OTG_DOEPCTL_EPENA) {
			out[ep_num].DOEPCTL |= (USB_OTG_DOEPCTL_SNAK | USB_OTG_DOEPCTL_EPDIS);
			while (out[ep_num].DOEPCTL & USB_OTG_DOEPCTL_EPENA);
		}
	}
}
static inline void USB_suspend_IRQ(PCD_HandleTypeDef* hpcd) {
	// TODO: redo handle and make it possible to pass usb ptr
	USB_OTG_GlobalTypeDef* 	usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*	device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	__IO uint32_t*			PCGCCTL =	(void*)(((uint32_t)usb) + USB_OTG_PCGCCTL_BASE);

	if ((device->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS) {
		USBD_HandleTypeDef* pdev = hpcd->pData;
		if (pdev->dev_state != USBD_STATE_SUSPENDED) {
			pdev->dev_old_state = pdev->dev_state;
		}
		pdev->dev_state = USBD_STATE_SUSPENDED;
		*PCGCCTL |= USB_OTG_PCGCCTL_STOPCLK;
		if (hpcd->Init.low_power_enable) {
			SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
		}
	}
	usb->GINTSTS |= USB_OTG_GINTSTS_USBSUSP;
}
static inline void USB_reset_IRQ(PCD_HandleTypeDef* hpcd) {
	// TODO: redo handle and make it possible to pass usb ptr
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE);

	device->DCTL &= ~USB_OTG_DCTL_RWUSIG;
	flush_TX_FIFOS(hpcd->Instance);
	for (uint8_t i = 0U; i < hpcd->Init.dev_endpoints; i++) {
		in[i].DIEPINT =		0xFB7FU;
		in[i].DIEPCTL &=	~USB_OTG_DIEPCTL_STALL;
		out[i].DOEPINT =	0xFB7FU;
		out[i].DOEPCTL &=	~USB_OTG_DOEPCTL_STALL;
		out[i].DOEPCTL |=	USB_OTG_DOEPCTL_SNAK;
	}
	device->DAINTMSK |= 0x10001U;
	if (hpcd->Init.use_dedicated_ep1) {
		device->DOUTEP1MSK |= (
			USB_OTG_DOEPMSK_STUPM		|
			USB_OTG_DOEPMSK_XFRCM		|
			USB_OTG_DOEPMSK_EPDM
		);
		device->DINEP1MSK |= (
			USB_OTG_DIEPMSK_TOM			|
			USB_OTG_DIEPMSK_XFRCM		|
		  	USB_OTG_DIEPMSK_EPDM
		);
	} else {
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
	}

	device->DCFG &= ~USB_OTG_DCFG_DAD;
	start_OEP0(usb);
	usb->GINTSTS |= USB_OTG_GINTSTS_USBRST;
}
static inline void USB_enumeration_done_IRQ(PCD_HandleTypeDef* hpcd) {
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE);
	USBD_HandleTypeDef*			pdev =		hpcd->pData;

	in[0].DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
	device->DCTL |= USB_OTG_DCTL_CGINAK;
	hpcd->Init.speed = USBD_FS_SPEED;

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

	pdev->dev_speed = USBD_SPEED_FULL;
	pdev->dev_state = USBD_STATE_DEFAULT;
	pdev->ep0_state = USBD_EP0_IDLE;
	pdev->dev_config = 0U;
	pdev->dev_remote_wakeup = 0U;
	pdev->dev_test_mode = 0U;
	if (pdev->pClass[0] != NULL) {
		if (pdev->pClass[0]->DeInit != NULL) {
			pdev->pClass[0]->DeInit(pdev, (uint8_t)pdev->dev_config);
		}
	}

	open_OEP(pdev->pData, 0x00U, USB_MAX_EP0_SIZE, USBD_EP_TYPE_CTRL);
	pdev->ep_out[0].is_used = 1U;
	pdev->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;

	open_IEP(pdev->pData, 0x00U, USB_MAX_EP0_SIZE, USBD_EP_TYPE_CTRL);
	pdev->ep_in[0].is_used = 1U;
	pdev->ep_in[0].maxpacket = USB_MAX_EP0_SIZE;

	usb->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE;
}
static inline void IEP_transfer_complete_IRQ(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USBD_HandleTypeDef*			pdev =		hpcd->pData;
	USBD_EndpointTypeDef*		ep;
	uint8_t*					data = hpcd->IN_ep[ep_num].xfer_buff;

	device->DIEPEMPMSK &= ~(0x1UL << ep_num);
	in->DIEPINT |= USB_OTG_DIEPINT_XFRC;

	if (!ep_num) {
		ep = &pdev->ep_in[0];
		if (pdev->ep0_state != USBD_EP0_DATA_IN) { return; }
		if (ep->rem_length > ep->maxpacket) {
			ep->rem_length -= ep->maxpacket;
			IN_transfer(hpcd, 0x00U, data, ep->rem_length);
			OUT_transfer(hpcd, 0x00U, NULL, 0U);
		} else if (
			(ep->maxpacket == ep->rem_length) &&
			(ep->total_length >= ep->maxpacket) &&
			(ep->total_length < pdev->ep0_data_len)
		) {
			IN_transfer(hpcd, 0x00U, NULL, 0U);
			pdev->ep0_data_len = 0U;
			OUT_transfer(hpcd, 0x00U, NULL, 0U);
		} else {
			if (pdev->dev_state == USBD_STATE_CONFIGURED && pdev->pClass[0]->EP0_TxSent != NULL) {
				pdev->classId = 0U;
				pdev->pClass[0]->EP0_TxSent(pdev);
			}
			stall_IEP(hpcd, 0x0U);
			pdev->ep0_state = USBD_EP0_STATUS_OUT;
			OUT_transfer(hpcd, 0x00U, NULL, 0U);
		}
		return;
	}
	if (pdev->dev_state == USBD_STATE_CONFIGURED && pdev->pClass[0]->DataIn != NULL) {
		pdev->classId = 0;
		(void)pdev->pClass[0]->DataIn(pdev, ep_num);
	}
}
static inline void IEP_disabled_IRQ(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_OTG_EPTypeDef*			ep =		&hpcd->IN_ep[ep_num];
	USBD_HandleTypeDef*			pdev =		hpcd->pData;

	flush_TX_FIFO(hpcd->Instance, ep_num);
	if (ep->is_iso_incomplete == 1U) {
		ep->is_iso_incomplete = 0U;
		if (
			pdev->pClass[pdev->classId] &&
			pdev->pClass[pdev->classId]->IsoINIncomplete &&
			pdev->dev_state == USBD_STATE_CONFIGURED
		) {
			(void)pdev->pClass[pdev->classId]->IsoINIncomplete(pdev, ep_num);
		}
	}
	in->DIEPINT |= USB_OTG_DIEPINT_EPDISD;
}
static inline void IEP_FIFO_empty_IRQ(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_OTG_EPTypeDef*			ep =		&hpcd->IN_ep[ep_num];
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
static inline void OEP_transfer_complete(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_OTG_EPTypeDef*			ep =		&hpcd->OUT_ep[ep_num];
	USBD_HandleTypeDef*			pdev =		hpcd->pData;
	out->DOEPINT |= USB_OTG_DOEPINT_XFRC;

	if (!(ep_num | ep->xfer_len)) { start_OEP0(usb); }

	if (!ep_num && pdev->ep0_state == USBD_EP0_DATA_OUT) {
		USBD_EndpointTypeDef*	USBD_ep = &pdev->ep_out[0];
		if (USBD_ep->rem_length > USBD_ep->maxpacket) {
			USBD_ep->rem_length -= USBD_ep->maxpacket;
			OUT_transfer(hpcd, 0x00U, ep->xfer_buff, MIN(USBD_ep->rem_length, USBD_ep->maxpacket));
			return;
		}
		if (pdev->dev_state == USBD_STATE_CONFIGURED && pdev->pClass[0]->EP0_RxReady) {
			pdev->classId = 0;
			pdev->pClass[0]->EP0_RxReady(pdev);
		}
		pdev->ep0_state = USBD_EP0_STATUS_IN;
		IN_transfer(hpcd, 0x00U, NULL, 0U);
	} else if (pdev->dev_state == USBD_STATE_CONFIGURED && pdev->pClass[0]->DataOut) {
		pdev->classId = 0;
		(void)pdev->pClass[0]->DataOut(pdev, ep_num);
	}
}
static inline void OEP_disabled(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USB_OTG_EPTypeDef*			ep =		&hpcd->OUT_ep[ep_num];
	USBD_HandleTypeDef*			pdev =		hpcd->pData;

	if (usb->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) { device->DCTL |= USB_OTG_DCTL_CGONAK; }
	if (ep->is_iso_incomplete == 1U) {
		ep->is_iso_incomplete = 0U;
		if (
			pdev->pClass[pdev->classId] &&
			pdev->dev_state == USBD_STATE_CONFIGURED &&
			pdev->pClass[pdev->classId]->IsoOUTIncomplete
		) {
			(void)pdev->pClass[pdev->classId]->IsoOUTIncomplete(pdev, ep_num);
		}
	}
	out->DOEPINT |= USB_OTG_DOEPINT_EPDISD;
}
static inline void OEP_setup_done(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	USBD_HandleTypeDef*			pdev =		hpcd->pData;
	out->DOEPINT |= USB_OTG_DOEPINT_STUP;

	uint8_t* data =		hpcd->Setup;
	uint8_t* header =	(uint8_t*)&pdev->header;
	*header++ =	*data++;
	*header++ =	*data++;
	*header++ =	*data++;
	*header++ =	*data++;
	*header++ =	*data++;
	*header++ =	*data++;
	*header++ =	*data++;
	*header =	*data;

	pdev->ep0_state = USBD_EP0_SETUP;
	pdev->ep0_data_len = pdev->header.length;
	switch (pdev->header.recipiant) {
	case RECIPIANT_DEVICE:		return device_setup_request(pdev);
	case RECIPIANT_INTERFACE:	return interface_setup_request(pdev);
	case RECIPIANT_ENDPOINT:	return endpoint_setup_request(pdev);
	default: break;
	}
	if (pdev->header.direction)	{ stall_IEP(pdev->pData, 0x0U); }
	else						{ stall_OEP(pdev->pData, 0x0U); }
}
static inline void USB_incomplete_ISO_IN_IRQ(PCD_HandleTypeDef* hpcd) {
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE);
	PCD_EPTypeDef*				ep;

	for (uint8_t ep_num = 1U; ep_num < hpcd->Init.dev_endpoints; ep_num++) {
		ep = &hpcd->IN_ep[ep_num];
		if (ep->type != EP_TYPE_ISOC || !(in[ep_num].DIEPCTL & USB_OTG_DIEPCTL_EPENA)) { continue; }
		ep->is_iso_incomplete = 1U;
		if (in[ep_num].DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
			in[ep_num].DIEPCTL |= (USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_EPDIS);
			while (in[ep_num].DIEPCTL & USB_OTG_DIEPCTL_EPENA);
		}
	}
	usb->GINTSTS |= USB_OTG_GINTSTS_IISOIXFR;
}
static inline void USB_incomplete_ISO_OUT_IRQ(PCD_HandleTypeDef* hpcd) {
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE);
	uint32_t					tmp;

	for (uint8_t ep_num = 1U; ep_num < hpcd->Init.dev_endpoints; ep_num++) {
		tmp = out[ep_num].DOEPCTL;
		if ((hpcd->OUT_ep[ep_num].type == EP_TYPE_ISOC) && tmp & USB_OTG_DOEPCTL_EPENA &&
			(tmp & (0x1U << 16)) == (hpcd->FrameNumber & 0x1U)) {
			hpcd->OUT_ep[ep_num].is_iso_incomplete = 1U;
			usb->GINTMSK |= USB_OTG_GINTMSK_GONAKEFFM;
			if ((usb->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) == 0U) {
				device->DCTL |= USB_OTG_DCTL_SGONAK;
				break;
			}
		}
	}
	usb->GINTSTS |= USB_OTG_GINTSTS_PXFR_INCOMPISOOUT;
}
static inline void USB_connection_IRQ(PCD_HandleTypeDef* hpcd) {
	USB_OTG_GlobalTypeDef*	usb =		hpcd->Instance;
	// TODO!!!
	usb->GINTSTS |= USB_OTG_GINTSTS_SRQINT;
}
static inline void USB_wake_up_IRQ(PCD_HandleTypeDef* hpcd) {
	USB_OTG_GlobalTypeDef*	usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*	device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USBD_HandleTypeDef*		pdev =		hpcd->pData;
	device->DCTL &= ~USB_OTG_DCTL_RWUSIG;
	if (pdev->dev_state == USBD_STATE_SUSPENDED) {
		pdev->dev_state = pdev->dev_old_state;
	}
	usb->GINTSTS |= USB_OTG_GINTSTS_WKUINT;
}


// handlers
static inline void IEP_common_handler(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	uint32_t					ep_int =	(in->DIEPINT & device->DIEPMSK) | ((device->DIEPEMPMSK >> ep_num) & 0x1U) << 7;

	/* transfer complete interrupt */
	if (ep_int & USB_OTG_DIEPINT_XFRC)			{ IEP_transfer_complete_IRQ(hpcd, ep_num); }
	/* endpoint disabled interrupt */
	if (ep_int & USB_OTG_DIEPINT_EPDISD)		{ IEP_disabled_IRQ(hpcd, ep_num); }
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
	if (ep_int & USB_OTG_DIEPINT_TXFE)			{ IEP_FIFO_empty_IRQ(hpcd, ep_num); }
	/* TX FIFO underrun interrupt */
	if (ep_int & USB_OTG_DIEPINT_TXFIFOUDRN)	{ in->DIEPINT |= USB_OTG_DIEPINT_TXFIFOUDRN; }
	/* buffer not available interrupt */
	if (ep_int & USB_OTG_DIEPINT_BNA)			{ in->DIEPINT |= USB_OTG_DIEPINT_BNA; }
	/* packet dropped interrupt */
	if (ep_int & USB_OTG_DIEPINT_PKTDRPSTS)		{ in->DIEPINT |= USB_OTG_DIEPINT_PKTDRPSTS; }
	/* NAK interrupt */
	if (ep_int & USB_OTG_DIEPINT_PKTDRPSTS)		{ in->DIEPINT |= USB_OTG_DIEPINT_NAK; }
}
static inline void OEP_common_handler(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	USB_OTG_GlobalTypeDef* 		usb =		hpcd->Instance;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
	uint32_t					ep_int =	out->DOEPINT & device->DOEPMSK;

	/* transfer complete interrupt */
	if (ep_int & USB_OTG_DOEPINT_XFRC)					{ OEP_transfer_complete(hpcd, ep_num); }
	/* endpoint disabled interrupt */
	if (ep_int & USB_OTG_DOEPINT_EPDISD)				{ OEP_disabled(hpcd, ep_num); }
	/* AHB error interrupt */
	if (ep_int & USB_OTG_DOEPINT_AHBERR)				{ out->DOEPINT |= USB_OTG_DOEPINT_AHBERR; }
	/* SETUP phase done */
	if (ep_int & USB_OTG_DOEPINT_STUP)					{ OEP_setup_done(hpcd, ep_num); }
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
static inline void USB_common_handler(PCD_HandleTypeDef* hpcd) {
	USB_OTG_GlobalTypeDef*	usb = hpcd->Instance;
	if (((usb->GINTSTS) & 0x1U) != USB_OTG_MODE_DEVICE) { return; }
	const uint32_t			irqs = usb->GINTSTS & usb->GINTMSK;
	if (!irqs)				{ return; }
	USB_OTG_DeviceTypeDef*	device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	uint8_t					ep_num;
	uint16_t				ep_gint;

	/* store current frame number */
	hpcd->FrameNumber = (device->DSTS & USB_OTG_DSTS_FNSOF_Msk) >> USB_OTG_DSTS_FNSOF_Pos;

	/* mode mismatch interrupt */
	if (irqs & USB_OTG_GINTSTS_OTGINT)					{ usb->GINTSTS |= USB_OTG_GINTSTS_MMIS; }
	/* OTG interrupt */
	if (irqs & USB_OTG_GINTSTS_OTGINT)					{ USB_OTG_IRQ(hpcd->pData, usb); }
	/* start of frame interrupt */
	if (irqs & USB_OTG_GINTSTS_SOF)						{ USB_SOF_IRQ(hpcd->pData, usb); }
	/* receive packet interrupt */
	if (irqs & USB_OTG_GINTSTS_RXFLVL)					{ USB_receive_packet_IRQ(hpcd); }
	/* global OUT NAK effective interrupt */
	if (irqs & USB_OTG_GINTSTS_BOUTNAKEFF)				{ USB_global_NAK_OUT_IRQ(hpcd); }
	/* suspend interrupt */
	if (irqs & USB_OTG_GINTSTS_USBSUSP)					{ USB_suspend_IRQ(hpcd); }
	/* reset interrupt */
	if (irqs & USB_OTG_GINTSTS_USBRST)					{ USB_reset_IRQ(hpcd); }
	/* enumeration done interrupt */
	if (irqs & USB_OTG_GINTSTS_ENUMDNE)					{ USB_enumeration_done_IRQ(hpcd); }
	/* IN endpoint interrupts */
	if (irqs & USB_OTG_GINTSTS_IEPINT) {
		ep_gint = device->DAINT & device->DAINTMSK & 0xFFFFU;
		ep_num = 0U;
		while (ep_gint) {
			if (ep_gint & 0b1UL)						{ IEP_common_handler(hpcd, ep_num); }
			ep_num++; ep_gint >>= 1U;
		}
	}
	/* OUT endpoint interrupts */
	if (irqs & USB_OTG_GINTSTS_OEPINT) {
		ep_gint = (device->DAINT & device->DAINTMSK) >> 0x10UL;
		ep_num = 0U;
		while (ep_gint) {
			if (ep_gint & 0x1U)							{ OEP_common_handler(hpcd, ep_num); }
			ep_num++; ep_gint >>= 1U;
		}
	}
	/* incomplete isochronous IN interrupt */
	if (irqs & USB_OTG_GINTSTS_IISOIXFR)				{ USB_incomplete_ISO_IN_IRQ(hpcd); }
	/* incomplete isochronous OUT interrupt */
	if (irqs & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)		{ USB_incomplete_ISO_OUT_IRQ(hpcd); }
	/* connection event interrupt */
	if (irqs & USB_OTG_GINTSTS_SRQINT)					{ USB_connection_IRQ(hpcd); }
	/* wake-up interrupt */
	if (irqs & USB_OTG_GINTSTS_WKUINT)					{ USB_wake_up_IRQ(hpcd); }
}
static inline void USB_wakeup_handler(PCD_HandleTypeDef* hpcd) {
	USB_OTG_GlobalTypeDef*	usb =		hpcd->Instance;
	__IO uint32_t*			PCGCCTL =	(void*)(((uint32_t)usb) + USB_OTG_PCGCCTL_BASE);
	if (hpcd->Init.low_power_enable) {
		SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
		sys_clock_init();
	}
	*PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK);
	EXTI->PR |= EXTI_PR_PR18;  // USB FS EXTI line TODO: modular
}


// interrupts
void OTG_FS_IRQHandler(void)		{ USB_common_handler(&hpcd_USB_OTG_FS); }
void OTG_FS_WKUP_IRQHandler(void)	{ USB_wakeup_handler(&hpcd_USB_OTG_FS); }