//
// Created by marijn on 4/2/24.
//
#include "usb/usb.h"
//#include "usb/hid.h"


#define inline __attribute__((always_inline))

/*!<
 * types
 * */
typedef __PACKED_STRUCT {
	uint32_t EPNUM		: 4;	// endpoint number
	uint32_t BCNT		: 11;	// byte count
	uint32_t DPID		: 2;	// data PID
	uint32_t PKTSTS		: 4;	// packet status
	uint32_t _			: 11;
}	GRXSTS_t;

// L1 ========================================= /
#define STS_GOUT_NAK                           1U
#define STS_DATA_UPDT                          2U
#define STS_XFER_COMP                          3U
#define STS_SETUP_COMP                         4U
#define STS_SETUP_UPDT                         6U

// L2 ========================================= /
#define EP_ADDR_MSK                            0xFU
#define USB_OTG_CORE_ID_300A          0x4F54300AU
#define USB_OTG_CORE_ID_310A          0x4F54310AU


// defs
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
USBD_StatusTypeDef USBD_CtlSendStatus(USBD_HandleTypeDef *pdev);
HAL_StatusTypeDef USB_EP0_OutStart(const USB_OTG_GlobalTypeDef *USBx, uint8_t dma, const uint8_t *psetup);
HAL_StatusTypeDef USB_WritePacket(const USB_OTG_GlobalTypeDef *USBx, uint8_t *src, uint8_t ch_ep_num, uint16_t len, uint8_t dma);


// L? ========================================= /
extern USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);
extern void /*L0*/ Error_Handler(void);
void USBD_CtlError(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	UNUSED(req);
	(void)USBD_LL_StallEP(pdev, 0x80U);
	(void)USBD_LL_StallEP(pdev, 0U);
}


// L7 ========================================= /
HAL_StatusTypeDef USB_EPStartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep, uint8_t dma){
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t epnum = (uint32_t)ep->num;
	uint16_t pktcnt;
	/* IN endpoint */
	if (ep->is_in == 1U) {
		/* Zero Length Packet? */
		if (ep->xfer_len == 0U) {
			USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
			USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19));
			USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
		}
		else {
			/* Program the transfer size and packet count
      * as follows: xfersize = N * maxpacket +
      * short_packet pktcnt = N + (short_packet
      * exist ? 1 : 0)
			 */
			USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
			USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);

			if (epnum == 0U) {
				if (ep->xfer_len > ep->maxpacket) {
					ep->xfer_len = ep->maxpacket;
				}

				USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19));
			}
			else {
				USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT &
											   (((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket) << 19));
			}

			USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & ep->xfer_len);

			if (ep->type == EP_TYPE_ISOC) {
				USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_MULCNT);
				USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_MULCNT & (1U << 29));
			}
		}

		if (dma == 1U) {
			if ((uint32_t)ep->dma_addr != 0U) {
				USBx_INEP(epnum)->DIEPDMA = (uint32_t)(ep->dma_addr);
			}

			if (ep->type == EP_TYPE_ISOC) {
				if ((USBx_DEVICE->DSTS & (1U << 8)) == 0U) {
					USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SODDFRM;
				}
				else {
					USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
				}
			}

			/* EP enable, IN data in FIFO */
			USBx_INEP(epnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
		}
		else {
			/* EP enable, IN data in FIFO */
			USBx_INEP(epnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

			if (ep->type != EP_TYPE_ISOC) {
				/* Enable the Tx FIFO Empty Interrupt for this EP */
				if (ep->xfer_len > 0U) {
					USBx_DEVICE->DIEPEMPMSK |= 1UL << (ep->num & EP_ADDR_MSK);
				}
			}
			else {
				if ((USBx_DEVICE->DSTS & (1U << 8)) == 0U) {
					USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SODDFRM;
				}
				else {
					USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
				}
				(void)USB_WritePacket(USBx, ep->xfer_buff, ep->num, (uint16_t)ep->xfer_len, dma);
			}
		}
	}
	else /* OUT endpoint */ {
		/* Program the transfer size and packet count as follows:
    * pktcnt = N
    * xfersize = N * maxpacket
		 */
		USBx_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
		USBx_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

		if (epnum == 0U) {
			if (ep->xfer_len > 0U) {
				ep->xfer_len = ep->maxpacket;
			}

			/* Store transfer size, for EP0 this is equal to endpoint max packet size */
			ep->xfer_size = ep->maxpacket;
			USBx_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & ep->xfer_size);
			USBx_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
		}
		else {
			if (ep->xfer_len == 0U) {
				USBx_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & ep->maxpacket);
				USBx_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
			}
			else {
				pktcnt = (uint16_t)((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket);
				ep->xfer_size = ep->maxpacket * pktcnt;

				USBx_OUTEP(epnum)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_PKTCNT & ((uint32_t)pktcnt << 19);
				USBx_OUTEP(epnum)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_XFRSIZ & ep->xfer_size;
			}
		}

		if (dma == 1U) {
			if ((uint32_t)ep->xfer_buff != 0U) {
				USBx_OUTEP(epnum)->DOEPDMA = (uint32_t)(ep->xfer_buff);
			}
		}

		if (ep->type == EP_TYPE_ISOC) {
			if ((USBx_DEVICE->DSTS & (1U << 8)) == 0U) {
				USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SODDFRM;
			}
			else {
				USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
			}
		}
		/* EP enable */
		USBx_OUTEP(epnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}
	return HAL_OK;
}
HAL_StatusTypeDef USB_EPSetStall(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t epnum = (uint32_t)ep->num;
	if (ep->is_in == 1U) {
		if (((USBx_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U) && (epnum != 0U)) {
			USBx_INEP(epnum)->DIEPCTL &= ~(USB_OTG_DIEPCTL_EPDIS);
		}
		USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
	}
	else {
		if (((USBx_OUTEP(epnum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == 0U) && (epnum != 0U)) {
			USBx_OUTEP(epnum)->DOEPCTL &= ~(USB_OTG_DOEPCTL_EPDIS);
		}
		USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
	}

	return HAL_OK;
}
HAL_StatusTypeDef USB_ActivateEndpoint(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t epnum = (uint32_t)ep->num;
	if (ep->is_in == 1U) {
		USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK));
		if ((USBx_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0U) {
			USBx_INEP(epnum)->DIEPCTL |= (ep->maxpacket & USB_OTG_DIEPCTL_MPSIZ) |
										 ((uint32_t)ep->type << 18) | (epnum << 22) |
										 USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
										 USB_OTG_DIEPCTL_USBAEP;
		}
	}
	else {
		USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16);
		if (((USBx_OUTEP(epnum)->DOEPCTL) & USB_OTG_DOEPCTL_USBAEP) == 0U) {
			USBx_OUTEP(epnum)->DOEPCTL |= (ep->maxpacket & USB_OTG_DOEPCTL_MPSIZ) |
										  ((uint32_t)ep->type << 18) |
										  USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
										  USB_OTG_DOEPCTL_USBAEP;
		}
	}
	return HAL_OK;
}
HAL_StatusTypeDef USB_DeactivateEndpoint(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t epnum = (uint32_t)ep->num;
	/* Read DEPCTLn register */
	if (ep->is_in == 1U) {
		if ((USBx_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA) {
			USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
			USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
		}
		USBx_DEVICE->DEACHMSK &= ~(USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK)));
		USBx_DEVICE->DAINTMSK &= ~(USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK)));
		USBx_INEP(epnum)->DIEPCTL &= ~(USB_OTG_DIEPCTL_USBAEP |
									   USB_OTG_DIEPCTL_MPSIZ |
									   USB_OTG_DIEPCTL_TXFNUM |
									   USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
									   USB_OTG_DIEPCTL_EPTYP);
	}
	else {
		if ((USBx_OUTEP(epnum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) {
			USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
			USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS;
		}
		USBx_DEVICE->DEACHMSK &= ~(USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16));
		USBx_DEVICE->DAINTMSK &= ~(USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16));
		USBx_OUTEP(epnum)->DOEPCTL &= ~(USB_OTG_DOEPCTL_USBAEP |
										USB_OTG_DOEPCTL_MPSIZ |
										USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
										USB_OTG_DOEPCTL_EPTYP);
	}
	return HAL_OK;
}
HAL_StatusTypeDef USB_EPClearStall(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t epnum = (uint32_t)ep->num;
	if (ep->is_in == 1U) {
		USBx_INEP(epnum)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
		if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK)) {
			USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM; /* DATA0 */
		}
	}
	else {
		USBx_OUTEP(epnum)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
		if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK)) {
			USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM; /* DATA0 */
		}
	}
	return HAL_OK;
}
HAL_StatusTypeDef USB_SetDevAddress(const USB_OTG_GlobalTypeDef *USBx, uint8_t address) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	USBx_DEVICE->DCFG &= ~(USB_OTG_DCFG_DAD);
	USBx_DEVICE->DCFG |= ((uint32_t)address << 4) & USB_OTG_DCFG_DAD;
	return HAL_OK;
}


// L6 ========================================= /
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len) {
	PCD_EPTypeDef *ep;
	ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
	/*setup and start the Xfer */
	ep->xfer_buff = pBuf;
	ep->xfer_len = len;
	ep->xfer_count = 0U;
	ep->is_in = 0U;
	ep->num = ep_addr & EP_ADDR_MSK;

	if (hpcd->Init.dma_enable == 1U) {
		ep->dma_addr = (uint32_t)pBuf;
	}
	(void)USB_EPStartXfer(hpcd->Instance, ep, (uint8_t)hpcd->Init.dma_enable);
	return HAL_OK;
}
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len) {
	PCD_EPTypeDef *ep;
	ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
	/*setup and start the Xfer */
	ep->xfer_buff = pBuf;
	ep->xfer_len = len;
	ep->xfer_count = 0U;
	ep->is_in = 1U;
	ep->num = ep_addr & EP_ADDR_MSK;

	if (hpcd->Init.dma_enable == 1U) {
		ep->dma_addr = (uint32_t)pBuf;
	}
	(void)USB_EPStartXfer(hpcd->Instance, ep, (uint8_t)hpcd->Init.dma_enable);
	return HAL_OK;
}
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr) {
	PCD_EPTypeDef *ep;
	if (((uint32_t)ep_addr & EP_ADDR_MSK) > hpcd->Init.dev_endpoints) {
		return HAL_ERROR;
	}
	if ((0x80U & ep_addr) == 0x80U){
		ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 1U;
	}
	else {
		ep = &hpcd->OUT_ep[ep_addr];
		ep->is_in = 0U;
	}
	ep->is_stall = 1U;
	ep->num = ep_addr & EP_ADDR_MSK;
	__HAL_LOCK(hpcd);
	(void)USB_EPSetStall(hpcd->Instance, ep);
	if ((ep_addr & EP_ADDR_MSK) == 0U) {
		(void)USB_EP0_OutStart(hpcd->Instance, (uint8_t)hpcd->Init.dma_enable, (uint8_t *)hpcd->Setup);
	}
	__HAL_UNLOCK(hpcd);
	return HAL_OK;
}
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type) {
	HAL_StatusTypeDef  ret = HAL_OK;
	PCD_EPTypeDef *ep;
	if ((ep_addr & 0x80U) == 0x80U) {
		ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 1U;
	}
	else {
		ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 0U;
	}
	ep->num = ep_addr & EP_ADDR_MSK;
	ep->maxpacket = ep_mps;
	ep->type = ep_type;

	if (ep->is_in != 0U) {
		/* Assign a Tx FIFO */
		ep->tx_fifo_num = ep->num;
	}
	/* Set initial data PID. */
	if (ep_type == EP_TYPE_BULK) {
		ep->data_pid_start = 0U;
	}
	__HAL_LOCK(hpcd);
	(void)USB_ActivateEndpoint(hpcd->Instance, ep);
	__HAL_UNLOCK(hpcd);
	return ret;
}
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr) {
	PCD_EPTypeDef *ep;
	if ((ep_addr & 0x80U) == 0x80U) {
		ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 1U;
	}
	else {
		ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 0U;
	}
	ep->num = ep_addr & EP_ADDR_MSK;
	__HAL_LOCK(hpcd);
	(void)USB_DeactivateEndpoint(hpcd->Instance, ep);
	__HAL_UNLOCK(hpcd);
	return HAL_OK;
}
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr){
	PCD_EPTypeDef *ep;
	if (((uint32_t)ep_addr & 0x0FU) > hpcd->Init.dev_endpoints) {
		return HAL_ERROR;
	}
	if ((0x80U & ep_addr) == 0x80U) {
		ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 1U;
	}
	else {
		ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 0U;
	}
	ep->is_stall = 0U;
	ep->num = ep_addr & EP_ADDR_MSK;
	__HAL_LOCK(hpcd);
	(void)USB_EPClearStall(hpcd->Instance, ep);
	__HAL_UNLOCK(hpcd);
	return HAL_OK;
}
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address) {
	__HAL_LOCK(hpcd);
	hpcd->USB_Address = address;
	(void)USB_SetDevAddress(hpcd->Instance, address);
	__HAL_UNLOCK(hpcd);

	return HAL_OK;
}
USBD_StatusTypeDef USBD_SetClassConfig(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	USBD_StatusTypeDef ret = USBD_OK;
	if (pdev->pClass[0] != NULL) {
		/* Set configuration and Start the Class */
		ret = (USBD_StatusTypeDef)pdev->pClass[0]->Init(pdev, cfgidx);
	}
	return ret;
}
USBD_StatusTypeDef USBD_ClrClassConfig(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	USBD_StatusTypeDef ret = USBD_OK;
	/* Clear configuration  and De-initialize the Class process */
	if (pdev->pClass[0]->DeInit(pdev, cfgidx) != 0U) {
		ret = USBD_FAIL;
	}
	return ret;
}


// L5 ========================================= /
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;
	hal_status = HAL_PCD_SetAddress(pdev->pData, dev_addr);
	usb_status =  USBD_Get_USB_Status(hal_status);
	return usb_status;
}
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;
	hal_status = HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
	usb_status =  USBD_Get_USB_Status(hal_status);
	return usb_status;
}
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;
	hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
	usb_status =  USBD_Get_USB_Status(hal_status);
	return usb_status;
}
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;
	hal_status = HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
	usb_status =  USBD_Get_USB_Status(hal_status);
	return usb_status;
}
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;
	hal_status = HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
	usb_status =  USBD_Get_USB_Status(hal_status);
	return usb_status;
}
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;
	hal_status = HAL_PCD_EP_Close(pdev->pData, ep_addr);
	usb_status =  USBD_Get_USB_Status(hal_status);
	return usb_status;
}
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;
	hal_status = HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
	usb_status =  USBD_Get_USB_Status(hal_status);
	return usb_status;
}
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr){
	PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;
	if((ep_addr & 0x80) == 0x80) {
		return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
	}
	else {
		return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
	}
}
USBD_StatusTypeDef USBD_CtlSendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint32_t len) {
	/* Set EP0 State */
	pdev->ep0_state = USBD_EP0_DATA_IN;
	pdev->ep_in[0].total_length = len;
	pdev->ep_in[0].rem_length = len;
	/* Start the transfer */
	(void)USBD_LL_Transmit(pdev, 0x00U, pbuf, len);
	return USBD_OK;
}
static void USBD_GetDescriptor(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	uint16_t len = 0U;
	uint8_t *pbuf = NULL;
	uint8_t err = 0U;

	switch (req->wValue >> 8) {
		case USB_DESC_TYPE_DEVICE:
			pbuf = pdev->pDesc->GetDeviceDescriptor(pdev->dev_speed, &len);
			break;
		case USB_DESC_TYPE_CONFIGURATION:
			if (pdev->dev_speed == USBD_SPEED_HIGH) { {
					pbuf = (uint8_t *)pdev->pClass[0]->GetHSConfigDescriptor(&len);
				}
				pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
			}
			else { {
					pbuf = (uint8_t *)pdev->pClass[0]->GetFSConfigDescriptor(&len);
				}
				pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
			}
			break;
		case USB_DESC_TYPE_STRING:
			switch ((uint8_t)(req->wValue)) {
				case USBD_IDX_LANGID_STR:
					if (pdev->pDesc->GetLangIDStrDescriptor != NULL) {
						pbuf = pdev->pDesc->GetLangIDStrDescriptor(pdev->dev_speed, &len);
					}
					else {
						USBD_CtlError(pdev, req);
						err++;
					}
					break;
				case USBD_IDX_MFC_STR:
					if (pdev->pDesc->GetManufacturerStrDescriptor != NULL) {
						pbuf = pdev->pDesc->GetManufacturerStrDescriptor(pdev->dev_speed, &len);
					}
					else {
						USBD_CtlError(pdev, req);
						err++;
					}
					break;
				case USBD_IDX_PRODUCT_STR:
					if (pdev->pDesc->GetProductStrDescriptor != NULL) {
						pbuf = pdev->pDesc->GetProductStrDescriptor(pdev->dev_speed, &len);
					}
					else {
						USBD_CtlError(pdev, req);
						err++;
					}
					break;
				case USBD_IDX_SERIAL_STR:
					if (pdev->pDesc->GetSerialStrDescriptor != NULL) {
						pbuf = pdev->pDesc->GetSerialStrDescriptor(pdev->dev_speed, &len);
					}
					else {
						USBD_CtlError(pdev, req);
						err++;
					}
					break;
				case USBD_IDX_CONFIG_STR:
					if (pdev->pDesc->GetConfigurationStrDescriptor != NULL) {
						pbuf = pdev->pDesc->GetConfigurationStrDescriptor(pdev->dev_speed, &len);
					}
					else {
						USBD_CtlError(pdev, req);
						err++;
					}
					break;
				case USBD_IDX_INTERFACE_STR:
					if (pdev->pDesc->GetInterfaceStrDescriptor != NULL) {
						pbuf = pdev->pDesc->GetInterfaceStrDescriptor(pdev->dev_speed, &len);
					}
					else {
						USBD_CtlError(pdev, req);
						err++;
					}
					break;
				default:
					break;
			}
			break;

		case USB_DESC_TYPE_DEVICE_QUALIFIER:
			if (pdev->dev_speed == USBD_SPEED_HIGH) { {
					pbuf = (uint8_t *)pdev->pClass[0]->GetDeviceQualifierDescriptor(&len);
				}
			}
			else {
				USBD_CtlError(pdev, req);
				err++;
			}
			break;
		case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
			if (pdev->dev_speed == USBD_SPEED_HIGH) { {
					pbuf = (uint8_t *)pdev->pClass[0]->GetOtherSpeedConfigDescriptor(&len);
				}
				pbuf[1] = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;
			}
			else {
				USBD_CtlError(pdev, req);
				err++;
			}
			break;
		default:
			USBD_CtlError(pdev, req);
			err++;
			break;
	}

	if (err != 0U) {
		return;
	}

	if (req->wLength != 0U) {
		if (len != 0U) {
			len = MIN(len, req->wLength);
			(void)USBD_CtlSendData(pdev, pbuf, len);
		}
		else
		{
			USBD_CtlError(pdev, req);
		}
	}
	else
	{
		(void)USBD_CtlSendStatus(pdev);
	}
}
static void USBD_SetAddress(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	uint8_t  dev_addr;
	if ((req->wIndex == 0U) && (req->wLength == 0U) && (req->wValue < 128U)) {
		dev_addr = (uint8_t)(req->wValue) & 0x7FU;
		if (pdev->dev_state == USBD_STATE_CONFIGURED) {
			USBD_CtlError(pdev, req);
		}
		else {
			pdev->dev_address = dev_addr;
			(void)USBD_LL_SetUSBAddress(pdev, dev_addr);
			(void)USBD_CtlSendStatus(pdev);

			if (dev_addr != 0U) {
				pdev->dev_state = USBD_STATE_ADDRESSED;
			}
			else {
				pdev->dev_state = USBD_STATE_DEFAULT;
			}
		}
	}
	else {
		USBD_CtlError(pdev, req);
	}
}
static USBD_StatusTypeDef USBD_SetConfig(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	USBD_StatusTypeDef ret = USBD_OK;
	static uint8_t cfgidx;
	cfgidx = (uint8_t)(req->wValue);
	if (cfgidx > USBD_MAX_NUM_CONFIGURATION) {
		USBD_CtlError(pdev, req);
		return USBD_FAIL;
	}

	switch (pdev->dev_state) {
		case USBD_STATE_ADDRESSED:
			if (cfgidx != 0U) {
				pdev->dev_config = cfgidx;
				ret = USBD_SetClassConfig(pdev, cfgidx);
				if (ret != USBD_OK) {
					USBD_CtlError(pdev, req);
					pdev->dev_state = USBD_STATE_ADDRESSED;
				}
				else {
					(void)USBD_CtlSendStatus(pdev);
					pdev->dev_state = USBD_STATE_CONFIGURED;
				}
			}
			else {
				(void)USBD_CtlSendStatus(pdev);
			}
			break;
		case USBD_STATE_CONFIGURED:
			if (cfgidx == 0U) {
				pdev->dev_state = USBD_STATE_ADDRESSED;
				pdev->dev_config = cfgidx;
				(void)USBD_ClrClassConfig(pdev, cfgidx);
				(void)USBD_CtlSendStatus(pdev);
			}
			else if (cfgidx != pdev->dev_config) {
				/* Clear old configuration */
				(void)USBD_ClrClassConfig(pdev, (uint8_t)pdev->dev_config);
				/* set new configuration */
				pdev->dev_config = cfgidx;
				ret = USBD_SetClassConfig(pdev, cfgidx);
				if (ret != USBD_OK){
					USBD_CtlError(pdev, req);
					(void)USBD_ClrClassConfig(pdev, (uint8_t)pdev->dev_config);
					pdev->dev_state = USBD_STATE_ADDRESSED;
				}
				else {
					(void)USBD_CtlSendStatus(pdev);
				}
			}
			else {
				(void)USBD_CtlSendStatus(pdev);
			}
			break;
		default:
			USBD_CtlError(pdev, req);
			(void)USBD_ClrClassConfig(pdev, cfgidx);
			ret = USBD_FAIL;
			break;
	}
	return ret;
}
static void USBD_GetConfig(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	if (req->wLength != 1U) {
		USBD_CtlError(pdev, req);
	}
	else {
		switch (pdev->dev_state) {
			case USBD_STATE_DEFAULT:
			case USBD_STATE_ADDRESSED:
				pdev->dev_default_config = 0U;
				(void)USBD_CtlSendData(pdev, (uint8_t *)&pdev->dev_default_config, 1U);
				break;
			case USBD_STATE_CONFIGURED:
				(void)USBD_CtlSendData(pdev, (uint8_t *)&pdev->dev_config, 1U);
				break;
			default:
				USBD_CtlError(pdev, req);
				break;
		}
	}
}
static void USBD_GetStatus(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	switch (pdev->dev_state) {
		case USBD_STATE_DEFAULT:
		case USBD_STATE_ADDRESSED:
		case USBD_STATE_CONFIGURED:
			if (req->wLength != 0x2U) {
				USBD_CtlError(pdev, req);
				break;
			}

#if (USBD_SELF_POWERED == 1U)
			pdev->dev_config_status = USB_CONFIG_SELF_POWERED;
#else
			pdev->dev_config_status = 0U;
#endif /* USBD_SELF_POWERED */

			if (pdev->dev_remote_wakeup != 0U) {
				pdev->dev_config_status |= USB_CONFIG_REMOTE_WAKEUP;
			}
			(void)USBD_CtlSendData(pdev, (uint8_t *)&pdev->dev_config_status, 2U);
			break;

		default:
			USBD_CtlError(pdev, req);
			break;
	}
}
static void USBD_SetFeature(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	if (req->wValue == USB_FEATURE_REMOTE_WAKEUP) {
		pdev->dev_remote_wakeup = 1U;
		(void)USBD_CtlSendStatus(pdev);
	}
	else if (req->wValue == USB_FEATURE_TEST_MODE) {
		pdev->dev_test_mode = (uint8_t)(req->wIndex >> 8);
		(void)USBD_CtlSendStatus(pdev);
	}
	else {
		USBD_CtlError(pdev, req);
	}
}
static void USBD_ClrFeature(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	switch (pdev->dev_state) {
		case USBD_STATE_DEFAULT:
		case USBD_STATE_ADDRESSED:
		case USBD_STATE_CONFIGURED:
			if (req->wValue == USB_FEATURE_REMOTE_WAKEUP) {
				pdev->dev_remote_wakeup = 0U;
				(void)USBD_CtlSendStatus(pdev);
			}
			break;
		default:
			USBD_CtlError(pdev, req);
			break;
	}
}


// L4 ========================================= /
USBD_StatusTypeDef USBD_CtlContinueRx(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint32_t len) {
	(void)USBD_LL_PrepareReceive(pdev, 0U, pbuf, len);
	return USBD_OK;
}
uint8_t USBD_CoreFindIF(USBD_HandleTypeDef *pdev, uint8_t index) {
	UNUSED(pdev);
	UNUSED(index);
	return 0x00U;
}
uint8_t USBD_CoreFindEP(USBD_HandleTypeDef *pdev, uint8_t index) {
	UNUSED(pdev);
	UNUSED(index);
	return 0x00U;
}
USBD_StatusTypeDef USBD_CtlSendStatus(USBD_HandleTypeDef *pdev) {
	/* Set EP0 State */
	pdev->ep0_state = USBD_EP0_STATUS_IN;
	/* Start the transfer */
	(void)USBD_LL_Transmit(pdev, 0x00U, NULL, 0U);
	return USBD_OK;
}
USBD_StatusTypeDef USBD_CtlContinueSendData(USBD_HandleTypeDef *pdev,uint8_t *pbuf, uint32_t len) {
	/* Start the next transfer */
	(void)USBD_LL_Transmit(pdev, 0x00U, pbuf, len);
	return USBD_OK;
}
USBD_StatusTypeDef USBD_CtlReceiveStatus(USBD_HandleTypeDef *pdev) {
	/* Set EP0 State */
	pdev->ep0_state = USBD_EP0_STATUS_OUT;
	/* Start the transfer */
	(void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
	return USBD_OK;
}
void USBD_ParseSetupRequest(USBD_SetupReqTypedef *req, uint8_t *pdata) {
	uint8_t *pbuff = pdata;
	req->bmRequest = *(uint8_t *)(pbuff);
	pbuff++;
	req->bRequest = *(uint8_t *)(pbuff);
	pbuff++;
	req->wValue = SWAPBYTE(pbuff);
	pbuff++;
	pbuff++;
	req->wIndex = SWAPBYTE(pbuff);
	pbuff++;
	pbuff++;
	req->wLength = SWAPBYTE(pbuff);
}
USBD_StatusTypeDef USBD_StdDevReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	USBD_StatusTypeDef ret = USBD_OK;
	switch (req->bmRequest & USB_REQ_TYPE_MASK) {
		case USB_REQ_TYPE_CLASS:
		case USB_REQ_TYPE_VENDOR:
			ret = (USBD_StatusTypeDef)pdev->pClass[pdev->classId]->Setup(pdev, req);
			break;
		case USB_REQ_TYPE_STANDARD:
			switch (req->bRequest) {
				case USB_REQ_GET_DESCRIPTOR:
					USBD_GetDescriptor(pdev, req);
					break;
				case USB_REQ_SET_ADDRESS:
					USBD_SetAddress(pdev, req);
					break;
				case USB_REQ_SET_CONFIGURATION:
					ret = USBD_SetConfig(pdev, req);
					break;
				case USB_REQ_GET_CONFIGURATION:
					USBD_GetConfig(pdev, req);
					break;
				case USB_REQ_GET_STATUS:
					USBD_GetStatus(pdev, req);
					break;
				case USB_REQ_SET_FEATURE:
					USBD_SetFeature(pdev, req);
					break;
				case USB_REQ_CLEAR_FEATURE:
					USBD_ClrFeature(pdev, req);
					break;
				default:
					USBD_CtlError(pdev, req);
					break;
			}
			break;
		default:
			USBD_CtlError(pdev, req);
			break;
	}
	return ret;
}
USBD_StatusTypeDef USBD_StdItfReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	USBD_StatusTypeDef ret = USBD_OK;
	uint8_t idx;
	switch (req->bmRequest & USB_REQ_TYPE_MASK) {
		case USB_REQ_TYPE_CLASS:
		case USB_REQ_TYPE_VENDOR:
		case USB_REQ_TYPE_STANDARD:
			switch (pdev->dev_state) {
				case USBD_STATE_DEFAULT:
				case USBD_STATE_ADDRESSED:
				case USBD_STATE_CONFIGURED:
					if (LOBYTE(req->wIndex) <= USBD_MAX_NUM_INTERFACES) {
						/* Get the class index relative to this interface */
						idx = USBD_CoreFindIF(pdev, LOBYTE(req->wIndex));
						if (((uint8_t)idx != 0xFFU) && (idx < USBD_MAX_SUPPORTED_CLASS)) {
							/* Call the class data out function to manage the request */
							if (pdev->pClass[idx]->Setup != NULL) {
								pdev->classId = idx;
								ret = (USBD_StatusTypeDef)(pdev->pClass[idx]->Setup(pdev, req));
							}
							else {
								/* should never reach this condition */
								ret = USBD_FAIL;
							}
						}
						else {
							/* No relative interface found */
							ret = USBD_FAIL;
						}
						if ((req->wLength == 0U) && (ret == USBD_OK)) {
							(void)USBD_CtlSendStatus(pdev);
						}
					}
					else {
						USBD_CtlError(pdev, req);
					}
					break;
				default:
					USBD_CtlError(pdev, req);
					break;
			}
			break;
		default:
			USBD_CtlError(pdev, req);
			break;
	}
	return ret;
}
USBD_StatusTypeDef USBD_StdEPReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	USBD_EndpointTypeDef *pep;
	uint8_t ep_addr;
	uint8_t idx;
	USBD_StatusTypeDef ret = USBD_OK;
	ep_addr = LOBYTE(req->wIndex);
	switch (req->bmRequest & USB_REQ_TYPE_MASK) {
		case USB_REQ_TYPE_CLASS:
		case USB_REQ_TYPE_VENDOR:
			/* Get the class index relative to this endpoint */
			idx = USBD_CoreFindEP(pdev, ep_addr);
			if (((uint8_t)idx != 0xFFU) && (idx < USBD_MAX_SUPPORTED_CLASS)){
				pdev->classId = idx;
				/* Call the class data out function to manage the request */
				if (pdev->pClass[idx]->Setup != NULL) {
					ret = (USBD_StatusTypeDef)pdev->pClass[idx]->Setup(pdev, req);
				}
			}
			break;
		case USB_REQ_TYPE_STANDARD:
			switch (req->bRequest) {
				case USB_REQ_SET_FEATURE:
					switch (pdev->dev_state) {
						case USBD_STATE_ADDRESSED:
							if ((ep_addr != 0x00U) && (ep_addr != 0x80U)) {
								(void)USBD_LL_StallEP(pdev, ep_addr);
								(void)USBD_LL_StallEP(pdev, 0x80U);
							}
							else {
								USBD_CtlError(pdev, req);
							}
							break;
						case USBD_STATE_CONFIGURED:
							if (req->wValue == USB_FEATURE_EP_HALT) {
								if ((ep_addr != 0x00U) && (ep_addr != 0x80U) && (req->wLength == 0x00U)) {
									(void)USBD_LL_StallEP(pdev, ep_addr);
								}
							}
							(void)USBD_CtlSendStatus(pdev);
							break;
						default:
							USBD_CtlError(pdev, req);
							break;
					}
					break;
				case USB_REQ_CLEAR_FEATURE:
					switch (pdev->dev_state) {
						case USBD_STATE_ADDRESSED:
							if ((ep_addr != 0x00U) && (ep_addr != 0x80U)) {
								(void)USBD_LL_StallEP(pdev, ep_addr);
								(void)USBD_LL_StallEP(pdev, 0x80U);
							}
							else {
								USBD_CtlError(pdev, req);
							}
							break;
						case USBD_STATE_CONFIGURED:
							if (req->wValue == USB_FEATURE_EP_HALT) {
								if ((ep_addr & 0x7FU) != 0x00U) {
									(void)USBD_LL_ClearStallEP(pdev, ep_addr);
								}
								(void)USBD_CtlSendStatus(pdev);
								/* Get the class index relative to this interface */
								idx = USBD_CoreFindEP(pdev, ep_addr);
								if (((uint8_t)idx != 0xFFU) && (idx < USBD_MAX_SUPPORTED_CLASS)) {
									pdev->classId = idx;
									/* Call the class data out function to manage the request */
									if (pdev->pClass[idx]->Setup != NULL) {
										ret = (USBD_StatusTypeDef)(pdev->pClass[idx]->Setup(pdev, req));
									}
								}
							}
							break;
						default:
							USBD_CtlError(pdev, req);
							break;
					}
					break;
				case USB_REQ_GET_STATUS:
					switch (pdev->dev_state) {
						case USBD_STATE_ADDRESSED:
							if ((ep_addr != 0x00U) && (ep_addr != 0x80U)) {
								USBD_CtlError(pdev, req);
								break;
							}
							pep = ((ep_addr & 0x80U) == 0x80U) ? &pdev->ep_in[ep_addr & 0x7FU] : &pdev->ep_out[ep_addr & 0x7FU];
							pep->status = 0x0000U;
							(void)USBD_CtlSendData(pdev, (uint8_t *)&pep->status, 2U);
							break;
						case USBD_STATE_CONFIGURED:
							if ((ep_addr & 0x80U) == 0x80U) {
								if (pdev->ep_in[ep_addr & 0xFU].is_used == 0U) {
									USBD_CtlError(pdev, req);
									break;
								}
							}
							else {
								if (pdev->ep_out[ep_addr & 0xFU].is_used == 0U) {
									USBD_CtlError(pdev, req);
									break;
								}
							}
							pep = ((ep_addr & 0x80U) == 0x80U) ? &pdev->ep_in[ep_addr & 0x7FU] : &pdev->ep_out[ep_addr & 0x7FU];
							if ((ep_addr == 0x00U) || (ep_addr == 0x80U)) {
								pep->status = 0x0000U;
							}
							else if (USBD_LL_IsStallEP(pdev, ep_addr) != 0U) {
								pep->status = 0x0001U;
							}
							else {
								pep->status = 0x0000U;
							}
							(void)USBD_CtlSendData(pdev, (uint8_t *)&pep->status, 2U);
							break;
						default:
							USBD_CtlError(pdev, req);
							break;
					}
					break;
				default:
					USBD_CtlError(pdev, req);
					break;
			}
			break;
		default:
			USBD_CtlError(pdev, req);
			break;
	}
	return ret;
}


// L3 ========================================= /
HAL_StatusTypeDef USB_EP0_OutStart(const USB_OTG_GlobalTypeDef *USBx, uint8_t dma, const uint8_t *psetup) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t gSNPSiD = *(__IO const uint32_t *)(&USBx->CID + 0x1U);

	if (gSNPSiD > USB_OTG_CORE_ID_300A) {
		if ((USBx_OUTEP(0U)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) {
			return HAL_OK;
		}
	}

	USBx_OUTEP(0U)->DOEPTSIZ = 0U;
	USBx_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
	USBx_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
	USBx_OUTEP(0U)->DOEPTSIZ |=  USB_OTG_DOEPTSIZ_STUPCNT;

	if (dma == 1U) {
		USBx_OUTEP(0U)->DOEPDMA = (uint32_t)psetup;
		/* EP enable */
		USBx_OUTEP(0U)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP;
	}

	return HAL_OK;
}
USBD_StatusTypeDef USBD_LL_DataOutStage(USBD_HandleTypeDef *pdev, uint8_t epnum, uint8_t *pdata) {
	USBD_EndpointTypeDef *pep;
	USBD_StatusTypeDef ret = USBD_OK;
	uint8_t idx;
	if (epnum == 0U) {
		pep = &pdev->ep_out[0];
		if (pdev->ep0_state == USBD_EP0_DATA_OUT) {
			if (pep->rem_length > pep->maxpacket) {
				pep->rem_length -= pep->maxpacket;
				(void)USBD_CtlContinueRx(pdev, pdata, MIN(pep->rem_length, pep->maxpacket));
			}
			else {
				/* Find the class ID relative to the current request */
				switch (pdev->request.bmRequest & 0x1FU) {
					case USB_REQ_RECIPIENT_DEVICE:
						/* Device requests must be managed by the first instantiated class
						   (or duplicated by all classes for simplicity) */
						idx = 0U;
						break;
					case USB_REQ_RECIPIENT_INTERFACE:
						idx = USBD_CoreFindIF(pdev, LOBYTE(pdev->request.wIndex));
						break;
					case USB_REQ_RECIPIENT_ENDPOINT:
						idx = USBD_CoreFindEP(pdev, LOBYTE(pdev->request.wIndex));
						break;
					default:
						/* Back to the first class in case of doubt */
						idx = 0U;
						break;
				}
				if (idx < USBD_MAX_SUPPORTED_CLASS) {
					/* Setup the class ID and route the request to the relative class function */
					if (pdev->dev_state == USBD_STATE_CONFIGURED) {
						if (pdev->pClass[idx]->EP0_RxReady != NULL) {
							pdev->classId = idx;
							pdev->pClass[idx]->EP0_RxReady(pdev);
						}
					}
				}

				(void)USBD_CtlSendStatus(pdev);
			}
		}
	}
	else {
		/* Get the class index relative to this interface */
		idx = USBD_CoreFindEP(pdev, (epnum & 0x7FU));
		if (((uint16_t)idx != 0xFFU) && (idx < USBD_MAX_SUPPORTED_CLASS)) {
			/* Call the class data out function to manage the request */
			if (pdev->dev_state == USBD_STATE_CONFIGURED) {
				if (pdev->pClass[idx]->DataOut != NULL) {
					pdev->classId = idx;
					ret = (USBD_StatusTypeDef)pdev->pClass[idx]->DataOut(pdev, epnum);
				}
			}
			if (ret != USBD_OK) {
				return ret;
			}
		}
	}

	return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_DataInStage(USBD_HandleTypeDef *pdev, uint8_t epnum, uint8_t *pdata) {
	USBD_EndpointTypeDef *pep;
	USBD_StatusTypeDef ret;
	uint8_t idx;

	if (epnum == 0U) {
		pep = &pdev->ep_in[0];

		if (pdev->ep0_state == USBD_EP0_DATA_IN) {
			if (pep->rem_length > pep->maxpacket) {
				pep->rem_length -= pep->maxpacket;
				(void)USBD_CtlContinueSendData(pdev, pdata, pep->rem_length);
				/* Prepare endpoint for premature end of transfer */
				(void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
			}
			else {
				/* last packet is MPS multiple, so send ZLP packet */
				if ((pep->maxpacket == pep->rem_length) &&
					(pep->total_length >= pep->maxpacket) &&
					(pep->total_length < pdev->ep0_data_len)) {
					(void)USBD_CtlContinueSendData(pdev, NULL, 0U);
					pdev->ep0_data_len = 0U;

					/* Prepare endpoint for premature end of transfer */
					(void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
				}
				else {
					if (pdev->dev_state == USBD_STATE_CONFIGURED) {
						if (pdev->pClass[0]->EP0_TxSent != NULL) {
							pdev->classId = 0U;
							pdev->pClass[0]->EP0_TxSent(pdev);
						}
					}
					(void)USBD_LL_StallEP(pdev, 0x80U);
					(void)USBD_CtlReceiveStatus(pdev);
				}
			}
		}
	}
	else {
		/* Get the class index relative to this interface */
		idx = USBD_CoreFindEP(pdev, ((uint8_t)epnum | 0x80U));
		if (((uint16_t)idx != 0xFFU) && (idx < USBD_MAX_SUPPORTED_CLASS)) {
			/* Call the class data out function to manage the request */
			if (pdev->dev_state == USBD_STATE_CONFIGURED) {
				if (pdev->pClass[idx]->DataIn != NULL) {
					pdev->classId = idx;
					ret = (USBD_StatusTypeDef)pdev->pClass[idx]->DataIn(pdev, epnum);

					if (ret != USBD_OK) {
						return ret;
					}
				}
			}
		}
	}

	return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_SetupStage(USBD_HandleTypeDef *pdev, uint8_t *psetup) {
	USBD_StatusTypeDef ret;
	USBD_ParseSetupRequest(&pdev->request, psetup);
	pdev->ep0_state = USBD_EP0_SETUP;
	pdev->ep0_data_len = pdev->request.wLength;
	switch (pdev->request.bmRequest & 0x1FU) {
		case USB_REQ_RECIPIENT_DEVICE:
			ret = USBD_StdDevReq(pdev, &pdev->request);
			break;
		case USB_REQ_RECIPIENT_INTERFACE:
			ret = USBD_StdItfReq(pdev, &pdev->request);
			break;
		case USB_REQ_RECIPIENT_ENDPOINT:
			ret = USBD_StdEPReq(pdev, &pdev->request);
			break;
		default:
			ret = USBD_LL_StallEP(pdev, (pdev->request.bmRequest & 0x80U));
			break;
	}

	return ret;
}
USBD_StatusTypeDef USBD_LL_IsoOUTIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	if (pdev->pClass[pdev->classId] == NULL) {
		return USBD_FAIL;
	}
	if (pdev->dev_state == USBD_STATE_CONFIGURED) {
		if (pdev->pClass[pdev->classId]->IsoOUTIncomplete != NULL) {
			(void)pdev->pClass[pdev->classId]->IsoOUTIncomplete(pdev, epnum);
		}
	}

	return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	if (pdev->pClass[pdev->classId] == NULL) {
		return USBD_FAIL;
	}
	if (pdev->dev_state == USBD_STATE_CONFIGURED) {
		if (pdev->pClass[pdev->classId]->IsoINIncomplete != NULL) {
			(void)pdev->pClass[pdev->classId]->IsoINIncomplete(pdev, epnum);
		}
	}

	return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_SetSpeed(USBD_HandleTypeDef *pdev, USBD_SpeedTypeDef speed) {
	pdev->dev_speed = speed;
	return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_Reset(USBD_HandleTypeDef *pdev) {
	USBD_StatusTypeDef ret = USBD_OK;
	/* Upon Reset call user call back */
	pdev->dev_state = USBD_STATE_DEFAULT;
	pdev->ep0_state = USBD_EP0_IDLE;
	pdev->dev_config = 0U;
	pdev->dev_remote_wakeup = 0U;
	pdev->dev_test_mode = 0U;

	if (pdev->pClass[0] != NULL) {
		if (pdev->pClass[0]->DeInit != NULL) {
			if (pdev->pClass[0]->DeInit(pdev, (uint8_t)pdev->dev_config) != USBD_OK) {
				ret = USBD_FAIL;
			}
		}
	}

	/* Open EP0 OUT */
	(void)USBD_LL_OpenEP(pdev, 0x00U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
	pdev->ep_out[0x00U & 0xFU].is_used = 1U;
	pdev->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;

	/* Open EP0 IN */
	(void)USBD_LL_OpenEP(pdev, 0x80U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
	pdev->ep_in[0x80U & 0xFU].is_used = 1U;
	pdev->ep_in[0].maxpacket = USB_MAX_EP0_SIZE;
	return ret;
}
HAL_StatusTypeDef USB_WritePacket(const USB_OTG_GlobalTypeDef *USBx, uint8_t *src, uint8_t ch_ep_num, uint16_t len, uint8_t dma) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint8_t *pSrc = src;
	uint32_t count32b;
	uint32_t i;

	if (dma == 0U) {
		count32b = ((uint32_t)len + 3U) / 4U;
		for (i = 0U; i < count32b; i++) {
			USBx_DFIFO((uint32_t)ch_ep_num) = __UNALIGNED_UINT32_READ(pSrc);
			pSrc++;
			pSrc++;
			pSrc++;
			pSrc++;
		}
	}

	return HAL_OK;
}
USBD_StatusTypeDef USBD_LL_Resume(USBD_HandleTypeDef *pdev) {
	if (pdev->dev_state == USBD_STATE_SUSPENDED) {
		pdev->dev_state = pdev->dev_old_state;
	}

	return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_DevConnected(USBD_HandleTypeDef *pdev) {
	/* Prevent unused argument compilation warning */
	UNUSED(pdev);
	return USBD_OK;
}
HAL_StatusTypeDef USB_EPStopXfer(const USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep) {
	__IO uint32_t count = 0U;
	HAL_StatusTypeDef ret = HAL_OK;
	uint32_t USBx_BASE = (uint32_t)USBx;
	/* IN endpoint */
	if (ep->is_in == 1U) {
		/* EP enable, IN data in FIFO */
		if (((USBx_INEP(ep->num)->DIEPCTL) & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA) {
			USBx_INEP(ep->num)->DIEPCTL |= (USB_OTG_DIEPCTL_SNAK);
			USBx_INEP(ep->num)->DIEPCTL |= (USB_OTG_DIEPCTL_EPDIS);

			do {
				count++;
				if (count > 10000U) {
					ret = HAL_ERROR;
					break;
				}
			} while (((USBx_INEP(ep->num)->DIEPCTL) & USB_OTG_DIEPCTL_EPENA) ==  USB_OTG_DIEPCTL_EPENA);
		}
	}
	else /* OUT endpoint */ {
		if (((USBx_OUTEP(ep->num)->DOEPCTL) & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) {
			USBx_OUTEP(ep->num)->DOEPCTL |= (USB_OTG_DOEPCTL_SNAK);
			USBx_OUTEP(ep->num)->DOEPCTL |= (USB_OTG_DOEPCTL_EPDIS);

			do {
				count++;
				if (count > 10000U) {
					ret = HAL_ERROR;
					break;
				}
			} while (((USBx_OUTEP(ep->num)->DOEPCTL) & USB_OTG_DOEPCTL_EPENA) ==  USB_OTG_DOEPCTL_EPENA);
		}
	}

	return ret;
}


// L2 ========================================= /
#define __HAL_PCD_CLEAR_FLAG(__HANDLE__, __INTERRUPT__)    (((__HANDLE__)->Instance->GINTSTS) &= (__INTERRUPT__))
#define CLEAR_IN_EP_INTR(__EPNUM__, __INTERRUPT__)          (USBx_INEP(__EPNUM__)->DIEPINT = (__INTERRUPT__))
#define CLEAR_OUT_EP_INTR(__EPNUM__, __INTERRUPT__)         (USBx_OUTEP(__EPNUM__)->DOEPINT = (__INTERRUPT__))
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
	USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
}
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	USBD_LL_IsoINIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) {
	USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);
}
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) {
	USBD_LL_DevConnected((USBD_HandleTypeDef*)hpcd->pData);
}
uint32_t USB_ReadDevOutEPInterrupt(const USB_OTG_GlobalTypeDef *USBx, uint8_t epnum) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t tmpreg;

	tmpreg  = USBx_OUTEP((uint32_t)epnum)->DOEPINT;
	tmpreg &= USBx_DEVICE->DOEPMSK;

	return tmpreg;
}
uint32_t USB_ReadDevInEPInterrupt(const USB_OTG_GlobalTypeDef *USBx, uint8_t epnum) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t tmpreg;
	uint32_t msk;
	uint32_t emp;

	msk = USBx_DEVICE->DIEPMSK;
	emp = USBx_DEVICE->DIEPEMPMSK;
	msk |= ((emp >> (epnum & EP_ADDR_MSK)) & 0x1U) << 7;
	tmpreg = USBx_INEP((uint32_t)epnum)->DIEPINT & msk;

	return tmpreg;
}
static HAL_StatusTypeDef PCD_EP_OutXfrComplete_int(PCD_HandleTypeDef *hpcd, uint32_t epnum) {
	USB_OTG_EPTypeDef *ep;
	const USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t gSNPSiD = *(__IO const uint32_t *)(&USBx->CID + 0x1U);
	uint32_t DoepintReg = USBx_OUTEP(epnum)->DOEPINT;

	if (hpcd->Init.dma_enable == 1U) {
		if ((DoepintReg & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP) /* Class C */ {
			/* StupPktRcvd = 1 this is a setup packet */
			if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
				((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX)) {
				CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
			}
		}
		else if ((DoepintReg & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR) /* Class E */ {
			CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
		}
		else if ((DoepintReg & (USB_OTG_DOEPINT_STUP | USB_OTG_DOEPINT_OTEPSPR)) == 0U) {
			/* StupPktRcvd = 1 this is a setup packet */
			if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
				((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX)) {
				CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
			}
			else {
				ep = &hpcd->OUT_ep[epnum];
				/* out data packet received over EP */
				ep->xfer_count = ep->xfer_size - (USBx_OUTEP(epnum)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ);
				if (epnum == 0U) {
					if (ep->xfer_len == 0U) {
						/* this is ZLP, so prepare EP0 for next setup */
						(void)USB_EP0_OutStart(hpcd->Instance, 1U, (uint8_t *)hpcd->Setup);
					}
					else {
						ep->xfer_buff += ep->xfer_count;
					}
				}

				HAL_PCD_DataOutStageCallback(hpcd, (uint8_t)epnum);
			}
		}
		else {
			/* ... */
		}
	}
	else {
		if (gSNPSiD == USB_OTG_CORE_ID_310A) {
			/* StupPktRcvd = 1 this is a setup packet */
			if ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX) {
				CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
			}
			else {
				if ((DoepintReg & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR) {
					CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
				}
				HAL_PCD_DataOutStageCallback(hpcd, (uint8_t)epnum);
			}
		}
		else {
			if ((epnum == 0U) && (hpcd->OUT_ep[epnum].xfer_len == 0U)) {
				/* this is ZLP, so prepare EP0 for next setup */
				(void)USB_EP0_OutStart(hpcd->Instance, 0U, (uint8_t *)hpcd->Setup);
			}
			HAL_PCD_DataOutStageCallback(hpcd, (uint8_t)epnum);
		}
	}
	return HAL_OK;
}
static HAL_StatusTypeDef PCD_EP_OutSetupPacket_int(PCD_HandleTypeDef *hpcd, uint32_t epnum) {
	const USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t gSNPSiD = *(__IO const uint32_t *)(&USBx->CID + 0x1U);
	uint32_t DoepintReg = USBx_OUTEP(epnum)->DOEPINT;

	if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
		((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX)) {
		CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
	}

	/* Inform the upper layer that a setup packet is available */
	HAL_PCD_SetupStageCallback(hpcd);

	if ((gSNPSiD > USB_OTG_CORE_ID_300A) && (hpcd->Init.dma_enable == 1U)) {
		(void)USB_EP0_OutStart(hpcd->Instance, 1U, (uint8_t *)hpcd->Setup);
	}

	return HAL_OK;
}
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
	USBD_SpeedTypeDef speed = USBD_SPEED_FULL;
	if ( hpcd->Init.speed != PCD_SPEED_FULL) {
		Error_Handler();
	}
	/* Set Speed. */
	USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);

	/* Reset Device. */
	USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}
static HAL_StatusTypeDef PCD_WriteEmptyTxFifo(PCD_HandleTypeDef *hpcd, uint32_t epnum) {
	USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
	uint32_t USBx_BASE = (uint32_t)USBx;
	USB_OTG_EPTypeDef *ep;
	uint32_t len;
	uint32_t len32b;
	uint32_t fifoemptymsk;
	ep = &hpcd->IN_ep[epnum];
	if (ep->xfer_count > ep->xfer_len) {
		return HAL_ERROR;
	}
	len = ep->xfer_len - ep->xfer_count;
	if (len > ep->maxpacket) {
		len = ep->maxpacket;
	}
	len32b = (len + 3U) / 4U;
	while (((USBx_INEP(epnum)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) >= len32b) &&
		   (ep->xfer_count < ep->xfer_len) && (ep->xfer_len != 0U)) {
		/* Write the FIFO */
		len = ep->xfer_len - ep->xfer_count;
		if (len > ep->maxpacket) {
			len = ep->maxpacket;
		}
		len32b = (len + 3U) / 4U;
		(void)USB_WritePacket(USBx, ep->xfer_buff, (uint8_t)epnum, (uint16_t)len,
							   (uint8_t)hpcd->Init.dma_enable);
		ep->xfer_buff  += len;
		ep->xfer_count += len;
	}

	if (ep->xfer_len <= ep->xfer_count) {
		fifoemptymsk = (uint32_t)(0x1UL << (epnum & EP_ADDR_MSK));
		USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;
	}

	return HAL_OK;
}
/*__weak*/ void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg) {
	UNUSED(hpcd);
	UNUSED(msg);
}
HAL_StatusTypeDef USB_ActivateSetup(const USB_OTG_GlobalTypeDef *USBx) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	/* Set the MPS of the IN EP0 to 64 bytes */
	USBx_INEP(0U)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
	USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;
	return HAL_OK;
}
uint8_t USB_GetDevSpeed(const USB_OTG_GlobalTypeDef *USBx) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint8_t speed;
	uint32_t DevEnumSpeed = USBx_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD;

	if (DevEnumSpeed == DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ) {
		speed = USBD_HS_SPEED;
	}
	else if ((DevEnumSpeed == DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ) ||
			 (DevEnumSpeed == DSTS_ENUMSPD_FS_PHY_48MHZ)) {
		speed = USBD_FS_SPEED;
	}
	else {
		speed = 0xFU;
	}

	return speed;
}
HAL_StatusTypeDef USB_SetTurnaroundTime(USB_OTG_GlobalTypeDef *USBx, uint32_t hclk, uint8_t speed) {
	uint32_t UsbTrd;
	/* The USBTRD is configured according to the tables below, depending on AHB frequency
	used by application. In the low AHB frequency range it is used to stretch enough the USB response
	time to IN tokens, the USB turnaround time, so to compensate for the longer AHB read access
	latency to the Data FIFO */
	if (speed == USBD_FS_SPEED) {
		if ((hclk >= 14200000U) && (hclk < 15000000U)) {
			/* hclk Clock Range between 14.2-15 MHz */
			UsbTrd = 0xFU;
		}
		else if ((hclk >= 15000000U) && (hclk < 16000000U)) {
			/* hclk Clock Range between 15-16 MHz */
			UsbTrd = 0xEU;
		}
		else if ((hclk >= 16000000U) && (hclk < 17200000U)) {
			/* hclk Clock Range between 16-17.2 MHz */
			UsbTrd = 0xDU;
		}
		else if ((hclk >= 17200000U) && (hclk < 18500000U)) {
			/* hclk Clock Range between 17.2-18.5 MHz */
			UsbTrd = 0xCU;
		}
		else if ((hclk >= 18500000U) && (hclk < 20000000U)) {
			/* hclk Clock Range between 18.5-20 MHz */
			UsbTrd = 0xBU;
		}
		else if ((hclk >= 20000000U) && (hclk < 21800000U)) {
			/* hclk Clock Range between 20-21.8 MHz */
			UsbTrd = 0xAU;
		}
		else if ((hclk >= 21800000U) && (hclk < 24000000U)) {
			/* hclk Clock Range between 21.8-24 MHz */
			UsbTrd = 0x9U;
		}
		else if ((hclk >= 24000000U) && (hclk < 27700000U)) {
			/* hclk Clock Range between 24-27.7 MHz */
			UsbTrd = 0x8U;
		}
		else if ((hclk >= 27700000U) && (hclk < 32000000U)) {
			/* hclk Clock Range between 27.7-32 MHz */
			UsbTrd = 0x7U;
		}
		else /* if(hclk >= 32000000) */ {
			/* hclk Clock Range between 32-200 MHz */
			UsbTrd = 0x6U;
		}
	}
	else if (speed == USBD_HS_SPEED) {
		UsbTrd = USBD_HS_TRDT_VALUE;
	}
	else {
		UsbTrd = USBD_DEFAULT_TRDT_VALUE;
	}

	USBx->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
	USBx->GUSBCFG |= (uint32_t)((UsbTrd << 10) & USB_OTG_GUSBCFG_TRDT);

	return HAL_OK;
}
uint32_t HAL_RCC_GetHCLKFreq(void) {
	return SYS_clock_frequency;  // TODO: valid?????
}

extern void flush_RX_FIFO(USB_OTG_GlobalTypeDef* usb);
extern void flush_TX_FIFO(USB_OTG_GlobalTypeDef* usb, uint8_t ep);
extern void flush_TX_FIFOS(USB_OTG_GlobalTypeDef* usb);


// L1.2 ========================================= /


// L1.1 ========================================= /
static inline void USB_OTG_IRQ(USBD_HandleTypeDef* pdev) {
	// TODO: redo handle and make it possible to pass usb ptr
	uint32_t tmp = USB_OTG_FS->GOTGINT;
	if (tmp & USB_OTG_GOTGINT_SEDET) {
		pdev->dev_state = USBD_STATE_DEFAULT;
		if (pdev->pClass[0] == NULL) { return; }
		pdev->pClass[0]->DeInit(pdev, (uint8_t)pdev->dev_config);
	}
	USB_OTG_FS->GOTGINT |= tmp;
}
static inline void USB_SOF_IRQ(USBD_HandleTypeDef* pdev) {
	// TODO: redo handle and make it possible to pass usb ptr
	if (pdev->dev_state == USBD_STATE_CONFIGURED) {
		if (pdev->pClass[0] == NULL)		{ return; }
		if (pdev->pClass[0]->SOF == NULL)	{ return; }
		(void)pdev->pClass[0]->SOF(pdev);
	}
	USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_SOF;
}
static inline void USB_receive_packet_IRQ(PCD_HandleTypeDef* hpcd) {
	// TODO: redo handle and make it possible to pass usb ptr
	USB_OTG_GlobalTypeDef*	usb = hpcd->Instance;
	usb->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
	uint32_t				tmp = usb->GRXSTSP;
	GRXSTS_t				status = *((GRXSTS_t*)&tmp);
	USB_OTG_EPTypeDef*		ep = &hpcd->OUT_ep[status.EPNUM];  // TODO: redo handle
	volatile uint32_t* FIFO = (void*)(((uint32_t)USB_OTG_FS) + USB_OTG_FIFO_BASE);

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
	uint8_t					ep_num;
	PCD_EPTypeDef*			ep;
	USB_OTG_GlobalTypeDef*	usb = hpcd->Instance;
	usb->GINTMSK &= ~USB_OTG_GINTMSK_GONAKEFFM;
	for (ep_num = 1U; ep_num < hpcd->Init.dev_endpoints; ep_num++) {
		ep = &hpcd->OUT_ep[ep_num];
		if (ep->is_iso_incomplete == 1U) { continue; }
		USB_EPStopXfer(usb, ep);
	}
}
static inline void USB_suspend_IRQ(PCD_HandleTypeDef* hpcd) {
	// TODO: redo handle and make it possible to pass usb ptr
	USB_OTG_GlobalTypeDef* usb = hpcd->Instance;
	uint32_t USBx_BASE = (uint32_t)usb;
	if ((USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS) {
		__IO uint32_t* PCGCCTL =	(void*)(USBx_BASE + USB_OTG_PCGCCTL_BASE);
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
// TODO<
static inline void USB_reset_IRQ(PCD_HandleTypeDef* hpcd) {
	uint32_t USBx_BASE = (uint32_t)hpcd->Instance;
	uint32_t i;

	USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
	flush_TX_FIFOS(hpcd->Instance);

	for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
	{
		USBx_INEP(i)->DIEPINT = 0xFB7FU;
		USBx_INEP(i)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
		USBx_OUTEP(i)->DOEPINT = 0xFB7FU;
		USBx_OUTEP(i)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
		USBx_OUTEP(i)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
	}
	USBx_DEVICE->DAINTMSK |= 0x10001U;

	if (hpcd->Init.use_dedicated_ep1 != 0U)
	{
		USBx_DEVICE->DOUTEP1MSK |= USB_OTG_DOEPMSK_STUPM |
								   USB_OTG_DOEPMSK_XFRCM |
								   USB_OTG_DOEPMSK_EPDM;

		USBx_DEVICE->DINEP1MSK |= USB_OTG_DIEPMSK_TOM |
								  USB_OTG_DIEPMSK_XFRCM |
								  USB_OTG_DIEPMSK_EPDM;
	}
	else
	{
		USBx_DEVICE->DOEPMSK |= USB_OTG_DOEPMSK_STUPM |
								USB_OTG_DOEPMSK_XFRCM |
								USB_OTG_DOEPMSK_EPDM |
								USB_OTG_DOEPMSK_OTEPSPRM |
								USB_OTG_DOEPMSK_NAKM;

		USBx_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_TOM |
								USB_OTG_DIEPMSK_XFRCM |
								USB_OTG_DIEPMSK_EPDM;
	}

	/* Set Default Address to 0 */
	USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

	/* setup EP0 to receive SETUP packets */
	(void)USB_EP0_OutStart(hpcd->Instance, (uint8_t)hpcd->Init.dma_enable,
							(uint8_t *)hpcd->Setup);

	__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_USBRST);
}
static inline void USB_enumeration_done_IRQ(PCD_HandleTypeDef* hpcd) {
	(void)USB_ActivateSetup(hpcd->Instance);
	hpcd->Init.speed = USB_GetDevSpeed(hpcd->Instance);

	/* Set USB Turnaround time */
	(void)USB_SetTurnaroundTime(hpcd->Instance,
								 HAL_RCC_GetHCLKFreq(),
								 (uint8_t)hpcd->Init.speed);

	HAL_PCD_ResetCallback(hpcd);

	__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_ENUMDNE);
}
static inline void IEP_common_handler(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	USB_OTG_EPTypeDef *ep;
	uint32_t i;
	uint32_t epint;
	uint32_t epnum = ep_num;
	USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t fifoemptymsk;

	epint = USB_ReadDevInEPInterrupt(hpcd->Instance, (uint8_t)epnum);

	if ((epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC)
	{
		fifoemptymsk = (uint32_t)(0x1UL << (epnum & EP_ADDR_MSK));
		USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;

		CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_XFRC);

		if (hpcd->Init.dma_enable == 1U)
		{
			hpcd->IN_ep[epnum].xfer_buff += hpcd->IN_ep[epnum].maxpacket;

			/* this is ZLP, so prepare EP0 for next setup */
			if ((epnum == 0U) && (hpcd->IN_ep[epnum].xfer_len == 0U))
			{
				/* prepare to rx more setup packets */
				(void)USB_EP0_OutStart(hpcd->Instance, 1U, (uint8_t *)hpcd->Setup);
			}
		}

		HAL_PCD_DataInStageCallback(hpcd, (uint8_t)epnum);
	}
	if ((epint & USB_OTG_DIEPINT_TOC) == USB_OTG_DIEPINT_TOC)
	{
		CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_TOC);
	}
	if ((epint & USB_OTG_DIEPINT_ITTXFE) == USB_OTG_DIEPINT_ITTXFE)
	{
		CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_ITTXFE);
	}
	if ((epint & USB_OTG_DIEPINT_INEPNE) == USB_OTG_DIEPINT_INEPNE)
	{
		CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_INEPNE);
	}
	if ((epint & USB_OTG_DIEPINT_EPDISD) == USB_OTG_DIEPINT_EPDISD)
	{
		flush_TX_FIFO(hpcd->Instance, epnum);

		ep = &hpcd->IN_ep[epnum];

		if (ep->is_iso_incomplete == 1U)
		{
			ep->is_iso_incomplete = 0U;

			HAL_PCD_ISOINIncompleteCallback(hpcd, (uint8_t)epnum);
		}

		CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_EPDISD);
	}
	if ((epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE)
	{
		(void)PCD_WriteEmptyTxFifo(hpcd, epnum);
	}
}
static inline void OEP_common_handler(PCD_HandleTypeDef* hpcd, uint8_t ep_num) {
	USB_OTG_EPTypeDef *ep;
	uint32_t i;
	uint32_t epint;
	uint32_t epnum = ep_num;
	USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
	uint32_t USBx_BASE = (uint32_t) USBx;
	uint32_t fifoemptymsk;

	epint = USB_ReadDevOutEPInterrupt(hpcd->Instance, (uint8_t)epnum);

	if ((epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC)
	{
		CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_XFRC);
		(void)PCD_EP_OutXfrComplete_int(hpcd, epnum);
	}

	if ((epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP)
	{
		CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STUP);
		/* Class B setup phase done for previous decoded setup */
		(void)PCD_EP_OutSetupPacket_int(hpcd, epnum);
	}

	if ((epint & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS)
	{
		CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPDIS);
	}

	/* Clear OUT Endpoint disable interrupt */
	if ((epint & USB_OTG_DOEPINT_EPDISD) == USB_OTG_DOEPINT_EPDISD)
	{
		if ((USBx->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) == USB_OTG_GINTSTS_BOUTNAKEFF)
		{
			USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGONAK;
		}

		ep = &hpcd->OUT_ep[epnum];

		if (ep->is_iso_incomplete == 1U)
		{
			ep->is_iso_incomplete = 0U;

			HAL_PCD_ISOOUTIncompleteCallback(hpcd, (uint8_t)epnum);
		}

		CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_EPDISD);
	}

	/* Clear Status Phase Received interrupt */
	if ((epint & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR)
	{
		CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
	}

	/* Clear OUT NAK interrupt */
	if ((epint & USB_OTG_DOEPINT_NAK) == USB_OTG_DOEPINT_NAK)
	{
		CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_NAK);
	}

}
static inline void USB_incomplete_ISO_IN_IRQ(PCD_HandleTypeDef* hpcd) {
	uint32_t USBx_BASE = (uint32_t)hpcd->Instance;
	PCD_EPTypeDef*			ep;
	uint8_t ep_num;
	uint32_t RegVal;
	for (ep_num = 1U; ep_num < hpcd->Init.dev_endpoints; ep_num++) {
		RegVal = USBx_INEP(ep_num)->DIEPCTL;
		ep = &hpcd->IN_ep[ep_num];
		if ((ep->type != EP_TYPE_ISOC) || ((RegVal & USB_OTG_DIEPCTL_EPENA) != USB_OTG_DIEPCTL_EPENA)) { continue; }
		ep->is_iso_incomplete = 1U;
		USB_EPStopXfer(hpcd->Instance, ep);
	}
	__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_IISOIXFR);
}
static inline void USB_incomplete_ISO_OUT_IRQ(PCD_HandleTypeDef* hpcd) {
	USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint8_t ep_num;
	uint32_t RegVal;
	for (ep_num = 1U; ep_num < hpcd->Init.dev_endpoints; ep_num++) {
		RegVal = USBx_OUTEP(ep_num)->DOEPCTL;
		if ((hpcd->OUT_ep[ep_num].type == EP_TYPE_ISOC) &&
			((RegVal & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) &&
			((RegVal & (0x1U << 16)) == (hpcd->FrameNumber & 0x1U))) {
			hpcd->OUT_ep[ep_num].is_iso_incomplete = 1U;
			USBx->GINTMSK |= USB_OTG_GINTMSK_GONAKEFFM;
			if ((USBx->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) == 0U) {
				USBx_DEVICE->DCTL |= USB_OTG_DCTL_SGONAK;
				break;
			}
		}
	}
	__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
}
static inline void USB_connection_IRQ(PCD_HandleTypeDef* hpcd) {
	HAL_PCD_ConnectCallback(hpcd);
	__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_SRQINT);
}
static inline void USB_wake_up_IRQ(PCD_HandleTypeDef* hpcd) {
	uint32_t USBx_BASE = (uint32_t)hpcd->Instance;
	USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
	if (hpcd->LPM_State == LPM_L1) {
		hpcd->LPM_State = LPM_L0;
		HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L0_ACTIVE);
	}
	else {
		HAL_PCD_ResumeCallback(hpcd);
	}
	__HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_WKUINT);
}


// L1 ========================================= /
static void HAL_PCD_IRQHandler(PCD_HandleTypeDef* hpcd) {
	USB_OTG_GlobalTypeDef*	usb = hpcd->Instance;  // TODO use this!
	if (((usb->GINTSTS) & 0x1U) != USB_OTG_MODE_DEVICE) { return; }
	const uint32_t			irqs = usb->GINTSTS & usb->GINTMSK;
	if (!irqs) { return; }
	USB_OTG_DeviceTypeDef*	device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	uint8_t					ep_num;
	uint16_t				ep_gint;

	/* store current frame number */
	hpcd->FrameNumber = (device->DSTS & USB_OTG_DSTS_FNSOF_Msk) >> USB_OTG_DSTS_FNSOF_Pos;
	/* mode mismatch interrupt */
	if (irqs & USB_OTG_GINTSTS_OTGINT)					{ usb->GINTSTS |= USB_OTG_GINTSTS_MMIS; }
	/* OTG interrupt */
	if (irqs & USB_OTG_GINTSTS_OTGINT)					{ USB_OTG_IRQ(hpcd->pData); }
	/* start of frame interrupt */
	if (irqs & USB_OTG_GINTSTS_SOF)						{ USB_SOF_IRQ(hpcd->pData); }
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


void OTG_FS_IRQHandler(void) {
	HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}


// L1 WKUP
#define USB_OTG_FS_WAKEUP_EXTI_LINE                                   (0x1U << 18)  /*!< USB FS EXTI Line WakeUp Interrupt */
#define __HAL_PCD_UNGATE_PHYCLOCK(__HANDLE__) \
  *(__IO uint32_t *)((uint32_t)((__HANDLE__)->Instance) + USB_OTG_PCGCCTL_BASE) &= ~(USB_OTG_PCGCCTL_STOPCLK)
#define __HAL_USB_OTG_FS_WAKEUP_EXTI_CLEAR_FLAG()   EXTI->PR = USB_OTG_FS_WAKEUP_EXTI_LINE


// L0 WKUP
void OTG_FS_WKUP_IRQHandler(void) {
	if ((&hpcd_USB_OTG_FS)->Init.low_power_enable) {
		SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
		sys_clock_init();
	}
	__HAL_PCD_UNGATE_PHYCLOCK(&hpcd_USB_OTG_FS);
	__HAL_USB_OTG_FS_WAKEUP_EXTI_CLEAR_FLAG();
}