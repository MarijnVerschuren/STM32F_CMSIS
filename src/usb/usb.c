//
// Created by marijn on 7/19/23.
//

#include "usb/usb.h"


// status
void /*L0*/ Error_Handler(void) {
	for(;;);
}
USBD_StatusTypeDef /*L2*/ USBD_Get_USB_Status(HAL_StatusTypeDef hal_status) {
	USBD_StatusTypeDef usb_status = USBD_OK;
	switch (hal_status) {
		case HAL_OK :
			usb_status = USBD_OK;
			break;
		case HAL_ERROR :
			usb_status = USBD_FAIL;
			break;
		case HAL_BUSY :
			usb_status = USBD_BUSY;
			break;
		case HAL_TIMEOUT :
			usb_status = USBD_FAIL;
			break;
		default :
			usb_status = USBD_FAIL;
			break;
	}
	return usb_status;
}

// macros


/*!<
 * variables
 * */
// L1 ========================================= /
USBD_HandleTypeDef hUsbDeviceFS;

// L2 ========================================= /
PCD_HandleTypeDef hpcd_USB_OTG_FS;



/*!<
 * static / hidden
 * */
// L5 ========================================= /
void flush_RX_FIFO(USB_OTG_GlobalTypeDef* usb) {
	__IO uint32_t why = 0;  // NOTE: creating a variable IS mandatory????
	while (!(usb->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));	// wait for AHB master IDLE state
	usb->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;					// flush RX FIFO
	while (usb->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH);			// wait until reset is processed
}
void flush_TX_FIFO(USB_OTG_GlobalTypeDef* usb, uint8_t ep) {
	while (!(usb->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
	usb->GRSTCTL = (
			ep << USB_OTG_GRSTCTL_TXFNUM_Pos		|			// select ep TX FIFO
			0b1UL << USB_OTG_GRSTCTL_TXFFLSH_Pos				// flush TX FIFO
	);
	while (usb->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);			// wait until reset is processed
}
void flush_TX_FIFOS(USB_OTG_GlobalTypeDef* usb) {
	while (!(usb->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
	usb->GRSTCTL = (
			0x10UL << USB_OTG_GRSTCTL_TXFNUM_Pos		|		// select all TX FIFOs
			0b1UL << USB_OTG_GRSTCTL_TXFFLSH_Pos				// flush TX FIFOs
	);
	while (usb->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);			// wait until reset is processed
}

// L4 ========================================= /
HAL_StatusTypeDef USB_DevConnect(const USB_OTG_GlobalTypeDef *USBx) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	/* In case phy is stopped, ensure to ungate and restore the phy CLK */
	USBx_PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
	USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
	return HAL_OK;
}
HAL_StatusTypeDef USB_EnableGlobalInt(USB_OTG_GlobalTypeDef *USBx) {
	USBx->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
	return HAL_OK;
}
#define __HAL_PCD_ENABLE(__HANDLE__)                       (void)USB_EnableGlobalInt ((__HANDLE__)->Instance)
HAL_StatusTypeDef USB_DisableGlobalInt(USB_OTG_GlobalTypeDef *USBx) {
	USBx->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
	return HAL_OK;
}
#define __HAL_PCD_DISABLE(__HANDLE__)                      (void)USB_DisableGlobalInt ((__HANDLE__)->Instance)

// L3 ========================================= / // X
HAL_StatusTypeDef HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size) {
	uint8_t i;
	uint32_t Tx_Offset;

	Tx_Offset = hpcd->Instance->GRXFSIZ;

	if (!fifo) {
		hpcd->Instance->DIEPTXF0_HNPTXFSIZ = ((uint32_t)size << 16) | Tx_Offset;
	}
	else {
		Tx_Offset += (hpcd->Instance->DIEPTXF0_HNPTXFSIZ >> 16);
		for (i = 0U; i < (fifo - 1U); i++) {
			Tx_Offset += (hpcd->Instance->DIEPTXF[i] >> 16);
		}
		/* Multiply Tx_Size by 2 to get higher performance */
		hpcd->Instance->DIEPTXF[fifo - 1U] = ((uint32_t)size << 16) | Tx_Offset;
	}
	return HAL_OK;
}

HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd) {
	USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
	__HAL_LOCK(hpcd);

	if (((USBx->GUSBCFG & USB_OTG_GUSBCFG_PHYSEL) != 0U) &&
		(hpcd->Init.battery_charging_enable == 1U)) {
		/* Enable USB Transceiver */
		USBx->GCCFG |= USB_OTG_GCCFG_PWRDWN;
	}

	__HAL_PCD_ENABLE(hpcd);
	(void)USB_DevConnect(hpcd->Instance);
	__HAL_UNLOCK(hpcd);
	return HAL_OK;
}


// L2 ========================================= / // X
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_Start(pdev->pData);
	usb_status =  USBD_Get_USB_Status(hal_status);
	return usb_status;
}


// L1 ========================================= / // X
USBD_StatusTypeDef USBD_RegisterClass(USBD_HandleTypeDef *pdev, USBD_ClassTypeDef *pclass) {
	uint16_t len = 0U;
	if (pclass == NULL) {
		return USBD_FAIL;
	}

	/* link the class to the USB Device handle */
	pdev->pClass[0] = pclass;

	/* Get Device Configuration Descriptor */
	if (pdev->pClass[pdev->classId]->GetFSConfigDescriptor != NULL) {
		pdev->pConfDesc = (void *)pdev->pClass[pdev->classId]->GetFSConfigDescriptor(&len);
	}

	/* Increment the NumClasses */
	pdev->NumClasses ++;
	return USBD_OK;
}
USBD_StatusTypeDef USBD_Start(USBD_HandleTypeDef *pdev) {
	/* Start the low level driver  */
	return USBD_LL_Start(pdev);
}


/*!<
 * init
 * */  // L0 ================================== / // X
void MX_USB_DEVICE_Init(void) {
	uint8_t i;
	USB_OTG_GlobalTypeDef*		usb =		USB_OTG_FS;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE);
	__IO uint32_t*				PCGCCTL =	(void*)(((uint32_t)usb) + USB_OTG_PCGCCTL_BASE);


	// USBD_Init
	hUsbDeviceFS.pClass[0] =		NULL;
	hUsbDeviceFS.pUserData[0] =		NULL;
	hUsbDeviceFS.pConfDesc =		NULL;
	hUsbDeviceFS.pDesc =			&FS_Desc;
	hUsbDeviceFS.dev_state =		USBD_STATE_DEFAULT;
	hUsbDeviceFS.id =				DEVICE_FS;
	// USBD_LL_Init
	hpcd_USB_OTG_FS.pData = &hUsbDeviceFS;
	hUsbDeviceFS.pData = &hpcd_USB_OTG_FS;
	hpcd_USB_OTG_FS.Instance = usb;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	// HAL_PCD_Init
	// HAL_PCD_Msp_Init
	fconfig_GPIO(GPIOA, 11, GPIO_alt_func, GPIO_no_pull, GPIO_push_pull, GPIO_very_high_speed, 10);
	fconfig_GPIO(GPIOA, 12, GPIO_alt_func, GPIO_no_pull, GPIO_push_pull, GPIO_very_high_speed, 10);

	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// TODO
	uint32_t prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(OTG_FS_IRQn, NVIC_EncodePriority(prioritygroup, 0, 0));
	NVIC_EnableIRQ(OTG_FS_IRQn);
	// ~ HAL_PCD_Msp_Init

	hpcd_USB_OTG_FS.State = HAL_PCD_STATE_BUSY;
	hpcd_USB_OTG_FS.Init.dma_enable = 0U;
	__HAL_PCD_DISABLE(&hpcd_USB_OTG_FS);

	// USB_CoreInit
	usb->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

	while (!(usb->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
	usb->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;					// reset the core
	while (usb->GRSTCTL & USB_OTG_GRSTCTL_CSRST);			// wait until reset is processed

	if (hpcd_USB_OTG_FS.Init.battery_charging_enable == 0U) {
		usb->GCCFG |= USB_OTG_GCCFG_PWRDWN;
	} else {
		usb->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);
	}
	// ~ USB_CoreInit

	usb->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);
	usb->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
	while (((usb->GINTSTS) & 0x1U) != (uint32_t)USB_DEVICE_MODE);

	for (i = 0U; i < hpcd_USB_OTG_FS.Init.dev_endpoints; i++) {
		hpcd_USB_OTG_FS.IN_ep[i].is_in = 1U;
		hpcd_USB_OTG_FS.IN_ep[i].num = i;
		hpcd_USB_OTG_FS.IN_ep[i].tx_fifo_num = i;
		hpcd_USB_OTG_FS.IN_ep[i].type = EP_TYPE_CTRL;
		hpcd_USB_OTG_FS.IN_ep[i].maxpacket = 0U;
		hpcd_USB_OTG_FS.IN_ep[i].xfer_buff = 0U;
		hpcd_USB_OTG_FS.IN_ep[i].xfer_len = 0U;

		hpcd_USB_OTG_FS.OUT_ep[i].is_in = 0U;
		hpcd_USB_OTG_FS.OUT_ep[i].num = i;
		hpcd_USB_OTG_FS.OUT_ep[i].type = EP_TYPE_CTRL;
		hpcd_USB_OTG_FS.OUT_ep[i].maxpacket = 0U;
		hpcd_USB_OTG_FS.OUT_ep[i].xfer_buff = 0U;
		hpcd_USB_OTG_FS.OUT_ep[i].xfer_len = 0U;
	}

	// USB_DevInit
	for (i = 0U; i < 15U; i++) {
		usb->DIEPTXF[i] = 0U;
	}
	if (hpcd_USB_OTG_FS.Init.vbus_sensing_enable == 0U) {
		/*
     * Disable HW VBUS sensing. VBUS is internally considered to be always
     * at VBUS-Valid level (5V).
		 */
		device->DCTL |= USB_OTG_DCTL_SDIS;
		usb->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
		usb->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
		usb->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
	}
	else {
		/* Enable HW VBUS sensing */
		usb->GCCFG &= ~USB_OTG_GCCFG_NOVBUSSENS;
		usb->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;
	}

	*PCGCCTL = 0U;

	// USB_SetDevSpeed
	device->DCFG |= USB_OTG_SPEED_FULL;
	// ~ USB_SetDevSpeed

	/* Flush the FIFOs */
	flush_TX_FIFOS(usb);
	flush_RX_FIFO(usb);

	device->DIEPMSK = 0U;
	device->DOEPMSK = 0U;
	device->DAINTMSK = 0U;

	for (i = 0U; i < hpcd_USB_OTG_FS.Init.dev_endpoints; i++) {
		if ((in[i].DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA) {
			if (i == 0U) {
				in[i].DIEPCTL = USB_OTG_DIEPCTL_SNAK;
			} else {
				in[i].DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
			}
		} else {
			in[i].DIEPCTL = 0U;
		}
		in[i].DIEPTSIZ = 0U;
		in[i].DIEPINT  = 0xFB7FU;
	}
	for (i = 0U; i < hpcd_USB_OTG_FS.Init.dev_endpoints; i++) {
		if ((out[i].DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) {
			if (i == 0U) {
				out[i].DOEPCTL = USB_OTG_DOEPCTL_SNAK;
			} else {
				out[i].DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
			}
		} else {
			out[i].DOEPCTL = 0U;
		}
		out[i].DOEPTSIZ = 0U;
		out[i].DOEPINT  = 0xFB7FU;
	}

	device->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);
	usb->GINTMSK = 0U;
	usb->GINTSTS = 0xBFFFFFFFU;

	usb->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_RXFLVLM |
					 USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
					 USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_IISOIXFRM |
					 USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM;

	if (hpcd_USB_OTG_FS.Init.Sof_enable != 0U) {
		usb->GINTMSK |= USB_OTG_GINTMSK_SOFM;
	}

	if (hpcd_USB_OTG_FS.Init.vbus_sensing_enable == 1U) {
		usb->GINTMSK |= (USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT);
	}
	// ~ USB_DevInit

	hpcd_USB_OTG_FS.USB_Address = 0U;
	hpcd_USB_OTG_FS.State = HAL_PCD_STATE_READY;
	// USB_DevDisconnect
	*PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
	device->DCTL |= USB_OTG_DCTL_SDIS;
	// ~ USB_DevDisconnect
	// ~ HAL_PCD_Init
	usb->GRXFSIZ = 0x80;											// TODO: argument
	usb->DIEPTXF0_HNPTXFSIZ = ((uint32_t)0x40 << 16) | 0x80;		// TODO: argument
	usb->DIEPTXF[0] = ((uint32_t)0x80 << 16) | 0xC0;				// TODO: argument + logic to select endpoints
	// ~ USBD_LL_Init
	// ~ USBD_Init
	

	if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID) != USBD_OK) {
		Error_Handler();
	}
	if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
		Error_Handler();
	}
}
