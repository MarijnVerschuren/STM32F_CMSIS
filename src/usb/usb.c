//
// Created by marijn on 7/19/23.
//

#include "usb/usb.h"

/*!<
 * defines
 * */
#define USB_OTG_FS_WAKEUP_EXTI_LINE	(0x1U << 18)  /*!< USB FS EXTI Line WakeUp Interrupt */


/*!<
 * variables
 * */
USBD_HandleTypeDef hUsbDeviceFS;
PCD_HandleTypeDef hpcd_USB_OTG_FS;


/*!<
 * functions
 * */
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


/*!<
 * init
 * */
void USB_device_init(USB_OTG_GlobalTypeDef*	usb) {
	uint8_t i;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_OTG_DEVICE_BASE);
	USB_OTG_INEndpointTypeDef*	in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE);
	USB_OTG_OUTEndpointTypeDef*	out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE);
	__IO uint32_t*				PCGCCTL =	(void*)(((uint32_t)usb) + USB_OTG_PCGCCTL_BASE);

	// USBD_Init
	hUsbDeviceFS.pClass =		NULL;
	hUsbDeviceFS.pDesc =			&FS_Desc;
	hUsbDeviceFS.dev_state =		USBD_STATE_DEFAULT;
	// USBD_LL_Init
	hpcd_USB_OTG_FS.pData = &hUsbDeviceFS;
	hUsbDeviceFS.pData = &hpcd_USB_OTG_FS;
	hpcd_USB_OTG_FS.Instance = usb;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
	hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = ENABLE;  // TODO!!!
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
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
	if(hpcd_USB_OTG_FS.Init.low_power_enable == 1) {
		EXTI->PR =		USB_OTG_FS_WAKEUP_EXTI_LINE;
		EXTI->FTSR &=	~(USB_OTG_FS_WAKEUP_EXTI_LINE);
		EXTI->RTSR |=	USB_OTG_FS_WAKEUP_EXTI_LINE;
		EXTI->IMR |=	USB_OTG_FS_WAKEUP_EXTI_LINE;
		NVIC_SetPriority(OTG_FS_WKUP_IRQn, NVIC_EncodePriority(prioritygroup, 0, 0));
		NVIC_EnableIRQ(OTG_FS_WKUP_IRQn);
	}
	// ~ HAL_PCD_Msp_Init

	usb->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;

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
	while ((usb->GINTSTS) & 0b1U);

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
	device->DCFG |= 3U;  // set full speed
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

	usb->GINTMSK |= (
		USB_OTG_GINTMSK_USBSUSPM		|
		USB_OTG_GINTMSK_USBRST			|
		USB_OTG_GINTMSK_RXFLVLM			|
		USB_OTG_GINTMSK_ENUMDNEM		|
		USB_OTG_GINTMSK_IEPINT			|
		USB_OTG_GINTMSK_OEPINT			|
		USB_OTG_GINTMSK_IISOIXFRM		|
		USB_OTG_GINTMSK_PXFRM_IISOOXFRM	|
		USB_OTG_GINTMSK_WUIM
	);

	if (hpcd_USB_OTG_FS.Init.Sof_enable != 0U) {
		usb->GINTMSK |= USB_OTG_GINTMSK_SOFM;
	}

	if (hpcd_USB_OTG_FS.Init.vbus_sensing_enable == 1U) {
		usb->GINTMSK |= (USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT);
	}
	// ~ USB_DevInit

	hpcd_USB_OTG_FS.USB_Address = 0U;
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

	// USBD_RegisterClass
	hUsbDeviceFS.pClass = &USBD_HID;
	// ~ USBD_RegisterClass

	// USBD_Start
	// USBD_LL_Start
	// HAL_PCD_Start
	if (((usb->GUSBCFG & USB_OTG_GUSBCFG_PHYSEL) != 0U) &&
		(hpcd_USB_OTG_FS.Init.battery_charging_enable == 1U)) {
		/* Enable USB Transceiver */
		usb->GCCFG |= USB_OTG_GCCFG_PWRDWN;
	}

	usb->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

	// USB_DevConnect
	*PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
	device->DCTL &= ~USB_OTG_DCTL_SDIS;
	// ~ USB_DevConnect
	// ~ HAL_PCD_Start
	// ~ USBD_LL_Start
	// ~ USBD_Start
}
