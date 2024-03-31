//
// Created by marijn on 7/19/23.
//

#include "usb/usb.h"



/*!<
 * variables
 * */
uint32_t USB_kernel_frequency = 0;
USB_handle_t* USB_handle = NULL;


/*!<
 * static / hidden
 * */
void flush_RX_FIFO() {
	while (!(USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
	USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;					// flush RX FIFO
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH);			// wait until reset is processed
}
void flush_TX_FIFO(uint8_t ep) {
	while (!(USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
	USB_OTG_FS->GRSTCTL = (
		ep << USB_OTG_GRSTCTL_TXFNUM_Pos		|			// select ep TX FIFO
		0b1UL << USB_OTG_GRSTCTL_TXFFLSH_Pos				// flush TX FIFO
	);
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);			// wait until reset is processed
}
void flush_TX_FIFOS() {
	while (!(USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
	USB_OTG_FS->GRSTCTL = (
		0x10UL << USB_OTG_GRSTCTL_TXFNUM_Pos		|		// select all TX FIFOs
		0b1UL << USB_OTG_GRSTCTL_TXFFLSH_Pos				// flush TX FIFOs
	);
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);			// wait until reset is processed
}

static inline void USB_GPIO_to_args(uint32_t usb_pin, uint8_t* alternate_function, GPIO_TypeDef** port, uint8_t* pin) {
	(*alternate_function) =	(usb_pin >> 8) & 0xfu;
	(*port) =				int_to_GPIO(usb_pin >> 4);
	(*pin) =				usb_pin & 0xfu;
}


/*!<
 * init
 * */
void fconfig_USB_FS_device(USB_GPIO_t dp, USB_GPIO_t dn, uint32_t RX_FIFO_size) {
	/* argument and variable setup */
	if (dp == USB_PIN_DISABLE || dn == USB_PIN_DISABLE) { return; }
	USB_OTG_DeviceTypeDef		*device =	(void*)((uint32_t)USB_OTG_FS + 0x800);
	USB_OTG_INEndpointTypeDef	*in =		(void*)((uint32_t)USB_OTG_FS + 0x900);
	USB_OTG_OUTEndpointTypeDef	*out =		(void*)((uint32_t)USB_OTG_FS + 0xB00);
	volatile uint32_t			*PCGCCTL =	(void*)((uint32_t)USB_OTG_FS + 0xE00);
	//GPIO_TypeDef				*dp_port, *dn_port;
	//uint8_t						dp_af, dn_af, dp_pin, dn_pin;

	/* GPIO config */
	fconfig_GPIO(GPIOA, 12, GPIO_alt_func, GPIO_no_pull, GPIO_push_pull, GPIO_very_high_speed, 10);
	fconfig_GPIO(GPIOA, 11, GPIO_alt_func, GPIO_no_pull, GPIO_push_pull, GPIO_very_high_speed, 10);

	/* enable USB device clock and global interrupt */
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	set_IRQ_priority(OTG_FS_IRQn, 0);
	enable_IRQ(OTG_FS_IRQn);

	USB_OTG_FS->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;

	USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;			// select internal PHY

	while (!(USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
	USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;					// reset the core
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST);			// wait until reset is processed

	/* mode config */
	USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;
	USB_OTG_FS->GUSBCFG |= (
		USB_OTG_GUSBCFG_FDMOD									// force device device mode
		/*
		USB_OTG_GUSBCFG_HNPCAP	|	// ?
		USB_OTG_GUSBCFG_SRPCAP		// ?
		 */
	);
	while (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD);			// wait until device mode is set

	// TODO: HAL EP STRUCT INIT

	/* FIFO buffer setup */
	uint8_t i;
	for (i = 0UL; i < 0xFUL; i++) {
		USB_OTG_FS->DIEPTXF[i] = 0x00000000UL;						// clear IN endpoint FIFO sizes/offsets
	}

	USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_NOVBUSSENS;
	USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;

	*PCGCCTL = 0x00000000UL;								// restart PHY clock

	flush_TX_FIFOS();
	flush_RX_FIFO();

	/* endpoint and device interrupt config */
	device->DIEPMSK =	0x00000000UL;						// mask all IN endpoint interrupts
	device->DOEPMSK =	0x00000000UL;						// mask all OUT endpoint interrupts
	device->DAINTMSK =	0x00000000UL;						// mask all endpoint generated interrupts

	for (i = 0x0UL; i < USB_OTG_ENDPOINT_COUNT; i++) {
		if (in[i].DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
			if (i == 0x0UL) { in[i].DIEPCTL = USB_OTG_DIEPCTL_SNAK; }
			else { in[i].DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK; }
		} else { in[i].DIEPCTL = 0x00000000UL; }

		if (out[i].DOEPCTL & USB_OTG_DOEPCTL_EPENA) {
			if (i == 0x0UL) { out[i].DOEPCTL = USB_OTG_DOEPCTL_SNAK; }
			else { out[i].DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK; }
		} else { out[i].DOEPCTL = 0x00000000UL; }

		in[i].DIEPTSIZ	= out[i].DOEPTSIZ	= 0x00000000UL;	// clear transfer config
		in[i].DIEPINT	= out[i].DOEPINT	= 0xFB7FUL;		// clear all interrupt status bits
	}

	device->DIEPMSK &= ~(
		USB_OTG_DIEPMSK_TXFURM
	);

	/* usb core interrupt config */
	USB_OTG_FS->GINTMSK = 0x00000000UL;							// mask all interrupts
	USB_OTG_FS->GINTSTS = 0xBFFFFFFFUL;							// clear all interrupts (except for session request interrupt)
	USB_OTG_FS->GINTMSK |= (
		USB_OTG_GINTMSK_RXFLVLM			|
		USB_OTG_GINTMSK_USBSUSPM		|
		USB_OTG_GINTMSK_USBRST			|
		USB_OTG_GINTMSK_ENUMDNEM		|
		USB_OTG_GINTMSK_IEPINT			|
		USB_OTG_GINTMSK_OEPINT			|
		USB_OTG_GINTMSK_IISOIXFRM		|
		USB_OTG_GINTMSK_PXFRM_IISOOXFRM |
		USB_OTG_GINTMSK_WUIM			|
		USB_OTG_GINTMSK_SRQIM			|
		USB_OTG_GINTMSK_OTGINT
	);

	/* disconnect */
	*PCGCCTL &= ~(
		USB_OTG_PCGCCTL_STOPCLK			|
		USB_OTG_PCGCCTL_GATECLK
	);
	device->DCTL |= USB_OTG_DCTL_SDIS;

	/* setup FIFO buffers */
	config_USB_RX_FIFO(RX_FIFO_size);
	config_USB_TX_FIFO(0, 0x40);		// max setup packet size

}

void config_USB_FS_device(USB_GPIO_t dp, USB_GPIO_t dn) {
	fconfig_USB_FS_device(dp, dn, 0x80);
}


void config_USB_RX_FIFO(uint32_t size) {
	USB_OTG_FS->GRXFSIZ = size;
	USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = (
		(USB_OTG_FS->DIEPTXF0_HNPTXFSIZ & 0xFFFF0000UL) |
		size		// offset
	);
	size += USB_OTG_FS->DIEPTXF0_HNPTXFSIZ >> 16;
	// fix FIFO offsets for all IN endpoints
	for (uint8_t i = 0; i < 0xEUL; i++) {
		USB_OTG_FS->DIEPTXF[i] = (
			(USB_OTG_FS->DIEPTXF[i] & 0xFFFF0000UL) |
			size	// offset
		);
		size += USB_OTG_FS->DIEPTXF[i] >> 16;
	}
}
void config_USB_TX_FIFO(uint8_t ep, uint32_t size) {
	uint32_t offset = USB_OTG_FS->GRXFSIZ;
	uint32_t block_size = (USB_OTG_FS->DIEPTXF0_HNPTXFSIZ >> 16);
	if (ep == 0) { block_size = size; }	ep -= 1;
	USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = (
		(block_size << 16) |
		offset
	);	offset += block_size;
	// fix FIFO offsets for all IN endpoints (and set size)
	for (uint8_t i = 0; i < 0xEUL; i++) {
		block_size = USB_OTG_FS->DIEPTXF[i] >> 16;
		if (ep == i) { block_size = size; }
		USB_OTG_FS->DIEPTXF[i] = (
			(block_size << 16) |
			offset
		);	offset += block_size;
	}
}


void start_USB() {
	USB_OTG_DeviceTypeDef	*device =	(void*)((uint32_t)USB_OTG_FS + 0x800);
	volatile uint32_t		*PCGCCTL =	(void*)((uint32_t)USB_OTG_FS + 0xE00);
	USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
	//(void)USB_OTG_FS->GRXSTSP;  // pop status register
	*PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
	device->DCTL &= ~USB_OTG_DCTL_SDIS;
}
void stop_USB() {
	USB_OTG_DeviceTypeDef	*device =	(void*)((uint32_t)USB_OTG_FS + 0x800);
	volatile uint32_t		*PCGCCTL =	(void*)((uint32_t)USB_OTG_FS + 0xE00);
	USB_OTG_FS->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
	*PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
	device->DCTL |= USB_OTG_DCTL_SDIS;
	flush_TX_FIFOS();
}