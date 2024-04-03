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
#define __HAL_RCC_SYSCFG_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);\
                                        (void)tmpreg; \
                                          } while(0U)
#define __HAL_RCC_USB_OTG_FS_CLK_ENABLE()  do {(RCC->AHB2ENR |= (RCC_AHB2ENR_OTGFSEN));\
                                               __HAL_RCC_SYSCFG_CLK_ENABLE();\
                                              }while(0U)


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
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority) {
	uint32_t prioritygroup = 0x00U;
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
}
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn) {
	NVIC_EnableIRQ(IRQn);
}
static HAL_StatusTypeDef USB_CoreReset(USB_OTG_GlobalTypeDef *USBx) {
	__IO uint32_t count = 0U;
	/* Wait for AHB master IDLE state. */
	do {
		count++;
		if (count > HAL_USB_TIMEOUT) {
			return HAL_TIMEOUT;
		}
	} while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);
	/* Core Soft Reset */
	count = 0U;
	USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

	do {
		count++;
		if (count > HAL_USB_TIMEOUT) {
			return HAL_TIMEOUT;
		}
	} while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);
	return HAL_OK;
}
uint32_t USB_GetMode(const USB_OTG_GlobalTypeDef *USBx) {
	return ((USBx->GINTSTS) & 0x1U);
}
HAL_StatusTypeDef USB_SetDevSpeed(const USB_OTG_GlobalTypeDef *USBx, uint8_t speed) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	USBx_DEVICE->DCFG |= speed;
	return HAL_OK;
}
HAL_StatusTypeDef USB_FlushTxFifo(USB_OTG_GlobalTypeDef *USBx, uint32_t num) {
	__IO uint32_t count = 0U;
	/* Wait for AHB master IDLE state. */
	do {
		count++;
		if (count > HAL_USB_TIMEOUT) {
			return HAL_TIMEOUT;
		}
	} while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);
	/* Flush TX Fifo */
	count = 0U;
	USBx->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (num << 6));

	do {
		count++;
		if (count > HAL_USB_TIMEOUT) {
			return HAL_TIMEOUT;
		}
	} while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);
	return HAL_OK;
}
HAL_StatusTypeDef USB_FlushRxFifo(USB_OTG_GlobalTypeDef *USBx) {
	__IO uint32_t count = 0U;
	/* Wait for AHB master IDLE state. */
	do {
		count++;
		if (count > HAL_USB_TIMEOUT) {
			return HAL_TIMEOUT;
		}
	} while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);
	/* Flush RX Fifo */
	count = 0U;
	USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;

	do {
		count++;
		if (count > HAL_USB_TIMEOUT) {
			return HAL_TIMEOUT;
		}
	} while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);
	return HAL_OK;
}


// L4 ========================================= /
void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle) {
	if(pcdHandle->Instance==USB_OTG_FS)
	{
		fconfig_GPIO(GPIOA, 11, GPIO_alt_func, GPIO_no_pull, GPIO_push_pull, GPIO_very_high_speed, 10);
		fconfig_GPIO(GPIOA, 12, GPIO_alt_func, GPIO_no_pull, GPIO_push_pull, GPIO_very_high_speed, 10);

		/* Peripheral clock enable */
		__HAL_RCC_USB_OTG_FS_CLK_ENABLE();

		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
	}
}
HAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg) {
	HAL_StatusTypeDef ret;
	/* FS interface (embedded Phy) */
	/* Select FS Embedded PHY */
	USBx->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

	/* Reset after a PHY select */
	ret = USB_CoreReset(USBx);

	if (cfg.battery_charging_enable == 0U) {
		/* Activate the USB Transceiver */
		USBx->GCCFG |= USB_OTG_GCCFG_PWRDWN;
	}
	else {
		/* Deactivate the USB Transceiver */
		USBx->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);
	}
	if (cfg.dma_enable == 1U) {
		USBx->GAHBCFG |= USB_OTG_GAHBCFG_HBSTLEN_2;
		USBx->GAHBCFG |= USB_OTG_GAHBCFG_DMAEN;
	}
	return ret;
}
HAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef *USBx, USB_OTG_ModeTypeDef mode) {
	uint32_t ms = 0U;

	USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);
	if (mode == USB_DEVICE_MODE) {
		USBx->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
		while (USB_GetMode(USBx) != (uint32_t)USB_DEVICE_MODE);
	}
	else {
		return HAL_ERROR;
	}

	if (ms == HAL_USB_CURRENT_MODE_MAX_DELAY_MS) {
		return HAL_ERROR;
	}

	return HAL_OK;
}
HAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg) {
	HAL_StatusTypeDef ret = HAL_OK;
	uint32_t USBx_BASE = (uint32_t)USBx;
	uint32_t i;

	for (i = 0U; i < 15U; i++) {
		USBx->DIEPTXF[i] = 0U;
	}

	/* VBUS Sensing setup */
	if (cfg.vbus_sensing_enable == 0U) {
		/*
     * Disable HW VBUS sensing. VBUS is internally considered to be always
     * at VBUS-Valid level (5V).
		 */
		USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
		USBx->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
		USBx->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
		USBx->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
	}
	else {
		/* Enable HW VBUS sensing */
		USBx->GCCFG &= ~USB_OTG_GCCFG_NOVBUSSENS;
		USBx->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;
	}
	/* Restart the Phy Clock */
	USBx_PCGCCTL = 0U;

	/* Set Core speed to Full speed mode */
	(void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_FULL);

	/* Flush the FIFOs */
	if (USB_FlushTxFifo(USBx, 0x10U) != HAL_OK) /* all Tx FIFOs */ {
		ret = HAL_ERROR;
	}
	if (USB_FlushRxFifo(USBx) != HAL_OK) {
		ret = HAL_ERROR;
	}

	/* Clear all pending Device Interrupts */
	USBx_DEVICE->DIEPMSK = 0U;
	USBx_DEVICE->DOEPMSK = 0U;
	USBx_DEVICE->DAINTMSK = 0U;

	for (i = 0U; i < cfg.dev_endpoints; i++) {
		if ((USBx_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA) {
			if (i == 0U) {
				USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
			} else {
				USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
			}
		} else {
			USBx_INEP(i)->DIEPCTL = 0U;
		}
		USBx_INEP(i)->DIEPTSIZ = 0U;
		USBx_INEP(i)->DIEPINT  = 0xFB7FU;
	}

	for (i = 0U; i < cfg.dev_endpoints; i++) {
		if ((USBx_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) {
			if (i == 0U) {
				USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
			} else {
				USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
			}
		} else {
			USBx_OUTEP(i)->DOEPCTL = 0U;
		}
		USBx_OUTEP(i)->DOEPTSIZ = 0U;
		USBx_OUTEP(i)->DOEPINT  = 0xFB7FU;
	}

	USBx_DEVICE->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);

	/* Disable all interrupts. */
	USBx->GINTMSK = 0U;

	/* Clear any pending interrupts */
	USBx->GINTSTS = 0xBFFFFFFFU;

	/* Enable the common interrupts */
	if (cfg.dma_enable == 0U) {
		USBx->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
	}

	/* Enable interrupts matching to the Device mode ONLY */
	USBx->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
					 USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
					 USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_IISOIXFRM |
					 USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM;
	if (cfg.Sof_enable != 0U) {
		USBx->GINTMSK |= USB_OTG_GINTMSK_SOFM;
	}

	if (cfg.vbus_sensing_enable == 1U) {
		USBx->GINTMSK |= (USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT);
	}
	return ret;
}
HAL_StatusTypeDef USB_DevDisconnect(const USB_OTG_GlobalTypeDef *USBx) {
	uint32_t USBx_BASE = (uint32_t)USBx;
	/* In case phy is stopped, ensure to ungate and restore the phy CLK */
	USBx_PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
	USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
	return HAL_OK;
}
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
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd) {
	const USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;;
	uint8_t i;

	/* Check the PCD handle allocation */
	if (hpcd == NULL) {
		return HAL_ERROR;
	}

	if (hpcd->State == HAL_PCD_STATE_RESET) {
		/* Allocate lock resource and initialize it */
		hpcd->Lock = HAL_UNLOCKED;

		/* Init the low level hardware : GPIO, CLOCK, NVIC... */
		HAL_PCD_MspInit(hpcd);
	}

	hpcd->State = HAL_PCD_STATE_BUSY;
	/* Disable DMA mode for FS instance */
	if (USBx == USB_OTG_FS) {
		hpcd->Init.dma_enable = 0U;
	}

	/* Disable the Interrupts */
	__HAL_PCD_DISABLE(hpcd);

	/*Init the Core (common init.) */
	if (USB_CoreInit(hpcd->Instance, hpcd->Init) != HAL_OK) {
		hpcd->State = HAL_PCD_STATE_ERROR;
		return HAL_ERROR;
	}

	/* Force Device Mode */
	if (USB_SetCurrentMode(hpcd->Instance, USB_DEVICE_MODE) != HAL_OK) {
		hpcd->State = HAL_PCD_STATE_ERROR;
		return HAL_ERROR;
	}

	/* Init endpoints structures */
	for (i = 0U; i < hpcd->Init.dev_endpoints; i++) {
		/* Init ep structure */
		hpcd->IN_ep[i].is_in = 1U;
		hpcd->IN_ep[i].num = i;
		hpcd->IN_ep[i].tx_fifo_num = i;
		/* Control until ep is activated */
		hpcd->IN_ep[i].type = EP_TYPE_CTRL;
		hpcd->IN_ep[i].maxpacket = 0U;
		hpcd->IN_ep[i].xfer_buff = 0U;
		hpcd->IN_ep[i].xfer_len = 0U;
	}
	for (i = 0U; i < hpcd->Init.dev_endpoints; i++) {
		hpcd->OUT_ep[i].is_in = 0U;
		hpcd->OUT_ep[i].num = i;
		/* Control until ep is activated */
		hpcd->OUT_ep[i].type = EP_TYPE_CTRL;
		hpcd->OUT_ep[i].maxpacket = 0U;
		hpcd->OUT_ep[i].xfer_buff = 0U;
		hpcd->OUT_ep[i].xfer_len = 0U;
	}

	/* Init Device */
	if (USB_DevInit(hpcd->Instance, hpcd->Init) != HAL_OK) {
		hpcd->State = HAL_PCD_STATE_ERROR;
		return HAL_ERROR;
	}

	hpcd->USB_Address = 0U;
	hpcd->State = HAL_PCD_STATE_READY;
	(void)USB_DevDisconnect(hpcd->Instance);
	return HAL_OK;
}
HAL_StatusTypeDef HAL_PCDEx_SetRxFiFo(PCD_HandleTypeDef *hpcd, uint16_t size) {
	hpcd->Instance->GRXFSIZ = size;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size) {
	uint8_t i;
	uint32_t Tx_Offset;

	/*  TXn min size = 16 words. (n  : Transmit FIFO index)
		When a TxFIFO is not used, the Configuration should be as follows:
			case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
		   --> Txm can use the space allocated for Txn.
		   case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
		   --> Txn should be configured with the minimum space of 16 words
	   The FIFO is used optimally when used TxFIFOs are allocated in the top
		   of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
	   When DMA is used 3n * FIFO locations should be reserved for internal DMA registers */
	Tx_Offset = hpcd->Instance->GRXFSIZ;

	if (fifo == 0U) {
		hpcd->Instance->DIEPTXF0_HNPTXFSIZ = ((uint32_t)size << 16) | Tx_Offset;
	}
	else {
		Tx_Offset += (hpcd->Instance->DIEPTXF0_HNPTXFSIZ) >> 16;
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
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev) {
	/* Init USB Ip. */
	if (pdev->id == DEVICE_FS) {
		/* Link the driver to the stack. */
		hpcd_USB_OTG_FS.pData = pdev;
		pdev->pData = &hpcd_USB_OTG_FS;

		hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
		hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
		hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
		hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
		hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
		hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
		hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
		hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
		hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
		hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
		if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
			Error_Handler();
		}
		HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);
		HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
		HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x80);
	}
	return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_Start(pdev->pData);
	usb_status =  USBD_Get_USB_Status(hal_status);
	return usb_status;
}


// L1 ========================================= / // X
USBD_StatusTypeDef USBD_Init(USBD_HandleTypeDef *pdev, USBD_DescriptorsTypeDef *pdesc, uint8_t id) {
	/* Check whether the USB Host handle is valid */
	if (pdev == NULL) {
		return USBD_FAIL;
	}

	/* Unlink previous class*/
	pdev->pClass[0] = NULL;
	pdev->pUserData[0] = NULL;
	pdev->pConfDesc = NULL;

	/* Assign USBD Descriptors */
	if (pdesc != NULL) {
		pdev->pDesc = pdesc;
	}

	/* Set Device initial State */
	pdev->dev_state = USBD_STATE_DEFAULT;
	pdev->id = id;

	/* Initialize low level driver */
	return USBD_LL_Init(pdev);
}
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
	if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK) {
		Error_Handler();
	}
	if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID) != USBD_OK) {
		Error_Handler();
	}
	if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
		Error_Handler();
	}
}