//
// Created by marijn on 4/2/24.
//
#include "usb/usb.h"
#include "usb/hid.h"


// L1 ========================================= /
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);


// L0 ========================================= /
USBD_ClassTypeDef USBD_HID = {
		USBD_HID_Init,
		USBD_HID_DeInit,
		USBD_HID_Setup,
		NULL,              /* EP0_TxSent */
		NULL,              /* EP0_RxReady */
		USBD_HID_DataIn,   /* DataIn */
		NULL,              /* DataOut */
		NULL,              /* SOF */
		NULL,
		NULL,
		NULL,  // HS
		USBD_HID_GetFSCfgDesc,
		USBD_HID_GetOtherSpeedCfgDesc,
		USBD_HID_GetDeviceQualifierDesc,
};

// L1 ========================================= /
#define HID_EPIN_ADDR                              0x81U
static uint8_t HIDInEpAdd = HID_EPIN_ADDR;
#define HID_EPIN_SIZE                              0x04U

#define USB_HID_CONFIG_DESC_SIZ                    34U
#define USB_HID_DESC_SIZ                           9U
#define HID_DESCRIPTOR_TYPE                        0x21U
#define HID_REPORT_DESC                            0x22U
#define HID_MOUSE_REPORT_DESC_SIZE                 63U

__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END = {
	0x09,                                               /* bLength: Configuration Descriptor size */
	USB_DESC_TYPE_CONFIGURATION,                        /* bDescriptorType: Configuration */
	USB_HID_CONFIG_DESC_SIZ,                            /* wTotalLength: Bytes returned */
	0x00,
	0x01,                                               /* bNumInterfaces: 1 interface */
	0x01,                                               /* bConfigurationValue: Configuration value */
	0x00,                                               /* iConfiguration: Index of string descriptor
			 describing the configuration */
#if (USBD_SELF_POWERED == 1U)
	0xE0,                                               /* bmAttributes: Bus Powered according to user configuration */
#else
	0xA0,                                               /* bmAttributes: Bus Powered according to user configuration */
#endif /* USBD_SELF_POWERED */
	USBD_MAX_POWER,                                     /* MaxPower (mA) */

	/************** Descriptor of Joystick Mouse interface ****************/
	/* 09 */
	0x09,                                               /* bLength: Interface Descriptor size */
	USB_DESC_TYPE_INTERFACE,                            /* bDescriptorType: Interface descriptor type */
	0x00,                                               /* bInterfaceNumber: Number of Interface */
	0x00,                                               /* bAlternateSetting: Alternate setting */
	0x01,                                               /* bNumEndpoints */
	0x03,                                               /* bInterfaceClass: HID */
	0x01,                                               /* bInterfaceSubClass : 1=BOOT, 0=no boot */
	0x01,                                               /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
	0,                                                  /* iInterface: Index of string descriptor */
	/******************** Descriptor of Joystick Mouse HID ********************/
	/* 18 */
	0x09,                                               /* bLength: HID Descriptor size */
	HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
	0x11,                                               /* bcdHID: HID Class Spec release number */
	0x01,
	0x00,                                               /* bCountryCode: Hardware target country */
	0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
	0x22,                                               /* bDescriptorType */
	HID_MOUSE_REPORT_DESC_SIZE,                         /* wItemLength: Total length of Report descriptor */
	0x00,
	/******************** Descriptor of Mouse endpoint ********************/
	/* 27 */
	0x07,                                               /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT,                             /* bDescriptorType:*/

	HID_EPIN_ADDR,                                      /* bEndpointAddress: Endpoint Address (IN) */
	0x03,                                               /* bmAttributes: Interrupt endpoint */
	HID_EPIN_SIZE,                                      /* wMaxPacketSize: 4 Bytes max */
	0x00,
	HID_FS_BINTERVAL,                                   /* bInterval: Polling Interval */
	/* 34 */
};
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
	USB_LEN_DEV_QUALIFIER_DESC,
	USB_DESC_TYPE_DEVICE_QUALIFIER,
	0x00,
	0x02,
	0x00,
	0x00,
	0x00,
	0x40,
	0x01,
	0x00,
};
// NOTE: this is actually a keyboard descriptor!!!!
__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[HID_MOUSE_REPORT_DESC_SIZE] __ALIGN_END = {
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
		0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
		0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
		0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
		0xc0                           // END_COLLECTION
};
__ALIGN_BEGIN static uint8_t USBD_HID_Desc[USB_HID_DESC_SIZ] __ALIGN_END = {
	/* 18 */
	0x09,                                               /* bLength: HID Descriptor size */
	HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
	0x11,                                               /* bcdHID: HID Class Spec release number */
	0x01,
	0x00,                                               /* bCountryCode: Hardware target country */
	0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
	0x22,                                               /* bDescriptorType */
	HID_MOUSE_REPORT_DESC_SIZE,                         /* wItemLength: Total length of Report descriptor */
	0x00,
};


// defs
extern void IEP0_transfer(USBD_HandleTypeDef* handle, uint8_t *buffer, uint32_t size);
extern void open_IEP(PCD_HandleTypeDef *hpcd, uint8_t ep_num, uint16_t ep_mps, uint8_t ep_type);
extern void open_OEP(PCD_HandleTypeDef *hpcd, uint8_t ep_num, uint16_t ep_mps, uint8_t ep_type);
extern void close_IEP(PCD_HandleTypeDef *hpcd, uint8_t ep_num);
extern void close_OEP(PCD_HandleTypeDef *hpcd, uint8_t ep_num);
extern void stall_IEP(PCD_HandleTypeDef *hpcd, uint8_t ep_num);
extern void stall_OEP(PCD_HandleTypeDef *hpcd, uint8_t ep_num);
extern void stall_EP(PCD_HandleTypeDef* hpcd, uint8_t ep_num);



// L3 ========================================= /
USBD_DescHeaderTypeDef *USBD_GetNextDesc(uint8_t *pbuf, uint16_t *ptr) {
	USBD_DescHeaderTypeDef *pnext = (USBD_DescHeaderTypeDef *)(void *)pbuf;
	*ptr += pnext->bLength;
	pnext = (USBD_DescHeaderTypeDef *)(void *)(pbuf + pnext->bLength);
	return (pnext);
}

// L2 ========================================= /
void *USBD_static_malloc(uint32_t size) {
	static uint32_t mem[(sizeof(USBD_HID_HandleTypeDef)/4)+1];/* On 32-bit boundary */
	return mem;
}
#define USBD_malloc         (void *)USBD_static_malloc
void USBD_static_free(void *p) {}
#define USBD_free           USBD_static_free
void *USBD_GetEpDesc(uint8_t *pConfDesc, uint8_t EpAddr) {
	USBD_DescHeaderTypeDef *pdesc = (USBD_DescHeaderTypeDef *)(void *)pConfDesc;
	USBD_ConfigDescTypeDef *desc = (USBD_ConfigDescTypeDef *)(void *)pConfDesc;
	USBD_EpDescTypeDef *pEpDesc = NULL;
	uint16_t ptr;

	if (desc->wTotalLength > desc->bLength) {
		ptr = desc->bLength;
		while (ptr < desc->wTotalLength) {
			pdesc = USBD_GetNextDesc((uint8_t *)pdesc, &ptr);
			if (pdesc->bDescriptorType == USB_DESC_TYPE_ENDPOINT) {
				pEpDesc = (USBD_EpDescTypeDef *)(void *)pdesc;
				if (pEpDesc->bEndpointAddress == EpAddr) {
					break;
				} else {
					pEpDesc = NULL;
				}
			}
		}
	}
	return (void *)pEpDesc;
}


// L1 ========================================= /
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	UNUSED(cfgidx);

	USBD_HID_HandleTypeDef *hhid;

	hhid = (USBD_HID_HandleTypeDef*)USBD_malloc(sizeof(USBD_HID_HandleTypeDef));

	if (hhid == NULL) {
		pdev->pClassDataCmsit[pdev->classId] = NULL;
		return (uint8_t)USBD_EMEM;
	}

	pdev->pClassDataCmsit[pdev->classId] = (void *)hhid;
	pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];

	pdev->ep_in[HIDInEpAdd & 0xFU].bInterval = HID_FS_BINTERVAL;

	/* Open EP IN */
	open_IEP(pdev->pData, HIDInEpAdd, HID_EPIN_SIZE, USBD_EP_TYPE_INTR);
	pdev->ep_in[HIDInEpAdd & 0xFU].is_used = 1U;

	hhid->state = USBD_HID_IDLE;
	return (uint8_t)USBD_OK;
}
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	UNUSED(cfgidx);

	/* Close HID EPs */
	close_IEP(pdev->pData, HIDInEpAdd);
	pdev->ep_in[HIDInEpAdd & 0xFU].is_used = 0U;
	pdev->ep_in[HIDInEpAdd & 0xFU].bInterval = 0U;

	/* Free allocated memory */
	if (pdev->pClassDataCmsit[pdev->classId] != NULL) {
		(void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
		pdev->pClassDataCmsit[pdev->classId] = NULL;
	}
	return (uint8_t)USBD_OK;
}
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
	USBD_StatusTypeDef ret = USBD_OK;
	uint16_t len;
	uint8_t *pbuf;
	uint16_t status_info = 0U;

	if (hhid == NULL)
	{
		return (uint8_t)USBD_FAIL;
	}

	switch (req->bmRequest & USB_REQ_TYPE_MASK)
	{
		case USB_REQ_TYPE_CLASS :
			switch (req->bRequest)
			{
				case USBD_HID_REQ_SET_PROTOCOL:
					hhid->Protocol = (uint8_t)(req->wValue);
					break;

				case USBD_HID_REQ_GET_PROTOCOL:
					IEP0_transfer(pdev, (uint8_t *)&hhid->Protocol, 1U);
					break;

				case USBD_HID_REQ_SET_IDLE:
					hhid->IdleState = (uint8_t)(req->wValue >> 8);
					break;

				case USBD_HID_REQ_GET_IDLE:
					IEP0_transfer(pdev, (uint8_t *)&hhid->IdleState, 1U);
					break;

				default:
					stall_EP(pdev->pData, 0x0U);
					ret = USBD_FAIL;
					break;
			}
			break;
		case USB_REQ_TYPE_STANDARD:
			switch (req->bRequest)
			{
				case USB_REQ_GET_STATUS:
					if (pdev->dev_state == USBD_STATE_CONFIGURED)
					{
						IEP0_transfer(pdev, (uint8_t *)&status_info, 2U);
					}
					else
					{
						stall_EP(pdev->pData, 0x0U);
						ret = USBD_FAIL;
					}
					break;

				case USB_REQ_GET_DESCRIPTOR:
					if ((req->wValue >> 8) == HID_REPORT_DESC)
					{
						len = MIN(HID_MOUSE_REPORT_DESC_SIZE, req->wLength);
						pbuf = HID_MOUSE_ReportDesc;
					}
					else if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE)
					{
						pbuf = USBD_HID_Desc;
						len = MIN(USB_HID_DESC_SIZ, req->wLength);
					}
					else
					{
						stall_EP(pdev->pData, 0x0U);
						ret = USBD_FAIL;
						break;
					}
					IEP0_transfer(pdev, pbuf, len);
					break;

				case USB_REQ_GET_INTERFACE :
					if (pdev->dev_state == USBD_STATE_CONFIGURED)
					{
						IEP0_transfer(pdev, (uint8_t *)&hhid->AltSetting, 1U);
					}
					else
					{
						stall_EP(pdev->pData, 0x0U);
						ret = USBD_FAIL;
					}
					break;

				case USB_REQ_SET_INTERFACE:
					if (pdev->dev_state == USBD_STATE_CONFIGURED)
					{
						hhid->AltSetting = (uint8_t)(req->wValue);
					}
					else
					{
						stall_EP(pdev->pData, 0x0U);
						ret = USBD_FAIL;
					}
					break;

				case USB_REQ_CLEAR_FEATURE:
					break;

				default:
					stall_EP(pdev->pData, 0x0U);
					ret = USBD_FAIL;
					break;
			}
			break;

		default:
			stall_EP(pdev->pData, 0x0U);
			ret = USBD_FAIL;
			break;
	}
	return (uint8_t)ret;
}

static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	UNUSED(epnum);
	/* Ensure that the FIFO is empty before a new transfer, this condition could
	be caused by  a new transfer before the end of the previous transfer */
	((USBD_HID_HandleTypeDef*)pdev->pClassDataCmsit[pdev->classId])->state = USBD_HID_IDLE;

	return (uint8_t)USBD_OK;
}
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length) {
	USBD_EpDescTypeDef *pEpDesc = USBD_GetEpDesc(USBD_HID_CfgDesc, HID_EPIN_ADDR);

	if (pEpDesc != NULL) {
		pEpDesc->bInterval = HID_FS_BINTERVAL;
	}

	*length = (uint16_t)sizeof(USBD_HID_CfgDesc);
	return USBD_HID_CfgDesc;
}
static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length) {
	USBD_EpDescTypeDef *pEpDesc = USBD_GetEpDesc(USBD_HID_CfgDesc, HID_EPIN_ADDR);

	if (pEpDesc != NULL) {
		pEpDesc->bInterval = HID_FS_BINTERVAL;
	}

	*length = (uint16_t)sizeof(USBD_HID_CfgDesc);
	return USBD_HID_CfgDesc;
}
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length) {
	*length = (uint16_t)sizeof(USBD_HID_DeviceQualifierDesc);
	return USBD_HID_DeviceQualifierDesc;
}


// L0 ========================================= /
extern void IN_transfer(PCD_HandleTypeDef *hpcd, uint8_t ep_num, void* buffer, uint32_t size);
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len) {
	USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	if (hhid == NULL) {
		return (uint8_t)USBD_FAIL;
	}

	if (pdev->dev_state == USBD_STATE_CONFIGURED) {
		if (hhid->state == USBD_HID_IDLE) {
			hhid->state = USBD_HID_BUSY;
			IN_transfer(pdev->pData, HIDInEpAdd, report, len);
		}
	}

	return (uint8_t)USBD_OK;
}