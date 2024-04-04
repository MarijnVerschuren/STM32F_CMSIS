//
// Created by marijn on 7/19/23.
//

#ifndef STM32H_CMSIS_USB_F
#define STM32H_CMSIS_USB_F
#include "main.h"
#include "sys.h"
#include "nvic.h"
#include "gpio.h"


// macros and other hot fixes for BS
#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */
#define __ALIGN_BEGIN
#define __ALIGN_END

#define __HAL_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if((__HANDLE__)->Lock == HAL_LOCKED)   \
                                    {                                      \
                                       return HAL_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->Lock = HAL_LOCKED;    \
                                    }                                      \
                                  }while (0U)
#define __HAL_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                      (__HANDLE__)->Lock = HAL_UNLOCKED;    \
                                    }while (0U)

#define USBx_PCGCCTL    *(__IO uint32_t *)((uint32_t)USBx_BASE + USB_OTG_PCGCCTL_BASE)
#define USBx_DEVICE     ((USB_OTG_DeviceTypeDef *)(USBx_BASE + USB_OTG_DEVICE_BASE))
#define USBx_INEP(i)    ((USB_OTG_INEndpointTypeDef *)(USBx_BASE + USB_OTG_IN_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USBx_OUTEP(i)   ((USB_OTG_OUTEndpointTypeDef *)(USBx_BASE + USB_OTG_OUT_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USBx_DFIFO(i)   *(__IO uint32_t *)(USBx_BASE + USB_OTG_FIFO_BASE + ((i) * USB_OTG_FIFO_SIZE))

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define LOBYTE(x)  ((uint8_t)((x) & 0x00FFU))
#define HIBYTE(x)  ((uint8_t)(((x) & 0xFF00U) >> 8U))
__STATIC_INLINE uint16_t SWAPBYTE(uint8_t *addr) {
	uint16_t _SwapVal;
	uint16_t _Byte1;
	uint16_t _Byte2;
	uint8_t *_pbuff = addr;

	_Byte1 = *(uint8_t *)_pbuff;
	_pbuff++;
	_Byte2 = *(uint8_t *)_pbuff;

	_SwapVal = (_Byte2 << 8) | _Byte1;

	return _SwapVal;
}

/*!<
 * definitions
 * */
// L1 ========================================= /
#define USBD_MAX_SUPPORTED_CLASS                       1U
#define USBD_MAX_NUM_CONFIGURATION     1U
#define USBD_MAX_NUM_INTERFACES     1U

#define USBD_STATE_DEFAULT                              0x01U
#define USBD_STATE_ADDRESSED                            0x02U
#define USBD_STATE_CONFIGURED                           0x03U
#define USBD_STATE_SUSPENDED                            0x04U

#define DEVICE_FS 		0
#define DEVICE_HS 		1
#define USB_OTG_MODE_DEVICE                    0U

#define USB_FS_MAX_PACKET_SIZE                          64U
#define USB_MAX_EP0_SIZE                                64U
#define HID_FS_BINTERVAL     0xAU

#define USBD_EP_TYPE_CTRL                               0x00U
#define USBD_EP_TYPE_ISOC                               0x01U
#define USBD_EP_TYPE_BULK                               0x02U
#define USBD_EP_TYPE_INTR                               0x03U

#define  USB_DESC_TYPE_DEVICE                           0x01U
#define  USB_DESC_TYPE_CONFIGURATION                    0x02U
#define  USB_DESC_TYPE_STRING                           0x03U
#define  USB_DESC_TYPE_INTERFACE                        0x04U
#define  USB_DESC_TYPE_ENDPOINT                         0x05U
#define  USB_DESC_TYPE_DEVICE_QUALIFIER                 0x06U
#define  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION        0x07U
#define  USB_DESC_TYPE_IAD                              0x0BU
#define  USB_DESC_TYPE_BOS                              0x0FU

#define  USB_LEN_DEV_QUALIFIER_DESC                     0x0AU
#define  USB_LEN_DEV_DESC                               0x12U
#define  USB_LEN_CFG_DESC                               0x09U
#define  USB_LEN_IF_DESC                                0x09U
#define  USB_LEN_EP_DESC                                0x07U
#define  USB_LEN_OTG_DESC                               0x03U
#define  USB_LEN_LANGID_STR_DESC                        0x04U
#define  USB_LEN_OTHER_SPEED_DESC_SIZ                   0x09U

#define USBD_MAX_POWER                                  0x32U /* 100 mA */


// L2 ========================================= /
#define USBD_HS_SPEED                          0U
#define USBD_FS_SPEED                          2U
#define PCD_SPEED_FULL               USBD_FS_SPEED

#define PCD_PHY_EMBEDDED             2U

#define DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ     (0U << 1)
#define DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ     (1U << 1)
#define DSTS_ENUMSPD_FS_PHY_48MHZ              (3U << 1)

#define USBD_HS_TRDT_VALUE                     9U
#define USBD_FS_TRDT_VALUE                     5U
#define USBD_DEFAULT_TRDT_VALUE                9U

// L3 ========================================= /
#define EP_TYPE_CTRL                           0U
#define EP_TYPE_ISOC                           1U
#define EP_TYPE_BULK                           2U
#define EP_TYPE_INTR                           3U
#define EP_TYPE_MSK                            3U

#define USBD_EP0_IDLE                                   0x00U
#define USBD_EP0_SETUP                                  0x01U
#define USBD_EP0_DATA_IN                                0x02U
#define USBD_EP0_DATA_OUT                               0x03U
#define USBD_EP0_STATUS_IN                              0x04U
#define USBD_EP0_STATUS_OUT                             0x05U
#define USBD_EP0_STALL                                  0x06U


#define  USB_REQ_TYPE_STANDARD                          0x00U
#define  USB_REQ_TYPE_CLASS                             0x20U
#define  USB_REQ_TYPE_VENDOR                            0x40U
#define  USB_REQ_TYPE_MASK                              0x60U

#define  USB_REQ_RECIPIENT_DEVICE                       0x00U
#define  USB_REQ_RECIPIENT_INTERFACE                    0x01U
#define  USB_REQ_RECIPIENT_ENDPOINT                     0x02U
#define  USB_REQ_RECIPIENT_MASK                         0x03U

#define  USB_REQ_GET_STATUS                             0x00U
#define  USB_REQ_CLEAR_FEATURE                          0x01U
#define  USB_REQ_SET_FEATURE                            0x03U
#define  USB_REQ_SET_ADDRESS                            0x05U
#define  USB_REQ_GET_DESCRIPTOR                         0x06U
#define  USB_REQ_SET_DESCRIPTOR                         0x07U
#define  USB_REQ_GET_CONFIGURATION                      0x08U
#define  USB_REQ_SET_CONFIGURATION                      0x09U
#define  USB_REQ_GET_INTERFACE                          0x0AU
#define  USB_REQ_SET_INTERFACE                          0x0BU
#define  USB_REQ_SYNCH_FRAME                            0x0CU

#define  USB_DESC_TYPE_DEVICE                           0x01U
#define  USB_DESC_TYPE_CONFIGURATION                    0x02U
#define  USB_DESC_TYPE_STRING                           0x03U
#define  USB_DESC_TYPE_INTERFACE                        0x04U
#define  USB_DESC_TYPE_ENDPOINT                         0x05U
#define  USB_DESC_TYPE_DEVICE_QUALIFIER                 0x06U
#define  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION        0x07U
#define  USB_DESC_TYPE_IAD                              0x0BU
#define  USB_DESC_TYPE_BOS                              0x0FU


// L4 ========================================= /
#define HAL_USB_CURRENT_MODE_MAX_DELAY_MS                           200U
#define USB_FEATURE_EP_HALT                             0x00U
#define USB_FEATURE_REMOTE_WAKEUP                       0x01U
#define USB_FEATURE_TEST_MODE                           0x02U


// L5 ========================================= /
#define HAL_USB_TIMEOUT                                       0xF000000U
#define USB_OTG_SPEED_FULL                     3U

#define  USBD_IDX_LANGID_STR                            0x00U
#define  USBD_IDX_MFC_STR                               0x01U
#define  USBD_IDX_PRODUCT_STR                           0x02U
#define  USBD_IDX_SERIAL_STR                            0x03U
#define  USBD_IDX_CONFIG_STR                            0x04U
#define  USBD_IDX_INTERFACE_STR                         0x05U

#define USB_CONFIG_REMOTE_WAKEUP                        0x02U
#define USB_CONFIG_SELF_POWERED                         0x01U



/*!<
 * enum types
 * */
// L1 ========================================= /
typedef enum {
	HAL_OK       = 0x00U,
	HAL_ERROR    = 0x01U,
	HAL_BUSY     = 0x02U,
	HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

typedef enum {
	USBD_SPEED_HIGH  = 0U,
	USBD_SPEED_FULL  = 1U,
	USBD_SPEED_LOW   = 2U,
} USBD_SpeedTypeDef;

// L2 ========================================= /
typedef enum {
	USBD_OK = 0U,
	USBD_BUSY,
	USBD_EMEM,
	USBD_FAIL,
} USBD_StatusTypeDef;

typedef enum {
	HAL_UNLOCKED = 0x00U,
	HAL_LOCKED   = 0x01U
} HAL_LockTypeDef;

typedef enum {
	HAL_PCD_STATE_RESET   = 0x00,
	HAL_PCD_STATE_READY   = 0x01,
	HAL_PCD_STATE_ERROR   = 0x02,
	HAL_PCD_STATE_BUSY    = 0x03,
	HAL_PCD_STATE_TIMEOUT = 0x04
} PCD_StateTypeDef;

typedef enum {
	LPM_L0 = 0x00, /* on */
	LPM_L1 = 0x01, /* LPM L1 sleep */
	LPM_L2 = 0x02, /* suspend */
	LPM_L3 = 0x03, /* off */
} PCD_LPM_StateTypeDef;

typedef enum {
	PCD_LPM_L0_ACTIVE = 0x00, /* on */
	PCD_LPM_L1_ACTIVE = 0x01, /* LPM L1 sleep */
} PCD_LPM_MsgTypeDef;


// L3 ========================================= /
typedef enum {
	USB_DEVICE_MODE = 0,
	USB_HOST_MODE   = 1,
	USB_DRD_MODE    = 2
} USB_ModeTypeDef;
typedef USB_ModeTypeDef     USB_OTG_ModeTypeDef;



/*!<
 * struct types
 * */
// L1 ========================================= /
typedef struct {
	uint32_t status;
	uint32_t total_length;
	uint32_t rem_length;
	uint32_t maxpacket;
	uint16_t is_used;
	uint16_t bInterval;
} USBD_EndpointTypeDef;

typedef  struct  usb_setup_req {
	uint8_t   bmRequest;
	uint8_t   bRequest;
	uint16_t  wValue;
	uint16_t  wIndex;
	uint16_t  wLength;
} USBD_SetupReqTypedef;

typedef struct {
	uint8_t *(*GetDeviceDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
	uint8_t *(*GetLangIDStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
	uint8_t *(*GetManufacturerStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
	uint8_t *(*GetProductStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
	uint8_t *(*GetSerialStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
	uint8_t *(*GetConfigurationStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
	uint8_t *(*GetInterfaceStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
} USBD_DescriptorsTypeDef;

typedef struct _Device_cb {
	uint8_t (*Init)(struct _USBD_HandleTypeDef *pdev, uint8_t cfgidx);
	uint8_t (*DeInit)(struct _USBD_HandleTypeDef *pdev, uint8_t cfgidx);
	/* Control Endpoints*/
	uint8_t (*Setup)(struct _USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef  *req);
	uint8_t (*EP0_TxSent)(struct _USBD_HandleTypeDef *pdev);
	uint8_t (*EP0_RxReady)(struct _USBD_HandleTypeDef *pdev);
	/* Class Specific Endpoints*/
	uint8_t (*DataIn)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
	uint8_t (*DataOut)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
	uint8_t (*SOF)(struct _USBD_HandleTypeDef *pdev);
	uint8_t (*IsoINIncomplete)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
	uint8_t (*IsoOUTIncomplete)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);

	uint8_t  *(*GetHSConfigDescriptor)(uint16_t *length);
	uint8_t  *(*GetFSConfigDescriptor)(uint16_t *length);
	uint8_t  *(*GetOtherSpeedConfigDescriptor)(uint16_t *length);
	uint8_t  *(*GetDeviceQualifierDescriptor)(uint16_t *length);
} USBD_ClassTypeDef;

typedef struct _USBD_HandleTypeDef {
	uint8_t                 id;
	uint32_t                dev_config;
	uint32_t                dev_default_config;
	uint32_t                dev_config_status;
	USBD_SpeedTypeDef       dev_speed;
	USBD_EndpointTypeDef    ep_in[16];
	USBD_EndpointTypeDef    ep_out[16];
	__IO uint32_t           ep0_state;
	uint32_t                ep0_data_len;
	__IO uint8_t            dev_state;
	__IO uint8_t            dev_old_state;
	uint8_t                 dev_address;
	uint8_t                 dev_connection_status;
	uint8_t                 dev_test_mode;
	uint32_t                dev_remote_wakeup;
	uint8_t                 ConfIdx;

	USBD_SetupReqTypedef    request;
	USBD_DescriptorsTypeDef *pDesc;
	USBD_ClassTypeDef       *pClass[USBD_MAX_SUPPORTED_CLASS];
	void                    *pClassData;
	void                    *pClassDataCmsit[USBD_MAX_SUPPORTED_CLASS];
	void                    *pUserData[USBD_MAX_SUPPORTED_CLASS];
	void                    *pData;
	void                    *pBosDesc;
	void                    *pConfDesc;
	uint32_t                classId;
	uint32_t                NumClasses;
} USBD_HandleTypeDef;


// L2 ========================================= /
typedef USB_OTG_GlobalTypeDef  PCD_TypeDef;

typedef struct {
	uint8_t dev_endpoints;            /*!< Device Endpoints number. This parameter depends on the used USB core. This parameter must be a number between Min_Data = 1 and Max_Data = 15 */
	uint8_t Host_channels;            /*!< Host Channels number. This parameter Depends on the used USB core. This parameter must be a number between Min_Data = 1 and Max_Data = 15 */
	uint8_t dma_enable;              /*!< USB DMA state. If DMA is not supported this parameter shall be set by default to zero */
	uint8_t speed;                   /*!< USB Core speed. This parameter can be any value of @ref PCD_Speed/HCD_Speed (HCD_SPEED_xxx, HCD_SPEED_xxx) */
	uint8_t ep0_mps;                 /*!< Set the Endpoint 0 Max Packet size.                                    */
	uint8_t phy_itface;              /*!< Select the used PHY interface. This parameter can be any value of @ref PCD_PHY_Module/HCD_PHY_Module  */
	uint8_t Sof_enable;              /*!< Enable or disable the output of the SOF signal.                        */
	uint8_t low_power_enable;        /*!< Enable or disable the low Power Mode.                                  */
	uint8_t lpm_enable;              /*!< Enable or disable Link Power Management.                               */
	uint8_t battery_charging_enable; /*!< Enable or disable Battery charging.                                    */
	uint8_t vbus_sensing_enable;     /*!< Enable or disable the VBUS Sensing feature.                            */
	uint8_t use_dedicated_ep1;       /*!< Enable or disable the use of the dedicated EP1 interrupt.              */
	uint8_t use_external_vbus;       /*!< Enable or disable the use of the external VBUS.                        */
} USB_CfgTypeDef;
typedef USB_CfgTypeDef      USB_OTG_CfgTypeDef;
typedef USB_OTG_CfgTypeDef     PCD_InitTypeDef;

typedef struct {
	uint8_t   num;                  /*!< Endpoint number This parameter must be a number between Min_Data = 1 and Max_Data = 15   */
	uint8_t   is_in;                /*!< Endpoint direction This parameter must be a number between Min_Data = 0 and Max_Data = 1    */
	uint8_t   is_stall;             /*!< Endpoint stall condition This parameter must be a number between Min_Data = 0 and Max_Data = 1    */
	uint8_t   is_iso_incomplete;    /*!< Endpoint isoc condition This parameter must be a number between Min_Data = 0 and Max_Data = 1    */
	uint8_t   type;                 /*!< Endpoint type This parameter can be any value of @ref USB_LL_EP_Type                   */
	uint8_t   data_pid_start;       /*!< Initial data PID This parameter must be a number between Min_Data = 0 and Max_Data = 1    */
	uint32_t  maxpacket;            /*!< Endpoint Max packet size This parameter must be a number between Min_Data = 0 and Max_Data = 64KB */
	uint8_t   *xfer_buff;           /*!< Pointer to transfer buffer                                               */
	uint32_t  xfer_len;             /*!< Current transfer length                                                  */
	uint32_t  xfer_count;           /*!< Partial transfer length in case of multi packet transfer                 */
	uint8_t   even_odd_frame;       /*!< IFrame parity This parameter must be a number between Min_Data = 0 and Max_Data = 1    */
	uint16_t  tx_fifo_num;          /*!< Transmission FIFO number This parameter must be a number between Min_Data = 1 and Max_Data = 15   */
	uint32_t  dma_addr;             /*!< 32 bits aligned transfer buffer address                                  */
	uint32_t  xfer_size;            /*!< requested transfer size                                                  */
} USB_EPTypeDef;
typedef USB_EPTypeDef       USB_OTG_EPTypeDef;
typedef USB_OTG_EPTypeDef      PCD_EPTypeDef;

typedef struct {
	PCD_TypeDef             *Instance;   /*!< Register base address             */
	PCD_InitTypeDef         Init;        /*!< PCD required parameters           */
	__IO uint8_t            USB_Address; /*!< USB Address                       */
	PCD_EPTypeDef           IN_ep[16];   /*!< IN endpoint parameters            */
	PCD_EPTypeDef           OUT_ep[16];  /*!< OUT endpoint parameters           */
	HAL_LockTypeDef         Lock;        /*!< PCD peripheral status             */
	__IO PCD_StateTypeDef   State;       /*!< PCD communication state           */
	__IO  uint32_t          ErrorCode;   /*!< PCD Error code                    */
	uint32_t                Setup[12];   /*!< Setup packet buffer               */
	PCD_LPM_StateTypeDef    LPM_State;   /*!< LPM State                         */
	uint32_t                BESL;
	uint32_t                FrameNumber; /*!< Store Current Frame number        */
	uint32_t lpm_active;                 /*!< Enable or disable the Link Power Management. This parameter can be set to ENABLE or DISABLE        */
	uint32_t battery_charging_active;    /*!< Enable or disable Battery charging. This parameter can be set to ENABLE or DISABLE        */
	void                    *pData;      /*!< Pointer to upper stack Handler */
} PCD_HandleTypeDef;


/*!<
 * variables
 * */
// L1 ========================================= /
extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef FS_Desc;  // TODO: init

// L2 ========================================= /
PCD_HandleTypeDef hpcd_USB_OTG_FS;

// HID ======================================== /
extern USBD_ClassTypeDef USBD_HID;  // TODO: init


/*!<
 * init
 * */
void USB_device_init(USB_OTG_GlobalTypeDef*	usb);
extern uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);


#endif // STM32H_CMSIS_USB_F
