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
#define __ALIGN_BEGIN
#define __ALIGN_END

/*!<
 * definitions
 * */
// L1 ========================================= /  // TODO: typedef enum
#define USBD_MAX_SUPPORTED_CLASS                       1U
#define USBD_MAX_NUM_CONFIGURATION     1U
#define USBD_MAX_NUM_INTERFACES     1U

#define USBD_STATE_DEFAULT                              0x01U
#define USBD_STATE_ADDRESSED                            0x02U
#define USBD_STATE_CONFIGURED                           0x03U
#define USBD_STATE_SUSPENDED                            0x04U

#define USB_OTG_MODE_DEVICE                    0U

#define USB_MAX_EP0_SIZE                                64U
#define HID_FS_BINTERVAL     0xAU

#define USBD_EP_TYPE_CTRL                               0x00U
#define USBD_EP_TYPE_ISOC                               0x01U
#define USBD_EP_TYPE_BULK                               0x02U
#define USBD_EP_TYPE_INTR                               0x03U


#define USBD_MAX_POWER                                  0x32U /* 100 mA */


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


// L4 ========================================= /
#define USB_FEATURE_EP_HALT                             0x00U
#define USB_FEATURE_REMOTE_WAKEUP                       0x01U
#define USB_FEATURE_TEST_MODE                           0x02U


/*!<
 * enum types
 * */
typedef enum {
   USB_DEVICE_DESCRIPTOR =			 	0x1U,
   USB_CONFIG_DESCRIPTOR =			 	0x2U,
   USB_STRING_DESCRIPTOR =			 	0x3U,
   USB_INTERFACE_DESCRIPTOR =			0x4U,
   USB_ENDPOINT_DESCRIPTOR =			0x5U,
   USB_QUALIFIER_DESCRIPTOR =			0x6U,
   USB_OTHER_SPEED_DESCRIPTOR =			0x7U,
   USB_IAD_DESCRIPTOR =					0xBU,
   USB_BOS_DESCRIPTOR =					0xFU
} USB_descriptor_type_t;
typedef enum {
	USB_LANGUAGE_ID_STRING_DESCRIPTOR =	0X0U,
	USB_MANUFACTURER_STRING_DESCRIPTOR=	0X1U,
	USB_PRODUCT_STRING_DESCRIPTOR =		0X2U,
	USB_SERIAL_STRING_DESCRIPTOR =		0X3U,
	USB_CONFIG_STRING_DESCRIPTOR =		0X4U,
	USB_INTERFACE_STRING_DESCRIPTOR =	0X5U,
} USB_string_descriptor_type_t;

typedef enum {
	STANDARD_REQUEST =		0b00,
	CLASS_REQUEST =			0b01,
	VENDOR_REQUEST =		0b10
}	SETUP_request_type_t;
typedef enum {
	RECIPIANT_DEVICE =		0b00000,
	RECIPIANT_INTERFACE =	0b00001,
	RECIPIANT_ENDPOINT =	0b00010
}	SETUP_recipiant_t;
typedef enum {
	GET_STATUS =			0x00,
	CLEAR_FEATURE =			0x01,
	SET_FEATURE =			0x03,
	SET_ADDRESS =			0x05,
	GET_DESCRIPTOR =		0x06,
	SET_DESCRIPTOR =		0x07,
	GET_CONFIGURATION =		0x08,
	SET_CONFIGURATION =		0x09,
	GET_INTERFACE =			0x0A,
	SET_INTERFACE =			0x0B
}	SETUP_command_t;


/*!<
 * struct types
 * */
// L0.0
typedef __PACKED_STRUCT {
	uint32_t EPNUM		: 4;	// endpoint number
	uint32_t BCNT		: 11;	// byte count
	uint32_t DPID		: 2;	// data PID
	uint32_t PKTSTS		: 4;	// packet status
	uint32_t _			: 11;
}	GRXSTS_t;

typedef __PACKED_STRUCT {
	SETUP_recipiant_t		    recipiant	: 5;		// |
	SETUP_request_type_t	    type		: 2;		// | bmRequest
	uint8_t					    direction	: 1;		// |  // TODO: MSB?!!!!!!!!!!!!!!!!!!
	SETUP_command_t			    command;				// bRequest
	uint16_t				    value;					// wValue
	uint16_t				    index;					// wIndex
	uint16_t				    length;					// wLength
}	setup_header_t;


// L1 ========================================= /
typedef struct {
	uint32_t status;
	uint32_t total_length;
	uint32_t rem_length;
	uint32_t maxpacket;
	uint16_t is_used;
	uint16_t bInterval;
} USBD_EndpointTypeDef;

typedef struct {
	uint8_t *(*GetDeviceDescriptor)(uint16_t *length);
	uint8_t *(*GetLangIDStrDescriptor)(uint16_t *length);
	uint8_t *(*GetManufacturerStrDescriptor)(uint16_t *length);
	uint8_t *(*GetProductStrDescriptor)(uint16_t *length);
	uint8_t *(*GetSerialStrDescriptor)(uint16_t *length);
	uint8_t *(*GetConfigurationStrDescriptor)(uint16_t *length);
	uint8_t *(*GetInterfaceStrDescriptor)(uint16_t *length);
} USBD_DescriptorsTypeDef;

typedef struct _Device_cb {
	uint8_t (*Init)(struct _USBD_HandleTypeDef *pdev, uint8_t cfgidx);
	void (*DeInit)(struct _USBD_HandleTypeDef *pdev, uint8_t cfgidx);
	/* Control Endpoints*/
	void (*Setup)(struct _USBD_HandleTypeDef *pdev, setup_header_t* req);
	void (*EP0_TxSent)(struct _USBD_HandleTypeDef *pdev);
	void (*EP0_RxReady)(struct _USBD_HandleTypeDef *pdev);
	/* Class Specific Endpoints*/
	void (*DataIn)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
	void (*DataOut)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
	void (*SOF)(struct _USBD_HandleTypeDef *pdev);
	void (*IsoINIncomplete)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
	void (*IsoOUTIncomplete)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);

	uint8_t  *(*GetFSConfigDescriptor)(uint16_t *length);
} USBD_ClassTypeDef;

typedef struct _USBD_HandleTypeDef {
	uint32_t                dev_config;
	uint32_t                dev_default_config;
	uint32_t                dev_config_status;
	USBD_EndpointTypeDef    ep_in[16];
	USBD_EndpointTypeDef    ep_out[16];
	__IO uint32_t           ep0_state;
	uint32_t                ep0_data_len;
	__IO uint8_t            dev_state;
	__IO uint8_t            dev_old_state;
	uint32_t                dev_remote_wakeup;

	setup_header_t			header;
	USBD_DescriptorsTypeDef *pDesc;
	USBD_ClassTypeDef       *pClass;
	void                    *pData;
} USBD_HandleTypeDef;


// L2 ========================================= /
typedef struct {
	uint8_t dev_endpoints;            /*!< Device Endpoints number. This parameter depends on the used USB core. This parameter must be a number between Min_Data = 1 and Max_Data = 15 */
	uint8_t ep0_mps;                 /*!< Set the Endpoint 0 Max Packet size.                                    */
	uint8_t Sof_enable;              /*!< Enable or disable the output of the SOF signal.                        */
	uint8_t low_power_enable;        /*!< Enable or disable the low Power Mode.                                  */
	uint8_t battery_charging_enable; /*!< Enable or disable Battery charging.                                    */
	uint8_t vbus_sensing_enable;     /*!< Enable or disable the VBUS Sensing feature.                            */
} USB_CfgTypeDef;

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

typedef struct {
	USB_OTG_GlobalTypeDef*	Instance;   /*!< Register base address             */
	USB_CfgTypeDef			Init;        /*!< PCD required parameters           */
	__IO uint8_t            USB_Address; /*!< USB Address                       */
	USB_EPTypeDef           IN_ep[16];   /*!< IN endpoint parameters            */
	USB_EPTypeDef           OUT_ep[16];  /*!< OUT endpoint parameters           */
	__IO  uint32_t          ErrorCode;   /*!< PCD Error code                    */
	uint32_t                Setup[12];   /*!< Setup packet buffer               */
	uint32_t                FrameNumber; /*!< Store Current Frame number        */
	uint32_t battery_charging_active;    /*!< Enable or disable Battery charging. This parameter can be set to ENABLE or DISABLE        */
	void                    *pData;      /*!< Pointer to upper stack Handler */
} PCD_HandleTypeDef;


/*!<
 * variables
 * */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef FS_Desc;  // TODO: init
PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_ClassTypeDef USBD_HID;  // TODO: init


/*!<
 * init
 * */
void USB_device_init(USB_OTG_GlobalTypeDef*	usb);


#endif // STM32H_CMSIS_USB_F
