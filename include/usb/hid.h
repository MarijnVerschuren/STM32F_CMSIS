//
// Created by marijn on 4/2/24.
//

#ifndef STM32F_CMSIS_USB_HAL_CPY_HID_H
#define STM32F_CMSIS_USB_HAL_CPY_HID_H


void USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);


#endif // STM32F_CMSIS_USB_HAL_CPY_HID_H
