/*
 * driver_usb.h
 *
 *  Created on: Sep 29, 2024
 *      Author: 14769
 */

#ifndef TINYUSB_DRIVER_USB_H_
#define TINYUSB_DRIVER_USB_H_

#include "tusb.h"

typedef enum
{
	USB_COMDATA_ITF = 0,
	USB_COMCMD_ITF,
	USB_COM_MAX
}usbChannel;

typedef void (*drvier_usb_rx_cb)(uint8_t*, size_t);

extern drvier_usb_rx_cb drvier_usb_rx_cbs[USB_COM_MAX];

void driver_usb_init();

void driver_usb_task();

void driver_usb_write(usbChannel itf, uint8_t* pdata, size_t len);

#endif /* TINYUSB_DRIVER_USB_H_ */
