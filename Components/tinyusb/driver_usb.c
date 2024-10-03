/*
 * driver_usb.c
 *
 *  Created on: Sep 29, 2024
 *      Author: 14769
 */

#include "driver_usb.h"

drvier_usb_rx_cb drvier_usb_rx_cbs[USB_COM_MAX];

void _default_cb(uint8_t* pdata, size_t size)
{
	(void) pdata;
	(void) size;
}

void driver_usb_init()
{
	for (uint8_t itf = 0; itf < USB_COM_MAX; itf++)
	{
		drvier_usb_rx_cbs[itf] = _default_cb;
	}
	tusb_init();
}

void driver_usb_task()
{
	tud_task_ext(1,0);

	for (uint8_t itf = 0; itf < USB_COM_MAX; itf++)
	{
		// connected() check for DTR bit
		// Most but not all terminal client set this when making connection
		// if ( tud_cdc_n_connected(itf) )
		{
		  if (tud_cdc_n_available(itf))
		  {
			uint8_t buf[64];

			uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

			drvier_usb_rx_cbs[itf](buf, count);
		  }
		}
	}
}

void driver_usb_write(usbChannel itf, uint8_t* pdata, size_t len)
{
	tud_cdc_n_write(itf, pdata, len);
	tud_cdc_n_write_flush(itf);
}
