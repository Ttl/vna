/*
 * Copyright 2012 Jared Boone
 * Copyright 2013 Benjamin Vernoux
 *
 * This file is part of HackRF.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <stdint.h>
#include <stddef.h>
#include "signal_mcu.h"
#include "m0_bin.h"

#include <libopencm3/cm3/vector.h>

#include <libopencm3/lpc43xx/ipc.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/m4/nvic.h>

#include <streaming.h>

#include "usb.h"
#include "usb_standard_request.h"

#include <rom_iap.h>
#include "usb_descriptor.h"

#include "usb_device.h"
#include "usb_endpoint.h"
#include "usb_api_board_info.h"
#include "usb_api_register.h"
#include "usb_api_spiflash.h"

#include "usb_api_transceiver.h"
#include "rf_path.h"
#include "sgpio_isr.h"
#include "usb_bulk_buffer.h"
#include "sgpio.h"

#define SLAVE_TXEV_FLAG ((uint32_t *) 0x40043400)
#define SLAVE_TXEV_QUIT() { *SLAVE_TXEV_FLAG = 0x0; }

static volatile transceiver_mode_t _transceiver_mode = TRANSCEIVER_MODE_OFF;

static unsigned int phase = 1;

void set_transceiver_mode(const transceiver_mode_t new_transceiver_mode) {
	baseband_streaming_disable();

	usb_endpoint_disable(&usb_endpoint_bulk_in);
	usb_endpoint_disable(&usb_endpoint_bulk_out);

	_transceiver_mode = new_transceiver_mode;

	if( _transceiver_mode == TRANSCEIVER_MODE_RX ) {
        usb_endpoint_init(&usb_endpoint_bulk_in);
        usb_bulk_buffer_offset = 0;
        phase = 1;
	} else if (_transceiver_mode == TRANSCEIVER_MODE_TX) {
		//usb_endpoint_init(&usb_endpoint_bulk_out);
	}

	if( _transceiver_mode != TRANSCEIVER_MODE_OFF ) {
		baseband_streaming_enable();
	}
}

transceiver_mode_t transceiver_mode(void) {
	return _transceiver_mode;
}

usb_request_status_t usb_vendor_request_set_transceiver_mode(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
	if( stage == USB_TRANSFER_STAGE_SETUP ) {
		switch( endpoint->setup.value ) {
		case TRANSCEIVER_MODE_OFF:
		case TRANSCEIVER_MODE_RX:
		case TRANSCEIVER_MODE_TX:
			set_transceiver_mode(endpoint->setup.value);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		default:
			return USB_REQUEST_STATUS_STALL;
		}
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

static const usb_request_handler_fn vendor_request_handler[] = {
	NULL, //0
	usb_vendor_request_set_transceiver_mode, //1
    usb_vendor_request_write_lo, //2
    usb_vendor_request_write_source, //3
    usb_vendor_request_set_gpio, //4
    usb_vendor_request_clear_gpio, //5
    usb_vendor_request_set_filter, //6
    usb_vendor_request_set_rx_ch, //7
    usb_vendor_request_set_tx_port, //8
    usb_vendor_request_write_att, //9
	usb_vendor_request_erase_spiflash, //10
	usb_vendor_request_write_spiflash, //11
	usb_vendor_request_read_spiflash, //12
	NULL,
	usb_vendor_request_read_board_id, //14
	usb_vendor_request_read_version_string, //15
    NULL,
    NULL,
	usb_vendor_request_read_partid_serialno, //18
    usb_vendor_request_read_mcp3021, //19
    NULL,
    NULL,
	NULL,
	NULL,
    NULL,
};

static const uint32_t vendor_request_handler_count =
	sizeof(vendor_request_handler) / sizeof(vendor_request_handler[0]);

usb_request_status_t usb_vendor_request(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
	usb_request_status_t status = USB_REQUEST_STATUS_STALL;

	if( endpoint->setup.request < vendor_request_handler_count ) {
		usb_request_handler_fn handler = vendor_request_handler[endpoint->setup.request];
		if( handler ) {
			status = handler(endpoint, stage);
		}
	}

	return status;
}

const usb_request_handlers_t usb_request_handlers = {
	.standard = usb_standard_request,
	.class = 0,
	.vendor = usb_vendor_request,
	.reserved = 0,
};

void usb_configuration_changed(
	usb_device_t* const device
) {
	/* Reset transceiver to idle state until other commands are received */
	set_transceiver_mode(TRANSCEIVER_MODE_OFF);
	if( device->configuration->number == 1 ) {
		// transceiver configuration
		cpu_clock_pll1_max_speed();
	} else {
		/* Configuration number equal 0 means usb bus reset. */
		cpu_clock_pll1_low_speed();
		//gpio_clear(PORT_LED1_3, PIN_LED1);
	}
}

void usb_set_descriptor_by_serial_number(void)
{
	iap_cmd_res_t iap_cmd_res;

	/* Read IAP Serial Number Identification */
	iap_cmd_res.cmd_param.command_code = IAP_CMD_READ_SERIAL_NO;
	iap_cmd_call(&iap_cmd_res);

	if (iap_cmd_res.status_res.status_ret == CMD_SUCCESS) {
		usb_descriptor_string_serial_number[0] = USB_DESCRIPTOR_STRING_SERIAL_BUF_LEN;
		usb_descriptor_string_serial_number[1] = USB_DESCRIPTOR_TYPE_STRING;

		/* 32 characters of serial number, convert to UTF-16LE */
		for (size_t i=0; i<USB_DESCRIPTOR_STRING_SERIAL_LEN; i++) {
			const uint_fast8_t nibble = (iap_cmd_res.status_res.iap_result[i >> 3] >> (28 - (i & 7) * 4)) & 0xf;
			const char c = (nibble > 9) ? ('a' + nibble - 10) : ('0' + nibble);
			usb_descriptor_string_serial_number[2 + i * 2] = c;
			usb_descriptor_string_serial_number[3 + i * 2] = 0x00;
		}
	} else {
		usb_descriptor_string_serial_number[0] = 2;
		usb_descriptor_string_serial_number[1] = USB_DESCRIPTOR_TYPE_STRING;
	}
}

//void m0_startup(void)
//{
//  uint32_t *src, *dest;
//
//  /* Halt M0 core (in case it was running) */
//  ipc_halt_m0();
//
//  /* Copy M0 code from M4 embedded addr to final addr M0 */
//  dest = &cm0_exec_baseaddr;
//  for(src = (uint32_t *)&m0_bin[0]; src < (uint32_t *)(&m0_bin[0]+m0_bin_size); )
//  {
//    *dest++ = *src++;
//  }
//
//  ipc_start_m0( (uint32_t)(&cm0_exec_baseaddr) );
//}

//void m0s_startup(void)
//{
//  uint32_t *src, *dest;
//
//  /* Halt M0 core (in case it was running) */
//  ipc_halt_m0s();
//
//  /* Copy M0 code from M4 embedded addr to final addr M0 */
//  dest = &cm0s_exec_baseaddr;
//  for(src = (uint32_t *)&m0s_bin[0]; src < (uint32_t *)(&m0s_bin[0]+m0s_bin_size); )
//  {
//    *dest++ = *src++;
//  }
//
//  ipc_start_m0s( (uint32_t)(&cm0s_exec_baseaddr) );
//}

int main(void) {
	pin_setup();
    gpio_set(PORT_LED1_3, PIN_LED1);

	cpu_clock_init();

    //m0_startup();

    ipc_halt_m0s();

    // Disable M0 Sub
    CCU1_CLK_PERIPH_CORE_CFG &= ~(1);

	usb_set_descriptor_by_serial_number();

	usb_set_configuration_changed_cb(usb_configuration_changed);

	usb_peripheral_reset();

	usb_device_init(0, &usb_device);

	usb_queue_init(&usb_endpoint_control_out_queue);
	usb_queue_init(&usb_endpoint_control_in_queue);
    usb_queue_init(&usb_endpoint_bulk_out_queue);
	usb_queue_init(&usb_endpoint_bulk_in_queue);

	usb_endpoint_init(&usb_endpoint_control_out);
	usb_endpoint_init(&usb_endpoint_control_in);

	nvic_set_priority(NVIC_USB0_IRQ, 255);
    //vector_table.irq[NVIC_SGPIO_IRQ] = sgpio_isr_rx;

	usb_run(&usb_device);
	ssp1_init();

    //sgpio_configure();

	while(true) {

		// Set up IN transfer of buffer 0.
		if ( usb_bulk_buffer_offset >= 16384
		     && phase == 1
		     && transceiver_mode() != TRANSCEIVER_MODE_OFF) {
			usb_transfer_schedule_block(
				(transceiver_mode() == TRANSCEIVER_MODE_RX)
				? &usb_endpoint_bulk_in : &usb_endpoint_bulk_out,
				&usb_bulk_buffer[0x0000],
				0x4000,
				NULL, NULL
				);
			phase = 0;
		}

		// Set up IN transfer of buffer 1.
		if ( usb_bulk_buffer_offset < 16384
		     && phase == 0
		     && transceiver_mode() != TRANSCEIVER_MODE_OFF) {
			usb_transfer_schedule_block(
				(transceiver_mode() == TRANSCEIVER_MODE_RX)
				? &usb_endpoint_bulk_in : &usb_endpoint_bulk_out,
				&usb_bulk_buffer[0x4000],
				0x4000-32, // Last 32 bytes are unused
				NULL, NULL
			);
			phase = 1;
		}
	}

	return 0;
}
