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

#include "usb_api_register.h"
#include <libopencm3/lpc43xx/gpio.h>
#include "hackrf_core.h"

#include <usb_queue.h>
#include "rf_path.h"

#include <stddef.h>
#include <stdint.h>

#include "sgpio.h"
#include "sgpio_isr.h"
#include "mcp3021.h"
#include "adchs.h"

uint8_t mcp3021_result[2];

usb_request_status_t usb_vendor_request_write_source(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    uint32_t data = (endpoint->setup.value<<16)|(endpoint->setup.index);

	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        source_write_register(data);
        usb_transfer_schedule_ack(endpoint->in);
        return USB_REQUEST_STATUS_OK;
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_write_lo(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    uint32_t data = (endpoint->setup.value<<16)|(endpoint->setup.index);

	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        lo_write_register(data);
        usb_transfer_schedule_ack(endpoint->in);
        return USB_REQUEST_STATUS_OK;
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_write_att(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    uint32_t data = (endpoint->setup.value<<16)|(endpoint->setup.index);

	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        att_write_register(data);
        usb_transfer_schedule_ack(endpoint->in);
        return USB_REQUEST_STATUS_OK;
	}
    return USB_REQUEST_STATUS_OK;
}


usb_request_status_t usb_vendor_request_set_tx_port(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    uint8_t data = (endpoint->setup.value & 0xFF);

	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        set_tx_port(data);
        usb_transfer_schedule_ack(endpoint->in);
        return USB_REQUEST_STATUS_OK;
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_rx_ch(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    uint8_t data = (endpoint->setup.value & 0xFF);

	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        set_rx_channel(data);
        usb_transfer_schedule_ack(endpoint->in);
        return USB_REQUEST_STATUS_OK;
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_filter(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    uint8_t data = (endpoint->setup.value & 0xFF);

	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        set_filter(data);
        usb_transfer_schedule_ack(endpoint->in);
        return USB_REQUEST_STATUS_OK;
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_gpio(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    uint32_t data = (endpoint->setup.value<<16)|(endpoint->setup.index);

	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        if (data & BIT0) {
            enable_signal();
        }
        if (data & BIT1) {
            ;
        }
        if (data & BIT2) {
            enable_pa();
        }
        if (data & BIT3) {
            enable_mixer();
        }
        if (data & BIT4) {
            gpio_set(PORT_LED1_3, PIN_LED1);
        }
        if (data & BIT5) {
            ;
        }
        usb_transfer_schedule_ack(endpoint->in);
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_clear_gpio(
    usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    uint32_t data = (endpoint->setup.value<<16)|(endpoint->setup.index);

	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        if (data & BIT0) {
            disable_signal();
        }
        if (data & BIT1) {
            ;
        }
        if (data & BIT2) {
            disable_pa();
        }
        if (data & BIT3) {
            disable_mixer();
        }
        if (data & BIT4) {
            gpio_clear(PORT_LED1_3, PIN_LED1);
        }
        usb_transfer_schedule_ack(endpoint->in);
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_read_mcp3021(
	usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{

	if (stage == USB_TRANSFER_STAGE_SETUP)
	{
        //mcp3021_start_conversion();
        uint16_t result = mcp3021_read_result();
        mcp3021_result[0] = result >> 8;
        mcp3021_result[1] = result & 0xFF;
        usb_transfer_schedule_block(endpoint->in, &mcp3021_result, 2, NULL, NULL);
		usb_transfer_schedule_ack(endpoint->out);
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_sample(
	usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{

	if (stage == USB_TRANSFER_STAGE_SETUP)
	{
        //ADCHS_restart_dma();
        sample();
        usb_transfer_schedule_ack(endpoint->in);
	}
    return USB_REQUEST_STATUS_OK;
}
