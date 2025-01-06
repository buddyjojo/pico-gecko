// SPDX-License-Identifier: MIT
/*
 * Copyright 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 * Copyright (c) 2022 Nicolai Electronics
 * Copyright (c) 2023 Chris Burton
 */

#include "bsp/board.h"
#include <hardware/irq.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <hardware/structs/pio.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <string.h>
#include <tusb.h>
#include <exitx.pio.h>
#include <exirx.pio.h>
#include "serial.h"

#define PIN_DO   3
#define PIN_DI   4

#if !defined(MIN)
#define MIN(a, b) ((a > b) ? b : a)
#endif /* MIN */

// might as well use our RAM
#define BUFFER_SIZE 2560

// activity LED on duration
#define LED_TICKER_COUNT 500

#define DEF_BIT_RATE 1500000
#define DEF_STOP_BITS 1
#define DEF_PARITY 0
#define DEF_DATA_BITS 8

uint    sm = 0;
uint8_t led_act_pin = 25;

cdc_line_coding_t usb_lc;
cdc_line_coding_t uart_lc;
mutex_t lc_mtx;
uint8_t uart_rx_buffer[BUFFER_SIZE];
uint32_t uart_rx_pos;
uint8_t uart_to_usb_buffer[BUFFER_SIZE];
uint32_t uart_to_usb_pos;
mutex_t uart_mtx;
uint8_t usb_to_uart_buffer[BUFFER_SIZE];
uint32_t usb_to_uart_pos;
uint32_t usb_to_uart_snd;
mutex_t usb_mtx;
uint32_t led_act_ticker;

uint rx_offset=0;
uint rxp_offset=0;

uint tx_offset=0;
uint txp_offset=0;

void usb_read_bytes() {
	uint32_t len = tud_cdc_n_available(0);

	if (len) {

		mutex_enter_blocking(&usb_mtx);

		len = MIN(len, BUFFER_SIZE - usb_to_uart_pos);
		if (len) {
			uint32_t count;

			count = tud_cdc_n_read(0, &usb_to_uart_buffer[usb_to_uart_pos], len);
			usb_to_uart_pos += count;
		}

		mutex_exit(&usb_mtx);
	}
}

void usb_write_bytes() {

	if (uart_to_usb_pos && mutex_try_enter(&uart_mtx, NULL)) {
		uint32_t count;

		count = tud_cdc_n_write(0, uart_to_usb_buffer, uart_to_usb_pos);
		if (count < uart_to_usb_pos)
			memcpy(uart_to_usb_buffer, &uart_to_usb_buffer[count],
			      uart_to_usb_pos - count);
		uart_to_usb_pos -= count;

		mutex_exit(&uart_mtx);

		if (count)
			tud_cdc_n_write_flush(0);
	}
}

void usb_cdc_process()
{

	mutex_enter_blocking(&lc_mtx);
	tud_cdc_n_get_line_coding(0, &usb_lc);
	mutex_exit(&lc_mtx);

	usb_read_bytes();
	usb_write_bytes();
}

void core1_entry(void)
{
	tusb_init();

	while (1) {

		tud_task();

		if (tud_cdc_n_connected(0)) {
			usb_cdc_process();
		}
	}
}

void uart_read_bytes()
{

	if (!pio_sm_is_rx_fifo_empty(pio0, sm)) {
		while (!pio_sm_is_rx_fifo_empty(pio0, sm) &&
				uart_rx_pos < BUFFER_SIZE) {
			uart_rx_buffer[uart_rx_pos] =  pio_sm_get_blocking(pio0, sm);
			uart_rx_pos++;
			led_act_ticker = LED_TICKER_COUNT;
		}
	}
	// If we can get the uart mutex then copy the UART data to the uart USB sender, otherwise we'll get it next time around
	if (mutex_try_enter(&uart_mtx, NULL)) {
		// Ensure we don't overflow the uart_to_usb_buffer
		uint32_t len = MIN(uart_rx_pos, BUFFER_SIZE - uart_to_usb_pos);
		memcpy(&uart_to_usb_buffer[uart_to_usb_pos], uart_rx_buffer, len);
		uart_to_usb_pos += len;
		uart_rx_pos = 0;
		mutex_exit(&uart_mtx);
	}

	if (led_act_ticker) {
		gpio_put(led_act_pin, 1);
		led_act_ticker--;
	} else {
		gpio_put(led_act_pin, 0);
	}
}

void uart_write_bytes() {

	// Try to get the usb_mutex and don't block if we cannot get it, we'll TX the data next passs
	if ((usb_to_uart_pos) && (usb_to_uart_snd < usb_to_uart_pos) &&
	    mutex_try_enter(&usb_mtx, NULL)) {

		led_act_ticker = LED_TICKER_COUNT;

		size_t bufspace=7-pio_sm_get_tx_fifo_level(pio0,1);
		size_t tosend=usb_to_uart_pos-usb_to_uart_snd;
		tosend = MIN(tosend,bufspace);

		for (size_t i = 0; i<tosend; ++i) {
			exi_put(pio0, 1, usb_to_uart_buffer[usb_to_uart_snd+i]);
		}
		usb_to_uart_snd+=tosend;
		// only reset buffers if we've sent everything
		if (usb_to_uart_snd == usb_to_uart_pos) {
			usb_to_uart_pos = 0;
			usb_to_uart_snd = 0;
		}
		mutex_exit(&usb_mtx);
	}
	if (led_act_ticker) {
		gpio_put(led_act_pin, 1);
		led_act_ticker--;
	} else {
		gpio_put(led_act_pin, 0);
	}
}

void init_uart_data() {

	/* USB CDC LC */
	usb_lc.bit_rate = DEF_BIT_RATE;
	usb_lc.data_bits = DEF_DATA_BITS;
	usb_lc.parity = DEF_PARITY;
	usb_lc.stop_bits = DEF_STOP_BITS;

	/* UART LC */
	uart_lc.bit_rate = DEF_BIT_RATE;
	uart_lc.data_bits = DEF_DATA_BITS;
	uart_lc.parity = DEF_PARITY;
	uart_lc.stop_bits = DEF_STOP_BITS;

	/* Buffer */
	uart_rx_pos = 0;
	uart_to_usb_pos = 0;
	usb_to_uart_pos = 0;
	usb_to_uart_snd = 0;

	/* Mutex */
	mutex_init(&lc_mtx);
	mutex_init(&uart_mtx);
	mutex_init(&usb_mtx);

	/* Activity LED */
	gpio_init(led_act_pin);
	gpio_set_dir(led_act_pin, GPIO_OUT);
	gpio_put(led_act_pin, 0);
	led_act_ticker = 0;

	// Set up the state machine we're going to use to for rx/tx
	exirx_program_init(pio0, 0, rx_offset, PIN_DI, PIN_DO, true);
	exitx_program_init(pio0, 1, tx_offset, PIN_DI, PIN_DO, true);
}

bool tud_vendor_control_complete_cb(uint8_t rhport, tusb_control_request_t const* request) {
    (void) rhport;
    (void) request;
    return true;
}

int main(void)
{
	set_sys_clock_khz(250000, true);

	// store our PIO programs in the instruction registers
	rx_offset = pio_add_program(pio0, &exirx_program);
	tx_offset = pio_add_program(pio0, &exitx_program);

	board_init();

	init_uart_data();

	multicore_launch_core1(core1_entry);

	while (1) {
		uart_write_bytes();
		uart_read_bytes();
	}

	return 0;
}

