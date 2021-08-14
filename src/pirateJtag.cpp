// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2020 Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 */

#include <libusb.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <cassert>

#include "pirateJtag.hpp"
#include "display.hpp"

using namespace std;

#define PIRATEJTAG_VID 0x1781
#define PIRATEJTAG_PID 0xC0C0

#define PIRATEJTAG_INTF		  0
#define PIRATEJTAG_WRITE_EP    0x02
#define PIRATEJTAG_READ_EP     0x81

/* JTAG usb commands */
#define JTAG_CMD_TAP_OUTPUT     0x0
#define JTAG_CMD_SET_TRST       0x1
#define JTAG_CMD_SET_SRST       0x2
#define JTAG_CMD_READ_INPUT     0x3
#define JTAG_CMD_TAP_OUTPUT_EMU 0x4
#define JTAG_CMD_SET_DELAY      0x5
#define JTAG_CMD_SET_SRST_TRST  0x6
#define JTAG_CMD_READ_CONFIG    0x7

PirateJtag::PirateJtag(uint32_t clkHZ, bool verbose):
			_verbose(verbose),
			dev_handle(NULL), usb_ctx(NULL), _curr_tms(0)
{
	int ret;

	if (libusb_init(&usb_ctx) < 0) {
		cerr << "libusb init failed" << endl;
		throw std::exception();
	}

	dev_handle = libusb_open_device_with_vid_pid(usb_ctx,
					PIRATEJTAG_VID, PIRATEJTAG_PID);
	if (!dev_handle) {
		cerr << "fails to open device" << endl;
		libusb_exit(usb_ctx);
		throw std::exception();
	}

	ret = libusb_claim_interface(dev_handle, PIRATEJTAG_INTF);
	if (ret) {
		cerr << "libusb error while claiming PirateJTAG interface" << endl;
		libusb_close(dev_handle);
		libusb_exit(usb_ctx);
		throw std::exception();
	}

	if (setClkFreq(clkHZ) < 0) {
		cerr << "Fail to set frequency" << endl;
		throw std::exception();
	}
}

PirateJtag::~PirateJtag()
{
	if (dev_handle)
		libusb_close(dev_handle);
	if (usb_ctx)
		libusb_exit(usb_ctx);
}

int PirateJtag::setClkFreq(uint32_t clkHZ)
{
	int actual_length;
	int ret, req_freq = clkHZ;

	_clkHZ = clkHZ;

	printInfo("Jtag frequency : requested " + std::to_string(req_freq) +
			"Hz -> real " + std::to_string(clkHZ) + "Hz");

	return clkHZ;
}

int PirateJtag::writeTMS(uint8_t *tms, int len, bool flush_buffer)
{
	int ret;

	if (_verbose)
		printf("writeTMS len %d flush %d\n", len, flush_buffer);

	/* fill buffer to reduce USB transaction */
	for (int i = 0; i < len; i++) {
		ret = append_step(tms[i >> 3] & (1 << (i & 0x07)), 1);
		if (ret < 0)
			return ret;
	}

	/* try to flush buffer */
	if (flush_buffer) {
		ret = flush();
		if (ret < 0)
			return ret;
	}

	return len;
}

int PirateJtag::toggleClk(uint8_t tms, uint8_t tdi, uint32_t clk_len)
{
	if (_verbose)
		printf("toggleClk tms %d tdi %d len %d\n", tms, tdi, clk_len);

	for (uint32_t i = 0; i < clk_len; i++) {
		append_step(tms, tdi);
	}

	int ret = flush();
	if (ret < 0)
		return ret;

	return clk_len;
}

int PirateJtag::flush()
{
	return write_buffer(NULL);
}

int PirateJtag::writeTDI(uint8_t *tx, uint8_t *rx, uint32_t len, bool end)
{
	int ret;
	uint32_t pos = 0;

	if (_verbose)
		printf("writeTDI len %d end %d\n", len, end);

	ret = flush();
	if (ret < 0)
		return ret;

	while (pos < len) {
		uint32_t tx_size = std::min(len - pos, (uint32_t)get_buffer_size() * 8);
		for (uint32_t i = 0; i < tx_size; i++) {
			if (end && (pos + i == len - 1))
				_curr_tms = 1;
			ret = append_step(_curr_tms, tx[(pos + i) >> 3] & (1 << (i & 0x07)));
			if (ret < 0)
				return ret;
		}
		if (rx != nullptr)
			ret = write_buffer(rx + (pos / 8));
		else
			ret = flush();
		if (ret < 0)
			return ret;
		pos += tx_size;
	}

	return len;
}

int PirateJtag::write_buffer(uint8_t *tdo)
{
	int ret = 0;
	int actual_length;

	if (_tx_bits == 0)
		return 0;

	int tx_bytes = 3 + (_tx_bits + 3) / 4;
	int rx_bytes = (_tx_bits + 7) / 8;
	int remainder = _tx_bits % 4;

	_tx_buffer[0] = (tx_bytes - 2) & 0xff;
	_tx_buffer[1] = ((tx_bytes - 2) >> 8) & 0xff;
	_tx_buffer[2] = JTAG_CMD_TAP_OUTPUT | (remainder << 4);

	ret = libusb_bulk_transfer(dev_handle, PIRATEJTAG_WRITE_EP,
							_tx_buffer, tx_bytes, &actual_length, 1000);
	if (ret < 0 || actual_length != tx_bytes) {
		cerr << "write: usb bulk write failed " << ret << " len " << actual_length << endl;
		return -EXIT_FAILURE;
	}
	ret = libusb_bulk_transfer(dev_handle, PIRATEJTAG_READ_EP,
							_rx_buffer, rx_bytes, &actual_length, 1000);
	if (ret < 0 || actual_length != rx_bytes) {
		cerr << "write: usb bulk read failed " << ret << " len " << actual_length << endl;
		return -EXIT_FAILURE;
	}

	if (tdo != nullptr) {
		memcpy(tdo, _rx_buffer, rx_bytes);

		if (_verbose) {
			printf("TDO = ");
			for (int i = 0; i < _tx_bits; i++) {
				printf("%d", (tdo[i >> 3] & (1 << (i & 0x07))) != 0);
			}
			printf("\n");
		}
	}

	ret = _tx_bits;
	_tx_bits = 0;

	return ret;
}

int PirateJtag::append_step(uint8_t tms, uint8_t tdi) {
	int ret = 0;

	_curr_tms = tms;
	unsigned char _tms = tms ? 1 : 0;
	unsigned char _tdi = tdi ? 1 : 0;

	if (isFull()) {
		ret = flush();
		if (ret < 0)
			return ret;
	}

	if (_verbose)
		printf("TMS = %d TDI = %d\n", _tms, _tdi);

	int tap_index = _tx_bits / 4 + 3;
	int bits = (_tx_bits % 4) * 2;

	if (!bits)
		_tx_buffer[tap_index] = 0;

	_tx_buffer[tap_index] |= (_tdi << bits)|(_tms << (bits + 1));
	_tx_bits++;

	return ret;
}
