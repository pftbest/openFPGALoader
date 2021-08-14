// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2020 Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 */

#ifndef SRC_PIRATEJTAG_HPP_
#define SRC_PIRATEJTAG_HPP_

#include <libusb.h>

#include "jtagInterface.hpp"

/*!
 * \file PirateJtag.hpp
 * \class PirateJtag
 * \brief concrete class between jtag implementation and FTDI capable bitbang mode
 * \author Gwenhael Goavec-Merou
 */

class PirateJtag : public JtagInterface {
 public:
	PirateJtag(uint32_t clkHZ, bool verbose);
	virtual ~PirateJtag();

	int setClkFreq(uint32_t clkHZ) override;

	/* TMS */
	int writeTMS(uint8_t *tms, int len, bool flush_buffer) override;
	/* TDI */
	int writeTDI(uint8_t *tx, uint8_t *rx, uint32_t len, bool end) override;
	/* clk */
	int toggleClk(uint8_t tms, uint8_t tdi, uint32_t clk_len) override;

	/*!
	 * \brief return internal buffer size (in byte).
	 * \return _buffer_size divided by 2 (one state == 2 bits)
	 */
	int get_buffer_size() override { return (2048 - 3) / 2; }

	bool isFull() override { return _tx_bits >= (get_buffer_size() * 8); }

	int flush() override;

 private:
	int append_step(uint8_t tms, uint8_t tdi);
	int write_buffer(uint8_t *tdo);

	bool _verbose;

    libusb_device_handle *dev_handle;
	libusb_context *usb_ctx;
	uint8_t _curr_tms;
	uint8_t _tx_buffer[2048];
	uint8_t _rx_buffer[1023];
	int _tx_bits;
};
#endif  // SRC_PIRATEJTAG_HPP_
