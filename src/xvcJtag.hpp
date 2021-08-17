// SPDX-License-Identifier: Apache-2.0

#ifndef SRC_XVC_JTAG_HPP_
#define SRC_XVC_JTAG_HPP_

#include "jtagInterface.hpp"

/*!
 * \file XvcJtag.hpp
 * \class XvcJtag
 * \brief concrete class between jtag implementation and FTDI capable bitbang
 * mode \author Gwenhael Goavec-Merou
 */

class XvcJtag : public JtagInterface
{
public:
	XvcJtag(uint32_t clkHZ, bool verbose);
	virtual ~XvcJtag();

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
	int get_buffer_size() override { return BUFFER_SIZE; }

	bool isFull() override { return _tx_bits >= BUFFER_SIZE; }

	int flush() override;

private:
	static constexpr size_t BUFFER_SIZE = 16384;

	int append_step(uint8_t tms, uint8_t tdi);
	int write_buffer(uint8_t *tdo);

	bool _verbose;
	int _client_fd;
	uint8_t _curr_tms;
	uint8_t _tms_buffer[BUFFER_SIZE];
	uint8_t _tdi_buffer[BUFFER_SIZE];
	uint8_t _tdo_buffer[BUFFER_SIZE];
	int _tx_bits;
};
#endif // SRC_XVC_JTAG_HPP_
