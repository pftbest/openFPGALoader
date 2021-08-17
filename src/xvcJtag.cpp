// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2020 Gwenhael Goavec-Merou
 * <gwenhael.goavec-merou@trabucayre.com>
 */

#include <stdio.h>
#include <string.h>

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

#include <cassert>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "display.hpp"
#include "xvcJtag.hpp"

using namespace std;

XvcJtag::XvcJtag(uint32_t clkHZ, bool verbose)
	: _verbose(verbose), _client_fd(0), _curr_tms(0)
{
	int ret;
	struct sockaddr_in serv_addr;
	struct hostent *server;

	_client_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (_client_fd < 0)
	{
		perror("socket");
		throw std::exception();
	}

	bzero((char *)&serv_addr, sizeof(serv_addr));
	server = gethostbyname("192.168.1.112");
	if (server == NULL)
	{
		cerr << "Failed finding server name" << std::endl;
		throw std::exception();
	}

	serv_addr.sin_family = AF_INET;
	memcpy((char *)&(serv_addr.sin_addr.s_addr), (char *)(server->h_addr),
		   server->h_length);
	serv_addr.sin_port = htons(2542);
	if (connect(_client_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) <
		0)
	{
		cerr << "Failed to connect to server" << std::endl;
		throw std::exception();
	}

	int yes = 1;
	if (setsockopt(_client_fd, IPPROTO_TCP, TCP_NODELAY, (char *)&yes, sizeof(int)) <
		0)
	{
		cerr << "Failed to set nodelay" << std::endl;
		throw std::exception();
	}
}

XvcJtag::~XvcJtag() { close(_client_fd); }

int XvcJtag::setClkFreq(uint32_t clkHZ)
{
	int actual_length;
	int ret, req_freq = clkHZ;

	_clkHZ = clkHZ;

	printInfo("Jtag frequency : requested " + std::to_string(req_freq) +
			  "Hz -> real " + std::to_string(clkHZ) + "Hz");

	return clkHZ;
}

int XvcJtag::writeTMS(uint8_t *tms, int len, bool flush_buffer)
{
	int ret;

	if (_verbose)
		printf("writeTMS len %d flush %d\n", len, flush_buffer);

	/* fill buffer to reduce USB transaction */
	for (int i = 0; i < len; i++)
	{
		ret = append_step(tms[i >> 3] & (1 << (i & 0x07)), 1);
		if (ret < 0)
			return ret;
	}

	/* try to flush buffer */
	if (flush_buffer)
	{
		ret = flush();
		if (ret < 0)
			return ret;
	}

	return len;
}

int XvcJtag::toggleClk(uint8_t tms, uint8_t tdi, uint32_t clk_len)
{
	if (_verbose)
		printf("toggleClk tms %d tdi %d len %d\n", tms, tdi, clk_len);

	for (uint32_t i = 0; i < clk_len; i++)
	{
		append_step(tms, tdi);
	}

	int ret = flush();
	if (ret < 0)
		return ret;

	return clk_len;
}

int XvcJtag::flush() { return write_buffer(NULL); }

int XvcJtag::writeTDI(uint8_t *tx, uint8_t *rx, uint32_t len, bool end)
{
	int ret;
	uint32_t pos = 0;

	if (_verbose)
		printf("writeTDI len %d end %d\n", len, end);

	ret = flush();
	if (ret < 0)
		return ret;

	while (pos < len)
	{
		uint32_t tx_size = std::min(len - pos, (uint32_t)get_buffer_size() * 8);
		for (uint32_t i = 0; i < tx_size; i++)
		{
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

static int sread(int fd, void *data, size_t len)
{
	while (len > 0)
	{
		int ret = read(fd, data, len);
		if (ret <= 0)
			return ret;
		data = (uint8_t *)data + ret;
		len -= ret;
	}
	return 1;
}

static int swrite(int fd, const void *data, size_t len)
{
	while (len > 0)
	{
		int ret = write(fd, data, len);
		if (ret <= 0)
			return ret;
		data = (uint8_t *)data + ret;
		len -= ret;
	}
	return 1;
}

int XvcJtag::write_buffer(uint8_t *tdo)
{
	int ret = 0;
	int actual_length;

	if (_tx_bits == 0)
		return 0;

	int byte_len = (_tx_bits + 7) / 8;
	uint32_t bit_len = _tx_bits;

	ret = swrite(_client_fd, "shift:", 6);
	if (ret <= 0) return ret;

	ret = swrite(_client_fd, &bit_len, 4);
	if (ret <= 0) return ret;

	ret = swrite(_client_fd, _tms_buffer, byte_len);
	if (ret <= 0) return ret;

	ret = swrite(_client_fd, _tdi_buffer, byte_len);
	if (ret <= 0) return ret;

	ret = sread(_client_fd, _tdo_buffer, byte_len);
	if (ret <= 0) return ret;

	if (tdo != nullptr)
	{
		memcpy(tdo, _tdo_buffer, byte_len);

		if (_verbose)
		{
			printf("TDO = ");
			for (int i = 0; i < _tx_bits; i++)
			{
				printf("%d", (tdo[i >> 3] & (1 << (i & 0x07))) != 0);
			}
			printf("\n");
		}
	}

	ret = _tx_bits;
	_tx_bits = 0;

	return ret;
}

int XvcJtag::append_step(uint8_t tms, uint8_t tdi)
{
	int ret = 0;

	_curr_tms = tms;
	unsigned char _tms = tms ? 1 : 0;
	unsigned char _tdi = tdi ? 1 : 0;

	if (isFull())
	{
		ret = flush();
		if (ret < 0)
			return ret;
	}

	if (_verbose)
		printf("TMS = %d TDI = %d\n", _tms, _tdi);

	int byte_index = _tx_bits / 8;
	int bit_index = _tx_bits % 8;

	if (bit_index == 0)
	{
		_tms_buffer[byte_index] = 0;
		_tdi_buffer[byte_index] = 0;
	}

	_tms_buffer[byte_index] |= _tms << bit_index;
	_tdi_buffer[byte_index] |= _tdi << bit_index;
	_tx_bits++;

	return ret;
}
