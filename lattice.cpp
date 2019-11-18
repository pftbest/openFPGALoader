/*
 * Copyright (C) 2019 Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <strings.h>
#include <string.h>
#include <unistd.h>

#include <iostream>

#include "ftdijtag.hpp"
#include "lattice.hpp"
#include "progressBar.hpp"

using namespace std;

#define ISC_ENABLE				0xc6
#  define ISC_ENABLE_FLASH_MODE	(1 << 3)
#  define ISC_ENABLE_SRAM_MODE	(0 << 3)
#define ISC_DISABLE				0x26
#define READ_DEVICE_ID_CODE		0xE0
#define FLASH_ERASE				0x0E
#  define FLASH_ERASE_UFM		(1<<3)
#  define FLASH_ERASE_CFG   	(1<<2)
#  define FLASH_ERASE_FEATURE	(1<<1)
#  define FLASH_ERASE_SRAM		(1<<0)
#  define FLASH_ERASE_ALL       0x0F
#define CHECK_BUSY_FLAG			0xF0
#  define CHECK_BUSY_FLAG_BUSY	(1 << 7)
#define RESET_CFG_ADDR			0x46
#define PROG_CFG_FLASH			0x70
#define PROG_FEATURE_ROW		0xE4
#define PROG_FEABITS			0xF8
#define PROG_DONE				0x5E
#define REFRESH					0x79

#define READ_FEATURE_ROW 0xE7
#define READ_FEABITS     0xFB
#define READ_STATUS_REGISTER 0x3C

Lattice::Lattice(FtdiJtag *jtag, const string filename):Device(jtag, filename)
{
	if (_filename != "")
		if (_file_extension == "jed")
			_mode = Device::FLASH_MODE;
}


void displayFeabits(uint16_t _featbits)
{
	uint8_t boot_sequence = (_featbits >> 12) & 0x03;
	uint8_t m = (_featbits >> 11) & 0x01;
	printf("\tboot mode                                :");
	switch (boot_sequence) {
		case 0:
			if (m != 0x01)
				printf(" Single Boot from NVCM/Flash\n");
			else
				printf(" Dual Boot from NVCM/Flash then External if there is a failure\n");
			break;
		case 1:
			if (m == 0x01)
				printf(" Single Boot from External Flash\n");
			else
				printf(" Error!\n");
			break;
		default:
			printf(" Error!\n");
	}
	printf("\tMaster Mode SPI                          : %s\n",
	    (((_featbits>>11)&0x01)?"enable":"disable"));
	printf("\tI2c port                                 : %s\n",
	    (((_featbits>>10)&0x01)?"disable":"enable"));
	printf("\tSlave SPI port                           : %s\n",
	    (((_featbits>>9)&0x01)?"disable":"enable"));
	printf("\tJTAG port                                : %s\n",
	    (((_featbits>>8)&0x01)?"disable":"enable"));
	printf("\tDONE                                     : %s\n",
	    (((_featbits>>7)&0x01)?"enable":"disable"));
	printf("\tINITN                                    : %s\n",
	    (((_featbits>>6)&0x01)?"enable":"disable"));
	printf("\tPROGRAMN                                 : %s\n",
	    (((_featbits>>5)&0x01)?"disable":"enable"));
	printf("\tMy_ASSP                                  : %s\n",
	    (((_featbits>>4)&0x01)?"enable":"disable"));
	printf("\tPassword (Flash Protect Key) Protect All : %s\n",
	    (((_featbits>>3)&0x01)?"Enaabled" : "Disabled"));
	printf("\tPassword (Flash Protect Key) Protect     : %s\n",
	    (((_featbits>>2)&0x01)?"Enabled" : "Disabled"));
}


void Lattice::program(unsigned int offset)
{
	(void) offset;

	if (_mode != FLASH_MODE)
		return;

	JedParser _jed(_filename);
	_jed.parse();

	/* read ID Code 0xE0 */
	printf("IDCode : %x\n", idCode());
	/* preload 0x1C */
	uint8_t tx_buf[26];
	memset(tx_buf, 0xff, 26);
	wr_rd(0x1C, tx_buf, 26, NULL, 0);
	/* ISC Enable 0xC6 */
	EnableISC(0x00);
	displayReadReg(readStatusReg());
	/* ISC ERASE */
	if (flashErase(0x01) == false)
		return;
	displayReadReg(readStatusReg());
	/* bypass */
	wr_rd(0xff, NULL, 0, NULL, 0);
	/* ISC Enable 0xC6 followed by 0x08 */
	EnableISC(0x08);
	displayReadReg(readStatusReg());
	/* ISC ERASE */
	if (flashErase(0x0e) == false)
		return;
	displayReadReg(readStatusReg());
	/* LSC_INIT_ADDRESS */
	wr_rd(0x46, NULL, 0, NULL, 0);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);

	/* flash UFM */
	if (false == flashProg(_jed.offset_for_section(0), _jed.data_for_section(0)))
		return;
	if (Verify(_jed) == false)
		return;

	/* LSC_INIT_ADDRESS */
	wr_rd(0x46, NULL, 0, NULL, 0);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);
	displayReadReg(readStatusReg());
	/* write feature row */
	if (writeFeaturesRow(_jed.featuresRow()) == false)
		return;
	readFeaturesRow();
	/* write feabits */
	if (writeFeabits(_jed.feabits()) == false)
		return;
	readFeabits();
	displayReadReg(readStatusReg());
	/* ISC program done 0x5E */
	if (writeProgramDone() == false) {
		cerr << "Error: writeProgramDone" << endl;
		return;
	}
	/* bypass */
	wr_rd(0xff, NULL, 0, NULL, 0);
	displayReadReg(readStatusReg());
	DisableISC();
	/* ISC REFRESH 0x26 */
	if (loadConfiguration() == false) {
		cerr << "Error: loadConfiguration" << endl;
		return;
	}
	displayReadReg(readStatusReg());
	/* bypass */
	wr_rd(0xff, NULL, 0, NULL, 0);
	displayReadReg(readStatusReg());
	_jtag->go_test_logic_reset();
	return;
}

bool Lattice::EnableISC(uint8_t flash_mode)
{
	wr_rd(ISC_ENABLE, &flash_mode, 1, NULL, 0);

	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);
	return pollBusyFlag();
}

bool Lattice::DisableISC()
{
	wr_rd(ISC_DISABLE, NULL, 0, NULL, 0);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);
	return pollBusyFlag();
}

bool Lattice::EnableCfgIf()
{
	uint8_t tx_buf = 0x08;
	wr_rd(0x74, &tx_buf, 1, NULL, 0);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);
	return pollBusyFlag();
}

bool Lattice::DisableCfg()
{
	uint8_t tx_buf, rx_buf;
	wr_rd(0x26, &tx_buf, 1, &rx_buf, 1);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);
	return true;
}

int Lattice::idCode()
{
	uint8_t device_id[4];
	wr_rd(READ_DEVICE_ID_CODE, NULL, 0, device_id, 4);
	return device_id[3] << 24 |
					device_id[2] << 16 |
					device_id[1] << 8  |
					device_id[0];
}

bool Lattice::checkID()
{
	printf("\n");
	printf("check ID\n");
	uint8_t tx[4];
	wr_rd(0xE2, tx, 4, NULL, 0);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);

	uint32_t reg = readStatusReg();
	displayReadReg(reg);

	tx[3] = 0x61;
	tx[2] = 0x2b;
	tx[1] = 0xd0;
	tx[0] = 0x43;
	wr_rd(0xE2, tx, 4, NULL, 0);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);
	reg = readStatusReg();
	displayReadReg(reg);
	printf("%08x\n", reg);
	printf("\n");
	return true;
}

/* feabits is MSB first
 * maybe this register too
 * or not
 */
uint32_t Lattice::readStatusReg()
{
	uint32_t reg;
	uint8_t rx[4], tx[4];
	wr_rd(0x3C, tx, 4, rx, 4, true);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);
	reg = rx[3] << 24 | rx[2] << 16 | rx[1] << 8 | rx[0];
	return reg;
}

bool Lattice::wr_rd(uint8_t cmd,
					uint8_t *tx, int tx_len,
					uint8_t *rx, int rx_len,
					bool verbose)
{
	int xfer_len = rx_len;
	if (tx_len > rx_len)
		xfer_len = tx_len;

	uint8_t xfer_tx[xfer_len];
	uint8_t xfer_rx[xfer_len];
	bzero(xfer_tx, xfer_len);
	int i;
	if (tx != NULL) {
		for (i = 0; i < tx_len; i++)
			xfer_tx[i] = tx[i];
	}

	_jtag->shiftIR(&cmd, NULL, 8, FtdiJtag::PAUSE_IR);
	if (rx || tx) {
		_jtag->shiftDR(xfer_tx, (rx) ? xfer_rx : NULL, 8 * xfer_len,
			FtdiJtag::PAUSE_DR);
	}
	if (rx) {
		if (verbose) {
			for (i=xfer_len-1; i >= 0; i--)
				printf("%02x ", xfer_rx[i]);
			printf("\n");
		}
	    for (i = 0; i < rx_len; i++)
			rx[i] = (xfer_rx[i]);
	}
	return true;
}

void Lattice::displayReadReg(uint32_t dev)
{
	printf("displayReadReg\n");
	if (dev & 1<<0)
		printf("\tTRAN Mode\n");
	printf("\tConfig Target Selection : %x\n", (dev >> 1) & 0x07);
	if (dev & 1<<4)
		printf("\tJTAG Active\n");
	if (dev & 1<<5)
		printf("\tPWD Protect\n");
	if (dev & 1<<6)
		printf("\tOTP\n");
	if (dev & 1<<7)
		printf("\tDecrypt Enable\n");
	if (dev & 1<<8)
		printf("\tDone Flag\n");
	if (dev & 1<<9)
		printf("\tISC Enable\n");
	if (dev & 1 << 10)
		printf("\tWrite Enable\n");
	if (dev & 1 << 11)
		printf("\tRead Enable\n");
	if (dev & 1 << 12)
		printf("\tBusy Flag\n");
	if (dev & 1 << 13)
		printf("\tFail Flag\n");
	if (dev & 1 << 14)
		printf("\tFFEA OTP\n");
	if (dev & 1 << 15)
		printf("\tDecrypt Only\n");
	if (dev & 1 << 16)
		printf("\tPWD Enable\n");
	if (dev & 1 << 17)
		printf("\tUFM OTP\n");
	if (dev & 1 << 18)
		printf("\tASSP\n");
	if (dev & 1 << 19)
		printf("\tSDM Enable\n");
	if (dev & 1 << 20)
		printf("\tEncryption PreAmble\n");
	if (dev & 1 << 21)
		printf("\tStd PreAmble\n");
	if (dev & 1 << 22)
		printf("\tSPIm Fail1\n");

	uint8_t err = (dev >> 23)&0x07;
	printf("\t");
	switch (err) {
		case 0:
			printf("No err\n");
			break;
		case 1:
			printf("ID ERR\n");
			break;
		case 2:
			printf("CMD ERR\n");
			break;
		case 3:
			printf("CRC ERR\n");
			break;
		case 4:
			printf("Preamble ERR\n");
			break;
		case 5:
			printf("Abort ERR\n");
			break;
		case 6:
			printf("Overflow ERR\n");
			break;
		case 7:
			printf("SDM EOF\n");
			break;
		default:
			printf("unknown %x\n", err);
	}
	if (dev & 1 << 26)
		printf("\tEXEC Error\n");
	if (dev & 1 << 27)
		printf("\tDevice failed to verify\n");
	if (dev & 1 << 28)
		printf("\tInvalid Command\n");
	if (dev & 1 << 29) printf("\tSED Error\n");
	if (dev & 1 << 30) printf("\tBypass Mode\n");
	if (dev & ((uint32_t)1 << 31)) printf("\tFT Mode\n");
}

bool Lattice::pollBusyFlag(bool verbose)
{
	uint8_t rx;
	int timeout = 0;
	do {
		wr_rd(CHECK_BUSY_FLAG, NULL, 0, &rx, 1);
		_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
		_jtag->toggleClk(1000);
		if (verbose)
			printf("pollBusyFlag :%02x\n", rx);
		if (timeout == 100000000){
			cerr << "timeout" << endl;
			return false;
		} else {
			timeout++;
		}
	} while (rx != 0);

	return true;
}

bool Lattice::flashEraseAll()
{
	return flashErase(0xf);
}

bool Lattice::flashErase(uint8_t mask)
{
	printf("flash erase\n");
	uint8_t tx[1] = {mask};
	wr_rd(FLASH_ERASE, tx, 1, NULL, 0);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);
	return pollBusyFlag();
}

bool Lattice::flashProg(uint32_t start_addr, std::vector<std::string> data)
{
	(void)start_addr;
	ProgressBar progress("Writing", data.size(), 50);
	for (uint32_t line = 0; line < data.size(); line++) {
		wr_rd(PROG_CFG_FLASH, (uint8_t *)data[line].c_str(),
				16, NULL, 0);
		_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
		_jtag->toggleClk(1000);
		progress.display(line);
		if (pollBusyFlag() == false)
			return false;
	}
	progress.done();
	return true;
}

bool Lattice::Verify(JedParser &_jed, bool unlock)
{
	uint8_t tx_buf[16], rx_buf[16];
	if (unlock)
		EnableISC(0x08);
	displayReadReg(readStatusReg());

	wr_rd(0x46, NULL, 0, NULL, 0);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);

	tx_buf[0] = 0x73;
	_jtag->shiftIR(tx_buf, NULL, 8, FtdiJtag::PAUSE_IR);

	bzero(tx_buf, 16);
	bool failure = false;
	vector<string> data = _jed.data_for_section(0);
	ProgressBar progress("Verifying", data.size(), 50);
	for (size_t line = 0;  line< data.size(); line++) {
		_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
		_jtag->toggleClk(2);
		_jtag->shiftDR(tx_buf, rx_buf, 16*8, FtdiJtag::PAUSE_DR);
		for (size_t i = 0; i < data[i].size(); i++) {
			if (rx_buf[i] != (unsigned char)data[line][i]) {
				printf("%3ld %3ld %02x -> %02x\n", line, i,
						rx_buf[i], (unsigned char)data[line][i]);
				failure = true;
			}
		}
		if (failure) {
			printf("Verify Failure\n");
			break;
		}
		progress.display(line);
	}
	if (unlock)
		DisableISC();

	progress.done();
	displayReadReg(readStatusReg());

	return true;
}

void Lattice::readFeaturesRow()
{
	uint8_t tx_buf[8];
	uint8_t rx_buf[8];
	bzero(tx_buf, 8);
	wr_rd(READ_FEATURE_ROW, tx_buf, 8, rx_buf, 8);
	int i;
	for (i=7; i >= 0; i--) {
		printf("%02x ", rx_buf[i]);
	}
	printf("\n");
}

void Lattice::readFeabits()
{
	uint8_t rx_buf[2];
	wr_rd(READ_FEABITS, NULL, 0, rx_buf, 2);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);

	uint16_t reg = rx_buf[0] | (((uint16_t)rx_buf[1]) << 8);
	printf("%04x\n", reg);
	displayFeabits(reg);
}

bool Lattice::writeFeaturesRow(uint64_t features)
{
	uint8_t tx_buf[8];
	for (int i=0; i < 8; i++)
		tx_buf[i] = ((features >> (i*8)) & 0x00ff);
	wr_rd(PROG_FEATURE_ROW, tx_buf, 8, NULL, 0);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);
	return pollBusyFlag();
}

bool Lattice::writeFeabits(uint16_t feabits)
{
	uint8_t tx_buf[2] = {(uint8_t)(feabits&0x00ff),
							(uint8_t)(0x00ff & (feabits>>8))};

	wr_rd(PROG_FEABITS, tx_buf, 2, NULL, 0);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);
	return pollBusyFlag();
}

bool Lattice::writeProgramDone()
{
	wr_rd(PROG_DONE, NULL, 0, NULL, 0);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);
	return pollBusyFlag();
}

bool Lattice::loadConfiguration()
{
	wr_rd(REFRESH, NULL, 0, NULL, 0);
	_jtag->set_state(FtdiJtag::RUN_TEST_IDLE);
	_jtag->toggleClk(1000);
	return pollBusyFlag();
}
