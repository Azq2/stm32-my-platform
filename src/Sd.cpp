#include "Sd.h"
#include "Sdio.h"

#include <task.h>

#include <cmath>
#include <libopencm3/cm3/nvic.h>

#include "Delay.h"

static Sd *sd_instance = nullptr;

Sd::Sd() {
	m_data_sem = xSemaphoreCreateBinary();
	m_cmd_sem = xSemaphoreCreateBinary();
	
	sd_instance = this;
}

Sd::~Sd() {
	sd_instance = nullptr;
}

int Sd::open() {
	m_capacity_class = CARD_SDSC;
	m_card_version = CARD_UNKNOWN;
	
	m_rca = 0;
	
	// Init SDIO bus
	initSdio();
	
	int ret;
	
	// Reset sdcard
	ret = sendCommand(CMD_GO_IDLE_STATE, 0);
	if (ret != ERR_SUCCESS)
		return ret;
	
	// Check operation conditions
	ret = cmdSendIfCond(R7_VHS_2V7_3V6, 0xAA);
	if (ret == ERR_NOT_USABLE) {
		return ret;
	} else if (ret != ERR_SUCCESS) {
		m_card_version = CARD_V1;
	} else {
		m_card_version = CARD_V2;
	}
	
	// Set HCS bit and powerup SD card
	ret = cmdAppSendOpCond();
	if (ret != ERR_SUCCESS)
		return ret;
	
	// Read CID register
	ret = cmdAllSendCid(&m_cid);
	if (ret != ERR_SUCCESS)
		return ret;
	
	// Read new RCA
	ret = cmdSendRelativeAddr(&m_rca);
	if (ret != ERR_SUCCESS)
		return ret;
	
	// Reac CSD register
	ret = cmdSendCsd(&m_csd);
	if (ret != ERR_SUCCESS)
		return ret;
	
	// Select card by RCA
	ret = sendCommand(CMD_SELECT_CARD, m_rca << 16, CMD_R1B);
	if (ret != ERR_SUCCESS)
		return ret;
	
	// Read SCR register
	ret = cmdSendScr(&m_scr);
	if (ret != ERR_SUCCESS)
		return ret;
	
	// Switch to 4B bus mode, if card supports it
	Sdio::BusWidth bus_width = Sdio::BUS_WIDTH_1;
	if ((m_scr.sd_bus_widths & SCR_BUS_WIDTH_4)) {
		bus_width = Sdio::BUS_WIDTH_4;
		ret = cmdSetBusWidth(SET_BUS_WIDTH_4);
		if (ret != ERR_SUCCESS)
			return ret;
	}
	
	// Update bus width on SDIO and set maximum frequency
	setClock(true, toClockDiv(m_clock_freq), bus_width);
	
	// Set default block length (512)
	ret = sendCommand(CMD_SET_BLOCKLEN, BLOCK_SIZE, CMD_R1);
	if (ret != ERR_SUCCESS)
		return ret;
	
	return ERR_SUCCESS;
}

int Sd::cmdAppSendOpCond() {
	TimeOut_t timeout;
	TickType_t ticks_to_wait = pdMS_TO_TICKS(1000);
	
	vTaskSetTimeOutState(&timeout);
	
	uint32_t host_ocr = OCR_VOLTAGE_3V2_3V3 | OCR_VOLTAGE_3V3_3V4;
	if (m_card_version == CARD_V2)
		host_ocr |= OCR_HCS;
	
	while (true) {
		// Enabe App command set
		int ret = sendCommand(CMD_APP_CMD, m_rca << 16, CMD_R1);
		if (ret != ERR_SUCCESS)
			return ret;
		
		// Send voltage range and HCS
		ret = sendCommand(CMD_SD_SEND_OP_COND, host_ocr, CMD_R3);
		if (ret != ERR_SUCCESS)
			return ret;
		
		// Wait for powerup
		if (SDIO_RESP1 & OCR_NOT_BUSY) {
			if (SDIO_RESP1 & OCR_HCS)
				m_capacity_class = CARD_SDHC;
			return ERR_SUCCESS;
		}
		
		taskYIELD();
		
		if (xTaskCheckForTimeOut(&timeout, &ticks_to_wait))
			return ERR_TIMEOUT;
	}
	
	return ERR_UNKNOWN;
}

int Sd::cmdSendIfCond(uint8_t voltage, uint8_t pattern) {
	uint32_t arg = (voltage << R7_VOLTAGE_SHIFT) | (pattern << R7_PATTERN_SHIFT);
	int ret = sendCommand(CMD_SEND_IF_COND, arg, CMD_R7);
	
	uint32_t card_voltage = (SDIO_RESP1 & R7_VOLTAGE_MASK) >> R7_VOLTAGE_SHIFT;
	uint32_t card_pattern = (SDIO_RESP1 & R7_PATTERN_MASK) >> R7_PATTERN_SHIFT;
	
	if (ret == ERR_SUCCESS) {
		if (card_pattern != pattern || card_voltage != voltage)
			return ERR_NOT_USABLE;
	}
	
	return ret;
}

int Sd::cmdSendCid(Cid *cid) {
	int ret = sendCommand(CMD_SEND_CID, m_rca << 16, CMD_R2);
	if (ret == ERR_SUCCESS) {
		if (!decodeCid((uint32_t *) &SDIO_RESP1, cid))
			return ERR_UNKNOWN;
	}
	return ret;
}

int Sd::cmdAllSendCid(Cid *cid) {
	int ret = sendCommand(CMD_ALL_SEND_CID, 0, CMD_R2);
	if (ret == ERR_SUCCESS) {
		if (!decodeCid((uint32_t *) &SDIO_RESP1, cid))
			return ERR_UNKNOWN;
	}
	return ret;
}

int Sd::cmdSendCsd(Csd *csd) {
	int ret = sendCommand(CMD_SEND_CSD, m_rca << 16, CMD_R2);
	if (ret == ERR_SUCCESS) {
		if (!decodeCsd((uint32_t *) &SDIO_RESP1, csd))
			return ERR_UNKNOWN;
	}
	return ret;
}

int Sd::cmdSendScr(Scr *scr) {
	uint32_t response[2] = {};
	
	int ret = sendCommand(CMD_APP_CMD, m_rca << 16, CMD_R1);
	if (ret != ERR_SUCCESS)
		return ret;
	
	transferDataBegin(MODE_READ, Sdio::BLOCK_SIZE_8, 1, (char *) &response);
	
	ret = sendCommand(CMD_SEND_SCR, 0, CMD_R1);
	if (ret != ERR_SUCCESS) {
		transferDataFinish();
		return ret;
	}
	
	ret = transferData(1000);
	if (ret != ERR_SUCCESS)
		return ret;
	
	if (!decodeScr(response, scr))
		return ERR_UNKNOWN;
	
	return ERR_SUCCESS;
}

int Sd::cmdSendRelativeAddr(uint16_t *new_rca) {
	for (int i = 0; i < 2; i++) {
		int ret = sendCommand(CMD_SEND_RELATIVE_ADDR, 0, CMD_R6);
		if (ret != ERR_SUCCESS)
			return ret;
		
		*new_rca = (SDIO_RESP1 & R6_RCA_MASK) >> R6_RCA_SHIFT;
		if (*new_rca)
			break;
	}
	return ERR_SUCCESS;
}

int Sd::cmdSetBusWidth(uint8_t bus_width) {
	int ret = sendCommand(CMD_APP_CMD, m_rca << 16, CMD_R1);
	if (ret != ERR_SUCCESS)
		return ret;
	return sendCommand(CMD_SET_BUS_WIDTH, bus_width, CMD_R1);
}

int Sd::cmdStopTransmission() {
	int ret;
	for (int i = 0; i < 3; i++) {
		ret = sendCommand(CMD_STOP_TRANSMISSION, 0, CMD_R1B);
		if (ret == ERR_SUCCESS)
			break;
	}
	return ret;
}

int Sd::cmdSendStatus(uint32_t *status) {
	int ret = sendCommand(CMD_SEND_STATUS, m_rca << 16, CMD_R1);
	if (ret == ERR_SUCCESS)
		*status = (SDIO_RESP1 & R1_STATUS_MASK) >> R1_STATUS_SHIFT;
	return ret;
}

int Sd::readBlocks(uint32_t addr, uint32_t n_blocks, char *buffer, uint32_t timeout_ms) {
	int ret;
	
	// Check addr and size
	if ((addr + n_blocks) * BLOCK_SIZE > m_csd.capacity || n_blocks > 0x7FFF)
		return ERR_OUT_OF_RANGE;
	
	// Init DPSM
	transferDataBegin(MODE_READ, Sdio::BLOCK_SIZE_512, n_blocks, buffer);
	
	// Send read command to SD
	uint8_t cmd = n_blocks > 1 ? CMD_READ_MULTIPLE_BLOCK : CMD_READ_SINGLE_BLOCK;
	ret = sendCommand(cmd, (m_capacity_class == CARD_SDHC ? addr : addr * 512), CMD_R1);
	if (ret != ERR_SUCCESS) {
		transferDataFinish();
		return ret;
	}
	
	// Wait for read done
	int read_status = transferData(timeout_ms);
	
	// Stop transmission
	if (n_blocks > 1) {
		ret = cmdStopTransmission();
		if (ret != ERR_SUCCESS)
			return ret;
	}
	
	return read_status;
}

int Sd::writeBlocks(uint32_t addr, uint32_t n_blocks, const char *buffer, uint32_t timeout_ms) {
	int ret;
	TimeOut_t timeout;
	TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
	
	vTaskSetTimeOutState(&timeout);
	
	// Check addr and size
	if ((addr + n_blocks) * BLOCK_SIZE > m_csd.capacity || n_blocks > 0x7FFF)
		return ERR_OUT_OF_RANGE;
	
	// Send write command to SD
	uint8_t cmd = n_blocks > 1 ? CMD_WRITE_MULTIPLE_BLOCK : CMD_WRITE_SINGLE_BLOCK;
	ret = sendCommand(cmd, (m_capacity_class == CARD_SDHC ? addr : addr * 512), CMD_R1);
	if (ret != ERR_SUCCESS) {
		cmdStopTransmission();
		return ret;
	}
	
	// Init DPSM
	transferDataBegin(MODE_WRITE, Sdio::BLOCK_SIZE_512, n_blocks, (char *) buffer);
	
	// Wait for write done
	ret = transferData(timeout_ms);
	if (ret != ERR_SUCCESS) {
		cmdStopTransmission();
		return ret;
	}
	
	// Stop transmission
	if (n_blocks > 1) {
		ret = cmdStopTransmission();
		if (ret != ERR_SUCCESS)
			return ret;
	}
	
	// Wait for programming done
	uint32_t status;
	do {
		if (xTaskCheckForTimeOut(&timeout, &ticks_to_wait))
			return ERR_TIMEOUT;
		ret = cmdSendStatus(&status);
		if (ret != ERR_SUCCESS)
			return ret;
	} while ((status == R1_STATUS_RCV || status == R1_STATUS_PRG));
	
	return ERR_SUCCESS;
}

void Sd::initSdio() {
	rcc_periph_clock_enable(RCC_SDIO);
	rcc_periph_reset_pulse(RST_SDIO);
	
	setClock(false, toClockDiv(400000), Sdio::BUS_WIDTH_1);
	Sdio::setPowerConfig(true);
	delayMs(1);
	setClock(true, toClockDiv(400000), Sdio::BUS_WIDTH_1);
	
	nvic_enable_irq(NVIC_SDIO_IRQ);
	nvic_set_priority(NVIC_SDIO_IRQ, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
}

void Sd::transferDataBegin(uint32_t mode, Sdio::BlockSize block_size, uint32_t n_blocks, char *data) {
	taskENTER_CRITICAL();
	uint32_t block_length = Sdio::toBlockLength(block_size);
	uint32_t data_length = block_length * n_blocks;
	
	m_isr.data.error = ERR_SUCCESS;
	m_isr.data.mode = mode;
	m_isr.data.ptr = data;
	m_isr.data.count = data_length >> 2;
	m_isr.data.block_length = block_length;
	
	Sdio::clearFlag(DATA_ICR_FLAGS);
	
	if (m_isr.data.mode == MODE_WRITE) {
		Sdio::maskFlag(DATA_MASK_FLAGS | SDIO_MASK_TXFIFOHEIE | SDIO_MASK_TXFIFOEIE);
	} else {
		if (block_length >= Sdio::FIFO_HALF_SIZE) {
			Sdio::maskFlag(DATA_MASK_FLAGS | SDIO_MASK_RXFIFOHFIE);
		} else {
			Sdio::maskFlag(DATA_MASK_FLAGS | SDIO_MASK_RXDAVLIE);
		}
	}
	
	Sdio::setDataTimeout(0xFFFFFFFF);
	Sdio::setDataLength(data_length);
	
	if (m_isr.data.mode == MODE_WRITE) {
		Sdio::setDpsmConfig(true, block_size, Sdio::TO_CARD, Sdio::BLOCK_TRANSFER_MODE, false);
	} else {
		Sdio::setDpsmConfig(true, block_size, Sdio::FROM_CARD, Sdio::BLOCK_TRANSFER_MODE, false);
	}
	
	taskEXIT_CRITICAL();
}

int Sd::transferData(uint32_t timeout_ms) {
	TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
	
	if (xSemaphoreTake(m_data_sem, ticks_to_wait)) {
		transferDataFinish();
		return m_isr.data.error;
	}
	
	transferDataFinish();
	
	return ERR_TIMEOUT;
}

void Sd::transferDataFinish() {
	taskENTER_CRITICAL();
	Sdio::setDpsmConfig(false, Sdio::BLOCK_SIZE_1, Sdio::FROM_CARD, Sdio::BLOCK_TRANSFER_MODE, false);
	Sdio::clearFlag(DATA_ICR_FLAGS);
	Sdio::unmaskFlag(DATA_UNMASK_FLAGS);
	taskEXIT_CRITICAL();
}

int Sd::sendCommand(uint8_t index, uint32_t arg, uint32_t flags, uint32_t timeout_ms) {
	TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
	
	auto finishCommand = [&]() {
		taskENTER_CRITICAL();
		Sdio::setCpsmConfig(false, 0, false, false);
		Sdio::clearFlag(CMD_ICR_FLAGS);
		Sdio::unmaskFlag(CMD_UNMASK_FLAGS);
		taskEXIT_CRITICAL();
	};
	
	taskENTER_CRITICAL();
	m_isr.cmd.error = ERR_SUCCESS;
	m_isr.cmd.flags = flags;
	
	Sdio::clearFlag(CMD_ICR_FLAGS);
	Sdio::maskFlag(CMD_MASK_FLAGS);
	Sdio::setCommandArgument(arg);
	Sdio::setCpsmConfig(true, index, (flags & CMD_LONG_RESP) != 0, (flags & CMD_WAIT_RESP) != 0);
	taskEXIT_CRITICAL();
	
	if (xSemaphoreTake(m_cmd_sem, ticks_to_wait)) {
		finishCommand();
		
		if (m_isr.cmd.error == ERR_CRC_FAIL && (flags & CMD_CRCFAIL))
			m_isr.cmd.error = ERR_SUCCESS;
		
		if (!m_isr.cmd.error && (flags & CMD_RESP_R1)) {
			if (SDIO_RESP1 & R1_ALL_ERRORS) {
				if (SDIO_RESP1 & (R1_CARD_ECC_FAILED | R1_COM_CRC_ERROR)) {
					m_isr.cmd.error = ERR_CRC_FAIL;
				} else if (SDIO_RESP1 & R1_ILLEGAL_COMMAND) {
					m_isr.cmd.error = ERR_ILLEGAL_COMMAND;
				} else {
					m_isr.cmd.error = ERR_UNKNOWN;
				}
			}
		}
		
		return m_isr.cmd.error;
	}
	
	finishCommand();
	
	return ERR_TIMEOUT;
}

bool Sd::decodeScr(const uint32_t *response, Scr *scr) {
	scr->scr_structure				= getBitsBE(response, 64, 60, 4);
	scr->sd_spec					= getBitsBE(response, 64, 56, 4);
	scr->data_stat_after_erase		= getBitsBE(response, 64, 55, 1) != 0;
	scr->sd_security				= getBitsBE(response, 64, 52, 3);
	scr->sd_bus_widths				= getBitsBE(response, 64, 48, 4);
	scr->sd_spec3					= getBitsBE(response, 64, 47, 1) != 0;
	scr->ex_security				= getBitsBE(response, 64, 43, 4);
	scr->cmd_support				= getBitsBE(response, 64, 32, 2);
	return scr->scr_structure == 0;
}

bool Sd::decodeCid(const uint32_t *response, Cid *cid) {
	cid->mid		= getBits(response, 128, 120, 8);
	cid->oid		= getBits(response, 128, 104, 16);
	
	for (int i = 0; i < 5; i++)
		cid->pnm[i] = getBits(response, 128, 96 - (i * 8), 8);
	cid->pnm[5] = 0;
	
	cid->prv		= getBits(response, 128, 56, 8);
	cid->psn		= getBits(response, 128, 24, 32);
	cid->mdt_year	= getBits(response, 128, 12, 8) + 2000;
	cid->mdt_month	= getBits(response, 128, 8, 4);
	
	return true;
}

bool Sd::decodeCsd(const uint32_t *response, Csd *csd) {
	static const int exp[] = {
		1, 10, 100, 1000, 10000, 100000, 1000000, 10000000
	};
	
	static const int mant[] = {
		0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80
	};
	
	uint32_t m, e;
	
	csd->csd_structure			= getBits(response, 128, 126, 2);
//	csd->spec_vers				= getBits(response, 128, 122, 4);
	
	m							= getBits(response, 128, 115, 4);
	e							= getBits(response, 128, 112, 3);
	csd->taac					= (exp[e] * mant[m] + 9) / 10;
	csd->nsac					= getBits(response, 128, 104, 8) * 100;
	
	m							= getBits(response, 128, 99, 4);
	e							= getBits(response, 128, 96, 3);
	csd->tran_speed				= (exp[e] * mant[m]) * 10000;
	
	csd->ccc					= getBits(response, 128, 84, 12);
	csd->read_bl_len			= 1 << getBits(response, 128, 80, 4);
	csd->read_bl_partial		= getBits(response, 128, 79, 1);
	csd->write_blk_misalign		= getBits(response, 128, 78, 1);
	csd->read_blk_misalign		= getBits(response, 128, 77, 1);
	csd->dsr_imp				= getBits(response, 128, 76, 1);
	
	if (csd->csd_structure == 0) {
		m						= getBits(response, 128, 62, 12);
		e						= getBits(response, 128, 47, 3);
		csd->capacity			= ((1 + m) << (e + 2)) * csd->read_bl_len;
	} else if (csd->csd_structure == 1) {
		csd->capacity			= (uint64_t) (getBits(response, 128, 48, 22) + 1) * 512 * 1024;
	}
	
//	if (csd->csd_structure == 0) {
//		csd->vdd_r_curr_min		= getBits(response, 128, 59, 3);
//		csd->vdd_r_curr_max		= getBits(response, 128, 56, 3);
//		csd->vdd_w_curr_min		= getBits(response, 128, 53, 3);
//		csd->vdd_w_curr_max		= getBits(response, 128, 50, 3);
//	} else {
//		csd->vdd_r_curr_min		= 0;
//		csd->vdd_r_curr_max		= 0;
//		csd->vdd_w_curr_min		= 0;
//		csd->vdd_w_curr_max		= 0;
//	}
	
	csd->erase_blk_en			= getBits(response, 128, 46, 1);
	csd->sector_size			= getBits(response, 128, 39, 7) + 1;
	
	csd->wp_grp_size			= getBits(response, 128, 32, 7);
	csd->wp_grp_enable			= getBits(response, 128, 31, 1);
//	csd->default_ecc			= getBits(response, 128, 29, 1);
	csd->r2w_factor				= 1 << getBits(response, 128, 26, 3);
	csd->write_bl_len			= 1 << getBits(response, 128, 22, 4);
	csd->write_bl_partial		= getBits(response, 128, 21, 1);
//	csd->content_prot_app		= getBits(response, 128, 16, 1);
//	csd->file_format_grp		= getBits(response, 128, 15, 1);
//	csd->copy					= getBits(response, 128, 14, 1);
	csd->perm_write_protect		= getBits(response, 128, 13, 1);
	csd->tmp_write_protect		= getBits(response, 128, 12, 1);
//	csd->file_format			= getBits(response, 128, 10, 2);
//	csd->ecc					= getBits(response, 128, 8, 2);
	
	return csd->csd_structure == 0 || csd->csd_structure == 1;
}

void Sd::setClock(bool enable, uint8_t div, Sdio::BusWidth bus_width) {
	Sdio::setClockConfig(
		enable,						// clock enable
		div,						// clock div
		bus_width,					// bus width
		Sdio::CLOCK_POSITIVE_EDGE,	// clock phase
		false,						// hw flow control
		false						// power saving
	);
}

uint8_t Sd::toClockDiv(uint32_t freq) {
	uint32_t sdioclk = getSdioClk();
	if (freq >= sdioclk)
		return 1;
	
	if ((sdioclk % freq) == 0)
		return sdioclk / freq;
	
	return (sdioclk / freq) + 1;
}

uint32_t Sd::getSdioClk() {
	uint32_t pllp = (((RCC_PLLCFGR >> RCC_PLLCFGR_PLLP_SHIFT) & RCC_PLLCFGR_PLLP_MASK) + 1) << 1;
	uint32_t pllq = (RCC_PLLCFGR >> RCC_PLLCFGR_PLLQ_SHIFT) & RCC_PLLCFGR_PLLQ_MASK;
	uint32_t vco = rcc_ahb_frequency * pllp;
	return vco / pllq;
}

void Sd::readHalfFifoFromISR() {
	constexpr uint32_t count = Sdio::FIFO_HALF_SIZE >> 2;
	
	if (m_isr.data.count < count) {
		m_isr.data.error = ERR_INTERNAL_ERROR;
		return;
	}
	
	uint32_t i = count;
	while (i--)
		*m_isr.data.ptr4++ = SDIO_FIFO;
	
	m_isr.data.count -= count;
}

void Sd::readFifoFromISR() {
	while (Sdio::getStatus() & SDIO_STA_RXDAVL) {
		if (m_isr.data.count > 0) {
			*m_isr.data.ptr4++ = SDIO_FIFO;
			m_isr.data.count--;
		} else {
			m_isr.data.error = ERR_INTERNAL_ERROR;
			break;
		}
	}
}

void Sd::writeFifoFromISR(uint32_t status) {
	while ((status & (SDIO_STA_TXFIFOHE | SDIO_STA_TXFIFOE))) {
		uint32_t max_cnt = (status & SDIO_STA_TXFIFOE) ? (Sdio::FIFO_SIZE >> 2) : (Sdio::FIFO_HALF_SIZE >> 2);
		uint32_t count = max_cnt > m_isr.data.count ? m_isr.data.count : max_cnt;
		
		while (count--) {
			SDIO_FIFO = *m_isr.data.ptr4++;
			m_isr.data.count--;
		}
		
		status = Sdio::getStatus();
	}
}

void Sd::handleIrq() {
	BaseType_t higher_task_woken = pdFALSE;
	uint32_t status = Sdio::getStatus();
	
	Sdio::clearFlag(Sdio::getIcrFromStatus(status));
	
	// DATA MODE
	if ((status & DATA_STA_FLAGS)) {
		if (m_isr.data.mode == MODE_WRITE) {
			writeFifoFromISR(status);
		} else {
			if (m_isr.data.block_length >= Sdio::FIFO_HALF_SIZE) {
				if ((status & SDIO_STA_RXFIFOHF))
					readHalfFifoFromISR();
			} else {
				if ((status & SDIO_STA_RXDAVL))
					readFifoFromISR();
			}
		}
		
		constexpr uint32_t error_flags = SDIO_STA_RXOVERR | SDIO_STA_TXUNDERR | SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT | SDIO_STA_STBITERR;
		if ((status & error_flags)) {
			if ((status & SDIO_STA_RXOVERR)) {
				m_isr.data.error = ERR_OVERRUN;
			} else if ((status & SDIO_STA_TXUNDERR)) {
				m_isr.data.error = ERR_UNDERRUN;
			} else if ((status & SDIO_STA_DCRCFAIL)) {
				m_isr.data.error = ERR_CRC_FAIL;
			} else if ((status & SDIO_STA_DTIMEOUT)) {
				m_isr.data.error = ERR_TIMEOUT;
			} else {
				m_isr.data.error = ERR_UNKNOWN;
			}
		}
		
		if ((status & SDIO_STA_DATAEND) || m_isr.data.error) {
			if (!m_isr.data.error && m_isr.data.count > 0)
				m_isr.data.error = ERR_INTERNAL_ERROR;
			
			Sdio::unmaskFlag(DATA_UNMASK_FLAGS);
			xSemaphoreGiveFromISR(m_data_sem, &higher_task_woken);
		}
	}
	
	// COMMAND MODE
	if ((status & CMD_STA_FLAGS)) {
		constexpr uint32_t error_flags = SDIO_STA_CCRCFAIL | SDIO_STA_CTIMEOUT;
		if ((status & error_flags)) {
			if ((status & SDIO_STA_CCRCFAIL)) {
				m_isr.cmd.error = ERR_CRC_FAIL;
			} else if ((status & SDIO_STA_CTIMEOUT)) {
				m_isr.cmd.error = ERR_TIMEOUT;
			} else {
				m_isr.cmd.error = ERR_UNKNOWN;
			}
		}
		
		uint32_t wait_flag = ((m_isr.cmd.flags & CMD_WAIT_RESP) ? SDIO_STA_CMDREND : SDIO_STA_CMDSENT);
		if ((status & wait_flag) || m_isr.cmd.error) {
			Sdio::unmaskFlag(CMD_UNMASK_FLAGS);
			xSemaphoreGiveFromISR(m_cmd_sem, &higher_task_woken);
		}
	}
	
	portYIELD_FROM_ISR(higher_task_woken);
}

extern "C" void sdio_isr(void) {
	if (sd_instance) {
		sd_instance->handleIrq();
	} else {
		Sdio::unmaskAllFlags();
		Sdio::clearAllFlags();
	}
}
