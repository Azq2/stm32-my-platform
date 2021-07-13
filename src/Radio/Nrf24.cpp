#include "Nrf24.h"

#include <cstdio>
#include <libopencm3/stm32/gpio.h>

Nrf24::Nrf24(Spi *spi, uint32_t cs_bank, uint32_t cs_pin, bool cs_negative) {
	m_spi = spi;
	m_cs_bank = cs_bank;
	m_cs_pin = cs_pin;
	m_cs_negative = cs_negative;
}

void Nrf24::begin() {
	if (m_cs_level == 0) {
		if (m_cs_negative) {
			gpio_clear(m_cs_bank, m_cs_pin);
		} else {
			gpio_set(m_cs_bank, m_cs_pin);
		}
	}
	m_cs_level++;
}

void Nrf24::end() {
	m_cs_level--;
	
	configASSERT(m_cs_level >= 0);
	
	if (m_cs_level == 0) {
		if (m_cs_negative) {
			gpio_set(m_cs_bank, m_cs_pin);
		} else {
			gpio_clear(m_cs_bank, m_cs_pin);
		}
	}
}

int Nrf24::reset() {
	int ret;
	
	// Default values from datasheet
	uint8_t default_values[][2] = {
		{Hw::REG_CONFIG,			0x08},
		{Hw::REG_EN_AA,				0x3F},
		{Hw::REG_EN_RXADDR,			0x03},
		{Hw::REG_SETUP_AW,			0x03},
		{Hw::REG_SETUP_RETR,		0x03},
		{Hw::REG_RF_CH,				0x02},
		{Hw::REG_RF_SETUP,			0x0E},
		{Hw::REG_STATUS,			0x00},
		{Hw::REG_RX_ADDR_P2,		0xC3},
		{Hw::REG_RX_ADDR_P3,		0xC4},
		{Hw::REG_RX_ADDR_P4,		0xC5},
		{Hw::REG_RX_ADDR_P5,		0xC6},
		{Hw::REG_RX_PW_P0,			0x00},
		{Hw::REG_RX_PW_P1,			0x00},
		{Hw::REG_RX_PW_P2,			0x00},
		{Hw::REG_RX_PW_P3,			0x00},
		{Hw::REG_RX_PW_P4,			0x00},
		{Hw::REG_RX_PW_P5,			0x00},
		{Hw::REG_DYNPD,				0x00},
		{Hw::REG_FEATURE,			0x00},
	};
		
	// Reset all registers to default values
	for (size_t i = 0; i < (sizeof(default_values) / sizeof(default_values[0])); i++) {
		ret = writeRegister(default_values[i][0], default_values[i][1]);
		if (ret != ERR_SUCCESS)
			return ret;
	}
	
	// Reset all addresses to datasheet defaults
	constexpr uint8_t addr1[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	constexpr uint8_t addr2[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	
	ret = setAddr(PIPE0, addr1, 5);
	if (ret != ERR_SUCCESS)
		return ret;
	
	ret = setAddr(PIPETX, addr1, 5);
	if (ret != ERR_SUCCESS)
		return ret;
	
	ret = setAddr(PIPETX, addr2, 5);
	if (ret != ERR_SUCCESS)
		return ret;
	
	// Reset all transmissions
	ret = flushTx();
	if (ret != ERR_SUCCESS)
		return ret;
	
	ret = flushRx();
	if (ret != ERR_SUCCESS)
		return ret;
	
	ret = clearIrqFlags();
	if (ret != ERR_SUCCESS)
		return ret;
	
	return ret;
}

int Nrf24::cmdR(uint8_t reg, uint8_t *value, int length) {
	begin();
	
	if (m_spi->write(&reg, 1) != 0) {
		end();
		return ERR_BUS;
	}
	
	if (m_spi->read(value, length) != 0) {
		end();
		return ERR_BUS;
	}
	
	end();
	return ERR_SUCCESS;
}

int Nrf24::cmdW(uint8_t reg, const uint8_t *value, int length) {
	begin();
	
	if (m_spi->write(&reg, 1) != 0) {
		end();
		return ERR_BUS;
	}
	
	if (value) {
		if (m_spi->write(value, length) != 0) {
			end();
			return ERR_BUS;
		}
	}
	
	end();
	return ERR_SUCCESS;
}

int Nrf24::updateRegister(uint8_t reg, uint8_t clear_bits, uint8_t set_bits) {
	int ret = readRegister(reg);
	if (ret < 0)
		return ret;
	return writeRegister(reg, (ret & ~clear_bits) | set_bits);
}

int Nrf24::readRegisterMask(uint8_t reg, uint8_t mask, uint8_t shift) {
	int ret = readRegister(reg);
	if (ret < 0)
		return ret;
	return (ret & mask) >> shift;
}

int Nrf24::setAddr(uint8_t pipe, const uint8_t *addr, uint8_t length) {
	if (pipe > 6)
		return ERR_INVALID_ARGS;
	
	int addr_width = getAddrWidth();
	if (addr_width < 0)
		return addr_width;
	
	if (addr_width != length)
		return ERR_INVALID_ARGS;
	
	if (pipe == PIPE0 || pipe == PIPE1 || pipe == PIPETX) {
		return writeRegister(Hw::REG_RX_ADDR_P0 + pipe, addr, length);
	} else {
		// Only LSB. MSBytes is equal to RX_ADDR_P1[39:8]
		return writeRegister(Hw::REG_RX_ADDR_P0 + pipe, addr[0]);
	}
}

void Nrf24::printState(int (*printf)(const char *, ... )) {
	int reg;
	
	// REG_CONFIG
	reg = readRegister(Hw::REG_CONFIG);
	printf("0x%02X: REG_CONFIG = 0x%02X\r\n", Hw::REG_CONFIG, reg);
	printf("  -> CONFIG_PRIM_RX      = %d\r\n", reg & Hw::CONFIG_PRIM_RX ? 1 : 0);
	printf("  -> CONFIG_PWR_UP       = %d\r\n", reg & Hw::CONFIG_PWR_UP ? 1 : 0);
	printf("  -> CONFIG_CRCO         = %d\r\n", reg & Hw::CONFIG_CRCO ? 1 : 0);
	printf("  -> CONFIG_EN_CRC       = %d\r\n", reg & Hw::CONFIG_EN_CRC ? 1 : 0);
	printf("  -> CONFIG_MASK_MAX_RT  = %d\r\n", reg & Hw::CONFIG_MASK_MAX_RT ? 1 : 0);
	printf("  -> CONFIG_MASK_TX_DS   = %d\r\n", reg & Hw::CONFIG_MASK_TX_DS ? 1 : 0);
	printf("  -> CONFIG_MASK_RX_DR   = %d\r\n", reg & Hw::CONFIG_MASK_RX_DR ? 1 : 0);
	
	// REG_EN_AA
	reg = readRegister(Hw::REG_EN_AA);
	printf("0x%02X: REG_EN_AA = 0x%02X\r\n", Hw::REG_EN_AA, reg);
	for (int i = 0; i < 6; i++) {
		printf("  -> PIPE%d = %d\r\n", i, (reg >> i ? 1 : 0));
	}
	
	// REG_EN_RXADDR
	reg = readRegister(Hw::REG_EN_RXADDR);
	printf("0x%02X: REG_EN_RXADDR = 0x%02X\r\n", Hw::REG_EN_RXADDR, reg);
	for (int i = 0; i < 6; i++) {
		printf("  -> PIPE%d = %d\r\n", i, (reg >> i ? 1 : 0));
	}
	
	// REG_EN_RXADDR
	reg = readRegister(Hw::REG_SETUP_AW);
	printf("0x%02X: REG_SETUP_AW = 0x%02X\r\n", Hw::REG_SETUP_AW, reg);
	printf("  -> WIDTH: %d\r\n", reg + 2);
	
	// REG_SETUP_RETR
	reg = readRegister(Hw::REG_SETUP_RETR);
	printf("0x%02X: REG_SETUP_RETR = 0x%02X\r\n", Hw::REG_SETUP_RETR, reg);
	int ard = (reg & Hw::SETUP_RETR_ARD_MASK) >> Hw::SETUP_RETR_ARD_SHIFT;
	int arc = (reg & Hw::SETUP_RETR_ARC_MASK) >> Hw::SETUP_RETR_ARC_SHIFT;
	printf("  -> ARD: %d ms [0x%02X]\r\n", (ard + 1) * 250, ard);
	printf("  -> ARC: %d\r\n", arc);
	
	// REG_RF_CH
	reg = readRegister(Hw::REG_RF_CH);
	printf("0x%02X: REG_RF_CH = 0x%02X\r\n", Hw::REG_RF_CH, reg);
	printf("  -> CHANNEL: %d MHz [%d]\r\n", 2400 + reg, reg);
	
	// REG_RF_SETUP
	reg = readRegister(Hw::REG_RF_SETUP);
	int power = (reg & Hw::SETUP_RF_PWR_MASK) >> Hw::SETUP_RF_PWR_SHIFT;
	printf("0x%02X: REG_RF_SETUP = 0x%02X\r\n", Hw::REG_RF_SETUP, reg);
	printf("  -> SETUP_RF_LNA_HCURR      = %d\r\n", reg & Hw::SETUP_RF_LNA_HCURR ? 1 : 0);
	printf("  -> SETUP_RF_PLL_LOCK       = %d\r\n", reg & Hw::SETUP_RF_PLL_LOCK ? 1 : 0);
	printf("  -> SETUP_RF_DR_HIGH        = %d\r\n", reg & Hw::SETUP_RF_DR_HIGH ? 1 : 0);
	printf("  -> SETUP_RF_DR_LOW         = %d\r\n", reg & Hw::SETUP_RF_DR_LOW ? 1 : 0);
	printf("  -> SETUP_RF_CONT_WAVE      = %d\r\n", reg & Hw::SETUP_RF_CONT_WAVE ? 1 : 0);
	printf("  -> SETUP_RF_PWR            = %d\r\n", power);
	
	if (power == TX_POWER_0dBm) {
		printf("  -> Tx Power: 0 dBm [%d]\r\n", power);
	} else if (power == TX_POWER_6dBm) {
		printf("  -> Tx Power: 6 dBm [%d]\r\n", power);
	} else if (power == TX_POWER_12dBm) {
		printf("  -> Tx Power: 12 dBm [%d]\r\n", power);
	} else if (power == TX_POWER_18dBm) {
		printf("  -> Tx Power: 18 dBm [%d]\r\n", power);
	}
	
	if (reg & Hw::SETUP_RF_DR_HIGH) {
		printf("  -> Data Rate: 2 Mbps\r\n");
	} else if (reg & Hw::SETUP_RF_DR_LOW) {
		printf("  -> Data Rate: 250 Kbps\r\n");
	} else {
		printf("  -> Data Rate: 1 Mbps\r\n");
	}
	
	// REG_STATUS
	reg = readRegister(Hw::REG_STATUS);
	int pipe = (reg & Hw::STATUS_RX_P_NO_MASK) >> Hw::STATUS_RX_P_NO_SHFIT;
	printf("0x%02X: REG_STATUS = 0x%02X\r\n", Hw::REG_STATUS, reg);
	printf("  -> STATUS_TX_FULL      = %d\r\n", reg & Hw::STATUS_TX_FULL ? 1 : 0);
	printf("  -> STATUS_MAX_RT       = %d\r\n", reg & Hw::STATUS_MAX_RT ? 1 : 0);
	printf("  -> STATUS_TX_DS        = %d\r\n", reg & Hw::STATUS_TX_DS ? 1 : 0);
	printf("  -> STATUS_RX_DR        = %d\r\n", reg & Hw::STATUS_RX_DR ? 1 : 0);
	printf("  -> STATUS_RX_P_NO      = %d\r\n", pipe);
	
	if (pipe > 5) {
		printf("  -> Pipe: Rx Fifo Empty\r\n");
	} else {
		printf("  -> Pipe: %d\r\n", pipe);
	}
	
	// REG_OBSERVE_TX
	reg = readRegister(Hw::REG_OBSERVE_TX);
	printf("0x%02X: REG_OBSERVE_TX = 0x%02X\r\n", Hw::REG_OBSERVE_TX, reg);
	int stat_plos = (reg & Hw::OBSERVE_TX_PLOS_CNT_MASK) >> Hw::OBSERVE_TX_PLOS_CNT_SHIFT;
	int stat_arc = (reg & Hw::OBSERVE_TX_ARC_CNT_MASK) >> Hw::OBSERVE_TX_ARC_CNT_SHIFT;
	printf("  -> OBSERVE_TX_PLOS_CNT  = %d\r\n", stat_plos);
	printf("  -> OBSERVE_TX_ARC_CNT   = %d\r\n", stat_arc);
	
	// REG_RPD
	reg = readRegister(Hw::REG_RPD);
	printf("0x%02X: REG_RPD = 0x%02X\r\n", Hw::REG_RPD, reg);
	printf("  -> DETECTED: %d\r\n", reg);
	
	// REG_RX_ADDR_P*
	for (int i = 0; i < 7; i++) {
		if (i == 6) {
			printf("0x%02X: REG_TX_ADDR\r\n", Hw::REG_RX_ADDR_P0 + i);
		} else {
			printf("0x%02X: REG_RX_ADDR_P%d\r\n", Hw::REG_RX_ADDR_P0 + i, i);
		}
		
		if (i == 0 || i == 1 || i == 6) {
			uint8_t addr[5] = {0};
			readRegister(Hw::REG_RX_ADDR_P0 + i, addr, 5);
			printf("  -> ADDR: %02X %02X %02X %02X %02X\r\n", addr[0], addr[1], addr[2], addr[3], addr[4]);
		} else {
			reg = readRegister(Hw::REG_RX_ADDR_P0 + i);
			printf("  -> ADDR: %02X\r\n", reg);
		}
	}
	
	// REG_RX_PW_P*
	for (int i = 0; i < 6; i++) {
		reg = readRegister(Hw::REG_RX_PW_P0 + i);
		printf("0x%02X: REG_RX_PW_P%d = 0x%02X\r\n", Hw::REG_RX_PW_P0 + i, i, reg);
		printf("  -> WIDTH: %d\r\n", reg);
	}
	
	// REG_FIFO_STATUS
	reg = readRegister(Hw::REG_FIFO_STATUS);
	printf("0x%02X: REG_FIFO_STATUS = 0x%02X\r\n", Hw::REG_FIFO_STATUS, reg);
	printf("  -> FIFO_STATUS_RX_EMPTY      = %d\r\n", reg & Hw::FIFO_STATUS_RX_EMPTY ? 1 : 0);
	printf("  -> FIFO_STATUS_RX_FILL       = %d\r\n", reg & Hw::FIFO_STATUS_RX_FILL ? 1 : 0);
	printf("  -> FIFO_STATUS_TX_EMPTY      = %d\r\n", reg & Hw::FIFO_STATUS_TX_EMPTY ? 1 : 0);
	printf("  -> FIFO_STATUS_TX_FILL       = %d\r\n", reg & Hw::FIFO_STATUS_TX_FILL ? 1 : 0);
	printf("  -> FIFO_STATUS_TX_REUSE      = %d\r\n", reg & Hw::FIFO_STATUS_TX_REUSE ? 1 : 0);
	
	// REG_DYNPD
	reg = readRegister(Hw::REG_DYNPD);
	printf("0x%02X: REG_DYNPD = 0x%02X\r\n", Hw::REG_DYNPD, reg);
	for (int i = 0; i < 6; i++) {
		printf("  -> PIPE%d = %d\r\n", i, (reg >> i ? 1 : 0));
	}
	
	// REG_FEATURE
	reg = readRegister(Hw::REG_FEATURE);
	printf("0x%02X: REG_FEATURE = 0x%02X\r\n", Hw::REG_FEATURE, reg);
	printf("  -> FEATURE_EN_DYN_ACK     = %d\r\n", reg & Hw::FEATURE_EN_DYN_ACK ? 1 : 0);
	printf("  -> FEATURE_EN_ACK_PAY     = %d\r\n", reg & Hw::FEATURE_EN_ACK_PAY ? 1 : 0);
	printf("  -> FEATURE_EN_DPL         = %d\r\n", reg & Hw::FEATURE_EN_DPL ? 1 : 0);
	
}

Nrf24::~Nrf24() {
	configASSERT(m_cs_level == 0);
}
