#include "Nrf24.h"
#include "Exti.h"
#include "Delay.h"

#include <cstdio>

Nrf24::Nrf24(Spi *spi, const Pinout &pins) : m_spi(spi), m_pins(pins) {
	m_irq_sem = xSemaphoreCreateBinary();
}

void Nrf24::ce(bool flag) {
	if (m_pins.cs.negative != flag) {
		gpio_clear(m_pins.ce.bank, m_pins.ce.pin);
	} else {
		gpio_set(m_pins.ce.bank, m_pins.ce.pin);
	}
}

void Nrf24::begin() {
	if (m_cs_level == 0) {
		if (m_pins.cs.negative) {
			gpio_clear(m_pins.cs.bank, m_pins.cs.pin);
		} else {
			gpio_set(m_pins.cs.bank, m_pins.cs.pin);
		}
	}
	m_cs_level++;
}

void Nrf24::end() {
	m_cs_level--;
	
	configASSERT(m_cs_level >= 0);
	
	if (m_cs_level == 0) {
		if (m_pins.cs.negative) {
			gpio_set(m_pins.cs.bank, m_pins.cs.pin);
		} else {
			gpio_clear(m_pins.cs.bank, m_pins.cs.pin);
		}
	}
}

int Nrf24::open() {
	Exti::Trigger trigger = m_pins.irq.negative ? Exti::FALLING : Exti::RISING;
	
	auto irq_callback = +[](bool state, void *self)->void {
		(void) state;
		static_cast<Nrf24 *>(self)->handleIrq();
	};
	
	if (Exti::set(m_pins.irq.bank, m_pins.irq.pin, trigger, irq_callback, this) < 0)
		return ERR_UNKNOWN;
	return ERR_SUCCESS;
}

int Nrf24::reset() {
	disable();
	
	constexpr uint8_t ALL_PIPES = (1 << PIPE0) | (1 << PIPE1) | (1 << PIPE2) | (1 << PIPE3) | (1 << PIPE4) | (1 << PIPE5);
	
	// Default values from datasheet with several changes:
	// - powerdown by default
	// - all RX pipes disabled
	// - enabled all features
	constexpr uint8_t default_values[][2] = {
		{Hw::REG_CONFIG,			Hw::CONFIG_CRCO | Hw::CONFIG_EN_CRC},
		{Hw::REG_EN_AA,				ALL_PIPES},
		{Hw::REG_EN_RXADDR,			0},
		{Hw::REG_SETUP_AW,			Hw::SETUP_AW_5B},
		{Hw::REG_SETUP_RETR,		(3 << Hw::SETUP_RETR_ARC_SHIFT) | (ARD_250US << Hw::SETUP_RETR_ARD_SHIFT)},
		{Hw::REG_RF_CH,				2},
		{Hw::REG_RF_SETUP,			Hw::SETUP_RF_DR_LOW | (TX_POWER_0dBm << Hw::SETUP_RF_PWR_SHIFT)},
		{Hw::REG_STATUS,			0x00},
		{Hw::REG_RX_ADDR_P2,		0xC3},
		{Hw::REG_RX_ADDR_P3,		0xC4},
		{Hw::REG_RX_ADDR_P4,		0xC5},
		{Hw::REG_RX_ADDR_P5,		0xC6},
		{Hw::REG_RX_PW_P0,			0},
		{Hw::REG_RX_PW_P1,			0},
		{Hw::REG_RX_PW_P2,			0},
		{Hw::REG_RX_PW_P3,			0},
		{Hw::REG_RX_PW_P4,			0},
		{Hw::REG_RX_PW_P5,			0},
		{Hw::REG_DYNPD,				ALL_PIPES},
		{Hw::REG_FEATURE,			Hw::FEATURE_EN_ACK_PAY | Hw::FEATURE_EN_DPL | Hw::FEATURE_EN_DYN_ACK},
	};
	
	// Reset all registers to default values
	for (size_t i = 0; i < (sizeof(default_values) / sizeof(default_values[0])); i++)
		writeRegister(default_values[i][0], default_values[i][1]);
	
	// Reset all addresses to datasheet defaults
	int ret;
	ret = setAddr(PIPE0, 0xE7E7E7E7E7);
	if (ret != ERR_SUCCESS)
		return ret;
	
	ret = setAddr(PIPE1, 0xE7E7E7E7E7);
	if (ret != ERR_SUCCESS)
		return ret;
	
	ret = setAddr(PIPETX, 0xC2C2C2C2C2);
	if (ret != ERR_SUCCESS)
		return ret;
	
	// Reset all transmissions
	flushAll();
	
	// Validate state
	if (!isConnected())
		return ERR_UNKNOWN;
	
	if (getFifoStatus() != (Hw::FIFO_STATUS_RX_EMPTY | Hw::FIFO_STATUS_TX_EMPTY))
		return ERR_UNKNOWN;
	
	if (getAddr(PIPE0) != 0xE7E7E7E7E7)
		return ERR_UNKNOWN;
	
	if (getAddr(PIPETX) != 0xC2C2C2C2C2)
		return ERR_UNKNOWN;
	
	return ERR_SUCCESS;
}

Nrf24::OperationMode Nrf24::getOperationMode() {
	if (isPowerUp()) {
		if (getMode() == MODE_RX) {
			return isChipEnabled() ? IN_RX_MODE : IN_STANDBY;
		} else {
			return IN_TX_MODE;
		}
	} else {
		return IN_POWER_DOWN;
	}
}

void Nrf24::startRx() {
	OperationMode mode = getOperationMode();
	if (mode == IN_RX_MODE)
		return;
	
	setMode(MODE_RX);
	if (mode == IN_POWER_DOWN)
		setPowerUp(true);
	flushAll();
	
	enable();
}

void Nrf24::startTx() {
	disable();
	
	OperationMode mode = getOperationMode();
	if (mode == IN_TX_MODE)
		return;
	
	setMode(MODE_TX);
	if (mode == IN_POWER_DOWN)
		setPowerUp(true);
	flushAll();
}

void Nrf24::stop() {
	disable();
	setMode(MODE_TX);
	flushAll();
	setPowerUp(false);
}

void Nrf24::flushAll() {
	flushRx();
	flushTx();
	clearIrqFlags(Hw::STATUS_RX_DR);
	clearIrqFlags(Hw::STATUS_TX_DS | Hw::STATUS_MAX_RT);
}

int Nrf24::write(const void *buffer, uint8_t size, bool no_ack, int retries) {
	writeTxPayload(buffer, size, no_ack);
	enable();
	int ret = writeFinish(retries);
	disable();
	return ret;
}

int Nrf24::writeAckPayload(uint8_t pipe, const void *buffer, uint8_t size) {
	configASSERT(pipe < 6);
	writeRegister(Hw::CMD_W_ACK_PAYLOAD | pipe, buffer, size);
	return ERR_SUCCESS;
}

int Nrf24::_writeFinish(int &retries) {
	uint8_t status = waitForIrqFlags(Hw::STATUS_TX_DS | Hw::STATUS_MAX_RT, DEFAULT_WRITE_TIMEOUT);
	
	// Timeout after ARD * ARC (when no_ack=0)
	if ((status & Hw::STATUS_MAX_RT)) {
		if (retries > 0) {
			// Start new transmit with previous payload
			clearIrqFlags(Hw::STATUS_MAX_RT);
			reuseTxPayload();
			disable();
			enable();
			retries--;
			return ERR_AGAIN;
		} else {
			// Not enought retries, stop
			clearIrqFlags(Hw::STATUS_TX_DS | Hw::STATUS_MAX_RT);
			flushTx();
			disable();
			return ERR_TIMEOUT;
		}
	}
	
	// Successful transmit
	if ((status & Hw::STATUS_TX_DS)) {
		clearIrqFlags(Hw::STATUS_TX_DS);
		return ERR_SUCCESS;
	}
	
	flushAll();
	disable();
	
	// Unknown error, may be chip fault and soft reset needed
	return ERR_UNKNOWN;
}

int Nrf24::streamWrite(const void *buffer, uint8_t size, bool no_ack) {
	int retries = m_stream_retries;
	
	// Wait for last transmit done if TX fifo is full
	if ((getStatus() & (Hw::STATUS_TX_FULL | Hw::STATUS_MAX_RT))) {
		int ret = writeFinish(retries);
		if (ret < 0)
			return ret;
	}
	
	// Add message to FIFO
	writeTxPayload(buffer, size, no_ack);
	
	// Start transmit
	enable();
	
	return ERR_SUCCESS;
}

int Nrf24::streamWriteFinish() {
	int retries = m_stream_retries;
	
	for (int i = 0; i < TX_FIFO_COUNT; i++) {
		// Wait for empty TX fifo
		 if ((getFifoStatus() & Hw::FIFO_STATUS_TX_EMPTY))
			break;
		 
		// Wait for transfer done
		int ret = writeFinish(retries);
		if (ret < 0)
			return ret;
	}
	
	disable();
	return ERR_SUCCESS;
}

bool Nrf24::waitForPacket(uint32_t timeout_ms) {
	uint8_t status = waitForIrqFlags(Hw::STATUS_RX_DR, timeout_ms);
	return (status & Hw::STATUS_RX_DR) != 0;
}

bool Nrf24::hasPacket() {
	return (getStatus() & Hw::STATUS_RX_DR) != 0;
}

int Nrf24::read(void *buffer, uint8_t *out_pipe, uint8_t max_size) {
	if (out_pipe)
		*out_pipe = NO_PIPE;
	
	uint8_t rx_pipe = getRxPipe();
	uint8_t payload_size = getDynamicPayloadSize();
	
	if (rx_pipe > PIPE5 || !payload_size || payload_size > 32) {
		// We received noise, clear IRQ and RX FIFO
		if (hasPacket()) {
			flushRx();
			clearIrqFlags(Hw::STATUS_RX_DR);
		}
		return 0;
	}
	
	if (out_pipe)
		*out_pipe = rx_pipe;
	
	uint8_t read_size = max_size < payload_size ? max_size : payload_size;
	readRxPayload(buffer, read_size);
	clearIrqFlags(Hw::STATUS_RX_DR);
	
	return read_size;
}

uint8_t Nrf24::waitForIrqFlags(uint8_t flags, uint32_t timeout_ms) {
	TimeOut_t timeout;
	TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
	
	vTaskSetTimeOutState(&timeout);
	
	while (true) {
		uint8_t status = getStatus();
		if ((status & flags))
			return status;
		
		if (xTaskCheckForTimeOut(&timeout, &ticks_to_wait))
			break;
		
		if (xSemaphoreTake(m_irq_sem, ticks_to_wait))
			break;
	}
	
	return getStatus();
}

void Nrf24::cmdR(uint8_t reg, void *value, int length) {
	begin();
	m_spi->write(&reg, 1);
	m_spi->read((uint8_t *) value, length);
	end();
}

void Nrf24::cmdW(uint8_t reg, const void *value, int length) {
	begin();
	m_spi->write(&reg, 1);
	m_spi->write((const uint8_t *) value, length);
	end();
}

void Nrf24::updateRegister(uint8_t reg, uint8_t clear_bits, uint8_t set_bits) {
	writeRegister(reg, (readRegister(reg) & ~clear_bits) | set_bits);
}

uint8_t Nrf24::readRegisterMask(uint8_t reg, uint8_t mask, uint8_t shift) {
	return (readRegister(reg) & mask) >> shift;
}

int Nrf24::setAddr(uint8_t pipe, uint64_t addr) {
	if (pipe > 6)
		return ERR_INVALID_ARGS;
	
	uint8_t addr_width = getAddrWidth();
	if (addr_width < 3 || addr_width > 5)
		return ERR_UNKNOWN;
	
	if (pipe == PIPE0 || pipe == PIPE1 || pipe == PIPETX) {
		writeRegister(Hw::REG_RX_ADDR_P0 + pipe, &addr, addr_width);
	} else {
		// Only LSB. MSBytes is equal to RX_ADDR_P1[39:8]
		writeRegister(Hw::REG_RX_ADDR_P0 + pipe, addr & 0xFF);
	}
	
	return ERR_SUCCESS;
}

uint64_t Nrf24::getAddr(uint8_t pipe) {
	uint8_t addr_width = getAddrWidth();
	
	configASSERT(addr_width >= 3 && addr_width <= 5);
	
	if (addr_width < 3 || addr_width > 5)
		return 0;
	
	uint64_t addr = 0;
	if (pipe == PIPE0 || pipe == PIPE1 || pipe == PIPETX) {
		readRegister(Hw::REG_RX_ADDR_P0 + pipe, &addr, addr_width);
	} else {
		readRegister(Hw::REG_RX_ADDR_P1, &addr, addr_width);
		readRegister(Hw::REG_RX_ADDR_P0 + pipe, &addr, 1);
	}
	
	return addr;
}

void Nrf24::handleIrq() {
	BaseType_t higher_task_woken = pdFALSE;
	xSemaphoreGiveFromISR(m_irq_sem, &higher_task_woken);
	
	if (higher_task_woken)
		portYIELD_FROM_ISR(higher_task_woken);
}

void Nrf24::printState(int (*printf)(const char *, ... )) {
	uint8_t reg;
	
	// REG_CONFIG
	reg = readRegister(Hw::REG_CONFIG);
	printf("0x%02X: REG_CONFIG = 0x%02X\r\n", Hw::REG_CONFIG, reg);
	printf("  -> CONFIG_PRIM_RX      = %d\r\n", (reg & Hw::CONFIG_PRIM_RX) ? 1 : 0);
	printf("  -> CONFIG_PWR_UP       = %d\r\n", (reg & Hw::CONFIG_PWR_UP) ? 1 : 0);
	printf("  -> CONFIG_CRCO         = %d\r\n", (reg & Hw::CONFIG_CRCO) ? 1 : 0);
	printf("  -> CONFIG_EN_CRC       = %d\r\n", (reg & Hw::CONFIG_EN_CRC) ? 1 : 0);
	printf("  -> CONFIG_MASK_MAX_RT  = %d\r\n", (reg & Hw::CONFIG_MASK_MAX_RT) ? 1 : 0);
	printf("  -> CONFIG_MASK_TX_DS   = %d\r\n", (reg & Hw::CONFIG_MASK_TX_DS) ? 1 : 0);
	printf("  -> CONFIG_MASK_RX_DR   = %d\r\n", (reg & Hw::CONFIG_MASK_RX_DR) ? 1 : 0);
	
	// REG_EN_AA
	reg = readRegister(Hw::REG_EN_AA);
	printf("0x%02X: REG_EN_AA = 0x%02X\r\n", Hw::REG_EN_AA, reg);
	for (int i = 0; i < 6; i++) {
		printf("  -> PIPE%d = %d\r\n", i, ((reg & (1 << i)) ? 1 : 0));
	}
	
	// REG_EN_RXADDR
	reg = readRegister(Hw::REG_EN_RXADDR);
	printf("0x%02X: REG_EN_RXADDR = 0x%02X\r\n", Hw::REG_EN_RXADDR, reg);
	for (int i = 0; i < 6; i++) {
		printf("  -> PIPE%d = %d\r\n", i, ((reg & (1 << i)) ? 1 : 0));
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
	printf("  -> SETUP_RF_LNA_HCURR      = %d\r\n", (reg & Hw::SETUP_RF_LNA_HCURR) ? 1 : 0);
	printf("  -> SETUP_RF_PLL_LOCK       = %d\r\n", (reg & Hw::SETUP_RF_PLL_LOCK) ? 1 : 0);
	printf("  -> SETUP_RF_DR_HIGH        = %d\r\n", (reg & Hw::SETUP_RF_DR_HIGH) ? 1 : 0);
	printf("  -> SETUP_RF_DR_LOW         = %d\r\n", (reg & Hw::SETUP_RF_DR_LOW) ? 1 : 0);
	printf("  -> SETUP_RF_CONT_WAVE      = %d\r\n", (reg & Hw::SETUP_RF_CONT_WAVE) ? 1 : 0);
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
	printf("  -> STATUS_TX_FULL      = %d\r\n", (reg & Hw::STATUS_TX_FULL) ? 1 : 0);
	printf("  -> STATUS_MAX_RT       = %d\r\n", (reg & Hw::STATUS_MAX_RT) ? 1 : 0);
	printf("  -> STATUS_TX_DS        = %d\r\n", (reg & Hw::STATUS_TX_DS) ? 1 : 0);
	printf("  -> STATUS_RX_DR        = %d\r\n", (reg & Hw::STATUS_RX_DR) ? 1 : 0);
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
	printf("  -> FIFO_STATUS_RX_EMPTY      = %d\r\n", (reg & Hw::FIFO_STATUS_RX_EMPTY) ? 1 : 0);
	printf("  -> FIFO_STATUS_RX_FULL       = %d\r\n", (reg & Hw::FIFO_STATUS_RX_FULL) ? 1 : 0);
	printf("  -> FIFO_STATUS_TX_EMPTY      = %d\r\n", (reg & Hw::FIFO_STATUS_TX_EMPTY) ? 1 : 0);
	printf("  -> FIFO_STATUS_TX_FULL       = %d\r\n", (reg & Hw::FIFO_STATUS_TX_FULL) ? 1 : 0);
	printf("  -> FIFO_STATUS_TX_REUSE      = %d\r\n", (reg & Hw::FIFO_STATUS_TX_REUSE) ? 1 : 0);
	
	// REG_DYNPD
	reg = readRegister(Hw::REG_DYNPD);
	printf("0x%02X: REG_DYNPD = 0x%02X\r\n", Hw::REG_DYNPD, reg);
	for (int i = 0; i < 6; i++) {
		printf("  -> PIPE%d = %d\r\n", i, ((reg & (1 << i)) ? 1 : 0));
	}
	
	// REG_FEATURE
	reg = readRegister(Hw::REG_FEATURE);
	printf("0x%02X: REG_FEATURE = 0x%02X\r\n", Hw::REG_FEATURE, reg);
	printf("  -> FEATURE_EN_DYN_ACK     = %d\r\n", (reg & Hw::FEATURE_EN_DYN_ACK) ? 1 : 0);
	printf("  -> FEATURE_EN_ACK_PAY     = %d\r\n", (reg & Hw::FEATURE_EN_ACK_PAY) ? 1 : 0);
	printf("  -> FEATURE_EN_DPL         = %d\r\n", (reg & Hw::FEATURE_EN_DPL) ? 1 : 0);
}

void Nrf24::close() {
	configASSERT(m_cs_level == 0);
	
	if (m_pins.irq.bank)
		Exti::remove(m_pins.irq.bank, m_pins.irq.pin);
}

Nrf24::~Nrf24() {
	close();
}
