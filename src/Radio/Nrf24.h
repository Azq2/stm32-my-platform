#pragma once

#include "../Spi.h"

#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <semphr.h>

class Nrf24 {
	public:
		static constexpr uint32_t TPD2STBY				= 150;
		
		// Minimum timeout: MAX_ARD (4000us) * MAX_ARC (15) = 60 ms
		// But make it with little margin, for worth case (eg. RTOS lags).
		static constexpr uint32_t DEFAULT_WRITE_TIMEOUT	= 150;
		
		// NRF24 hardware registers and bits
		struct Hw {
			enum Cmd: uint8_t {
				CMD_R_REGISTER			= 0x00,
				CMD_W_REGISTER			= 0x20,
				
				CMD_LOCK_UNLOCK			= 0x50,
				CMD_R_RX_PL_WID			= 0x60,
				CMD_R_RX_PAYLOAD		= 0x61,
				CMD_W_TX_PAYLOAD		= 0xA0,
				CMD_W_ACK_PAYLOAD		= 0xA8,
				CMD_W_TX_PAYLOAD_NOACK	= 0xB0,
				
				CMD_FLUSH_TX			= 0xE1,
				CMD_FLUSH_RX			= 0xE2,
				
				CMD_REUSE_TX_P			= 0xE3,
				CMD_NOP					= 0xFF
			};
			
			enum Reg: uint8_t {
				REG_CONFIG				= 0x00,
				REG_EN_AA				= 0x01,
				REG_EN_RXADDR			= 0x02,
				REG_SETUP_AW			= 0x03,
				REG_SETUP_RETR			= 0x04,
				REG_RF_CH				= 0x05,
				REG_RF_SETUP			= 0x06,
				REG_STATUS				= 0x07,
				REG_OBSERVE_TX			= 0x08,
				REG_RPD					= 0x09,
				REG_RX_ADDR_P0			= 0x0A,
				REG_RX_ADDR_P1			= 0x0B,
				REG_RX_ADDR_P2			= 0x0C,
				REG_RX_ADDR_P3			= 0x0D,
				REG_RX_ADDR_P4			= 0x0E,
				REG_RX_ADDR_P5			= 0x0F,
				REG_TX_ADDR				= 0x10,
				REG_RX_PW_P0			= 0x11,
				REG_RX_PW_P1			= 0x12,
				REG_RX_PW_P2			= 0x13,
				REG_RX_PW_P3			= 0x14,
				REG_RX_PW_P4			= 0x15,
				REG_RX_PW_P5			= 0x16,
				REG_FIFO_STATUS			= 0x17,
				REG_DYNPD				= 0x1C,
				REG_FEATURE				= 0x1D
			};
			
			enum Masks {
				REG_MASK				= 0x1F,
				PIPE_MASK				= 0x3F,
				
				// REG_CONFIG
				CONFIG_CRC_SHIFT		= 2,
				CONFIG_CRC_MASK			= 0x0C,
				
				CONFIG_IRQ_SHIFT		= 4,
				CONFIG_IRQ_MASK			= 0x70,
				
				// REG_SETUP_AW
				SETUP_AW_SHIFT			= 0,
				SETUP_AW_MASK			= 0x03,
				
				// REG_SETUP_RETR
				SETUP_RETR_ARC_SHIFT	= 0,
				SETUP_RETR_ARC_MASK		= 0x0F,
				
				SETUP_RETR_ARD_SHIFT	= 4,
				SETUP_RETR_ARD_MASK		= 0xF0,
				
				// REG_RF_CH
				RF_CH_SHIFT				= 0,
				RF_CH_MASK				= 0x3F,
				
				// REG_RF_SETUP
				SETUP_RF_PWR_SHIFT		= 1,
				SETUP_RF_PWR_MASK		= 0x06,
				
				STATUS_RX_P_NO_SHFIT	= 1,
				STATUS_RX_P_NO_MASK		= 0xE,
				
				// REG_OBSERVE_TX
				OBSERVE_TX_ARC_CNT_SHIFT	= 0,
				OBSERVE_TX_ARC_CNT_MASK		= 0x0F,
				
				OBSERVE_TX_PLOS_CNT_SHIFT	= 4,
				OBSERVE_TX_PLOS_CNT_MASK	= 0xF0,
				
				// REG_RX_PW_*
				RX_PW_SHIFT					= 0,
				RX_PW_MASK					= 0x3F,
				
				// REG_DYNPD
				DYNPD_SHIFT					= 0,
				DYNPD_MASK					= 0x3F
			};
			
			enum Bits: uint8_t {
				// REG_CONFIG
				CONFIG_PRIM_RX			= 1 << 0,
				CONFIG_PWR_UP			= 1 << 1,
				CONFIG_CRCO				= 1 << 2,
				CONFIG_EN_CRC			= 1 << 3,
				CONFIG_MASK_MAX_RT		= 1 << 4,
				CONFIG_MASK_TX_DS		= 1 << 5,
				CONFIG_MASK_RX_DR		= 1 << 6,
				
				// REG_RF_SETUP
				SETUP_RF_LNA_HCURR		= 1 << 0,
				SETUP_RF_PLL_LOCK		= 1 << 4,
				SETUP_RF_DR_HIGH		= 1 << 3,
				SETUP_RF_DR_LOW			= 1 << 5,
				SETUP_RF_CONT_WAVE		= 1 << 7,
				
				// REG_STATUS
				STATUS_TX_FULL			= 1 << 0,
				
				STATUS_MAX_RT			= 1 << 4,
				STATUS_TX_DS			= 1 << 5,
				STATUS_RX_DR			= 1 << 6,
				
				// REG_FIFO_STATUS
				FIFO_STATUS_RX_EMPTY		= 1 << 0,
				FIFO_STATUS_RX_FULL			= 1 << 1,
				FIFO_STATUS_TX_EMPTY		= 1 << 4,
				FIFO_STATUS_TX_FULL			= 1 << 5,
				FIFO_STATUS_TX_REUSE		= 1 << 6,
				
				// REG_FEATURE
				FEATURE_EN_DYN_ACK			= 1 << 0,
				FEATURE_EN_ACK_PAY			= 1 << 1,
				FEATURE_EN_DPL				= 1 << 2,
			};
			
			enum Values: uint8_t {
				// REG_SETUP_AW
				SETUP_AW_3B		= 1,
				SETUP_AW_4B		= 2,
				SETUP_AW_5B		= 3,
			};
		};
		
		enum Pipe: uint8_t {
			PIPE0		= 0,
			PIPE1		= 1,
			PIPE2		= 2,
			PIPE3		= 3,
			PIPE4		= 4,
			PIPE5		= 5,
			PIPETX		= 6,
			NO_PIPE		= 7
		};
		
		enum Features {
			FEATURE_DYN_ACK		= Hw::FEATURE_EN_DYN_ACK,
			FEATURE_ACK_PAY		= Hw::FEATURE_EN_ACK_PAY,
			FEATURE_DPL			= Hw::FEATURE_EN_DPL,
		};
		
		enum DataRate: uint8_t {
			DATA_RATE_250K		= Hw::SETUP_RF_DR_LOW,
			DATA_RATE_1M		= 0,
			DATA_RATE_2M		= Hw::SETUP_RF_DR_HIGH,
			DATA_RATE_INVALID	= Hw::SETUP_RF_DR_HIGH | Hw::SETUP_RF_DR_LOW
		};
		
		enum CrcMode: uint8_t {
			CRC_OFF		= 0,
			CRC_8BIT	= Hw::CONFIG_EN_CRC,
			CRC_16BIT	= Hw::CONFIG_EN_CRC | Hw::CONFIG_CRCO,
		};
		
		enum IrqMask: uint8_t {
			IRQ_MASK_MAX_RT	= Hw::CONFIG_MASK_MAX_RT,
			IRQ_MASK_TX_DS	= Hw::CONFIG_MASK_TX_DS,
			IRQ_MASK_RX_DR	= Hw::CONFIG_MASK_RX_DR
		};
		
		enum AutoRetrDelay: uint8_t {
			ARD_250US	= 0x00,
			ARD_500US	= 0x01,
			ARD_750US	= 0x02,
			ARD_1000US	= 0x03,
			ARD_1250US	= 0x04,
			ARD_1500US	= 0x05,
			ARD_1750US	= 0x06,
			ARD_2000US	= 0x07,
			ARD_2250US	= 0x08,
			ARD_2500US	= 0x09,
			ARD_2750US	= 0x0A,
			ARD_3000US	= 0x0B,
			ARD_3250US	= 0x0C,
			ARD_3500US	= 0x0D,
			ARD_3750US	= 0x0E,
			ARD_4000US	= 0x0F
		};
		
		enum TxPower: uint8_t {
			TX_POWER_18dBm		= 0,
			TX_POWER_12dBm		= 1,
			TX_POWER_6dBm		= 2,
			TX_POWER_0dBm		= 3
		};
		
		enum Mode {
			MODE_TX	= 0,
			MODE_RX	= 1,
		};
		
		enum Errors {
			ERR_SUCCESS			= 0,
			ERR_FIFO_EMPTY		= ERR_SUCCESS,
			ERR_UNKNOWN			= -1,
			ERR_INVALID_ARGS	= -2,
			ERR_TIMEOUT			= -3
		};
		
		struct Pin {
			uint32_t bank;
			uint16_t pin;
			bool negative;
		};
		
		struct Pinout {
			Pin irq;
			Pin cs;
			Pin ce;
		};
		
		// Strange value for unlocking some features
		static constexpr uint8_t UNLOCK_MAGIC_VALUE = 0x73;
	protected:
		Spi *m_spi;
		
		Pinout m_pins;
		int m_cs_level = 0;
		
		SemaphoreHandle_t m_irq_sem = nullptr;
		
		Nrf24 &operator=(const Nrf24 &);
		Nrf24(const Nrf24 &);
		
	public:
		explicit Nrf24(Spi *spi, const Pinout &pins);
		
		inline bool hasIrq() {
			bool value = gpio_get(m_pins.irq.bank, m_pins.irq.pin) != 0;
			return value != m_pins.irq.negative;
		}
		
		int open();
		void close();
		
		int reset();
		
		void ce(bool flag);
		
		inline void enable() {
			ce(true);
		}
		
		inline void disable() {
			ce(true);
		}
		
		void begin();
		void end();
		
		// TX
		int write(const void *buffer, uint8_t size, bool no_ack = false, uint32_t timeout_ms = DEFAULT_WRITE_TIMEOUT);
		
		// RX
		bool waitForPacket(uint32_t timeout_ms);
		bool hasPacket();
		int read(void *buffer, uint8_t *pipe = nullptr, uint8_t max_size = 32);
		
		// Power up
		inline void setPowerUp(bool enable) {
			updateRegister(Hw::REG_CONFIG, Hw::CONFIG_PWR_UP, (enable ? Hw::CONFIG_PWR_UP : 0));
		}
		
		inline bool isPowerUp() {
			return readRegisterMask(Hw::REG_CONFIG, Hw::CONFIG_PWR_UP, 0) != 0;
		}
		
		// Mode
		inline void setMode(Mode mode) {
			updateRegister(Hw::REG_CONFIG, Hw::CONFIG_PRIM_RX, (mode == MODE_RX ? Hw::CONFIG_PRIM_RX : 0));
		}
		
		inline Mode getMode() {
			return readRegisterMask(Hw::REG_CONFIG, Hw::CONFIG_PRIM_RX, 0) ? MODE_RX : MODE_TX;
		}
		
		// CRC
		inline void setCrcMode(CrcMode crc) {
			updateRegister(Hw::REG_CONFIG, Hw::CONFIG_CRC_MASK, crc);
		}
		
		inline CrcMode getCrcMode() {
			return (CrcMode) readRegisterMask(Hw::REG_CONFIG, Hw::CONFIG_CRC_MASK, 0);
		}
		
		// Channel
		inline void setChannel(uint8_t channel) {
			configASSERT(channel <= 127);
			writeRegister(Hw::REG_RF_CH, channel);
		}
		
		inline void setFrequency(uint8_t freq) {
			configASSERT(freq >= 2400 && freq <= 2527);
			writeRegister(Hw::REG_RF_CH, freq - 2400);
		}
		
		inline uint8_t getChannel() {
			return readRegister(Hw::REG_RF_CH);
		}
		
		inline uint8_t getFrequency() {
			return 2400 + readRegister(Hw::REG_RF_CH);
		}
		
		// Auto retransmit
		inline void setAutoRetransmit(uint8_t ard, uint8_t arc) {
			configASSERT(arc <= 15);
			updateRegister(Hw::REG_SETUP_RETR, Hw::SETUP_RETR_ARC_MASK | Hw::SETUP_RETR_ARD_MASK,
				(ard << Hw::SETUP_RETR_ARD_SHIFT) | (arc << Hw::SETUP_RETR_ARC_SHIFT));
		}
		
		inline uint8_t getAutoRetransmitRetr() {
			return readRegisterMask(Hw::REG_SETUP_RETR, Hw::SETUP_RETR_ARD_MASK, Hw::SETUP_RETR_ARD_SHIFT);
		}
		
		inline uint8_t getAutoRetransmitCount() {
			return readRegisterMask(Hw::REG_SETUP_RETR, Hw::SETUP_RETR_ARC_MASK, Hw::SETUP_RETR_ARC_SHIFT);
		}
		
		// Address width
		inline void setAddrWidth(uint8_t width) {
			configASSERT(width >= 3 && width <= 5);
			writeRegister(Hw::REG_SETUP_AW, width - 2);
		}
		
		inline uint8_t getAddrWidth() {
			return readRegister(Hw::REG_SETUP_AW) + 2;
		}
		
		// Address
		int setAddr(uint8_t pipe, uint64_t addr);
		uint64_t getAddr(uint8_t pipe);
		
		// Tx power
		inline void setTxPower(TxPower power) {
			updateRegister(Hw::REG_RF_SETUP, Hw::SETUP_RF_PWR_MASK, power << Hw::SETUP_RF_PWR_SHIFT);
		}
		
		inline TxPower getTxPower() {
			return (TxPower) readRegisterMask(Hw::REG_RF_SETUP, Hw::SETUP_RF_PWR_MASK, Hw::SETUP_RF_PWR_SHIFT);
		}
		
		// LNA
		inline void enableLna(bool enable) {
			updateRegister(Hw::REG_RF_SETUP, Hw::SETUP_RF_LNA_HCURR, (enable ? Hw::SETUP_RF_LNA_HCURR : 0));
		}
		
		inline bool isLnaEnabled() {
			return readRegisterMask(Hw::REG_RF_SETUP, Hw::SETUP_RF_LNA_HCURR, 0) != 0;
		}
		
		// Data rate
		inline void setDataRate(DataRate dr) {
			updateRegister(Hw::REG_RF_SETUP, Hw::SETUP_RF_DR_HIGH | Hw::SETUP_RF_DR_LOW, dr);
		}
		
		inline DataRate getDataRate() {
			return (DataRate) readRegisterMask(Hw::REG_RF_SETUP, Hw::SETUP_RF_DR_HIGH | Hw::SETUP_RF_DR_LOW, 0);
		}
		
		// Rx pipe enabling
		inline void enableRxPipe(uint8_t pipe, bool enable) {
			configASSERT(pipe < 6);
			updateRegister(Hw::REG_EN_RXADDR, (1 << pipe), (enable ? 1 << pipe : 0));
		}
		
		inline bool isRxPipeEnabled(uint8_t pipe) {
			configASSERT(pipe < 6);
			return readRegisterMask(Hw::REG_EN_RXADDR, (1 << pipe), 0) != 0;
		}
		
		// Auto Ack
		inline void enableAutoAck(uint8_t pipe, bool enable) {
			configASSERT(pipe < 6);
			updateRegister(Hw::REG_EN_AA, (1 << pipe), (enable ? 1 << pipe : 0));
		}
		
		inline bool isAutoAckEnabled(uint8_t pipe) {
			configASSERT(pipe < 6);
			return readRegisterMask(Hw::REG_EN_AA, (1 << pipe), 0) != 0;
		}
		
		// Dynamic Payload
		inline void enableDynamicPayload(uint8_t pipe, bool enable) {
			configASSERT(pipe < 6);
			return updateRegister(Hw::REG_DYNPD, (1 << pipe), (enable ? 1 << pipe : 0));
		}
		
		inline bool isDynamicPayloadEnabled(uint8_t pipe) {
			configASSERT(pipe < 6);
			return readRegisterMask(Hw::REG_DYNPD, (1 << pipe), 0) != 0;
		}
		
		inline void setPayloadSize(uint8_t pipe, uint8_t size) {
			configASSERT(pipe < 6);
			writeRegister(Hw::REG_RX_PW_P0 + pipe, size);
		}
		
		inline uint8_t getPayloadSize(uint8_t pipe) {
			configASSERT(pipe < 6);
			return readRegister(Hw::REG_RX_PW_P0 + pipe);
		}
		
		inline uint8_t getDynamicPayloadSize() {
			return cmdR(Hw::CMD_R_RX_PL_WID);
		}
		
		// Status
		inline uint8_t getStatus() {
			return readRegister(Hw::REG_STATUS);
		}
		
		inline uint8_t getFifoStatus() {
			return readRegister(Hw::REG_FIFO_STATUS);
		}
		
		inline uint8_t getRxPipe() {
			return readRegisterMask(Hw::REG_STATUS, Hw::STATUS_RX_P_NO_MASK, Hw::STATUS_RX_P_NO_SHFIT);
		}
		
		inline bool isConnected() {
			// TODO: more better way...
			uint8_t addr_width = getAddrWidth();
			return addr_width >= 3 && addr_width <= 5;
		}
		
		// Statistic
		inline uint8_t getStatPacketLoss() {
			return readRegisterMask(Hw::REG_OBSERVE_TX, Hw::OBSERVE_TX_PLOS_CNT_MASK, Hw::OBSERVE_TX_PLOS_CNT_SHIFT);
		}
		
		inline uint8_t getStatRetrCount() {
			return readRegisterMask(Hw::REG_OBSERVE_TX, Hw::OBSERVE_TX_ARC_CNT_MASK, Hw::OBSERVE_TX_ARC_CNT_SHIFT);
		}
		
		inline void clearStatPacketLoss() {
			// Dummy write to REG_RF_CH reseting PLOS counter
			updateRegister(Hw::REG_RF_CH, 0, 0);
		}
		
		// RPD (Received Power Detector)
		inline bool getRpd() {
			return readRegister(Hw::REG_RPD) != 0;
		}
		
		// Payload
		inline void writeTxPayload(const void *buffer, uint8_t length, bool no_ack = false) {
			writeRegister(no_ack ? Hw::CMD_W_TX_PAYLOAD_NOACK : Hw::CMD_W_TX_PAYLOAD, buffer, length);
		}
		
		inline void readRxPayload(void *buffer, uint8_t length) {
			readRegister(Hw::CMD_R_RX_PAYLOAD, buffer, length);
		}
		
		// Features
		inline void setFeatures(Features features) {
			updateRegister(Hw::REG_FEATURE, FEATURE_ACK_PAY | FEATURE_DPL | FEATURE_DYN_ACK, features);
		}
		
		// FIFO management
		inline void flushTx() {
			cmdW(Hw::CMD_FLUSH_TX);
		}
		
		inline void flushRx() {
			cmdW(Hw::CMD_FLUSH_RX);
		}
		
		// Irq
		inline void clearIrqTxFlags() {
			updateRegister(Hw::REG_STATUS, 0, Hw::STATUS_MAX_RT | Hw::STATUS_TX_DS);
		}
		
		inline void clearIrqRxFlags() {
			updateRegister(Hw::REG_STATUS, 0, Hw::STATUS_RX_DR);
		}
		
		inline void maskIrq(IrqMask mask) {
			updateRegister(Hw::REG_CONFIG, mask, mask);
		}
		
		inline void unmaskIrq(IrqMask mask) {
			updateRegister(Hw::REG_CONFIG, mask, 0);
		}
		
		inline IrqMask getIrqMask() {
			return (IrqMask) readRegisterMask(Hw::REG_CONFIG, Hw::CONFIG_IRQ_MASK, 0);
		}
		
		uint8_t waitForIrqFlags(uint8_t flags, uint32_t timeout_ms);
		
		// Commands
		void cmdR(uint8_t reg, void *value, int length);
		
		inline uint8_t cmdR(uint8_t reg) {
			uint8_t value;
			cmdR(reg, &value, 1);
			return value;
		}
		
		void cmdW(uint8_t reg, const void *value = nullptr, int length = 1);
		
		inline void cmdW(uint8_t reg, uint8_t value) {
			cmdW(reg, &value, 1);
		}
		
		// Registers
		void updateRegister(uint8_t reg, uint8_t clear_bits, uint8_t set_bits);
		
		uint8_t readRegisterMask(uint8_t reg, uint8_t mask, uint8_t shift);
		
		inline void readRegister(uint8_t reg, void *value, int length) {
			cmdR(Hw::CMD_R_REGISTER | reg, value, length);
		}
		
		inline uint8_t readRegister(uint8_t reg) {
			return cmdR(Hw::CMD_R_REGISTER | reg);
		}
		
		inline void writeRegister(uint8_t reg, const void *value, int length) {
			cmdW(Hw::CMD_W_REGISTER | reg, value, length);
		}
		
		inline void writeRegister(uint8_t reg, uint8_t value) {
			cmdW(Hw::CMD_W_REGISTER | reg, &value, 1);
		}
		
		void handleIrq();
		
		void printState(int (*printf)(const char *, ... ));
		
		~Nrf24();
};
