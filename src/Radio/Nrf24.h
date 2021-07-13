#pragma once

#include "../Spi.h"

class Nrf24 {
	public:
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
				FIFO_STATUS_RX_FILL			= 1 << 1,
				FIFO_STATUS_TX_EMPTY		= 1 << 4,
				FIFO_STATUS_TX_FILL			= 1 << 5,
				FIFO_STATUS_TX_REUSE		= 1 << 6,
				
				// REG_FEATURE
				FEATURE_EN_DYN_ACK			= 1 << 0,
				FEATURE_EN_ACK_PAY			= 1 << 1,
				FEATURE_EN_DPL				= 1 << 2,
			};
		};
		
		enum Pipe: uint8_t {
			PIPE0	= 0,
			PIPE1	= 1,
			PIPE2	= 2,
			PIPE3	= 3,
			PIPE4	= 4,
			PIPE5	= 5,
			PIPETX	= 6
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
			ERR_BUS				= -1,
			ERR_INVALID_ARGS	= -2,
			ERR_UNKNOWN			= -3
		};
		
		// Strange value for unlocking some features
		static constexpr uint8_t UNLOCK_MAGIC_VALUE = 0x73;
	protected:
		Spi *m_spi = nullptr;
		uint32_t m_cs_bank = 0;
		uint32_t m_cs_pin = 0;
		bool m_cs_negative = true;
		int m_cs_level = 0;
		
		Nrf24 &operator=(const Nrf24 &);
		Nrf24(const Nrf24 &);
		
	public:
		Nrf24(Spi *spi, uint32_t cs_bank, uint32_t cs_pin, bool cs_negative = true);
		int reset();
		
		void begin();
		void end();
		
		inline int getDynamicPayloadSize() {
			return cmdR(Hw::CMD_R_RX_PL_WID);
		}
		
		inline int setPowerUp(bool enable) {
			return updateRegister(Hw::REG_CONFIG, Hw::CONFIG_PWR_UP, (enable ? Hw::CONFIG_PWR_UP : 0));
		}
		
		inline int setMode(Mode mode) {
			return updateRegister(Hw::REG_CONFIG, Hw::CONFIG_PRIM_RX, (mode == MODE_RX ? Hw::CONFIG_PRIM_RX : 0));
		}
		
		inline int setCrcMode(CrcMode crc) {
			return updateRegister(Hw::REG_CONFIG, Hw::CONFIG_CRC_MASK, crc);
		}
		
		inline int setChannel(uint8_t channel) {
			return writeRegister(Hw::REG_RF_CH, channel);
		}
		
		inline int setAutoRetransmit(uint8_t ard, uint8_t arc) {
			return updateRegister(Hw::REG_SETUP_RETR, Hw::SETUP_RETR_ARC_MASK | Hw::SETUP_RETR_ARD_MASK,
				(ard << Hw::SETUP_RETR_ARD_SHIFT) | (arc << Hw::SETUP_RETR_ARC_SHIFT));
		}
		
		inline int setAddrWidth(uint8_t width) {
			if (width < 3 || width > 5)
				return ERR_INVALID_ARGS;
			return writeRegister(Hw::REG_SETUP_AW, width - 2);
		}
		
		inline int getAddrWidth() {
			int ret = readRegister(Hw::REG_SETUP_AW);
			return ret < 0 ? ret : ret + 2;
		}
		
		int setAddr(uint8_t pipe, const uint8_t *addr, uint8_t length);
		
		inline int setTxPower(TxPower power) {
			return updateRegister(Hw::REG_RF_SETUP, Hw::SETUP_RF_PWR_MASK, power << Hw::SETUP_RF_PWR_SHIFT);
		}
		
		inline int enableLna(bool enable) {
			return updateRegister(Hw::REG_RF_SETUP, Hw::SETUP_RF_LNA_HCURR, (enable ? Hw::SETUP_RF_LNA_HCURR : 0));
		}
		
		inline int setDataRate(DataRate dr) {
			return updateRegister(Hw::REG_RF_SETUP, Hw::SETUP_RF_DR_HIGH | Hw::SETUP_RF_DR_LOW, dr);
		}
		
		inline int enableRxPipe(uint8_t pipe, bool enable) {
			if (pipe > 5)
				return ERR_INVALID_ARGS;
			return updateRegister(Hw::REG_EN_RXADDR, (1 << pipe), (enable ? 1 << pipe : 0));
		}
		
		inline int enableAA(uint8_t pipe, bool enable) {
			if (pipe > 5)
				return ERR_INVALID_ARGS;
			return updateRegister(Hw::REG_EN_AA, (1 << pipe), (enable ? 1 << pipe : 0));
		}
		
		inline int setPayloadSize(uint8_t pipe, uint8_t size) {
			if (pipe > 5 || size > 32)
				return ERR_INVALID_ARGS;
			return writeRegister(Hw::REG_RX_PW_P0 + pipe, size);
		}
		
		inline int getPayloadSize(uint8_t pipe) {
			if (pipe > 5 )
				return ERR_INVALID_ARGS;
			int ret = readRegister(Hw::REG_RX_PW_P0 + pipe);
			if (ret > 32)
				return ERR_UNKNOWN;
			return ret;
		}
		
		inline int getStatus() {
			return readRegister(Hw::REG_STATUS);
		}
		
		inline int getFifoStatus() {
			return readRegister(Hw::REG_FIFO_STATUS);
		}
		
		inline int getRxPipe() {
			return readRegisterMask(Hw::REG_FIFO_STATUS, Hw::STATUS_RX_P_NO_MASK, Hw::STATUS_RX_P_NO_SHFIT);
		}
		
		inline int getPlos() {
			return readRegisterMask(Hw::REG_OBSERVE_TX, Hw::OBSERVE_TX_PLOS_CNT_MASK, Hw::OBSERVE_TX_PLOS_CNT_SHIFT);
		}
		
		inline int getArc() {
			return readRegisterMask(Hw::REG_OBSERVE_TX, Hw::OBSERVE_TX_ARC_CNT_MASK, Hw::OBSERVE_TX_ARC_CNT_SHIFT);
		}
		
		inline int getRpd() {
			return readRegister(Hw::REG_RPD);
		}
		
		inline int writeTxPayload(uint8_t *buffer, uint8_t length) {
			return writeRegister(Hw::CMD_W_TX_PAYLOAD, buffer, length);
		}
		
		inline int readRxPayload(uint8_t *buffer, uint8_t length) {
			return readRegister(Hw::CMD_R_RX_PAYLOAD, buffer, length);
		}
		
		inline int setFeatures(Features features) {
			return updateRegister(Hw::REG_FEATURE, FEATURE_ACK_PAY | FEATURE_DPL | FEATURE_DYN_ACK, features);
		}
		
		inline int flushTx() {
			return cmdW(Hw::CMD_FLUSH_TX);
		}
		
		inline int flushRx() {
			return cmdW(Hw::CMD_FLUSH_RX);
		}
		
		inline int clearIrqFlags() {
			return updateRegister(Hw::REG_STATUS, 0, Hw::STATUS_MAX_RT | Hw::STATUS_TX_DS | Hw::STATUS_RX_DR);
		}
		
		inline int clearPlos() {
			// Dummy write to REG_RF_CH reseting PLOS counter
			return updateRegister(Hw::REG_RF_CH, 0, 0);
		}
		
		// Commands
		int cmdR(uint8_t reg, uint8_t *value, int length);
		
		inline int cmdR(uint8_t reg) {
			uint8_t value;
			int ret = cmdR(reg, &value, 1);
			return ret < 0 ? ret : value;
		}
		
		int cmdW(uint8_t reg, const uint8_t *value = nullptr, int length = 1);
		
		inline int cmdW(uint8_t reg, uint8_t value) {
			return cmdW(reg, &value, 1);
		}
		
		// Registers
		int updateRegister(uint8_t reg, uint8_t clear_bits, uint8_t set_bits);
		
		int readRegisterMask(uint8_t reg, uint8_t mask, uint8_t shift);
		
		inline int readRegister(uint8_t reg, uint8_t *value, int length) {
			return cmdR(Hw::CMD_R_REGISTER | reg, value, length);
		}
		
		inline int readRegister(uint8_t reg) {
			return cmdR(Hw::CMD_R_REGISTER | reg);
		}
		
		inline int writeRegister(uint8_t reg, const uint8_t *value, int length) {
			return cmdW(Hw::CMD_W_REGISTER | reg, value, length);
		}
		
		inline int writeRegister(uint8_t reg, uint8_t value) {
			return cmdW(Hw::CMD_W_REGISTER | reg, &value, 1);
		}
		
		void printState(int (*printf)(const char *, ... ));
		
		~Nrf24();
};
