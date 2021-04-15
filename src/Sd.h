#pragma once

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <cstdio>
#include <cerrno>
#include <cstring>
#include <unistd.h>

#include "Sdio.h"

class Sd {
	public:
		struct Csd {
			uint32_t taac;
			uint16_t nsac;
			uint32_t tran_speed;
			
			uint16_t read_bl_len;
			uint16_t write_bl_len;
			
			uint64_t capacity;
			
			// uint32_t vdd_r_curr_min;
			// uint32_t vdd_r_curr_max;
			// uint32_t vdd_w_curr_min;
			// uint32_t vdd_w_curr_max;
			
			uint8_t sector_size;
			uint8_t wp_grp_size;
			
			uint8_t r2w_factor;
			
			uint16_t ccc:12;
			bool wp_grp_enable:1;
			bool erase_blk_en:1;
			bool write_bl_partial:1;
			bool perm_write_protect:1;
			bool tmp_write_protect:1;
			bool read_bl_partial:1;
			bool write_blk_misalign:1;
			bool read_blk_misalign:1;
			bool dsr_imp:1;
			uint8_t csd_structure:2;
			
			// uint8_t ecc:2;
			// uint8_t default_ecc:2;
			// uint8_t content_prot_app:1;
			// uint8_t file_format_grp:1;
			// uint8_t copy:1;
			// uint8_t file_format:2;
			// uint8_t spec_vers:4;
		};
		
		struct Cid {
			uint8_t mid;
			uint16_t oid;
			char pnm[6];
			uint8_t prv;
			uint32_t psn;
			uint16_t mdt_year;
			uint8_t mdt_month;
		};
		
		enum {
			SCR_BUS_WIDTH_1		= 1 << 0,
			SCR_BUS_WIDTH_4		= 1 << 2,
		};
		
		struct Scr {
			uint8_t scr_structure:4;
			uint8_t sd_spec:4;
			uint8_t sd_bus_widths:4;
			uint8_t ex_security:4;
			uint8_t sd_security:3;
			uint8_t cmd_support:2;
			bool data_stat_after_erase:1;
			bool sd_spec3:1;
		};
	
	protected:
		enum {
			CARD_UNKNOWN	= 0,
			CARD_V1			= 1,
			CARD_V2			= 2
		};
		
		enum {
			CARD_SDSC		= 0,
			CARD_SDHC		= 1
		};
		
		enum {
			OCR_VOLTAGE_MASK		= 0x1FFFFF0,
			OCR_VOLTAGE_SHIFT		= 4,
			
			OCR_VOLTAGE_2V7_2V8		= 1 << 15,
			OCR_VOLTAGE_2V8_2V9		= 1 << 16,
			OCR_VOLTAGE_2V9_3V0		= 1 << 17,
			OCR_VOLTAGE_3V0_3V1		= 1 << 18,
			OCR_VOLTAGE_3V1_3V2		= 1 << 19,
			OCR_VOLTAGE_3V2_3V3		= 1 << 20,
			OCR_VOLTAGE_3V3_3V4		= 1 << 21,
			OCR_VOLTAGE_3V4_3V5		= 1 << 22,
			OCR_VOLTAGE_3V5_3V6		= 1 << 23,
			OCR_S18R				= 1 << 24,
			OCR_S18A				= OCR_S18R,
			OCR_HCS					= 1 << 30,
			OCR_NOT_BUSY			= 1 << 31
		};
		
		enum {
			R7_VHS_2V7_3V6			= 1,
		};
		
		enum {
			R7_PATTERN_SHIFT		= 0,
			R7_PATTERN_MASK			= 0xFF,
			
			R7_VOLTAGE_SHIFT		= 8,
			R7_VOLTAGE_MASK			= 0xF00,
			
			R7_CMD_VERSION_SHIFT	= 28,
			R7_CMD_VERSION_MASK		= 0xF0000000
		};
		
		enum {
			R6_AKE_SEQ_ERROR		= 1 << 3,	// ER
			R6_APP_CMD				= 1 << 5,	// SR
			R6_FX_EVENT				= 1 << 6,	// SX
			R6_READY_FOR_DATA		= 1 << 8,	// SX
			R6_ERROR				= 1 << 13,	// ERX
			R6_ILLEGAL_COMMAND		= 1 << 14,	// ER
			R6_COM_CRC_ERROR		= 1 << 15,	// ER
			
			R6_ALL_ERRORS			= (
				R6_AKE_SEQ_ERROR | R6_ERROR | R6_ILLEGAL_COMMAND | R6_COM_CRC_ERROR
			),
			
			R6_RCA_SHIFT			= 16,
			R6_RCA_MASK				= 0xFFFF0000,
			
			R6_STATUS_SHIFT			= 0,
			R6_STATUS_MASK			= 0xFFFF
		};
		
		enum {
			CMD_GO_IDLE_STATE			= 0,
			CMD_ALL_SEND_CID			= 2,
			CMD_SEND_RELATIVE_ADDR		= 3,
			CMD_SET_BUS_WIDTH			= 6,
			CMD_SELECT_CARD				= 7,
			CMD_SEND_IF_COND			= 8,
			CMD_SEND_CSD				= 9,
			CMD_SEND_CID				= 10,
			CMD_STOP_TRANSMISSION		= 12,
			CMD_SEND_STATUS				= 13,
			CMD_SET_BLOCKLEN			= 16,
			CMD_READ_SINGLE_BLOCK		= 17,
			CMD_READ_MULTIPLE_BLOCK		= 18,
			CMD_WRITE_SINGLE_BLOCK		= 24,
			CMD_WRITE_MULTIPLE_BLOCK	= 25,
			CMD_SEND_SCR				= 51,
			CMD_APP_CMD					= 55,
			CMD_SD_SEND_OP_COND			= 41
		};
		
		enum {
			R1_AKE_SEQ_ERROR		= 1 << 3,	// ER
			R1_APP_CMD				= 1 << 5,	// SR
			R1_FX_EVENT				= 1 << 6,	// SX
			R1_READY_FOR_DATA		= 1 << 8,	// SX
			R1_ERASE_RESET			= 1 << 13,	// SR
			R1_CARD_ECC_DISABLED	= 1 << 14,	// SX
			R1_WP_ERASE_SKIP		= 1 << 15,	// ERX
			R1_CSD_OVERWRITE		= 1 << 16,	// ERX
			R1_UNDERRUN				= 1 << 17,	// EX
			R1_OVERRUN				= 1 << 17,	// EX
			R1_ERROR				= 1 << 19,	// ERX
			R1_CC_ERROR				= 1 << 20,	// ERX
			R1_CARD_ECC_FAILED		= 1 << 21,	// ERX
			R1_ILLEGAL_COMMAND		= 1 << 22,	// ER
			R1_COM_CRC_ERROR		= 1 << 23,	// ER
			R1_LOCK_UNLOCK_FAILED	= 1 << 24,	// ERX
			R1_CARD_IS_LOCKED		= 1 << 25,	// SX
			R1_WP_VIOLATION			= 1 << 26,	// ERX
			R1_ERASE_PARAM			= 1 << 27,	// ERX
			R1_ERASE_SEQ_ERROR		= 1 << 28,	// ER
			R1_BLOCK_LEN_ERROR		= 1 << 29,	// ERX
			R1_ADDRESS_ERROR		= 1 << 30,	// ERX
			R1_OUT_OF_RANGE			= 1 << 31,	// ERX
			
			R1_ALL_ERRORS			= (
				R1_AKE_SEQ_ERROR | R1_WP_ERASE_SKIP | R1_CSD_OVERWRITE |
				R1_UNDERRUN | R1_OVERRUN | R1_ERROR | R1_CC_ERROR | R1_CARD_ECC_FAILED |
				R1_ILLEGAL_COMMAND | R1_COM_CRC_ERROR | R1_LOCK_UNLOCK_FAILED | R1_WP_VIOLATION | R1_ERASE_PARAM |
				R1_ERASE_SEQ_ERROR | R1_BLOCK_LEN_ERROR | R1_ADDRESS_ERROR | R1_OUT_OF_RANGE |
				R1_ERASE_RESET
			),
			
			R1_STATUS_SHIFT			= 9,
			R1_STATUS_MASK			= 0x1E00
		};
		
		enum {
			R1_STATUS_IDLE		= 0,
			R1_STATUS_READY		= 1,
			R1_STATUS_IDENT		= 2,
			R1_STATUS_STBY		= 3,
			R1_STATUS_TRAN		= 4,
			R1_STATUS_DATA		= 5,
			R1_STATUS_RCV		= 6,
			R1_STATUS_PRG		= 7,
			R1_STATUS_DIS		= 8,
		};
		
		enum {
			CMD_WAIT_RESP	= 1 << 0,
			CMD_LONG_RESP	= 1 << 1,
			CMD_CRCFAIL		= 1 << 2,
			
			CMD_RESP_R1		= 1 << 3,
			CMD_RESP_R2		= 1 << 4,
			CMD_RESP_R3		= 1 << 5,
			CMD_RESP_R6		= 1 << 6,
			CMD_RESP_R7		= 1 << 7,
			
			CMD_R1			= CMD_RESP_R1 | CMD_WAIT_RESP,
			CMD_R1B			= CMD_RESP_R1 | CMD_WAIT_RESP,
			CMD_R2			= CMD_RESP_R2 | CMD_WAIT_RESP | CMD_LONG_RESP,
			CMD_R3			= CMD_RESP_R3 | CMD_WAIT_RESP | CMD_CRCFAIL,
			CMD_R6			= CMD_RESP_R6 | CMD_WAIT_RESP,
			CMD_R7			= CMD_RESP_R7 | CMD_WAIT_RESP,
		};
		
		// CMD_SET_BUS_WIDTH
		enum {
			SET_BUS_WIDTH_1		= 0,
			SET_BUS_WIDTH_4		= 2
		};
		
		enum {
			MODE_READ	= 0,
			MODE_WRITE	= 1
		};
		
		enum {
			CMD_ICR_FLAGS		= SDIO_ICR_CMDSENTC | SDIO_ICR_CMDRENDC | SDIO_ICR_CTIMEOUTC | SDIO_ICR_CCRCFAILC,
			CMD_MASK_FLAGS		= SDIO_MASK_CMDRENDIE | SDIO_MASK_CMDSENTIE | SDIO_MASK_CCRCFAILIE | SDIO_MASK_CTIMEOUTIE,
			CMD_UNMASK_FLAGS	= CMD_MASK_FLAGS,
			CMD_STA_FLAGS		= SDIO_STA_CMDSENT | SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL,
			
			DATA_ICR_FLAGS		= (
				SDIO_ICR_DATAENDC |
				SDIO_ICR_RXOVERRC |
				SDIO_ICR_TXUNDERRC |
				SDIO_ICR_DCRCFAILC |
				SDIO_ICR_DTIMEOUTC |
				SDIO_ICR_STBITERRC |
				SDIO_ICR_DBCKENDC
			),
			DATA_MASK_FLAGS		= (
				SDIO_MASK_DATAENDIE |
				SDIO_MASK_RXOVERRIE |
				SDIO_MASK_TXUNDERRIE |
				SDIO_MASK_DCRCFAILIE |
				SDIO_MASK_DTIMEOUTIE |
				SDIO_MASK_STBITERRIE |
				SDIO_MASK_DBCKENDIE
			),
			DATA_UNMASK_FLAGS	= (
				DATA_MASK_FLAGS |
				SDIO_MASK_RXDAVLIE |
				SDIO_MASK_RXFIFOHFIE |
				SDIO_MASK_TXFIFOHEIE |
				SDIO_MASK_TXFIFOEIE
			),
			DATA_STA_FLAGS		= (
				SDIO_STA_RXDAVL |
				SDIO_STA_DATAEND |
				SDIO_STA_RXOVERR |
				SDIO_STA_TXUNDERR |
				SDIO_STA_DCRCFAIL |
				SDIO_STA_DTIMEOUT |
				SDIO_STA_STBITERR |
				SDIO_STA_RXFIFOHF |
				SDIO_STA_TXFIFOHE |
				SDIO_STA_TXFIFOE |
				SDIO_STA_DBCKEND
			),
		};
		
		SemaphoreHandle_t m_data_sem = nullptr;
		SemaphoreHandle_t m_cmd_sem = nullptr;
		
		struct {
			struct {
				int error;
				uint8_t mode;
				union {
					char *ptr;
					uint32_t *ptr4;
				};
				uint32_t count;
				uint32_t block_length;
			} data;
			struct {
				int error;
				uint32_t flags;
			} cmd;
		} m_isr = {};
		
		uint32_t m_clock_freq = 25000000;
		uint8_t m_capacity_class = CARD_SDSC;
		uint8_t m_card_version = CARD_UNKNOWN;
		bool m_exclusive = false;
		
		uint16_t m_rca = 0;
		Csd m_csd = {};
		Cid m_cid = {};
		Scr m_scr = {};
		
		Sd &operator=(const Sd &);
		Sd(const Sd &);
		
		void initSdio();
		uint8_t toClockDiv(uint32_t freq);
		void setClock(bool enable, uint8_t div, Sdio::BusWidth bus_width);
		uint32_t getSdioClk();
		
		int sendCommand(uint8_t index, uint32_t arg, uint32_t flags = 0, uint32_t timeout_ms = 5000);
		
		int checkVoltageRange();
		
		bool decodeCid(const uint32_t *response, Cid *cid);
		bool decodeCsd(const uint32_t *response, Csd *csd);
		bool decodeScr(const uint32_t *response, Scr *scr);
		
		int cmdAppSendOpCond();
		int cmdSendIfCond(uint8_t voltage, uint8_t pattern);
		int cmdAllSendCid(Cid *cid);
		int cmdSendCid(Cid *cid);
		int cmdSendCsd(Csd *csd);
		int cmdSendScr(Scr *scr);
		int cmdSendRelativeAddr(uint16_t *new_rca);
		int cmdSetBusWidth(uint8_t bus_width);
		int cmdStopTransmission();
		int cmdSendStatus(uint32_t *status);
		
		void transferDataBegin(uint32_t mode, Sdio::BlockSize block_size, uint32_t n_blocks, char *data);
		int transferData(uint32_t timeout_ms);
		void transferDataFinish();
		
		void readFifoFromISR();
		void readHalfFifoFromISR();
		
		void writeFifoFromISR(uint32_t status);
		
		constexpr uint32_t getBits(const uint32_t *response, uint32_t bits, uint32_t start, uint32_t size) {
			uint32_t offset = (bits / 32) - (start / 32) - 1;
			uint32_t shift = start & 31;
			uint32_t value = (response[offset] >> shift);
			
			if (shift + size > 32)
				value |= response[offset - 1] << (32 - shift);
			return value & ((1llu << size) - 1);
		}
		
		constexpr uint32_t getBitsBE(const uint32_t *response, uint32_t bits, uint32_t start, uint32_t size) {
			uint32_t offset = (bits / 32) - (start / 32) - 1;
			uint32_t shift = start & 31;
			uint32_t value = (__builtin_bswap32(response[offset]) >> shift);
			
			if (shift + size > 32)
				value |= __builtin_bswap32(response[offset - 1]) << (32 - shift);
			return value & ((1llu << size) - 1);
		}
	public:
		static constexpr uint32_t BLOCK_SIZE = 512;
		
		enum {
			ERR_SUCCESS				= 0,
			ERR_UNKNOWN				= -1,
			ERR_TIMEOUT				= -2,
			ERR_CRC_FAIL			= -3,
			ERR_ILLEGAL_COMMAND		= -4,
			ERR_NOT_USABLE			= -5,
			ERR_OVERRUN				= -6,
			ERR_UNDERRUN			= -7,
			ERR_OUT_OF_RANGE		= -8,
			ERR_INTERNAL_ERROR		= -9
		};
		
		void handleIrq();
		
		constexpr void setClockFreq(uint32_t freq) {
			m_clock_freq = freq;
		}
		
		inline uint32_t getRealClockFreq() {
			return getSdioClk() / toClockDiv(m_clock_freq);
		}
		
		const constexpr Csd *csd() {
			return &m_csd;
		}
		
		const constexpr Cid *cid() {
			return &m_cid;
		}
		
		const constexpr Scr *scr() {
			return &m_scr;
		}
		
		constexpr uint16_t rca() {
			return m_rca;
		}
		
		int readBlocks(uint32_t addr, uint32_t n_blocks, char *buffer, uint32_t timeout_ms);
		int writeBlocks(uint32_t addr, uint32_t n_blocks, const char *buffer, uint32_t timeout_ms);
		
		Sd();
		~Sd();
		
		int open();
		void close();
};
