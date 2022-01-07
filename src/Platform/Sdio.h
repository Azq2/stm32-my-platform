#pragma once

#include <libopencm3/stm32/sdio.h>

namespace Sdio {
	constexpr uint32_t FIFO_SIZE		= 32 * 4;
	constexpr uint32_t FIFO_HALF_SIZE	= 8 * 4;
	
	enum ReadWaitMode {
		READ_WAIT_D2	= 0,
		READ_WAIT_CK	= SDIO_DCTRL_RWMOD
	};
	
	enum ClockPhase {
		CLOCK_POSITIVE_EDGE		= 0,
		CLOCK_NEGATIVE_EDGE		= SDIO_CLKCR_NEGEDGE
	};
	
	enum DataTransferMode {
		STREAM_TRANSFER_MODE	= 0,
		BLOCK_TRANSFER_MODE		= SDIO_DCTRL_DTMODE,
	};
	
	enum DataDirection {
		TO_CARD		= 0,
		FROM_CARD	= SDIO_DCTRL_DTDIR
	};
	
	enum BusWidth {
		BUS_WIDTH_1		= SDIO_CLKCR_WIDBUS_1,
		BUS_WIDTH_4		= SDIO_CLKCR_WIDBUS_4,
		BUS_WIDTH_8		= SDIO_CLKCR_WIDBUS_8
	};
	
	enum BlockSize {
		BLOCK_SIZE_1		= SDIO_DCTRL_DBLOCKSIZE_0,
		BLOCK_SIZE_2		= SDIO_DCTRL_DBLOCKSIZE_1,
		BLOCK_SIZE_4		= SDIO_DCTRL_DBLOCKSIZE_2,
		BLOCK_SIZE_8		= SDIO_DCTRL_DBLOCKSIZE_3,
		BLOCK_SIZE_16		= SDIO_DCTRL_DBLOCKSIZE_4,
		BLOCK_SIZE_32		= SDIO_DCTRL_DBLOCKSIZE_5,
		BLOCK_SIZE_64		= SDIO_DCTRL_DBLOCKSIZE_6,
		BLOCK_SIZE_128		= SDIO_DCTRL_DBLOCKSIZE_7,
		BLOCK_SIZE_256		= SDIO_DCTRL_DBLOCKSIZE_8,
		BLOCK_SIZE_512		= SDIO_DCTRL_DBLOCKSIZE_9,
		BLOCK_SIZE_1K		= SDIO_DCTRL_DBLOCKSIZE_10,
		BLOCK_SIZE_2K		= SDIO_DCTRL_DBLOCKSIZE_11,
		BLOCK_SIZE_4K		= SDIO_DCTRL_DBLOCKSIZE_12,
		BLOCK_SIZE_8K		= SDIO_DCTRL_DBLOCKSIZE_13,
		BLOCK_SIZE_16K		= SDIO_DCTRL_DBLOCKSIZE_14
	};
	
	constexpr uint32_t toBlockLength(BlockSize block_size) {
		return (1 << ((uint32_t) block_size >> SDIO_DCTRL_DBLOCKSIZE_SHIFT));
	}
	
	inline void setPowerConfig(bool pwron) {
		SDIO_POWER = pwron ? SDIO_POWER_PWRCTRL_PWRON : 0;
	}
	
	inline void setClockConfig(bool enable, uint8_t div, BusWidth bus_width, ClockPhase phase, bool hwfc_en, bool pwrsave) {
		uint32_t tmpreg = 0;
		
		if (hwfc_en)
			tmpreg |= SDIO_CLKCR_HWFC_EN;
		
		tmpreg |= phase;
		tmpreg |= bus_width;
		
		if (pwrsave)
			tmpreg |= SDIO_CLKCR_PWRSAV;
		
		if (enable)
			tmpreg |= SDIO_CLKCR_CLKEN;
		
		if (div >= 2) {
			tmpreg |= ((div - 2) << SDIO_CLKCR_CLKDIV_SHIFT);
		} else {
			tmpreg |= SDIO_CLKCR_BYPASS;
		}
		
		SDIO_CLKCR = tmpreg;
	}
	
	inline void setDpsmConfig(bool enable, BlockSize block_size, DataDirection data_direction, DataTransferMode transfer_mode, bool dma,
		bool sdio_specific = false, ReadWaitMode rwmode = READ_WAIT_D2, bool rwstop = false, bool rwstart = false)
	{
		uint32_t tmpreg = 0;
		
		if (enable)
			tmpreg |= SDIO_DCTRL_DTEN;
		
		if (sdio_specific)
			tmpreg |= SDIO_DCTRL_SDIOEN;
		
		tmpreg |= rwmode;
		
		if (rwstop)
			tmpreg |= SDIO_DCTRL_RWSTOP;
		
		if (rwstart)
			tmpreg |= SDIO_DCTRL_RWSTART;
		
		if (dma)
			tmpreg |= SDIO_DCTRL_DMAEN;
		
		tmpreg |= block_size;
		tmpreg |= transfer_mode;
		tmpreg |= data_direction;
		
		SDIO_DCTRL = tmpreg;
	}
	
	inline void setCpsmConfig(bool enable, uint8_t command_index, bool long_response, bool wait_for_response, bool wait_for_pend = false,
		bool wait_for_irq = false, bool ceata_command = false, bool ceata_irq = true, bool ceata_completion_signal = false)
	{
		uint32_t tmpreg = command_index;
		
		if (enable)
			tmpreg |= SDIO_CMD_CPSMEN;
		
		if (ceata_command)
			tmpreg |= SDIO_CMD_ATACMD;
		
		if (!ceata_irq)
			tmpreg |= SDIO_CMD_NIEN;
		
		if (ceata_completion_signal)
			tmpreg |= SDIO_CMD_ENCMDCOMPL;
		
		if (wait_for_pend)
			tmpreg |= SDIO_CMD_WAITPEND;
		
		if (wait_for_irq)
			tmpreg |= SDIO_CMD_WAITINT;
		
		if (long_response) {
			if (wait_for_response) {
				tmpreg |= SDIO_CMD_WAITRESP_LONG;
			} else {
				tmpreg |= SDIO_CMD_WAITRESP_NO_2;
			}
		} else {
			if (wait_for_response) {
				tmpreg |= SDIO_CMD_WAITRESP_SHORT;
			} else {
				tmpreg |= SDIO_CMD_WAITRESP_NO_0;
			}
		}
		
		SDIO_CMD = tmpreg;
	}
	
	inline void setCommandArgument(uint32_t arg) {
		SDIO_ARG = arg;
	}
	
	inline void setDataTimeout(uint32_t time) {
		SDIO_DTIMER = time;
	}
	
	inline void setDataLength(uint32_t length) {
		SDIO_DLEN = length;
	}
	
	inline uint32_t getResponseCommand() {
		return SDIO_RESPCMD;
	}
	
	inline uint32_t getDataCount() {
		return SDIO_DCOUNT;
	}
	
	inline uint32_t geFifoCount() {
		return SDIO_FIFOCNT;
	}
	
	inline uint32_t getStatus() {
		return SDIO_STA;
	}
	
	inline uint32_t getStatusFlag(uint32_t flag) {
		return SDIO_STA & flag;
	}
	
	constexpr inline uint32_t getIcrFromStatus(uint32_t status) {
		return status & (
			SDIO_STA_CCRCFAIL |
			SDIO_STA_DCRCFAIL |
			SDIO_STA_CTIMEOUT |
			SDIO_STA_DTIMEOUT |
			SDIO_STA_TXUNDERR |
			SDIO_STA_RXOVERR |
			SDIO_STA_CMDREND |
			SDIO_STA_CMDSENT |
			SDIO_STA_DATAEND |
			SDIO_STA_STBITERR |
			SDIO_STA_DBCKEND |
			SDIO_STA_SDIOIT |
			SDIO_STA_CEATAEND
		);
	}
	
	inline void clearFlag(uint32_t flag) {
		SDIO_ICR = flag;
	}
	
	inline void clearAllFlags() {
		SDIO_ICR = (
			SDIO_ICR_CEATAENDC |
			SDIO_ICR_SDIOITC |
			SDIO_ICR_DBCKENDC |
			SDIO_ICR_STBITERRC |
			SDIO_ICR_DATAENDC |
			SDIO_ICR_CMDSENTC |
			SDIO_ICR_CMDRENDC |
			SDIO_ICR_RXOVERRC |
			SDIO_ICR_TXUNDERRC |
			SDIO_ICR_DTIMEOUTC |
			SDIO_ICR_CTIMEOUTC |
			SDIO_ICR_DCRCFAILC |
			SDIO_ICR_CCRCFAILC
		);
	}
	
	inline void maskFlag(uint32_t flag) {
		SDIO_MASK = SDIO_MASK | flag;
	}
	
	inline void maskAllFlags() {
		SDIO_MASK = (
			SDIO_MASK_CEATAENDIE |
			SDIO_MASK_SDIOITIE |
			SDIO_MASK_RXDAVLIE |
			SDIO_MASK_TXDAVLIE |
			SDIO_MASK_RXFIFOEIE |
			SDIO_MASK_TXFIFOEIE |
			SDIO_MASK_RXFIFOFIE |
			SDIO_MASK_TXFIFOFIE |
			SDIO_MASK_RXFIFOHFIE |
			SDIO_MASK_TXFIFOHEIE |
			SDIO_MASK_RXACTIE |
			SDIO_MASK_TXACTIE |
			SDIO_MASK_CMDACTIE |
			SDIO_MASK_DBCKENDIE |
			SDIO_MASK_STBITERRIE |
			SDIO_MASK_DATAENDIE |
			SDIO_MASK_CMDSENTIE |
			SDIO_MASK_CMDRENDIE |
			SDIO_MASK_RXOVERRIE |
			SDIO_MASK_TXUNDERRIE |
			SDIO_MASK_DTIMEOUTIE |
			SDIO_MASK_CTIMEOUTIE |
			SDIO_MASK_DCRCFAILIE |
			SDIO_MASK_CCRCFAILIE
		);
	}
	
	inline void unmaskFlag(uint32_t flag) {
		SDIO_MASK = SDIO_MASK & ~flag;
	}
	
	inline void unmaskAllFlags() {
		SDIO_MASK = 0;
	}
}
