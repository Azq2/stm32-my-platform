#pragma once

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include <cstddef>

#include <FreeRTOS.h>
#include <semphr.h>

class Spi {
	protected:
		static constexpr uint32_t SPI_ALL_IRQ = SPI_CR2_RXNEIE | SPI_CR2_TXEIE | SPI_CR2_ERRIE;
		static constexpr uint32_t SPI_ERRORS_FLAGS = SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR | SPI_SR_UDR | SPI_SR_CHSIDE;
		
		struct HwConfig {
			uint32_t spi;
			uint32_t irq;
			rcc_periph_rst rst;
			rcc_periph_clken rcc;
			uint32_t bus;
		};
		
		enum Mode: uint8_t {
			MODE_0		= 0,
			MODE_1		= 1,
			MODE_2		= 2,
			MODE_3		= 3,
		};
		
		enum BitOrder: uint32_t {
			LSB_FIRST		= SPI_CR1_LSBFIRST,
			MSB_FIRST		= SPI_CR1_MSBFIRST
		};
		
		int m_id = -1;
		const HwConfig *m_config = nullptr;
		Mode m_mode = MODE_0;
		uint32_t m_speed = 400000;
		uint32_t m_real_speed = 0;
		bool m_crc = true;
		BitOrder m_data_bit_order = MSB_FIRST;
		uint16_t m_default_write_value = 0xFFFF;
		int m_one_byte_timeout = 10;
		
		struct {
			struct {
				union {
					void *buffer;
					uint8_t *buffer8;
					uint16_t *buffer16;
				};
				int size;
			} read;
			
			struct {
				union {
					const void *buffer;
					const uint8_t *buffer8;
					const uint16_t *buffer16;
				};
				int size;
			} write;
			
			bool wide;
			int error;
			int size;
		} m_isr = {};
		
		SemaphoreHandle_t m_transfer_sem = nullptr;
		
		// Hardware config
		constexpr static HwConfig m_spi_ports[] = {
			#ifdef STM32F4
				{SPI1,		NVIC_SPI1_IRQ,		RST_SPI1,		RCC_SPI1,	PERIPH_BASE_APB2},
				{SPI2,		NVIC_SPI2_IRQ,		RST_SPI2,		RCC_SPI2,	PERIPH_BASE_APB1},
				{SPI3,		NVIC_SPI3_IRQ,		RST_SPI3,		RCC_SPI3,	PERIPH_BASE_APB1},
				{SPI4,		NVIC_SPI4_IRQ,		RST_SPI4,		RCC_SPI4,	PERIPH_BASE_APB2},
				{SPI5,		NVIC_SPI5_IRQ,		RST_SPI5,		RCC_SPI5,	PERIPH_BASE_APB2},
				{SPI6,		NVIC_SPI6_IRQ,		RST_SPI6,		RCC_SPI6,	PERIPH_BASE_APB2},
			#else
				#error Unsupported hardware
			#endif
		};
		constexpr static int SPI_COUNT = sizeof(m_spi_ports) / sizeof(m_spi_ports[0]);
		static Spi *m_instances[SPI_COUNT];
		
		Spi &operator=(const Spi &);
		Spi(const Spi &);
		
		constexpr uint32_t getSpiBusClock() {
			switch (m_config->bus) {
				case PERIPH_BASE_APB1:
					return rcc_apb1_frequency;
				
				case PERIPH_BASE_APB2:
					return rcc_apb2_frequency;
			}
			return 0;
		}
		
		int transfer(void *buffer_tx, int size_tx, void *buffer_rx, int size_rx, bool wide);
	public:
		enum {
			ERR_SUCCESS				= 0,
			ERR_UNKNOWN				= -1,
			ERR_TIMEOUT				= -2,
			ERR_CRC_FAIL			= -3,
			ERR_OVERRUN				= -4,
			ERR_FAULT				= -5,
		};
		
		static inline void irq(uint32_t index) {
			Spi *instance = m_instances[index];
			if (instance)
				instance->handleIrq();
		}
		
		int open();
		int close();
		
		void setSpeed(uint32_t speed);
		void setMode(Mode mode);
		void setCrcControl(bool enable);
		void setBitOrder(BitOrder order);
		
		constexpr uint32_t getRealSpeed() {
			return m_real_speed;
		}
		
		void handleIrq();
		
		constexpr void setOneByteTimeout(int timeout) {
			m_one_byte_timeout = timeout;
		}
		
		// Send one byte and return response or error
		inline int transfer(uint8_t byte) {
			int ret = transfer(&byte, 1, &byte, 1, false);
			return ret < 0 ? ret : byte;
		}
		
		inline int transfer16(uint16_t byte) {
			int ret = transfer(&byte, 1, &byte, 1, true);
			return ret < 0 ? ret : byte;
		}
		
		// Read-only
		inline int read(uint8_t *buffer_rx, int size_rx) {
			return transfer(nullptr, 0, buffer_rx, size_rx, false);
		}
		
		inline int read16(uint8_t *buffer_rx, int size_rx) {
			return transfer(nullptr, 0, buffer_rx, size_rx, true);
		}
		
		// Write-only
		inline int write(uint8_t *buffer_tx, int size_tx) {
			return transfer(nullptr, 0, buffer_tx, size_tx, false);
		}
		
		inline int write16(uint8_t *buffer_tx, int size_tx) {
			return transfer(nullptr, 0, buffer_tx, size_tx, true);
		}
		
		// Write+Read
		inline int transfer(uint8_t *buffer_tx, int size_tx, uint8_t *buffer_rx, int size_rx) {
			return transfer(buffer_tx, size_tx, buffer_rx, size_rx, false);
		}
		
		inline int transfer16(uint16_t *buffer_tx, int size_tx, uint16_t *buffer_rx, int size_rx) {
			return transfer(buffer_tx, size_tx, buffer_rx, size_rx, true);
		}
		
		inline int transfer(uint8_t *buffer_trx, int size_trx) {
			return transfer(buffer_trx, size_trx, buffer_trx, size_trx, false);
		}
		
		inline int transfer16(uint16_t *buffer_trx, int size_trx) {
			return transfer(buffer_trx, size_trx, buffer_trx, size_trx, true);
		}
		
		void configure();
		
		Spi(uint32_t spi);
		~Spi();
};
