#pragma once

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include <cstddef>

#include <FreeRTOS.h>
#include <semphr.h>

class Spi {
	protected:
		static constexpr int ONE_BYTE_TIMEOUT = 10;
		
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
		
		enum BitOrder: uint8_t {
			LSB_FIRST		= 0,
			MSB_FIRST		= 1
		};
		
		BitOrder m_data_bit_order = MSB_FIRST;
		Mode m_mode = MODE_0;
		
		int m_id = -1;
		const HwConfig *m_config = nullptr;
		uint32_t m_speed = 400000;
		uint32_t m_real_speed = 0;
		
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
			uint16_t default_write;
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
		
		int transfer(const void *buffer_tx, int size_tx, void *buffer_rx, int size_rx, uint16_t default_write, bool wide);
	public:
		enum {
			ERR_SUCCESS		= 0,
			ERR_UNKNOWN		= -1
		};
		
		enum {
			SPEED_MIN		= 0,
			SPEED_MAX		= 1000000000
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
		void setBitOrder(BitOrder order);
		
		constexpr uint32_t getRealSpeed() {
			return m_real_speed;
		}
		
		void handleIrq();
		
		// Send one byte and return response
		inline int transfer(uint8_t byte) {
			int ret = transfer(&byte, 1, &byte, 1, 0xFF, false);
			return ret < 0 ? ret : byte;
		}
		
		inline int transfer16(uint16_t byte) {
			int ret = transfer(&byte, 1, &byte, 1, 0xFFFF, true);
			return ret < 0 ? ret : byte;
		}
		
		// Read-only
		inline int read(uint8_t *buffer_rx, int size_rx, uint8_t default_write = 0xFF) {
			return transfer(nullptr, 0, buffer_rx, size_rx, default_write, false);
		}
		
		inline int read16(uint8_t *buffer_rx, int size_rx, uint16_t default_write = 0xFFFF) {
			return transfer(nullptr, 0, buffer_rx, size_rx, default_write, true);
		}
		
		// Write-only
		inline int write(const uint8_t *buffer_tx, int size_tx) {
			return transfer(buffer_tx, size_tx, nullptr, 0, 0xFF, false);
		}
		
		inline int write16(const uint8_t *buffer_tx, int size_tx) {
			return transfer(buffer_tx, size_tx, nullptr, 0, 0xFFFF, true);
		}
		
		// Write+Read
		inline int transfer(const uint8_t *buffer_tx, int size_tx, uint8_t *buffer_rx, int size_rx, uint8_t default_write = 0xFF) {
			return transfer(buffer_tx, size_tx, buffer_rx, size_rx, default_write, false);
		}
		
		inline int transfer16(const uint16_t *buffer_tx, int size_tx, uint16_t *buffer_rx, int size_rx, uint16_t default_write = 0xFFFF) {
			return transfer(buffer_tx, size_tx, buffer_rx, size_rx, default_write, true);
		}
		
		inline int transfer(uint8_t *buffer_trx, int size_trx) {
			return transfer(buffer_trx, size_trx, buffer_trx, size_trx, 0xFF, false);
		}
		
		inline int transfer16(uint16_t *buffer_trx, int size_trx) {
			return transfer(buffer_trx, size_trx, buffer_trx, size_trx, 0xFFFF, true);
		}
		
		void configure();
		
		explicit Spi(uint32_t spi);
		~Spi();
};
