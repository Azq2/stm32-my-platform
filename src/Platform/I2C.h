#pragma once

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include <cstddef>

#include <FreeRTOS.h>
#include <semphr.h>

struct I2CMessage {
	static constexpr uint8_t READ		= 1 << 0;
	static constexpr uint8_t WRITE		= 1 << 1;
	static constexpr uint8_t NOSTART	= 1 << 2;
	static constexpr uint8_t STOP		= 1 << 3;
	
	uint8_t addr = 0;
	uint8_t *buffer = 0;
	int size = 0;
	uint8_t flags = 0;
	int timeout = 60000;
};

class I2C {
	protected:
		static constexpr uint32_t MAX_STANDART_SPEED	= 100000;
		static constexpr uint32_t MAX_FAST_SPEED		= 400000;
		
		static constexpr uint32_t BUSY_WAIT_TIMEOUT		= 25;
		
		static constexpr uint32_t I2C_ALL_ERRORS = I2C_SR1_TIMEOUT | I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR;
		static constexpr uint32_t I2C_ALL_IRQ = I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
		
		enum TransferFlags {
			I2C_TRANSFER_SEND_START	= 1 << 0,
			I2C_TRANSFER_SEND_STOP	= 1 << 1,
		};
		
		struct HwConfig {
			uint32_t i2c;
			uint32_t irq_ev;
			uint32_t irq_er;
			rcc_periph_clken rcc;
		};
		
		int m_id = -1;
		const HwConfig *m_config = nullptr;
		uint32_t m_speed = MAX_STANDART_SPEED;
		uint32_t m_real_speed = 0;
		bool m_start = false;
		
		struct {
			uint8_t *buffer = nullptr;
			uint16_t addr = 0;
			int size = 0;
			int remain = 0;
			int error = ERR_SUCCESS;
			uint32_t flags = 0;
		} m_isr = {};
		
		SemaphoreHandle_t m_transfer_sem = nullptr;
		SemaphoreHandle_t m_mutex = nullptr;
		
		// Hardware config
		constexpr static HwConfig m_i2c_ports[] = {
			#ifdef STM32F4
				{I2C1,		NVIC_I2C1_EV_IRQ,	NVIC_I2C1_ER_IRQ,		RCC_I2C1},
				{I2C2,		NVIC_I2C2_EV_IRQ,	NVIC_I2C2_ER_IRQ,		RCC_I2C2},
				{I2C3,		NVIC_I2C3_EV_IRQ,	NVIC_I2C3_ER_IRQ,		RCC_I2C3},
			#else
				#error Unsupported hardware
			#endif
		};
		constexpr static int I2C_COUNT = sizeof(m_i2c_ports) / sizeof(m_i2c_ports[0]);
		static I2C *m_instances[I2C_COUNT];
		
		I2C &operator=(const I2C &);
		I2C(const I2C &);
		
		uint32_t getI2CBusClock() {
			return rcc_apb1_frequency;
		}
		
		void handleIrqEv();
		void handleIrqEr();
		
		void configure();
		
		void lock();
		void unlock();
		
		int waitForBusyFlag(TimeOut_t *timeout, TickType_t *ticks_to_wait);
		int waitForBtfFlag(TimeOut_t *timeout, TickType_t *ticks_to_wait);
		
		int _read(uint16_t addr, uint8_t *buffer, int size, uint32_t flags, int timeout_ms = 60000);
		int _write(uint16_t addr, const uint8_t *buffer, int size, uint32_t flags, int timeout_ms = 60000);
		void _abort();
		void _stop();
	public:
		enum {
			ERR_SUCCESS					= 0,
			ERR_NACK					= -1,
			ERR_UNKNOWN					= -2,
			ERR_BUSY					= -3,
			ERR_TIMEOUT					= -4,
			ERR_OVERRUN_OR_UNDERRUN		= -5,
			ERR_ARBITRATION_LOST		= -6,
			ERR_BUS						= -7,
			ERR_INVALID					= -8
		};
		
		static inline void irqEvent(uint32_t index) {
			I2C *instance = m_instances[index];
			if (instance)
				instance->handleIrqEv();
		}
		
		static inline void irqError(uint32_t index) {
			I2C *instance = m_instances[index];
			if (instance)
				instance->handleIrqEr();
		}
		
		int open();
		int close();
		
		int read(uint16_t addr, uint8_t *buffer, int size, int timeout_ms = 60000);
		int write(uint16_t addr, const uint8_t *buffer, int size, int timeout_ms = 60000);
		int transfer(uint16_t addr, I2CMessage *msgs, int count);
		
		bool ping(uint8_t addr, int tries = 3, int timeout_ms = 60000);
		
		void setSpeed(uint32_t speed);
		
		constexpr uint32_t getRealSpeed() {
			return m_real_speed;
		}
		
		explicit I2C(uint32_t spi);
		~I2C();
};
