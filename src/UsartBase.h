#pragma once

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include <cstddef>

class UsartBase {
	protected:
		enum StopBits: uint32_t {
			STOPBITS_1		= USART_STOPBITS_1,
			STOPBITS_1_5	= USART_STOPBITS_1_5,
			STOPBITS_2		= USART_STOPBITS_2
		};
		
		enum Parity: uint32_t {
			PARITY_NONE		= USART_PARITY_NONE,
			PARITY_ODD		= USART_PARITY_ODD,
			PARITY_EVEN		= USART_PARITY_EVEN
		};
		
		enum Mode: uint32_t {
			MODE_TX		= USART_MODE_TX,
			MODE_RX		= USART_MODE_RX,
			MODE_TX_RX	= USART_MODE_TX_RX
		};
		
		struct HwConfig {
			uint32_t usart;
			uint32_t irq;
			rcc_periph_rst rst;
			rcc_periph_clken rcc;
		};
		
		// Hardware config
		constexpr static HwConfig m_usarts[] = {
			#ifdef STM32F4
				{USART1,	NVIC_USART1_IRQ,	RST_USART1,		RCC_USART1},
				{USART2,	NVIC_USART2_IRQ,	RST_USART2,		RCC_USART2},
				{USART3,	NVIC_USART3_IRQ,	RST_USART3,		RCC_USART3},
				{UART4,		NVIC_UART4_IRQ,		RST_UART4,		RCC_UART4},
				{UART5,		NVIC_UART5_IRQ,		RST_UART5,		RCC_UART5},
				{USART6,	NVIC_USART6_IRQ,	RST_USART6,		RCC_USART6},
				{UART7,		NVIC_UART7_IRQ,		RST_UART7,		RCC_UART7},
				{UART8,		NVIC_UART8_IRQ,		RST_UART8,		RCC_UART8},
			#else
				#error Unsupported hardware
			#endif
		};
		constexpr static int USARTS_COUNT = sizeof(m_usarts) / sizeof(m_usarts[0]);
		static UsartBase *m_instances[USARTS_COUNT];
		
		uint32_t m_baudrate = 115200;
		uint32_t m_databits = 8;
		Parity m_parity = PARITY_NONE;
		StopBits m_stopbits = STOPBITS_1;
		Mode m_mode = MODE_TX_RX;
		uint32_t m_flowcontrol = 0;
		bool m_half_duplex = false;
		
		void setBaurd(uint32_t baurd);
		void setPortConfig(uint32_t databits, Parity parity, StopBits stopbits);
		void setMode(Mode mode);
		void setFlowControl(bool rts, bool cts);
		void setHalfDuplex(bool enable);
		
		int m_id = -1;
		const HwConfig *m_config = nullptr;
		
		UsartBase &operator=(const UsartBase &);
		UsartBase(const UsartBase &);
		
		void configure();
		
		virtual void init();
		virtual void deinit();
		virtual void handleIrq() = 0;
	public:
		static inline void irq(uint32_t index) {
			UsartBase *instance = m_instances[index];
			if (instance)
				instance->handleIrq();
		}
		
		int open();
		int close();
		
		virtual int read(char *buffer, int size, int timeout_ms = 60000) = 0;
		virtual int write(const char *buffer, int size, int timeout_ms = 60000) = 0;
		
		UsartBase(uint32_t usart);
		virtual ~UsartBase();
};
