#pragma once

#include "UsartBase.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <stream_buffer.h>

class UsartBuffered : public UsartBase {
	protected:
		UsartBuffered &operator=(const UsartBuffered &);
		UsartBuffered(const UsartBuffered &);
		
		StreamBufferHandle_t m_tx_stream = nullptr;
		StreamBufferHandle_t m_rx_stream = nullptr;
		
		int m_rx_buffer_size = 60;
		int m_tx_buffer_size = 60;
		
		int m_curr_rx_buffer_size = 0;
		int m_curr_tx_buffer_size = 0;
		
		bool m_continouous_read = false;
	public:
		explicit UsartBuffered(uint32_t usart);
		
		void init() override;
		void deinit() override;
		void handleIrq() override;
		
		void purgeTx();
		void purgeRx();
		
		void setContinuousRead(bool enable);
		
		constexpr void setRxBufferSize(int size) {
			m_rx_buffer_size = size;
		}
		
		constexpr void setTxBufferSize(int size) {
			m_tx_buffer_size = size;
		}
		
		int read(char *buffer, int size, int timeout_ms = 60000) override;
		int write(const char *buffer, int size, int timeout_ms = 60000) override;
};
