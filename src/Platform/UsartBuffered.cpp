#include "UsartBuffered.h"

UsartBuffered::UsartBuffered(uint32_t usart) : UsartBase(usart) {
	
}

void UsartBuffered::setContinuousRead(bool enable) {
	m_continouous_read = enable;
	
	if (m_instances[m_id]) {
		if (m_continouous_read) {
			USART_CR1(m_config->usart) |= USART_CR1_RXNEIE;
		} else {
			USART_CR1(m_config->usart) &= ~USART_CR1_RXNEIE;
		}
	}
}

void UsartBuffered::deinit() {
	if (m_tx_stream) {
		vStreamBufferDelete(m_tx_stream);
		m_tx_stream = nullptr;
	}
	
	if (m_rx_stream) {
		vStreamBufferDelete(m_rx_stream);
		m_rx_stream = nullptr;
	}
}

void UsartBuffered::init() {
	if (m_tx_stream && m_tx_buffer_size != m_curr_tx_buffer_size) {
		vStreamBufferDelete(m_tx_stream);
		m_tx_stream = nullptr;
	}
	
	if (m_rx_stream && m_rx_buffer_size != m_curr_rx_buffer_size) {
		vStreamBufferDelete(m_rx_stream);
		m_rx_stream = nullptr;
	}
	
	if (!m_tx_stream) {
		m_tx_stream = xStreamBufferCreate(m_tx_buffer_size, 1);
		m_curr_tx_buffer_size = m_tx_buffer_size;
	}
	
	if (!m_rx_stream) {
		m_rx_stream = xStreamBufferCreate(m_rx_buffer_size, 1);
		m_curr_rx_buffer_size = m_rx_buffer_size;
	}
	
	setContinuousRead(m_continouous_read);
}

int UsartBuffered::read(char *buffer, int size, int timeout_ms) {
	TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
	TimeOut_t timeout;
	
	vTaskSetTimeOutState(&timeout);
	
	if (!m_continouous_read)
		USART_CR1(m_config->usart) |= USART_CR1_RXNEIE;
	
	int received = 0;
	while (received < size) {
		received += xStreamBufferReceive(m_rx_stream, buffer + received, size - received, ticks_to_wait);
		if (xTaskCheckForTimeOut(&timeout, &ticks_to_wait))
			break;
	}
	
	if (!m_continouous_read)
		USART_CR1(m_config->usart) &= ~USART_CR1_RXNEIE;
	
	return received;
}

void UsartBuffered::purgeRx() {
	xStreamBufferReset(m_rx_stream);
}

void UsartBuffered::purgeTx() {
	xStreamBufferReset(m_tx_stream);
}

int UsartBuffered::write(const char *buffer, int size, int timeout_ms) {
	TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
	TimeOut_t timeout;
	
	vTaskSetTimeOutState(&timeout);
	
	int written = 0;
	while (written < size) {
		written += xStreamBufferSend(m_tx_stream, buffer + written, size - written, ticks_to_wait);
		USART_CR1(m_config->usart) |= USART_CR1_TXEIE;
		
		if (xTaskCheckForTimeOut(&timeout, &ticks_to_wait))
			break;
	}
	
	return written;
}

void UsartBuffered::handleIrq() {
	uint8_t byte;
	BaseType_t higher_task_woken = pdFALSE;
	
	// RX
	if ((USART_CR1(m_config->usart) & USART_CR1_RXNEIE)) {
		while ((USART_SR(m_config->usart) & USART_SR_RXNE)) {
			byte = usart_recv(m_config->usart) & 0xFF;
			
			// FIXME: Simulate circular buffer in continouous mode
			if (m_continouous_read && xStreamBufferIsFull(m_rx_stream)) {
				uint8_t dummy;
				xStreamBufferReceiveFromISR(m_rx_stream, &dummy, 1, nullptr);
			}
			
			if (!xStreamBufferSendFromISR(m_rx_stream, &byte, 1, &higher_task_woken)) {
				USART_CR1(m_config->usart) &= ~USART_CR1_RXNEIE;
				break;
			}
		}
	}
	
	if (higher_task_woken) {
		portYIELD_FROM_ISR(higher_task_woken);
		return;
	}
	
	// TX
	if ((USART_CR1(m_config->usart) & USART_CR1_TXEIE)) {
		while ((USART_SR(m_config->usart) & USART_SR_TXE)) {
			size_t avail = xStreamBufferReceiveFromISR(m_tx_stream, &byte, 1, &higher_task_woken);
			if (avail > 0) {
				usart_send(m_config->usart, byte);
			} else {
				USART_CR1(m_config->usart) &= ~USART_CR1_TXEIE;
				break;
			}
		}
	}
	
	if (higher_task_woken) {
		portYIELD_FROM_ISR(higher_task_woken);
		return;
	}
}
