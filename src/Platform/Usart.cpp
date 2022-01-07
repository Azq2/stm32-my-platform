#include "Usart.h"

Usart::Usart(uint32_t usart) : UsartBase(usart) {
	m_read_sem = xSemaphoreCreateBinary();
	m_write_sem = xSemaphoreCreateBinary();
}

Usart::~Usart() {
	vSemaphoreDelete(m_read_sem);
	vSemaphoreDelete(m_write_sem);
}

int Usart::read(char *buffer, int size, int timeout_ms) {
	TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
	
	m_isr.read.buffer = buffer;
	m_isr.read.size = size;
	USART_CR1(m_config->usart) |= USART_CR1_RXNEIE;
	
	if (xSemaphoreTake(m_read_sem, ticks_to_wait))
		return size - m_isr.read.size;
	
	USART_CR1(m_config->usart) &= ~USART_CR1_RXNEIE;
	
	return size - m_isr.read.size;
}

int Usart::write(const char *buffer, int size, int timeout_ms) {
	TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
	
	m_isr.write.buffer = buffer;
	m_isr.write.size = size;
	USART_CR1(m_config->usart) |= USART_CR1_TXEIE;
	
	if (xSemaphoreTake(m_write_sem, ticks_to_wait))
		return size - m_isr.write.size;
	
	USART_CR1(m_config->usart) &= ~USART_CR1_TXEIE;
	
	return size - m_isr.write.size;
}

void Usart::handleIrq() {
	BaseType_t higher_task_woken = pdFALSE;
	
	// RX
	if ((USART_CR1(m_config->usart) & USART_CR1_RXNEIE)) {
		while ((USART_SR(m_config->usart) & USART_SR_RXNE)) {
			if (m_isr.read.size > 0) {
				*m_isr.read.buffer++ = usart_recv(m_config->usart) & 0xFF;
				m_isr.read.size--;
			}
			
			if (!m_isr.read.size) {
				USART_CR1(m_config->usart) &= ~USART_CR1_RXNEIE;
				xSemaphoreGiveFromISR(m_read_sem, &higher_task_woken);
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
			if (m_isr.write.size > 0) {
				usart_send(m_config->usart, *m_isr.write.buffer++);
				m_isr.write.size--;
			}
			
			if (!m_isr.write.size) {
				USART_CR1(m_config->usart) &= ~USART_CR1_TXEIE;
				xSemaphoreGiveFromISR(m_write_sem, &higher_task_woken);
				break;
			}
		}
	}
	
	if (higher_task_woken) {
		portYIELD_FROM_ISR(higher_task_woken);
		return;
	}
}
