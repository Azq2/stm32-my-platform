#include "I2C.h"

#include "Delay.h"

#include <cerrno>
#include <algorithm>
#include <cstdio>

#include <FreeRTOS.h>
#include <task.h>

I2C *I2C::m_instances[I2C::I2C_COUNT] = {};

I2C::I2C(uint32_t i2c) {
	for (int i = 0; i < I2C_COUNT; i++) {
		if (m_i2c_ports[i].i2c == i2c) {
			m_id = i;
			m_config = &m_i2c_ports[i];
			break;
		}
	}
	
	m_transfer_sem = xSemaphoreCreateBinary();
	m_mutex = xSemaphoreCreateMutex();
	
	configASSERT(m_id != -1);
}

void I2C::lock() {
	xSemaphoreTake(m_mutex, portMAX_DELAY);
}

void I2C::unlock() {
	xSemaphoreGive(m_mutex);
}

void I2C::setSpeed(uint32_t speed) {
	lock();
	m_speed = speed;
	configure();
	unlock();
}

void I2C::configure() {
	if (!m_instances[m_id])
		return;
	
	taskENTER_CRITICAL();
	
	i2c_peripheral_disable(m_config->i2c);
	
	// Frequenncy
	uint32_t pclk = getI2CBusClock();
	uint32_t pclk_mhz = pclk / 1000000;
	
	i2c_set_dutycycle(m_config->i2c, I2C_CCR_DUTY_DIV2);
	i2c_set_clock_frequency(m_config->i2c, pclk_mhz);
	
	uint32_t ccr;
	uint32_t trise;
	
	m_speed = std::min(MAX_FAST_SPEED, std::max(1UL, m_speed));
	
	if (m_speed > MAX_STANDART_SPEED) {
		i2c_set_fast_mode(m_config->i2c);
		ccr = std::min(0xFFFFUL, std::max(0x1UL, pclk / (m_speed * 3)));
		trise = (pclk_mhz * 300 / 1000) + 1; // 300ns
		m_real_speed = pclk / (ccr * 3);
	} else {
		i2c_set_standard_mode(m_config->i2c);
		ccr = std::min(0xFFFFUL, std::max(0x4UL, pclk / (m_speed * 2)));
		trise = pclk_mhz + 1; // 1000ns
		m_real_speed = pclk / (ccr * 2);
	}
	
	i2c_set_ccr(m_config->i2c, ccr);
	i2c_set_trise(m_config->i2c, trise);
	
	i2c_peripheral_enable(m_config->i2c);
	
	taskEXIT_CRITICAL();
}

int I2C::open() {
	lock();
	if (m_instances[m_id]) {
		unlock();
		return ERR_BUSY;
	}
	
	m_instances[m_id] = this;
	
	taskENTER_CRITICAL();
	// Enable clock
	rcc_periph_clock_enable(m_config->rcc);
	
	// Reset
	I2C_CR1(m_config->i2c) |= I2C_CR1_SWRST;
	I2C_CR1(m_config->i2c) &= ~I2C_CR1_SWRST;
	
	// Configure peripheral
	configure();
	
	// Enable IRQ
	nvic_enable_irq(m_config->irq_ev);
	nvic_enable_irq(m_config->irq_er);
	
	nvic_set_priority(m_config->irq_ev, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	nvic_set_priority(m_config->irq_er, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	
	m_start = false;
	
	taskEXIT_CRITICAL();
	unlock();
	return ERR_SUCCESS;
}

int I2C::getTimeout(int size) {
	// timeout for one byte: 30 clocks
	int one_byte_timeout = ((1000000 / m_real_speed)) * 2 * 30;
	// use 5s static timeout for worst case (rtos lags)
	// with additional timeout for requested size
	return (one_byte_timeout * (size + 2)) / 1000 + 5000;
}

int I2C::waitForBtfFlag(TimeOut_t *timeout, TickType_t *ticks_to_wait) {
	while ((I2C_SR1(m_config->i2c) & I2C_SR1_BTF)) {
		if (xTaskCheckForTimeOut(timeout, ticks_to_wait))
			return false;
		taskYIELD();
	}
	return true;
}

int I2C::waitForBusyFlag(TimeOut_t *timeout, TickType_t *ticks_to_wait) {
	while ((I2C_SR2(m_config->i2c) & I2C_SR2_BUSY)) {
		if (xTaskCheckForTimeOut(timeout, ticks_to_wait))
			return false;
		taskYIELD();
	}
	return true;
}

bool I2C::ping(uint8_t addr, int tries) {
	for (int i = 0; i < tries; i++) {
		if (write(addr, nullptr, 0) == ERR_SUCCESS)
			return true;
	}
	return false;
}

int I2C::read(uint16_t addr, uint8_t *buffer, int size) {
	lock();
	int ret = _read(addr, buffer, size, I2C_TRANSFER_SEND_START | I2C_TRANSFER_SEND_STOP);
	unlock();
	return ret;
}

int I2C::write(uint16_t addr, const uint8_t *buffer, int size) {
	lock();
	int ret = _write(addr, buffer, size, I2C_TRANSFER_SEND_START | I2C_TRANSFER_SEND_STOP);
	unlock();
	return ret;
}

int I2C::transfer(uint16_t addr, I2CMessage *msgs, int count) {
	lock();
	
	bool last_dir = false;
	for (int i = 0; i < count; i++) {
		I2CMessage *msg = &msgs[i];
		
		bool dir = (msg->flags & I2CMessage::WRITE) != 0;
		uint32_t flags = 0;
		
		if (i == 0 || !(msg->flags & I2CMessage::NOSTART) || last_dir != dir) {
			// Force send START bit on first message
			// Or if direction changed
			// Or if I2CMessage::NOSTART flag not set
			flags |= I2C_TRANSFER_SEND_START;
		}
		
		if (i == count - 1 || (msg->flags & I2CMessage::STOP)) {
			// Force send STOP bit on last message
			// Or if I2CMessage::STOP flag is set
			flags |= I2C_TRANSFER_SEND_STOP;
		}
		
		int ret;
		if (dir) {
			ret = _write(addr, msg->buffer, msg->size, flags);
		} else {
			ret = _read(addr, msg->buffer, msg->size, flags);
		}
		
		last_dir = dir;
		
		if (ret != ERR_SUCCESS) {
			unlock();
			return ret;
		}
	}
	
	unlock();
	return ERR_SUCCESS;
}

int I2C::_read(uint16_t addr, uint8_t *buffer, int size, uint32_t flags) {
	TickType_t ticks_to_wait = pdMS_TO_TICKS(getTimeout(size));
	TimeOut_t timeout;
	
	vTaskSetTimeOutState(&timeout);
	
	if (!m_instances[m_id])
		return ERR_BUSY;
	
	if (!size) {
		_abort();
		return ERR_INVALID;
	}
	
	if (!(flags & I2C_TRANSFER_SEND_START)) {
		// Not supported due to stm32 limitations
		_abort();
		return ERR_INVALID;
	}
	
	m_isr.error = ERR_SUCCESS;
	m_isr.buffer = buffer;
	m_isr.size = size;
	m_isr.remain = size;
	m_isr.addr = (addr << 1) | 1;
	m_isr.flags = flags;
	
	i2c_peripheral_enable(m_config->i2c);
	i2c_nack_current(m_config->i2c);
	i2c_enable_ack(m_config->i2c);
	
	if (!m_start) {
		if (!waitForBusyFlag(&timeout, &ticks_to_wait)) {
			_abort();
			return ERR_BUSY;
		}
	}
	
	i2c_send_start(m_config->i2c);
	m_start = true;
	
	// In some cases BTF flag is stuck after previous transfer 
	waitForBtfFlag(&timeout, &ticks_to_wait);
	
	i2c_enable_interrupt(m_config->i2c, I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
	
	int ret;
	if (xSemaphoreTake(m_transfer_sem, ticks_to_wait)) {
		ret = m_isr.error;
	} else {
		_abort();
		ret = ERR_BUSY;
	}
	
	return ret;
}

int I2C::_write(uint16_t addr, const uint8_t *buffer, int size, uint32_t flags) {
	TickType_t ticks_to_wait = pdMS_TO_TICKS(getTimeout(size));
	TimeOut_t timeout;
	
	vTaskSetTimeOutState(&timeout);
	
	if (!m_instances[m_id])
		return ERR_BUSY;
	
	m_isr.error = ERR_SUCCESS;
	m_isr.buffer = const_cast<uint8_t *>(buffer);
	m_isr.size = size;
	m_isr.remain = size;
	m_isr.addr = (addr << 1);
	m_isr.flags = flags;
	
	i2c_peripheral_enable(m_config->i2c);
	i2c_nack_current(m_config->i2c);
	i2c_enable_ack(m_config->i2c);
	
	if ((flags & I2C_TRANSFER_SEND_START) || !m_start) {
		if (!m_start) {
			if (!waitForBusyFlag(&timeout, &ticks_to_wait)) {
				_abort();
				return ERR_BUSY;
			}
		}
		
		i2c_send_start(m_config->i2c);
		m_start = true;
		
		// In some cases BTF flag is stuck after previous transfer 
		waitForBtfFlag(&timeout, &ticks_to_wait);
	} else {
		i2c_enable_interrupt(m_config->i2c, I2C_CR2_ITBUFEN);
	}
	
	i2c_enable_interrupt(m_config->i2c, I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
	
	int ret;
	if (xSemaphoreTake(m_transfer_sem, ticks_to_wait)) {
		ret = m_isr.error;
	} else {
		_abort();
		ret = ERR_BUSY;
	}
	
	return ret;
}

void I2C::_abort() {
	if (m_start) {
		// Disable all IRQ
		i2c_disable_interrupt(m_config->i2c, I2C_ALL_IRQ);
		
		// ACK=0
		i2c_disable_ack(m_config->i2c);
		
		// STOP=1
		_stop();
		
		// POS=0
		i2c_nack_current(m_config->i2c);
		
		// Dummy read
		if ((I2C_SR1(m_config->i2c) & I2C_SR1_RxNE))
			(void) I2C_DR(m_config->i2c);
		
		// Disable peripheral
		i2c_peripheral_disable(m_config->i2c);
	}
}

void I2C::_stop() {
	i2c_send_stop(m_config->i2c);
	m_start = false;
}

int I2C::close() {
	lock();
	if (m_instances[m_id]) {
		taskENTER_CRITICAL();
		// Disable IRQ
		nvic_disable_irq(m_config->irq_er);
		nvic_disable_irq(m_config->irq_ev);
		
		// Disable i2c
		i2c_peripheral_disable(m_config->i2c);
		
		// Disable i2c clock
		rcc_periph_clock_disable(m_config->rcc);
		
		m_start = false;
		
		m_instances[m_id] = nullptr;
		
		taskEXIT_CRITICAL();
	}
	unlock();
	return ERR_SUCCESS;
}

// Flow similar to application note AN2824
void I2C::handleIrqEv() {
	BaseType_t higher_task_woken = pdFALSE;
	uint32_t sr1 = I2C_SR1(m_config->i2c);
	
	if ((sr1 & I2C_SR1_SB)) {
		// Clear SB=1 by reading SR2
		(void) I2C_SR2(m_config->i2c);
		
		// Send slave addr
		I2C_DR(m_config->i2c) = m_isr.addr;
		
		return;
	}
	
	// Read from slave
	if ((m_isr.addr & 1)) {
		// Read 1 byte
		if (m_isr.size == 1) {
			if ((sr1 & I2C_SR1_ADDR)) {
				// ACK=0
				i2c_disable_ack(m_config->i2c);
				
				// Clear ADDR=1 by reading SR2
				(void) I2C_SR2(m_config->i2c);
				
				if ((m_isr.flags & I2C_TRANSFER_SEND_STOP)) {
					// STOP=1
					_stop();
				}
				
				// Wait for RxNE
				i2c_enable_interrupt(m_config->i2c, I2C_CR2_ITBUFEN);
			} else if ((sr1 & I2C_SR1_RxNE)) {
				// Read data
				*m_isr.buffer++ = I2C_DR(m_config->i2c);
				m_isr.remain--;
				
				// Finish transfer
				i2c_disable_interrupt(m_config->i2c, I2C_ALL_IRQ);
				xSemaphoreGiveFromISR(m_transfer_sem, &higher_task_woken);
			}
		}
		// Read 2 bytes
		else if (m_isr.size == 2) {
			if ((sr1 & I2C_SR1_ADDR)) {
				// POS=1
				i2c_nack_next(m_config->i2c);
				
				// Clear ADDR=1 by reading SR2
				(void) I2C_SR2(m_config->i2c);
				
				// ACK=0
				i2c_disable_ack(m_config->i2c);
			} else if ((sr1 & I2C_SR1_BTF)) {
				if ((m_isr.flags & I2C_TRANSFER_SEND_STOP)) {
					// STOP=1
					_stop();
				}
				
				// Read 2 bytes
				for (int i = 0; i < 2; i++) {
					*m_isr.buffer++ = I2C_DR(m_config->i2c);
					m_isr.remain--;
				}
				
				// Finish transfer
				i2c_disable_interrupt(m_config->i2c, I2C_ALL_IRQ);
				xSemaphoreGiveFromISR(m_transfer_sem, &higher_task_woken);
			}
		}
		// Read 3+ bytes
		else {
			if ((sr1 & I2C_SR1_ADDR)) {
				// Clear ADDR=1 by reading SR2
				(void) I2C_SR2(m_config->i2c);
				
				// Enable RxNE
				i2c_enable_interrupt(m_config->i2c, I2C_CR2_ITBUFEN);
			} else if ((I2C_CR2(m_config->i2c) & I2C_CR2_ITBUFEN) && (sr1 & I2C_SR1_RxNE)) {
				if (m_isr.remain == 3) {
					// N-2 and N-1 bytes must read after BTF event, disable RxNE
					i2c_disable_interrupt(m_config->i2c, I2C_CR2_ITBUFEN);
				} else if (m_isr.remain == 1) {
					// Read last byte
					*m_isr.buffer++ = I2C_DR(m_config->i2c);
					m_isr.remain--;
					
					// Finish transfer
					i2c_disable_interrupt(m_config->i2c, I2C_ALL_IRQ);
					xSemaphoreGiveFromISR(m_transfer_sem, &higher_task_woken);
				} else {
					// Read data
					*m_isr.buffer++ = I2C_DR(m_config->i2c);
					m_isr.remain--;
				}
			} else if (!(I2C_CR2(m_config->i2c) & I2C_CR2_ITBUFEN) && (sr1 & I2C_SR1_BTF)) {
				// ACK=0
				i2c_disable_ack(m_config->i2c);
				
				// Read N-2 byte
				*m_isr.buffer++ = I2C_DR(m_config->i2c);
				m_isr.remain--;
				
				if ((m_isr.flags & I2C_TRANSFER_SEND_STOP)) {
					// STOP=1
					_stop();
				}
				
				// Read N-1 byte
				*m_isr.buffer++ = I2C_DR(m_config->i2c);
				m_isr.remain--;
				
				// Last byte must read after RxNE event, enable it
				i2c_enable_interrupt(m_config->i2c, I2C_CR2_ITBUFEN);
			}
		}
	}
	// Write to slave
	else {
		if ((sr1 & I2C_SR1_ADDR)) {
			// Clear ADDR=1 by reading SR2
			(void) I2C_SR2(m_config->i2c);
			
			if (m_isr.remain > 0) {
				I2C_DR(m_config->i2c) = *m_isr.buffer++;
				m_isr.remain--;
			}
			
			if (m_isr.remain > 0) {
				// We have more data for write
				i2c_enable_interrupt(m_config->i2c, I2C_CR2_ITBUFEN);
			} else {
				if ((m_isr.flags & I2C_TRANSFER_SEND_STOP)) {
					// STOP=1
					_stop();
				}
				
				// Finish transfer
				i2c_disable_interrupt(m_config->i2c, I2C_ALL_IRQ);
				xSemaphoreGiveFromISR(m_transfer_sem, &higher_task_woken);
			}
		} else if ((I2C_CR2(m_config->i2c) & I2C_CR2_ITBUFEN) && (sr1 & I2C_SR1_TxE)) {
			I2C_DR(m_config->i2c) = *m_isr.buffer++;
			m_isr.remain--;
			
			if (!m_isr.remain) {
				// No more data, wait for BTF event
				i2c_disable_interrupt(m_config->i2c, I2C_CR2_ITBUFEN);
			}
		} else if (!(I2C_CR2(m_config->i2c) & I2C_CR2_ITBUFEN) && (sr1 & I2C_SR1_BTF)) {
			if ((m_isr.flags & I2C_TRANSFER_SEND_STOP)) {
				// STOP=1
				_stop();
			}
			
			// Finish transfer
			i2c_disable_interrupt(m_config->i2c, I2C_ALL_IRQ);
			xSemaphoreGiveFromISR(m_transfer_sem, &higher_task_woken);
		}
	}
	
	if (higher_task_woken)
		portYIELD_FROM_ISR(higher_task_woken);
}

void I2C::handleIrqEr() {
	BaseType_t higher_task_woken = pdFALSE;
	uint32_t status = I2C_SR1(m_config->i2c);
	
	I2C_SR1(m_config->i2c) &= ~I2C_ALL_ERRORS;
	I2C_SR1(m_config->i2c) &= ~I2C_SR1_ADDR;
	
	if ((status & I2C_SR1_TIMEOUT)) {
		m_isr.error = ERR_TIMEOUT;
		m_start = false;
	} else if ((status & I2C_SR1_OVR)) {
		m_isr.error = ERR_OVERRUN_OR_UNDERRUN;
		m_start = false;
	} else if ((status & I2C_SR1_AF)) {
		m_isr.error = ERR_NACK;
		
		// STOP=1
		_stop();
	} else if ((status & I2C_SR1_ARLO)) {
		m_isr.error = ERR_ARBITRATION_LOST;
		m_start = false;
	} else if ((status & I2C_SR1_BERR)) {
		m_isr.error = ERR_BUS;
		m_start = false;
	} else {
		m_isr.error = ERR_UNKNOWN;
	}
	
	// Disable all IRQ
	i2c_disable_interrupt(m_config->i2c, I2C_ALL_IRQ);
	
	// POS=0
	i2c_nack_current(m_config->i2c);
	
	// Dummy read
	if ((I2C_SR1(m_config->i2c) & I2C_SR1_RxNE))
		(void) I2C_DR(m_config->i2c);
	
	xSemaphoreGiveFromISR(m_transfer_sem, &higher_task_woken);
	portYIELD_FROM_ISR(higher_task_woken);
}

I2C::~I2C() {
	vSemaphoreDelete(m_transfer_sem);
	vSemaphoreDelete(m_mutex);
	close();
}

#ifdef STM32F4
extern "C" void i2c1_ev_isr(void) {
	I2C::irqEvent(0);
}

extern "C" void i2c1_er_isr(void) {
	I2C::irqError(0);
}

extern "C" void i2c2_ev_isr(void) {
	I2C::irqEvent(1);
}

extern "C" void i2c2_er_isr(void) {
	I2C::irqError(1);
}

extern "C" void i2c3_ev_isr(void) {
	I2C::irqEvent(2);
}

extern "C" void i2c3_er_isr(void) {
	I2C::irqError(2);
}
#else
	#error Unsupported hardware
#endif
