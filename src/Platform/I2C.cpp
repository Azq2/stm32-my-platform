#include "I2C.h"

#include "Delay.h"

#include <cerrno>
#include <algorithm>
#include <cstdio>

#include <FreeRTOS.h>
#include <task.h>

I2C *I2C::m_instances[I2C::I2C_COUNT] = {};

static void dump_sr1(uint32_t status) {
	struct {
		uint32_t bit;
		const char *name;
	} regs[] = {
		{I2C_SR1_SMBALERT, "I2C_SR1_SMBALERT"},
		{I2C_SR1_TIMEOUT, "I2C_SR1_TIMEOUT"},
		{I2C_SR1_PECERR, "I2C_SR1_PECERR"},
		{I2C_SR1_OVR, "I2C_SR1_OVR"},
		{I2C_SR1_AF, "I2C_SR1_AF"},
		{I2C_SR1_ARLO, "I2C_SR1_ARLO"},
		{I2C_SR1_BERR, "I2C_SR1_BERR"},
		{I2C_SR1_TxE, "I2C_SR1_TxE"},
		{I2C_SR1_RxNE, "I2C_SR1_RxNE"},
		{I2C_SR1_STOPF, "I2C_SR1_STOPF"},
		{I2C_SR1_ADD10, "I2C_SR1_ADD10"},
		{I2C_SR1_BTF, "I2C_SR1_BTF"},
		{I2C_SR1_ADDR, "I2C_SR1_ADDR"},
		{I2C_SR1_SB, "I2C_SR1_SB"},
	};
	
	printf("I2C_SR1:\r\n");
	for (int i = 0; i < (sizeof(regs) / sizeof(regs[0])); i++) {
		if ((status & regs[i].bit)) {
			printf("  %s\r\n", regs[i].name);
		}
	}
	printf("\r\n");
}

static void dump_sr2(uint32_t status) {
	struct {
		uint32_t bit;
		const char *name;
	} regs[] = {
		{I2C_SR2_DUALF, "I2C_SR2_DUALF"},
		{I2C_SR2_SMBHOST, "I2C_SR2_SMBHOST"},
		{I2C_SR2_SMBDEFAULT, "I2C_SR2_SMBDEFAULT"},
		{I2C_SR2_GENCALL, "I2C_SR2_GENCALL"},
		{I2C_SR2_TRA, "I2C_SR2_TRA"},
		{I2C_SR2_MSL, "I2C_SR2_MSL"},
	};
	
	printf("I2C_SR2:\r\n");
	for (int i = 0; i < (sizeof(regs) / sizeof(regs[0])); i++) {
		if ((status & regs[i].bit)) {
			printf("  %s\r\n", regs[i].name);
		}
	}
	printf("\r\n");
}

I2C::I2C(uint32_t i2c) {
	for (int i = 0; i < I2C_COUNT; i++) {
		if (m_i2c_ports[i].i2c == i2c) {
			m_id = i;
			m_config = &m_i2c_ports[i];
			break;
		}
	}
	
	m_transfer_sem = xSemaphoreCreateBinary();
	
	configASSERT(m_id != -1);
}

void I2C::setSpeed(uint32_t speed) {
	m_speed = speed;
	configure();
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
	
	if (m_speed > MAX_STANDART_SPEED) {
		i2c_set_fast_mode(m_config->i2c);
		ccr = std::min((uint32_t) 0xFFF, std::max((uint32_t) 0x1, (pclk / (m_speed * 3)) << 1));
		trise = (pclk_mhz * 300 / 1000) + 1;
	} else {
		i2c_set_standard_mode(m_config->i2c);
		ccr = std::min((uint32_t) 0xFFF, std::max((uint32_t) 0x4, (pclk / m_speed) << 1));
		trise = pclk_mhz + 1;
	}
	
	i2c_set_ccr(m_config->i2c, ccr);
	i2c_set_trise(m_config->i2c, trise);
	
	i2c_peripheral_enable(m_config->i2c);
	
	taskEXIT_CRITICAL();
}

int I2C::open() {
	if (m_instances[m_id])
		return -EEXIST;
	
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
	return 0;
}

int I2C::waitForBusyFlag(TimeOut_t *timeout, TickType_t *ticks_to_wait) {
	while ((I2C_SR2(m_config->i2c) & I2C_SR2_BUSY)) {
		if (xTaskCheckForTimeOut(timeout, ticks_to_wait))
			return false;
		taskYIELD();
	}
	return true;
}

int I2C::start() {
	TickType_t ticks_to_wait = pdMS_TO_TICKS(25);
	TimeOut_t timeout;
	
	if (!m_start) {
		if (!waitForBusyFlag(&timeout, &ticks_to_wait))
			return ERR_BUSY;
	}
	
	taskENTER_CRITICAL();
	i2c_send_start(m_config->i2c);
	m_start = true;
	taskEXIT_CRITICAL();
	
	return ERR_SUCCESS;
}

int I2C::stop() {
	taskENTER_CRITICAL();
	i2c_send_stop(m_config->i2c);
	m_start = false;
	taskEXIT_CRITICAL();
	
	return ERR_SUCCESS;
}

int I2C::read(uint16_t addr, uint8_t *buffer, int size, bool repeated, int timeout_ms) {
	TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
	TimeOut_t timeout;
	
	vTaskSetTimeOutState(&timeout);
	
	if (!size)
		return ERR_SUCCESS;
	
	m_isr.error = ERR_SUCCESS;
	m_isr.buffer = buffer;
	m_isr.size = size;
	m_isr.remain = size;
	m_isr.addr = (addr << 1) | 1;
	m_isr.repeated = repeated;
	
	printf("read m_start=%d\r\n", m_start);
	
	i2c_nack_current(m_config->i2c);
	i2c_enable_ack(m_config->i2c);
	
	if (!m_start)
		start();
	
	i2c_enable_interrupt(m_config->i2c, I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
	
	xSemaphoreTake(m_transfer_sem, ticks_to_wait);
	
	if (m_isr.error == ERR_SUCCESS)
		return size - m_isr.remain;
	return m_isr.error;
}

int I2C::write(uint16_t addr, const uint8_t *buffer, int size, bool repeated, int timeout_ms) {
	TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
	TimeOut_t timeout;
	
	vTaskSetTimeOutState(&timeout);
	
	if (!size)
		return ERR_SUCCESS;
	
	m_isr.error = ERR_SUCCESS;
	m_isr.buffer = const_cast<uint8_t *>(buffer);
	m_isr.size = size;
	m_isr.remain = size;
	m_isr.addr = (addr << 1);
	m_isr.repeated = repeated;
	
	printf("write m_start=%d\r\n", m_start);
	
	i2c_nack_current(m_config->i2c);
	i2c_enable_ack(m_config->i2c);
	
	if (!m_start)
		start();
	
	i2c_enable_interrupt(m_config->i2c, I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
	
	xSemaphoreTake(m_transfer_sem, ticks_to_wait);
	
	if (m_isr.error == ERR_SUCCESS)
		return size - m_isr.remain;
	return m_isr.error;
}

int I2C::close() {
	if (!m_instances[m_id])
		return -ENOENT;
	
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
	return 0;
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
				
				if (m_isr.repeated) {
					// START=1
					i2c_send_start(m_config->i2c);
					m_start = true;
				} else {
					// STOP=1
					i2c_send_stop(m_config->i2c);
					m_start = false;
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
			} else {
				printf("Unknown IRQ [1b]\r\n");
				dump_sr1(sr1);
				dump_sr1(I2C_SR2(m_config->i2c));
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
				if (m_isr.repeated) {
					// START=1
					i2c_send_start(m_config->i2c);
					m_start = true;
				} else {
					// STOP=1
					i2c_send_stop(m_config->i2c);
					m_start = false;
				}
				
				// Read 2 bytes
				for (int i = 0; i < 2; i++) {
					*m_isr.buffer++ = I2C_DR(m_config->i2c);
					m_isr.remain--;
				}
				
				// Finish transfer
				i2c_disable_interrupt(m_config->i2c, I2C_ALL_IRQ);
				xSemaphoreGiveFromISR(m_transfer_sem, &higher_task_woken);
			} else {
				printf("Unknown IRQ [2b]\r\n");
				dump_sr1(sr1);
				dump_sr1(I2C_SR2(m_config->i2c));
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
				
				if (m_isr.repeated) {
					// START=1
					i2c_send_start(m_config->i2c);
					m_start = true;
				} else {
					// STOP=1
					i2c_send_stop(m_config->i2c);
					m_start = false;
				}
				
				// Read N-1 byte
				*m_isr.buffer++ = I2C_DR(m_config->i2c);
				m_isr.remain--;
				
				// Last byte must read after RxNE event, enable it
				i2c_enable_interrupt(m_config->i2c, I2C_CR2_ITBUFEN);
			} else {
				printf("Unknown IRQ [3b]\r\n");
				dump_sr1(sr1);
				dump_sr1(I2C_SR2(m_config->i2c));
			}
		}
	}
	// Write to slave
	else {
		if ((sr1 & I2C_SR1_ADDR)) {
			// Clear ADDR=1 by reading SR2
			(void) I2C_SR2(m_config->i2c);
			
			I2C_DR(m_config->i2c) = *m_isr.buffer++;
			m_isr.remain--;
			
			if (m_isr.remain > 0) {
				// We have more data for write
				i2c_enable_interrupt(m_config->i2c, I2C_CR2_ITBUFEN);
			}
		} else if ((I2C_CR2(m_config->i2c) & I2C_CR2_ITBUFEN) && (sr1 & I2C_SR1_TxE)) {
			I2C_DR(m_config->i2c) = *m_isr.buffer++;
			m_isr.remain--;
			
			if (!m_isr.remain) {
				// No more data, wait for BTF event
				i2c_disable_interrupt(m_config->i2c, I2C_CR2_ITBUFEN);
			}
		} else if (!(I2C_CR2(m_config->i2c) & I2C_CR2_ITBUFEN) && (sr1 & I2C_SR1_BTF)) {
			if (m_isr.repeated) {
				// START=1
				i2c_send_start(m_config->i2c);
				m_start = true;
			} else {
				// STOP=1
				i2c_send_stop(m_config->i2c);
				m_start = false;
			}
			
			// Finish transfer
			i2c_disable_interrupt(m_config->i2c, I2C_ALL_IRQ);
			xSemaphoreGiveFromISR(m_transfer_sem, &higher_task_woken);
		} else {
			printf("Unknown IRQ [w]\r\n");
			dump_sr1(sr1);
			dump_sr1(I2C_SR2(m_config->i2c));
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
	} else if ((status & I2C_SR1_OVR)) {
		m_isr.error = ERR_OVERRUN_OR_UNDERRUN;
	} else if ((status & I2C_SR1_AF)) {
		m_isr.error = ERR_NACK;
		
		// Send stop on AF
		i2c_send_stop(m_config->i2c);
		m_start = false;
	} else if ((status & I2C_SR1_ARLO)) {
		m_isr.error = ERR_ARBITRATION_LOST;
	} else if ((status & I2C_SR1_BERR)) {
		m_isr.error = ERR_BUS;
	} else {
		m_isr.error = ERR_UNKNOWN;
	}
	
	if ((m_isr.addr & 1)) {
		// POS=0
		i2c_nack_current(m_config->i2c);
		
		// Dummy read last byte from FIFO
		if ((status & I2C_SR1_RxNE))
			(void) I2C_DR(m_config->i2c);
	}
	
	i2c_disable_interrupt(m_config->i2c, I2C_ALL_IRQ);
	xSemaphoreGiveFromISR(m_transfer_sem, &higher_task_woken);
	portYIELD_FROM_ISR(higher_task_woken);
}

I2C::~I2C() {
	vSemaphoreDelete(m_transfer_sem);
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
