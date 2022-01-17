#include "Spi.h"

#include <cerrno>

#include <FreeRTOS.h>
#include <task.h>

Spi *Spi::m_instances[Spi::SPI_COUNT] = {};

Spi::Spi(uint32_t spi) {
	for (int i = 0; i < SPI_COUNT; i++) {
		if (m_spi_ports[i].spi == spi) {
			m_id = i;
			m_config = &m_spi_ports[i];
			break;
		}
	}
	
	m_transfer_sem = xSemaphoreCreateBinary();
	
	configASSERT(m_id != -1);
}

void Spi::setSpeed(uint32_t speed) {
	m_speed = speed;
	configure();
}

void Spi::setMode(Mode mode) {
	m_mode = mode;
	configure();
}

void Spi::setBitOrder(Spi::BitOrder order) {
	m_data_bit_order = order;
	configure();
}

void Spi::configure() {
	if (!m_instances[m_id])
		return;
	
	taskENTER_CRITICAL();
	uint32_t bus = getSpiBusClock();
	
	static constexpr struct {
		uint32_t br;
		uint16_t div;
	} baudrates[] = {
		{SPI_CR1_BAUDRATE_FPCLK_DIV_2, 2},
		{SPI_CR1_BAUDRATE_FPCLK_DIV_4, 4},
		{SPI_CR1_BAUDRATE_FPCLK_DIV_8, 8},
		{SPI_CR1_BAUDRATE_FPCLK_DIV_16, 16},
		{SPI_CR1_BAUDRATE_FPCLK_DIV_32, 32},
		{SPI_CR1_BAUDRATE_FPCLK_DIV_64, 64},
		{SPI_CR1_BAUDRATE_FPCLK_DIV_128, 128},
		{SPI_CR1_BAUDRATE_FPCLK_DIV_256, 256}
	};
	
	uint32_t br, div;
	for (size_t i = 0; i < (sizeof(baudrates) / sizeof(baudrates[0])); i++) {
		div = baudrates[i].div;
		br = baudrates[i].br;
		
		if (m_speed >= (bus / baudrates[i].div))
			break;
	}
	
	m_real_speed = bus / div;
	spi_init_master(
		m_config->spi,
		br,
		(m_mode & SPI_CR1_CPOL),
		(m_mode & SPI_CR1_CPHA),
		SPI_CR1_DFF_8BIT,
		(m_data_bit_order == MSB_FIRST ? SPI_CR1_MSBFIRST : SPI_CR1_LSBFIRST)
	);
	spi_disable_crc(m_config->spi);
	
	taskEXIT_CRITICAL();
}

int Spi::open() {
	if (m_instances[m_id])
		return -EEXIST;
	
	m_instances[m_id] = this;
	
	taskENTER_CRITICAL();
	// Enable clock & reset
	rcc_periph_clock_enable(m_config->rcc);
	rcc_periph_reset_pulse(m_config->rst);
	
	// Configure peripheral
	configure();
	
	spi_set_full_duplex_mode(m_config->spi);
	spi_enable_software_slave_management(m_config->spi);
	spi_set_nss_high(m_config->spi);
	
	// Enable spi
	spi_enable(m_config->spi);
	
	// Enable IRQ
	nvic_enable_irq(m_config->irq);
	nvic_set_priority(m_config->irq, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	
	taskEXIT_CRITICAL();
	return 0;
}

int Spi::close() {
	if (!m_instances[m_id])
		return -ENOENT;
	
	taskENTER_CRITICAL();
	// Disable IRQ
	nvic_disable_irq(m_config->irq);
	
	// Disable spi
	spi_disable(m_config->spi);
	
	// Disable spi clock
	rcc_periph_clock_disable(m_config->rcc);
	
	m_instances[m_id] = nullptr;
	
	taskEXIT_CRITICAL();
	return 0;
}

int Spi::transfer(const void *buffer_tx, int size_tx, void *buffer_rx, int size_rx, uint16_t default_write, bool wide) {
	int total_size = size_tx > size_rx ? size_tx : size_rx;
	if (!total_size)
		return ERR_SUCCESS;
	
	m_isr.error = ERR_SUCCESS;
	m_isr.write.buffer = buffer_tx;
	m_isr.write.size = size_tx;
	m_isr.read.buffer = buffer_rx;
	m_isr.read.size = size_rx;
	m_isr.size = total_size;
	m_isr.wide = wide;
	m_isr.default_write = default_write;
	
	if (wide) {
		spi_set_dff_16bit(m_config->spi);
	} else {
		spi_set_dff_8bit(m_config->spi);
	}
	
	SPI_CR2(m_config->spi) |= SPI_CR2_TXEIE | SPI_CR2_ERRIE;
	
	xSemaphoreTake(m_transfer_sem, portMAX_DELAY);
	
	if (m_isr.error == ERR_SUCCESS)
		return total_size - m_isr.size;
	return m_isr.error;
}

void Spi::handleIrq() {
	BaseType_t higher_task_woken = pdFALSE;
	
	// Errors
	if ((SPI_CR2(m_config->spi) & SPI_CR2_ERRIE) && (SPI_SR(m_config->spi) & SPI_ERRORS_FLAGS)) {
		// Error in SPI Master mode is nearly impossible.
		// Only possible error is MODF (Mode fault), but this error throws only with invalid SPI configuration.
		// All other errors related to I2S or slave mode.
		m_isr.error = ERR_UNKNOWN;
		
		SPI_SR(m_config->spi) &= ~SPI_ERRORS_FLAGS;
		SPI_CR2(m_config->spi) &= ~SPI_ALL_IRQ;
		
		xSemaphoreGiveFromISR(m_transfer_sem, &higher_task_woken);
	}
	
	// TX
	if ((SPI_CR2(m_config->spi) & SPI_CR2_TXEIE) && (SPI_SR(m_config->spi) & SPI_SR_TXE)) {
		SPI_CR2(m_config->spi) &= ~SPI_CR2_TXEIE;
		
		if (m_isr.write.size > 0) {
			// Send data from TX buffer
			if (m_isr.wide) {
				SPI_DR(m_config->spi) = *m_isr.write.buffer16++;
			} else {
				SPI_DR(m_config->spi) = *m_isr.write.buffer8++;
			}
			
			m_isr.write.size--;
		} else {
			// If TX buffer empty, send default write value
			if (m_isr.wide) {
				SPI_DR(m_config->spi) = m_isr.default_write;
			} else {
				SPI_DR(m_config->spi) = m_isr.default_write & 0xFF;
			}
		}
		
		SPI_CR2(m_config->spi) |= SPI_CR2_RXNEIE;
	}
	
	// RX
	if ((SPI_CR2(m_config->spi) & SPI_CR2_RXNEIE) && (SPI_SR(m_config->spi) & SPI_SR_RXNE)) {
		SPI_CR2(m_config->spi) &= ~SPI_SR_RXNE;
		
		if (m_isr.read.size > 0) {
			// Read data to RX buffer
			if (m_isr.wide) {
				*m_isr.read.buffer16++ = SPI_DR(m_config->spi);
			} else {
				*m_isr.read.buffer8++ = SPI_DR(m_config->spi);
			}
			m_isr.read.size--;
		} else {
			// If RX buffer full, make dummy read
			(void) SPI_DR(m_config->spi);
		}
		
		m_isr.size--;
		
		if (m_isr.size > 0) {
			SPI_CR2(m_config->spi) |= SPI_CR2_TXEIE;
		} else {
			SPI_CR2(m_config->spi) &= ~SPI_ALL_IRQ;
			xSemaphoreGiveFromISR(m_transfer_sem, &higher_task_woken);
		}
	}
	
	if (higher_task_woken)
		portYIELD_FROM_ISR(higher_task_woken);
}

Spi::~Spi() {
	vSemaphoreDelete(m_transfer_sem);
	close();
}

#ifdef STM32F4
extern "C" void spi1_isr(void) {
	Spi::irq(0);
}

extern "C" void spi2_isr(void) {
	Spi::irq(1);
}

extern "C" void spi3_isr(void) {
	Spi::irq(2);
}

extern "C" void spi4_isr(void) {
	Spi::irq(3);
}

extern "C" void spi5_isr(void) {
	Spi::irq(4);
}

extern "C" void spi6_isr(void) {
	Spi::irq(5);
}
#else
	#error Unsupported hardware
#endif
