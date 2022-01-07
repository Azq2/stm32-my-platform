#include "UsartBase.h"

#include <cerrno>

#include <FreeRTOS.h>
#include <task.h>

UsartBase *UsartBase::m_instances[UsartBase::USARTS_COUNT] = {};

UsartBase::UsartBase(uint32_t usart) {
	for (int i = 0; i < USARTS_COUNT; i++) {
		if (m_usarts[i].usart == usart) {
			m_id = i;
			m_config = &m_usarts[i];
			break;
		}
	}
	
	configASSERT(m_id != -1);
}

UsartBase::~UsartBase() {
	close();
}

void UsartBase::init() {
	// Stub
}

void UsartBase::deinit() {
	// Stub
}

void UsartBase::setBaurd(uint32_t baurd) {
	m_baudrate = baurd;
	configure();
}

void UsartBase::setPortConfig(uint32_t databits, UsartBase::Parity parity, UsartBase::StopBits stopbits) {
	m_databits = databits;
	m_parity = parity;
	m_stopbits = stopbits;
	configure();
}

void UsartBase::setMode(UsartBase::Mode mode) {
	m_mode = mode;
	configure();
}

void UsartBase::setFlowControl(bool rts, bool cts) {
	m_flowcontrol = (rts ? USART_FLOWCONTROL_RTS : 0) | (cts ? USART_FLOWCONTROL_CTS : 0);
	configure();
}

void UsartBase::setHalfDuplex(bool enable) {
	m_half_duplex = enable;
	configure();
}

void UsartBase::configure() {
	if (!m_instances[m_id])
		return;
	
	usart_set_baudrate(m_config->usart, m_baudrate);
	usart_set_databits(m_config->usart, m_databits);
	usart_set_stopbits(m_config->usart, m_stopbits);
	usart_set_parity(m_config->usart, m_parity);
	usart_set_flow_control(m_config->usart, m_flowcontrol);
	usart_set_mode(m_config->usart, m_mode);
	
	if (m_half_duplex) {
		USART_CR3(m_config->usart) = USART_CR3_HDSEL;
	} else {
		USART_CR3(m_config->usart) &= ~USART_CR3_HDSEL;
	}
}

int UsartBase::open() {
	if (m_instances[m_id])
		return -EEXIST;
	
	m_instances[m_id] = this;
	
	taskENTER_CRITICAL();
	// Enable clock & reset
	rcc_periph_clock_enable(m_config->rcc);
	rcc_periph_reset_pulse(m_config->rst);
	
	// Configure peripheral
	configure();
	
	// Implementation specific init
	init();
	
	// Enable usart
	usart_enable(m_config->usart);
	
	// Enable IRQ
	nvic_enable_irq(m_config->irq);
	nvic_set_priority(m_config->irq, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	
	taskEXIT_CRITICAL();
	return 0;
}

int UsartBase::close() {
	if (!m_instances[m_id])
		return -ENOENT;
	
	taskENTER_CRITICAL();
	// Disable IRQ
	nvic_disable_irq(m_config->irq);
	
	// Disable usart
	usart_disable(m_config->usart);
	
	// Disable usart clock
	rcc_periph_clock_disable(m_config->rcc);
	
	m_instances[m_id] = nullptr;
	
	// Implementation specific deinit
	deinit();
	
	taskEXIT_CRITICAL();
	return 0;
}

#ifdef STM32F4
extern "C" void usart1_isr(void) {
	UsartBase::irq(0);
}

extern "C" void usart2_isr(void) {
	UsartBase::irq(1);
}

extern "C" void usart3_isr(void) {
	UsartBase::irq(2);
}

extern "C" void uart4_isr(void) {
	UsartBase::irq(3);
}

extern "C" void uart5_isr(void) {
	UsartBase::irq(4);
}

extern "C" void usart6_isr(void) {
	UsartBase::irq(5);
}

extern "C" void uart7_isr(void) {
	UsartBase::irq(6);
}

extern "C" void uart8_isr(void) {
	UsartBase::irq(7);
}
#else
	#error Unsupported hardware
#endif
