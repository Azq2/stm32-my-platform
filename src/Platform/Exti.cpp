#include "Exti.h"

#include <libopencm3/stm32/gpio.h>

#include <cstdio>

Exti::Channel Exti::m_channels[EXTI_COUNT];

int Exti::set(uint32_t bank, uint32_t pin, Trigger trigger, Callback *callback, void *user_data) {
	uint8_t id = Gpio::pin2id(pin);
	uint32_t exti = id2exti(id);
	
	taskENTER_CRITICAL();
	if (m_channels[id].bank) {
		taskEXIT_CRITICAL();
		return ERR_EXISTS;
	}
	
	m_channels[id].bank = bank;
	m_channels[id].callback = callback;
	m_channels[id].user_data = user_data;
	
	// Enable EXTI and connect to specified GPIO bank
	exti_select_source(exti, bank);
	exti_set_trigger(exti, (exti_trigger_type) trigger);
	exti_enable_request(exti);
	
	// Enable IRQ for EXTI
	int irq_n = getIrq(id);
	nvic_enable_irq(irq_n);
	nvic_set_priority(irq_n, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	
	taskEXIT_CRITICAL();
	
	return ERR_SUCCESS;
}

int Exti::remove(uint32_t bank, uint32_t pin) {
	int id = Gpio::pin2id(pin);
	uint32_t exti = id2exti(id);
	
	taskENTER_CRITICAL();
	if (!m_channels[id].bank || m_channels[id].bank != bank) {
		taskEXIT_CRITICAL();
		return ERR_SUCCESS;
	}
	
	m_channels[id].bank = 0;
	m_channels[id].callback = nullptr;
	m_channels[id].user_data = nullptr;
	
	// Disconnect GPIO bank from EXTI
	exti_select_source(exti, 0);
	
	// Disable exti
	exti_disable_request(exti);
	
	// Disable IRQ, if it not used by another EXTI
	int irq_n = getIrq(id);
	if (!isSharedIrqUsed(irq_n))
		nvic_disable_irq(irq_n);
	
	taskEXIT_CRITICAL();
	
	return ERR_SUCCESS;
}

void Exti::handleIrq(uint8_t id) {
	uint32_t exti = id2exti(id);
	Channel &channel = m_channels[id];
	
	bool state = false;
	if ((EXTI_RTSR & exti) && (EXTI_FTSR & exti)) {
		uint16_t pin = Gpio::id2pin(id);
		state = gpio_get(channel.bank, pin) != 0;
	} else if ((EXTI_RTSR & exti)) {
		state = true;
	}
	
	exti_reset_request(exti);
	channel.callback(state, channel.user_data);
}

#ifdef STM32F4
extern "C" void exti0_isr(void) {
	Exti::handleIrq(0);
}

extern "C" void exti1_isr(void) {
	Exti::handleIrq(1);
}

extern "C" void exti2_isr(void) {
	Exti::handleIrq(2);
}

extern "C" void exti3_isr(void) {
	Exti::handleIrq(3);
}

extern "C" void exti4_isr(void) {
	Exti::handleIrq(4);
}

extern "C" void exti9_5_isr(void) {
	Exti::handleIrqRange(5, 9);
}

extern "C" void exti15_19_isr(void) {
	Exti::handleIrqRange(10, 15);
}
#else
	#error Unsupported hardware
#endif
