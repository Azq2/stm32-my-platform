#include "UsartRaw.h"

UsartRaw::UsartRaw(uint32_t usart) : UsartBase(usart) {
	
}

UsartRaw::~UsartRaw() {
	
}

void UsartRaw::handleIrq() {
	
}

int UsartRaw::read(char *buffer, int size, int timeout_ms) {
	int received = 0;
	while (size--) {
		*buffer++ = usart_recv_blocking(m_config->usart) & 0xFF;
		received++;
	}
	return received;
}

int UsartRaw::write(const char *buffer, int size, int timeout_ms) {
	int written = 0;
	while (size--) {
		usart_send_blocking(m_config->usart, *buffer++);
		written++;
	}
	return written;
}
