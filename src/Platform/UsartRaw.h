#pragma once

#include "UsartBase.h"

class UsartRaw : public UsartBase {
	protected:
		UsartRaw &operator=(const UsartRaw &);
		UsartRaw(const UsartRaw &);
	public:
		explicit UsartRaw(uint32_t usart);
		~UsartRaw();
		
		void handleIrq() override;
		
		int read(char *buffer, int size, int timeout_ms = 60000) override;
		int write(const char *buffer, int size, int timeout_ms = 60000) override;
};
