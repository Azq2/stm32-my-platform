#pragma once

#include "UsartBase.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

class Usart : public UsartBase {
	protected:
		Usart &operator=(const Usart &);
		Usart(const Usart &);
		
		struct {
			struct {
				char *buffer;
				size_t size;
			} read;
			
			struct {
				const char *buffer;
				size_t size;
			} write;
		} m_isr = {};
		
		SemaphoreHandle_t m_read_sem = nullptr;
		SemaphoreHandle_t m_write_sem = nullptr;
	
	public:
		explicit Usart(uint32_t usart);
		~Usart();
		
		void handleIrq() override;
		
		int read(char *buffer, int size, int timeout_ms = 60000) override;
		int write(const char *buffer, int size, int timeout_ms = 60000) override;
};
