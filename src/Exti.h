#pragma once

#include "Gpio.h"

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

class Exti {
	public:
		enum Trigger: int {
			RISING		= EXTI_TRIGGER_RISING,
			FALLING		= EXTI_TRIGGER_FALLING,
			BOTH		= EXTI_TRIGGER_BOTH
		};
		
		typedef void (Callback)(bool state, void *data);
	protected:
		
		static constexpr int EXTI_COUNT = 16;
		
		static constexpr uint32_t id2exti(int id) {
			return 1 << id;
		}
		
		struct Channel {
			Callback *callback = nullptr;
			void *user_data = nullptr;
			uint32_t bank = 0;
		};
		
		static Channel m_channels[EXTI_COUNT];
		
		static constexpr int getIrq(uint8_t id) {
			#ifdef STM32F4
			switch (id) {
				case 0:		return NVIC_EXTI0_IRQ;
				case 1:		return NVIC_EXTI1_IRQ;
				case 2:		return NVIC_EXTI2_IRQ;
				case 3:		return NVIC_EXTI3_IRQ;
				case 4:		return NVIC_EXTI3_IRQ;
				
				case 5: case 6: case 7: case 8: case 9:
					return NVIC_EXTI9_5_IRQ;
				case 10: case 11: case 12: case 13: case 14: case 15:
					return NVIC_EXTI15_10_IRQ;
			}
			#else
				#error Unsupported hardware
			#endif
			return -1;
		}
		
		static constexpr bool isSharedIrqUsed(int irq_n) {
			#ifdef STM32F4
			switch (irq_n) {
				case NVIC_EXTI9_5_IRQ:
					for (int i = 5; i <= 9; i++) {
						if (m_channels[i].bank)
							return true;
					}
				break;
				
				case NVIC_EXTI15_10_IRQ:
					for (int i = 10; i <= 15; i++) {
						if (m_channels[i].bank)
							return true;
					}
				break;
			}
			#else
				#error Unsupported hardware
			#endif
			return false;
		}
	public:
		enum Errors {
			ERR_SUCCESS		= 0,
			ERR_EXISTS		= -1,
		};
		
		static void handleIrq(uint8_t id);
		static inline void handleIrqRange(uint8_t from, uint8_t to) {
			for (uint8_t i = from; i <= to; i++) {
				if (exti_get_flag_status(id2exti(i)))
					Exti::handleIrq(i);
			}
		};
		static int set(uint32_t bank, uint32_t pin, Trigger trigger, Callback *callback, void *user_data = nullptr);
		static int remove(uint32_t bank, uint32_t pin);
};
