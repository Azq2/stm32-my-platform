#pragma once

#include <FreeRTOS.h>
#include <task.h>
#include <cstdint>

inline void delayMs(uint32_t ms) {
	vTaskDelay(pdMS_TO_TICKS(ms));
}

void delayNs(uint32_t ns);
void delayUs(uint32_t us);
