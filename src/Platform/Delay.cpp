#include "Delay.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm/dwt.h>

#if defined(STM32F0)
	// Cortext-M0
	// SUBS (1) + BCS (3)
	constexpr uint32_t LOOP_TICKS = 4;
#elif defined(STM32L0) || defined(STM32G0)
	// Cortext-M0+
	// SUBS (1) + BCS (2)
	constexpr uint32_t LOOP_TICKS = 3;
#elif defined(STM32F1) || defined(STM32F2) || defined(STM32L1)
	// Cortext-M3
	// SUBS (1) + BCS (2)
	constexpr uint32_t LOOP_TICKS = 3;
#elif defined(STM32F3) || defined(STM32F4) || defined(STM32G4) || defined(STM32L4)
	// Cortext-M4F
	// SUBS (1) + BCS (2)
	constexpr uint32_t LOOP_TICKS = 3;
#elif defined(STM32F7) || defined(STM32H7)
	// Cortext-M7F
	// SUBS (1) + BCS (1)
	constexpr uint32_t LOOP_TICKS = 2;
#else
	#error Unsupported hardware
#endif

// TODO: use DWT, if available?
void delayNs(uint32_t ns) {
	uint32_t cycles = (ns * 100) / ((1000000000 * LOOP_TICKS) / (rcc_ahb_frequency / 100));
	__asm__ volatile (
		"1: \n"
		"SUBS %0, #1\n"
		"BCS 1b \n" : : "r" (cycles) : "memory"
	);
}

void delayUs(uint32_t us) {
	while (us > 1000) {
		delayNs(1000000);
		us -= 1000;
	}
	
	if (us > 0)
		delayNs(us * 1000);
}
