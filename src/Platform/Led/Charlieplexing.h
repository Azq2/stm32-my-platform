#pragma once

#include <libopencm3/stm32/gpio.h>

#include <cstddef>

class Charlieplexing {
	public:
		struct Pin {
			uint32_t bank;
			uint32_t pin;
			uint32_t state;
		};
		
		Charlieplexing(Pin *pins, int n, int leds_n = 0);
		~Charlieplexing() { };
		
		void frame();
		void set(int led, bool state);
		bool get(int led);
		void toggle(int led);
		
		constexpr int getDelay(int fps) {
			return 1000 / fps / m_rows_cnt;
		}
	protected:
		Pin *m_pins = nullptr;
		int m_pins_cnt = 0;
		int m_rows_cnt = 0;
		int m_counter = 0;
	
	private:
		Charlieplexing(const Charlieplexing &p);
		Charlieplexing &operator=(const Charlieplexing &p);
};
