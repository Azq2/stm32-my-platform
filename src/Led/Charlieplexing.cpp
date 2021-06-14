#include "Charlieplexing.h"

#include <cstdio>
#include <cmath>

Charlieplexing::Charlieplexing(Charlieplexing::Pin *pins, int n, int leds_n) {
	if (!leds_n)
		leds_n = n * (n - 1);
	
	m_pins = pins;
	m_pins_cnt = n;
	m_rows_cnt = (int) ceil((double) leds_n / (double) (n - 1));
}

void Charlieplexing::toggle(int led) {
	set(led, !get(led));
}

bool Charlieplexing::get(int led) {
	int n = m_pins_cnt - 1;
	int x = led / n;
	int y = led - (x * n);
	return m_pins[x].state & (1 << y);
}

void Charlieplexing::set(int led, bool state) {
	int n = m_pins_cnt - 1;
	int x = led / n;
	int y = led - (x * n);
	
	if (state) {
		m_pins[x].state |= 1 << y;
	} else {
		m_pins[x].state &= ~(1 << y);
	}
}

void Charlieplexing::frame() {
	// Reset previous cathode
	const Pin &prev_cathode = m_pins[m_counter > 0 ? m_counter - 1 : m_rows_cnt - 1];
	gpio_mode_setup(prev_cathode.bank, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, prev_cathode.pin);
	
	int led_n = 0;
	const Pin &cathode = m_pins[m_counter];
	for (int i = 0; i < m_pins_cnt; i++) {
		const Pin &p = m_pins[i];
		if (i != m_counter) {
			if ((cathode.state & (1 << led_n))) {
				gpio_mode_setup(p.bank, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, p.pin);
				gpio_set(p.bank, p.pin);
			} else {
				gpio_mode_setup(p.bank, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, p.pin);
			}
			
			led_n++;
		}
	}
	
	// Enable new cathode
	gpio_mode_setup(cathode.bank, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, cathode.pin);
	gpio_clear(cathode.bank, cathode.pin);
	
	m_counter = (m_counter + 1) % m_rows_cnt;
}
