#pragma once

#include <stdint.h>
#include "tim/tim.hpp"
#include "gpio/gpio.hpp"

namespace hal
{
class pwm
{
	public:
		enum ch_t
		{
			CH_1,
			CH_2,
			CH_3,
			CH_4,
			CH_END
		};
		
		enum mode_t
		{
			MODE_NONINVERTED,
			MODE_INVERTED
		};
		
		pwm(tim::tim_t tim, ch_t ch, gpio &gpio, mode_t mode = MODE_NONINVERTED);
		~pwm();
		
		void freq(uint32_t freq);
		uint32_t freq() const { return _freq; }
		void duty(uint8_t duty);
		uint8_t duty() const { return _duty; }
		
		void start() const;
		void stop() const;
	
	private:
		tim::tim_t _tim;
		ch_t _ch;
		uint32_t _freq;
		uint8_t _duty;
		mode_t _mode;
		gpio &_gpio;
		static void gpio_af_init(tim::tim_t tim, gpio &gpio);
		static void calc_freq(tim::tim_t tim, uint32_t freq, uint16_t &presc,
			uint16_t &reload);
		static uint16_t calc_ccr(tim::tim_t tim, uint8_t duty);
};
}