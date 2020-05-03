#pragma once

#include <stdint.h>
#include "gpio/gpio.hpp"

namespace hal
{
class dac
{
	public:
		enum dac_t
		{
			DAC_1,
			DAC_2,
			DAC_END
		};
		
		enum align_t
		{
			ALIGN_R_8,
			ALIGN_R_12,
			ALIGN_L_12
		};
		
		dac(dac_t dac, align_t align, gpio &gpio);
		~dac();
		
		void set(uint16_t val) const;
		void set(float val) const;
		uint16_t get() const;
	
	private:
		dac_t _dac;
		align_t _align;
		gpio &_gpio;
};
}