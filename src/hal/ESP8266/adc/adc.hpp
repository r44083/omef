#pragma once

#include <stdint.h>
#include <stddef.h>

namespace hal
{
class adc
{
	public:
		enum adc_t
		{
			ADC_1 // TOUT (Pin 6)
		};
		
		enum adc_ch_t
		{
			ADC_CH_0,
			ADC_CH_VDD
		};
		
		enum adc_resol_t
		{
			ADC_RESOL_10BIT
		};
		
		adc(adc_t adc, adc_ch_t ch, adc_resol_t resol = ADC_RESOL_10BIT,
			uint8_t samples_number = 10);
		~adc();
		
		uint16_t code();
		float vltg();
		
		void resol(adc_resol_t resol);
		adc_resol_t resol() const { return _resol; }
	
	private:
		adc_t _adc;
		adc_ch_t _ch;
		adc_resol_t _resol;
		uint8_t _samples_number;
};
}