#include <string.h>
#include <stdlib.h>
#include "common/assert.h"
#include "adc.hpp"

using namespace hal;

extern "C" void phy_adc_read_fast(uint16_t *adc_addr, uint16_t adc_num,
	uint8_t adc_clk_div);
extern "C" uint16_t phy_get_vdd33();

adc::adc(adc_t adc, adc_ch_t ch, adc_resol_t resol, uint8_t samples_number):
	_adc(adc),
	_resol(resol),
	_samples_number(samples_number)
{
	ASSERT(_adc <= ADC_1);
	ASSERT(_ch <= ADC_CH_VDD);
	// ESP8266 supports only 10 bit resolution
	ASSERT(_resol == ADC_RESOL_10BIT);
	ASSERT(_samples_number > 0);
}

adc::~adc()
{
}

uint16_t adc::code()
{
	uint16_t res = 0;
	
	if(_ch == ADC_CH_0)
	{
		uint16_t data[_samples_number];
		uint8_t clk_div = 8;// 8..32
		phy_adc_read_fast(data, _samples_number, clk_div);
		
		uint32_t sum = 0;
		for(uint8_t i = 0; i < _samples_number; i++)
			sum += data[i];
		res = sum / _samples_number;
	}
	else if(_ch == ADC_CH_VDD)
		res = phy_get_vdd33();
	
	return res;
}

float adc::vltg()
{
	return code() / 1024;
}

void adc::resol(adc_resol_t resol)
{
	// ESP8266 supports only 10 bit resolution
	ASSERT(resol == ADC_RESOL_10BIT);
}