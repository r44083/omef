#pragma once

#include <stdint.h>
#include <stddef.h>
#include "dma/dma.hpp"

namespace hal
{
class adc
{
	public:
		enum adc_t
		{
			ADC_1,
			ADC_END
		};
		
		// Muliplexed ADC inputs (channels)
		enum adc_ch_t
		{
			ADC_CH_0,
			ADC_CH_1,
			ADC_CH_2,
			ADC_CH_3,
			ADC_CH_4,
			ADC_CH_5,
			ADC_CH_6,
			ADC_CH_7,
			ADC_CH_8,
			ADC_CH_9,
			ADC_CH_10,
			ADC_CH_11,
			ADC_CH_12,
			ADC_CH_13,
			ADC_CH_14,
			ADC_CH_15,
			ADC_CH_TEMP,
			ADC_CH_VREF,
			ADC_CH_END
		};
		
		enum adc_tim_t
		{
			ADC_TIM_1,
			ADC_TIM_2,
			ADC_TIM_3,
			ADC_TIM_4,
			ADC_TIM_END
		};
		
		enum adc_resol_t
		{
			ADC_RESOL_12BIT,
			ADC_RESOL_10BIT,
			ADC_RESOL_8BIT,
			ADC_RESOL_6BIT,
			ADC_RESOL_END
		};
		
		typedef void (*adc_cb_t)(adc *adc, adc_ch_t ch, float val, void *ctx);
		
		adc(adc_t adc, adc_ch_t ch_list[], size_t ch_list_size, adc_tim_t tim,
			hal::dma &dma, adc_resol_t resol = ADC_RESOL_12BIT,
			uint32_t freq = 1000, uint8_t num_of_samples = 16);
		~adc();
		
		void resol(adc_resol_t resol);
		adc_resol_t resol() const { return _resol; }
		
		void freq(uint32_t freq);
		uint32_t freq() const { return _freq; }
		
		// Link callback to specific ADC channel
		void add_clbk(adc_ch_t ch, adc_cb_t clbk, void *ctx);
		
		void enable(bool enable);
	
	private:
		adc_t _adc;
		adc_ch_t *_ch_list;
		size_t _ch_list_size;
		adc_tim_t _tim;
		dma &_dma;
		adc_resol_t _resol;
		uint32_t _freq;
		uint16_t *_dma_buff;
		uint8_t _num_of_samples;
		struct
		{
			adc_cb_t clbk;
			void *ctx;
		} _clbks[ADC_CH_END];
		
		void tim_init(uint32_t freq);
		static void calc_clk(adc_tim_t tim, uint32_t freq, uint16_t &presc, uint16_t &reload);
		void init_regular_chnls(uint8_t index, adc_ch_t ch);
		static void on_dma(dma *dma, dma::event_t event, void *ctx);
};
}