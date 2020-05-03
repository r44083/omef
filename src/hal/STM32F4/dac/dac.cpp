#include <stddef.h>
#include "common/assert.h"
#include "dac.hpp"
#include "gpio/gpio.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"

#if !defined(STM32F405xx) && !defined(STM32F407xx) && !defined(STM32F410Cx) && \
	!defined(STM32F410Rx) && !defined(STM32F410Tx) && !defined(STM32F413xx) && \
	!defined(STM32F415xx) && !defined(STM32F417xx) && !defined(STM32F423xx) && \
	!defined(STM32F427xx) && !defined(STM32F429xx) && !defined(STM32F437xx) && \
	!defined(STM32F439xx) && !defined(STM32F446xx) && !defined(STM32F469xx) && \
	!defined(STM32F479xx)
	#error "MCU doesn't have DAC"
#endif

using namespace hal;

dac::dac(dac_t dac, align_t align, gpio &gpio):
	_dac(dac),
	_align(align),
	_gpio(gpio)
{
	ASSERT(_dac < DAC_END);
	ASSERT(_align <= ALIGN_L_12);
	ASSERT(_gpio.mode() == gpio::mode::AN);
	
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	uint32_t cr = DAC_CR_EN1 | DAC_CR_TSEL1;
	DAC->CR = cr << (_dac == DAC_1 ? 0 : DAC_CR_EN2_Pos);
}

dac::~dac()
{
	DAC->CR &= ~(DAC_CR_EN1 << (_dac == DAC_1 ? 0 : DAC_CR_EN2_Pos));
	RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
}

void dac::set(uint16_t val) const
{
	ASSERT(val < 4096);
	ASSERT(val < 256 || _align != ALIGN_R_8);
	
	if(_dac == DAC_1)
	{
		if(_align == ALIGN_R_8)
			DAC->DHR8R1 = val;
		else if(_align == ALIGN_R_12)
			DAC->DHR12R1 = val;
		else
			DAC->DHR12L1 = val;
		DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
	}
	else
	{
		if(_align == ALIGN_R_8)
			DAC->DHR8R2 = val;
		else if(_align == ALIGN_R_12)
			DAC->DHR12R2 = val;
		else
			DAC->DHR12L2 = val;
		DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;
	}
}

void dac::set(float voltage) const
{
#define V_REF (float)3.3
	ASSERT(voltage <= V_REF);
	
	uint16_t code = 0;
	
	if(_align == ALIGN_R_8)
		code = (voltage / V_REF) * 255;
	else
		code = (voltage / V_REF) * 4095;
	
	set(code);
}

uint16_t dac::get() const
{
	return _dac == DAC_1 ? DAC->DOR1 : DAC->DOR2;
}