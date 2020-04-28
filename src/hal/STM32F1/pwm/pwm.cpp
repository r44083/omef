#include <stddef.h>

#include "common/assert.h"

#include "pwm.hpp"
#include "tim/tim.hpp"
#include "rcc/rcc.hpp"
#include "gpio/gpio.hpp"
#include "CMSIS/Device/STM32F1xx/Include/stm32f1xx.h"

using namespace hal;

#define MAX_RESOL 0xFFFF

static TIM_TypeDef *const tim_list[tim::TIM_END] =
{
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F103x6) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM1,
#else
	NULL,
#endif
	TIM2, TIM3,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM4,
#else
	NULL,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	TIM5,
#else
	NULL,
#endif
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xE) || \
	defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM6, TIM7,
#else
	NULL, NULL,
#endif
#if defined(STM32F103xE) || defined(STM32F103xG)
	TIM8,
#else
	NULL,
#endif
#if defined(STM32F101xG) || defined(STM32F103xG)
	TIM9, TIM10, TIM11,
#else
	NULL, NULL, NULL,
#endif
#if defined(STM32F100xE) || defined(STM32F101xG) || defined(STM32F103xG)
	TIM12, TIM13, TIM14,
#else
	NULL, NULL, NULL,
#endif
#if defined(STM32F100xB) || defined(STM32F100xE)
	TIM15, TIM16, TIM17
#else
	NULL, NULL, NULL
#endif
};

static uint32_t const rcc_list[tim::TIM_END] =
{
	RCC_APB2ENR_TIM1EN, RCC_APB1ENR_TIM2EN, RCC_APB1ENR_TIM3EN,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	RCC_APB1ENR_TIM4EN,
#else
	0,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	RCC_APB1ENR_TIM5EN,
#else
	0,
#endif
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xE) || \
	defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	RCC_APB1ENR_TIM6EN, RCC_APB1ENR_TIM7EN,
#else
	0, 0,
#endif
#if defined(STM32F103xE) || defined(STM32F103xG)
	RCC_APB2ENR_TIM8EN,
#else
	0,
#endif
#if defined(STM32F101xG) || defined(STM32F103xG)
	RCC_APB2ENR_TIM9EN, RCC_APB2ENR_TIM10EN, RCC_APB2ENR_TIM11EN,
#else
	0, 0, 0,
#endif
#if defined(STM32F100xE) || defined(STM32F101xG) || defined(STM32F103xG)
	RCC_APB1ENR_TIM12EN, RCC_APB1ENR_TIM13EN, RCC_APB1ENR_TIM14EN,
#else
	0, 0, 0,
#endif
#if defined(STM32F100xB) || defined(STM32F100xE)
	RCC_APB2ENR_TIM15EN, RCC_APB2ENR_TIM16EN, RCC_APB2ENR_TIM17EN
#else
	0, 0, 0
#endif
};

static volatile uint32_t *rcc_bus_list[tim::TIM_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR,
	&RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR,
	&RCC->APB1ENR, &RCC->APB2ENR, &RCC->APB2ENR,
	&RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB1ENR,
	&RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR,
	&RCC->APB2ENR, &RCC->APB2ENR
};

static rcc_src_t const rcc_src_list[tim::TIM_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB2, RCC_SRC_APB2,
	RCC_SRC_APB2, RCC_SRC_APB2, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB2,
	RCC_SRC_APB2, RCC_SRC_APB2
};

static pwm::ch_t const max_ch_list[tim::TIM_END] =
{
	pwm::CH_4,   pwm::CH_4, pwm::CH_4,
	pwm::CH_4,   pwm::CH_4, pwm::CH_END,
	pwm::CH_END, pwm::CH_4, pwm::CH_2,
	pwm::CH_1,   pwm::CH_1, pwm::CH_2,
	pwm::CH_1,   pwm::CH_1
};

static uint32_t const ccmr_reg_list[pwm::CH_END][pwm::MODE_INVERTED + 1] = 
{
	{TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1, TIM_CCMR1_OC1M},
	{TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1, TIM_CCMR1_OC2M},
	{TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1, TIM_CCMR2_OC3M},
	{TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1, TIM_CCMR2_OC4M}
};

static void calc_freq(tim::tim_t tim, uint32_t freq, uint16_t *presc,
	uint16_t *reload);

static uint16_t calc_ccr(tim::tim_t tim, uint8_t duty);

pwm::pwm(tim::tim_t tim, ch_t ch, gpio &gpio, mode_t mode):
	_tim(tim),
	_ch(ch),
	_freq(0),
	_duty(0),
	_mode(mode),
	_gpio(gpio)
{
	ASSERT(_tim < tim::TIM_END && tim_list[_tim]);
	ASSERT(_ch < CH_END);
	ASSERT(_ch <= max_ch_list[_tim]);
	ASSERT(_mode <= MODE_NONINVERTED);
	ASSERT(_gpio.mode() == gpio::mode::AF);
	
	*rcc_bus_list[_tim] |= rcc_list[_tim];
	
	/* Enable PWM output */
	tim_list[_tim]->CCER |= TIM_CCER_CC1E << (_ch * 4);
	
	switch(_ch)
	{
		case CH_1:
			tim_list[_tim]->CCMR1 &= ~TIM_CCMR1_OC1M;
			tim_list[_tim]->CCMR1 |= ccmr_reg_list[_ch][_mode];
			break;
		
		case CH_2:
			tim_list[_tim]->CCMR1 &= ~TIM_CCMR1_OC2M;
			tim_list[_tim]->CCMR1 |= ccmr_reg_list[_ch][_mode];
			break;
		
		case CH_3:
			tim_list[_tim]->CCMR2 &= ~TIM_CCMR2_OC3M;
			tim_list[_tim]->CCMR2 |= ccmr_reg_list[_ch][_mode];
			break;
		
		case CH_4:
			tim_list[_tim]->CCMR2 &= ~TIM_CCMR2_OC4M;
			tim_list[_tim]->CCMR2 |= ccmr_reg_list[_ch][_mode];
			break;
		
		default: ASSERT(0);
	}
	
	/* Enable output for advanced timers */
	/* RM0090 chapter 17.4.18 (page 575) */
	if(_tim == tim::TIM_1 || _tim == tim::TIM_8)
		tim_list[_tim]->BDTR |= TIM_BDTR_MOE;
}

pwm::~pwm()
{
	
}

void pwm::freq(uint32_t freq)
{
	ASSERT(freq > 0);
	
	uint16_t presc = 0;
	uint16_t reload = 0;
	
	_freq = freq;
	calc_freq(_tim, _freq, &presc, &reload);
	
	tim_list[_tim]->PSC = presc;
	tim_list[_tim]->ARR = reload;
}

void pwm::duty(uint8_t duty)
{
	ASSERT(duty <= 100);
	
	_duty = duty;
	uint16_t ccr = calc_ccr(_tim, _duty);
	switch(_ch)
	{
		case CH_1: tim_list[_tim]->CCR1 = ccr; break;
		case CH_2: tim_list[_tim]->CCR2 = ccr; break;
		case CH_3: tim_list[_tim]->CCR3 = ccr; break;
		case CH_4: tim_list[_tim]->CCR4 = ccr; break;
		default: ASSERT(0);
	}
}

void pwm::start() const
{
	ASSERT(_freq > 0);
	
	tim_list[_tim]->CR1 &= ~TIM_CR1_OPM;
	tim_list[_tim]->CR1 |= TIM_CR1_CEN;
}

void pwm::stop() const
{
	tim_list[_tim]->CR1 &= ~TIM_CR1_CEN;
}

static void calc_freq(tim::tim_t tim, uint32_t freq, uint16_t *presc,
	uint16_t *reload)
{
	uint32_t tmp_presc = 0;
	uint32_t tmp_reload = 0;
	uint32_t clk_freq = rcc_get_freq(rcc_src_list[tim]);
	
	/* If APBx prescaller more then 1, TIMx prescaller multiplies by 2 */
	if(clk_freq != rcc_get_freq(RCC_SRC_AHB))
		clk_freq *= 2;
	
	/* Increase timer clock frequency or use timer with higher clock frequency
	to pass this assert */
	ASSERT(freq < clk_freq);
	
	tmp_reload = (clk_freq + (freq / 2)) / freq;
	if(tmp_reload <= MAX_RESOL)
		tmp_presc = 1;
	else
	{
		tmp_presc = ((tmp_reload + (MAX_RESOL / 2)) / MAX_RESOL) + 1;
		tmp_reload /= tmp_presc;
	}
	
	/* Minimum value for correct duty cycle setup (in percent). Increase timer
	clock frequency or use timer with higher clock frequency to pass this
	assert */
	ASSERT(tmp_reload > 100);
	
	ASSERT(tmp_presc <= MAX_RESOL);
	ASSERT(tmp_reload <= MAX_RESOL);
	
	*presc = (uint16_t)(tmp_presc - 1);
	*reload = (uint16_t)(tmp_reload - 1);
}

static uint16_t calc_ccr(tim::tim_t tim, uint8_t duty)
{
	uint16_t res = 0;
	
	if(duty > 0)
	{
		res = (tim_list[tim]->ARR * duty) / 100;
		res += 1;
	}
	
	return res;
}
