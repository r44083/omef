#include <stddef.h>
#include <limits>
#include "common/assert.h"
#include "pwm.hpp"
#include "pwm_priv.hpp"
#include "tim/tim.hpp"
#include "tim/tim_priv.hpp"
#include "rcc/rcc.hpp"
#include "gpio/gpio.hpp"
#include "gpio/gpio_priv.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"

using namespace hal;

pwm::pwm(tim::tim_t tim, ch_t ch, gpio &gpio, mode_t mode):
	_tim(tim),
	_ch(ch),
	_freq(0),
	_duty(0),
	_mode(mode),
	_gpio(gpio)
{
	ASSERT(_tim < tim::TIM_END && tim_priv::tim[_tim]);
	ASSERT(_ch < CH_END);
	ASSERT(_ch <= pwm_priv::max_ch[_tim]);
	ASSERT(_mode <= MODE_INVERTED);
	ASSERT(_gpio.mode() == gpio::mode::AF);
	
	*tim_priv::rcc_en_reg[_tim] |= tim_priv::rcc_en[_tim];
	
	gpio_af_init(_tim, _gpio);
	
	TIM_TypeDef *tim_reg = tim_priv::tim[_tim];
	
	// Enable PWM output
	tim_reg->CCER |= TIM_CCER_CC1E << (_ch * TIM_CCER_CC2E_Pos);
	
	switch(_ch)
	{
		case CH_1:
			tim_reg->CCMR1 &= ~TIM_CCMR1_OC1M;
			tim_reg->CCMR1 |= pwm_priv::ccmr[_ch][_mode];
			break;
		
		case CH_2:
			tim_reg->CCMR1 &= ~TIM_CCMR1_OC2M;
			tim_reg->CCMR1 |= pwm_priv::ccmr[_ch][_mode];
			break;
		
		case CH_3:
			tim_reg->CCMR2 &= ~TIM_CCMR2_OC3M;
			tim_reg->CCMR2 |= pwm_priv::ccmr[_ch][_mode];
			break;
		
		case CH_4:
			tim_reg->CCMR2 &= ~TIM_CCMR2_OC4M;
			tim_reg->CCMR2 |= pwm_priv::ccmr[_ch][_mode];
			break;
		
		default: ASSERT(0);
	}
	
	// Enable output for advanced timers: RM0090 chapter 17.4.18 (page 575)
	if(_tim == tim::TIM_1 || _tim == tim::TIM_8)
		tim_reg->BDTR |= TIM_BDTR_MOE;
}

pwm::~pwm()
{
	tim_priv::tim[_tim]->CR1 &= ~TIM_CR1_CEN;
	*tim_priv::rcc_en_reg[_tim] &= ~tim_priv::rcc_en[_tim];
}

void pwm::freq(uint32_t freq)
{
	ASSERT(freq > 0);
	
	_freq = freq;
	uint16_t presc, reload;
	calc_freq(_tim, _freq, presc, reload);
	
	tim_priv::tim[_tim]->PSC = presc;
	tim_priv::tim[_tim]->ARR = reload;
}

void pwm::duty(uint8_t duty)
{
	ASSERT(duty <= 100);
	
	_duty = duty;
	uint16_t ccr = calc_ccr(_tim, _duty);
	switch(_ch)
	{
		case CH_1: tim_priv::tim[_tim]->CCR1 = ccr; break;
		case CH_2: tim_priv::tim[_tim]->CCR2 = ccr; break;
		case CH_3: tim_priv::tim[_tim]->CCR3 = ccr; break;
		case CH_4: tim_priv::tim[_tim]->CCR4 = ccr; break;
		default: ASSERT(0);
	}
}

void pwm::start() const
{
	ASSERT(_freq > 0);
	
	tim_priv::tim[_tim]->CR1 &= ~TIM_CR1_OPM;
	tim_priv::tim[_tim]->CR1 |= TIM_CR1_CEN;
}

void pwm::stop() const
{
	tim_priv::tim[_tim]->CR1 &= ~TIM_CR1_CEN;
}

void pwm::gpio_af_init(tim::tim_t tim, gpio &gpio)
{
	GPIO_TypeDef *gpio_reg = gpio_priv::gpio[gpio.port()];
	uint8_t pin = gpio.pin();
	
	// Push-pull
	gpio_reg->OTYPER &= ~(GPIO_OTYPER_OT0 << pin);
	
	// Configure alternate function
	gpio_reg->AFR[pin / 8] &= ~(GPIO_AFRL_AFSEL0 << ((pin % 8) * 4));
	gpio_reg->AFR[pin / 8] |= pwm_priv::gpio_af_list[tim] << ((pin % 8) * 4);
}

void pwm::calc_freq(tim::tim_t tim, uint32_t freq, uint16_t &presc,
	uint16_t &reload)
{
	uint32_t clk_freq = rcc_get_freq(tim_priv::rcc_src[tim]);
	
	// If APBx prescaller more then 1, TIMx prescaller multiplies by 2
	if(clk_freq != rcc_get_freq(RCC_SRC_AHB))
		clk_freq *= 2;
	
	/* Increase timer clock frequency or use timer with higher clock frequency
	to pass this assert */
	ASSERT(freq < clk_freq);
	
	uint32_t tmp_presc = 0;
	uint32_t tmp_reload = (clk_freq + (freq / 2)) / freq;
	constexpr auto max_resol = std::numeric_limits<uint16_t>::max();
	
	if(tmp_reload <= max_resol)
		tmp_presc = 1;
	else
	{
		tmp_presc = ((tmp_reload + (max_resol / 2)) / max_resol) + 1;
		tmp_reload /= tmp_presc;
	}
	
	/* Minimum value for correct duty cycle setup (in percent). Increase timer
	clock frequency or use timer with higher clock frequency to pass this
	assert */
	ASSERT(tmp_reload > 100);
	
	ASSERT(tmp_presc <= max_resol);
	ASSERT(tmp_reload <= max_resol);
	
	presc = tmp_presc - 1;
	reload = tmp_reload - 1;
}

uint16_t pwm::calc_ccr(tim::tim_t tim, uint8_t duty)
{
	uint16_t ccr = 0;
	
	if(duty > 0)
	{
		ccr = (tim_priv::tim[tim]->ARR * duty) / 100;
		ccr += 1;
	}
	
	return ccr;
}