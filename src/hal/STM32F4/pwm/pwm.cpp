#include <stddef.h>

#include "common/assert.h"
#include "pwm.hpp"
#include "tim/tim.hpp"
#include "rcc/rcc.hpp"
#include "gpio/gpio.hpp"
#include "gpio/gpio_priv.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"

using namespace hal;

#define MAX_RESOL 0xFFFF

static TIM_TypeDef *const tim_list[tim::TIM_END] =
{
	TIM1,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM2, TIM3, TIM4,
#else
	NULL, NULL, NULL,
#endif
	TIM5,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F410Cx) || \
	defined(STM32F410Rx) || defined(STM32F410Tx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM6,
#else
	NULL,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM7, TIM8,
#else
	NULL, NULL,
#endif
	TIM9,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM10,
#else
	NULL,
#endif
	TIM11,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM12, TIM13, TIM14
#else
	NULL, NULL, NULL
#endif
};

static uint32_t const rcc_list[tim::TIM_END] =
{
	RCC_APB2ENR_TIM1EN,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1ENR_TIM2EN, RCC_APB1ENR_TIM3EN, RCC_APB1ENR_TIM4EN,
#else
	0, 0, 0,
#endif
	RCC_APB1ENR_TIM5EN,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F410Cx) || \
	defined(STM32F410Rx) || defined(STM32F410Tx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1ENR_TIM6EN,
#else
	0,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1ENR_TIM7EN, RCC_APB2ENR_TIM8EN,
#else
	0, 0,
#endif
	RCC_APB2ENR_TIM9EN,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB2ENR_TIM10EN,
#else
	0,
#endif
	RCC_APB2ENR_TIM11EN,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1ENR_TIM12EN, RCC_APB1ENR_TIM13EN, RCC_APB1ENR_TIM14EN
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
	&RCC->APB1ENR, &RCC->APB1ENR
};

static rcc_src_t const rcc_src_list[tim::TIM_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB2, RCC_SRC_APB2,
	RCC_SRC_APB2, RCC_SRC_APB2, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB1
};

static pwm::ch_t const max_ch_list[tim::TIM_END] =
{
	pwm::CH_4,   pwm::CH_4, pwm::CH_4,
	pwm::CH_4,   pwm::CH_4, pwm::CH_END,
	pwm::CH_END, pwm::CH_4, pwm::CH_2,
	pwm::CH_1,   pwm::CH_1, pwm::CH_2,
	pwm::CH_1,   pwm::CH_1
};

static uint8_t const gpio_af_list[tim::TIM_END] =
{
	/* TIM2 for STM32F446xx (AF0) not implemented */
	0x01, 0x01, 0x02,
	0x02, 0x02, 0x00,
	0x00, 0x03, 0x03,
	0x03, 0x03, 0x09,
	0x09, 0x09
};

static uint32_t const ccmr_reg_list[pwm::CH_END][pwm::MODE_INVERTED + 1] = 
{
	{TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1, TIM_CCMR1_OC1M},
	{TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1, TIM_CCMR1_OC2M},
	{TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1, TIM_CCMR2_OC3M},
	{TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1, TIM_CCMR2_OC4M}
};

static void gpio_af_init(tim::tim_t tim, gpio &gpio);

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
	ASSERT(_mode <= MODE_INVERTED);
	ASSERT(_gpio.mode() == gpio::mode::AF);
	
	*rcc_bus_list[_tim] |= rcc_list[_tim];
	
	gpio_af_init(_tim, _gpio);
	
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

static void gpio_af_init(tim::tim_t tim, gpio &gpio)
{
	GPIO_TypeDef *gpio_reg = gpio_priv::ports[gpio.port()];
	
	/* Push-pull type */
	gpio_reg->OTYPER &= ~(GPIO_OTYPER_OT0 << 1);
	
	/* Configure alternate function */
	uint8_t pin = gpio.pin();
	if(pin < 8)
	{
		gpio_reg->AFR[0] &= ~(0x0F << (pin * 4));
		gpio_reg->AFR[0] |= gpio_af_list[tim] << (pin * 4);
	}
	else
	{
		gpio_reg->AFR[1] &= ~(0x0F << ((pin - 8) * 4));
		gpio_reg->AFR[1] |= gpio_af_list[tim] << ((pin - 8) * 4);
	}
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
