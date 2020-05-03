#pragma once

#include "pwm.hpp"
#include "tim/tim_priv.hpp"

namespace hal::pwm_priv
{
constexpr pwm::ch_t max_ch[tim::TIM_END] =
{
	pwm::CH_4, pwm::CH_4, pwm::CH_4, pwm::CH_4, pwm::CH_4, pwm::CH_END,
	pwm::CH_END, pwm::CH_4, pwm::CH_2, pwm::CH_1, pwm::CH_1, pwm::CH_2,
	pwm::CH_1, pwm::CH_1
};

constexpr uint32_t ccmr[pwm::CH_END][pwm::MODE_INVERTED + 1] = 
{
	{TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1, TIM_CCMR1_OC1M},
	{TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1, TIM_CCMR1_OC2M},
	{TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1, TIM_CCMR2_OC3M},
	{TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1, TIM_CCMR2_OC4M}
};
};