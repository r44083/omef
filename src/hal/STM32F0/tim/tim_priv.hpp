#pragma once

#include "rcc/rcc.hpp"
#include "tim.hpp"
#include "CMSIS/Device/STM32F0xx/Include/stm32f0xx.h"

namespace hal::tim_priv
{
constexpr TIM_TypeDef *const tim[tim::TIM_END] =
{
	TIM1,
#if defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
	defined(STM32F091xC) || defined(STM32F098xx)
	TIM2,
#else
	NULL,
#endif
	TIM3, NULL, NULL,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	TIM6,
#else
	NULL,
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	TIM7,
#else
	NULL,
#endif
	NULL, NULL, NULL, NULL, NULL, NULL, TIM14,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	TIM15,
#else
	NULL,
#endif
	TIM16,
	TIM17
};

constexpr uint32_t rcc_en[tim::TIM_END] =
{
	RCC_APB2ENR_TIM1EN,
#if defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
	defined(STM32F091xC) || defined(STM32F098xx)
	RCC_APB1ENR_TIM2EN,
#else
	0,
#endif
	RCC_APB1ENR_TIM3EN,
	0,
	0,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	RCC_APB1ENR_TIM6EN,
#else
	0,
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	RCC_APB1ENR_TIM7EN,
#else
	0,
#endif
	0, 0, 0, 0, 0, 0, RCC_APB1ENR_TIM14EN,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	RCC_APB2ENR_TIM15EN,
#else
	0,
#endif
	RCC_APB2ENR_TIM16EN, RCC_APB2ENR_TIM17EN
};

constexpr volatile uint32_t *const rcc_en_reg[tim::TIM_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, NULL, NULL, &RCC->APB1ENR,
	&RCC->APB1ENR, NULL, NULL, NULL, NULL, NULL, NULL, &RCC->APB1ENR,
	&RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB2ENR
};

constexpr rcc_src_t rcc_src[tim::TIM_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1, static_cast<rcc_src_t>(0),
	static_cast<rcc_src_t>(0), RCC_SRC_APB1, RCC_SRC_APB1,
	static_cast<rcc_src_t>(0), static_cast<rcc_src_t>(0),
	static_cast<rcc_src_t>(0), static_cast<rcc_src_t>(0),
	static_cast<rcc_src_t>(0), static_cast<rcc_src_t>(0),
	RCC_SRC_APB1, RCC_SRC_APB2, RCC_SRC_APB2, RCC_SRC_APB2
};

constexpr IRQn_Type irqn[tim::TIM_END] =
{
	TIM1_CC_IRQn,
#if defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
	defined(STM32F091xC) || defined(STM32F098xx)
	TIM2_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
	TIM3_IRQn, static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	TIM6_DAC_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	TIM7_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0), TIM14_IRQn,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || \
	defined(STM32F058xx) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	TIM15_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
	TIM16_IRQn, TIM17_IRQn
};
};