#pragma once

#include "adc.hpp"
#include "rcc/rcc.hpp"
#include "CMSIS/Device/STM32F1xx/Include/stm32f1xx.h"

namespace hal::adc_priv
{
constexpr TIM_TypeDef *const tim[adc::ADC_TIM_END] =
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
};

constexpr uint32_t rcc_en[adc::ADC_TIM_END] =
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
};

constexpr volatile uint32_t *const rcc_en_reg[adc::ADC_TIM_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR
};

constexpr rcc_src_t rcc_src[adc::ADC_TIM_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1
};
};