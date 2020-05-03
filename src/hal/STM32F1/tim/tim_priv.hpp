#pragma once

#include "rcc/rcc.hpp"
#include "tim.hpp"
#include "CMSIS/Device/STM32F1xx/Include/stm32f1xx.h"

namespace hal::tim_priv
{
constexpr TIM_TypeDef *const tim[tim::TIM_END] =
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

constexpr uint32_t rcc_en[tim::TIM_END] =
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

constexpr volatile uint32_t *const rcc_en_reg[tim::TIM_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR,
	&RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB2ENR,
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR,
	&RCC->APB2ENR, &RCC->APB2ENR
};

constexpr rcc_src_t rcc_src[tim::TIM_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB2, RCC_SRC_APB2, RCC_SRC_APB2,
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB2,
	RCC_SRC_APB2, RCC_SRC_APB2
};

constexpr IRQn_Type irqn[tim::TIM_END] =
{
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F103x6) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM1_CC_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
	TIM2_IRQn, TIM3_IRQn,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM4_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	TIM5_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xE) || \
	defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM6_IRQn, TIM7_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F103xE) || defined(STM32F103xG)
	TIM8_CC_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F101xG) || defined(STM32F103xG)
	TIM9_IRQn, TIM10_IRQn, TIM11_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xE) || defined(STM32F101xG) || defined(STM32F103xG)
	TIM12_IRQn, TIM13_IRQn, TIM14_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xG) || \
	defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || \
	defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
	TIM1_BRK_TIM15_IRQn, TIM1_UP_TIM16_IRQn, TIM1_TRG_COM_TIM17_IRQn
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0)
#endif
};
};