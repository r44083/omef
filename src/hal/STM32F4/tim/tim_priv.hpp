#pragma once

#include "rcc/rcc.hpp"
#include "tim.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"

namespace hal::tim_priv
{
constexpr TIM_TypeDef *const tim[tim::TIM_END] =
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

constexpr uint32_t rcc_en[tim::TIM_END] =
{
	RCC_APB2ENR_TIM1EN, RCC_APB1ENR_TIM2EN, RCC_APB1ENR_TIM3EN,
	RCC_APB1ENR_TIM4EN, RCC_APB1ENR_TIM5EN, RCC_APB1ENR_TIM6EN,
	RCC_APB1ENR_TIM7EN, RCC_APB2ENR_TIM8EN, RCC_APB2ENR_TIM9EN,
	RCC_APB2ENR_TIM10EN, RCC_APB2ENR_TIM11EN, RCC_APB1ENR_TIM12EN,
	RCC_APB1ENR_TIM13EN, RCC_APB1ENR_TIM14EN
};

constexpr uint32_t rcc_rst[tim::TIM_END] =
{
	RCC_APB2RSTR_TIM1RST, RCC_APB1RSTR_TIM2RST, RCC_APB1RSTR_TIM3RST,
	RCC_APB1RSTR_TIM4RST, RCC_APB1RSTR_TIM5RST, RCC_APB1RSTR_TIM6RST,
	RCC_APB1RSTR_TIM7RST, RCC_APB2RSTR_TIM9RST, RCC_APB2RSTR_TIM9RST,
	RCC_APB2RSTR_TIM10RST, RCC_APB2RSTR_TIM11RST, RCC_APB1RSTR_TIM12RST,
	RCC_APB1RSTR_TIM13RST, RCC_APB1RSTR_TIM14RST
};

constexpr volatile uint32_t *rcc_en_reg[tim::TIM_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR,
	&RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB2ENR,
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR
};

constexpr volatile uint32_t *rcc_rst_reg[tim::TIM_END] =
{
	&RCC->APB2RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR,
	&RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB2RSTR,
	&RCC->APB2RSTR, &RCC->APB2RSTR, &RCC->APB2RSTR, &RCC->APB1RSTR,
	&RCC->APB1RSTR, &RCC->APB1RSTR
};

constexpr rcc_src_t rcc_src[tim::TIM_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1,
	RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB2, RCC_SRC_APB2, RCC_SRC_APB2,
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1
};

constexpr IRQn_Type irqn[tim::TIM_END] =
{
	TIM1_CC_IRQn,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM2_IRQn, TIM3_IRQn, TIM4_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0),
#endif
	TIM5_IRQn,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F410Cx) || \
	defined(STM32F410Rx) || defined(STM32F410Tx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM6_DAC_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM7_IRQn, TIM8_CC_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#endif
	TIM1_BRK_TIM9_IRQn,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM1_UP_TIM10_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
	TIM1_TRG_COM_TIM11_IRQn,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	TIM8_BRK_TIM12_IRQn, TIM8_UP_TIM13_IRQn, TIM8_TRG_COM_TIM14_IRQn
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0)
#endif
};
};