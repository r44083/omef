#pragma once
#include "gpio.hpp"
#include "CMSIS/Device/STM32F1xx/Include/stm32f1xx.h"

namespace hal::gpio_priv
{
constexpr GPIO_TypeDef *const gpio[gpio::ports] =
{
	GPIOA, GPIOB, GPIOC, GPIOD,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xB) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	GPIOE,
#else
	NULL,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG)
	GPIOF, GPIOG
#else
	NULL, NULL
#endif
};

constexpr uint32_t rcc_en[gpio::ports] =
{
	RCC_APB2ENR_IOPAEN, RCC_APB2ENR_IOPBEN, RCC_APB2ENR_IOPCEN,
	RCC_APB2ENR_IOPDEN, RCC_APB2ENR_IOPEEN,
#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
	RCC_APB2ENR_IOPFEN, RCC_APB2ENR_IOPGEN
#else
	0, 0
#endif
};
};