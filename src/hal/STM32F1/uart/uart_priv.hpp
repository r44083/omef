#pragma once

#include "rcc/rcc.hpp"
#include "uart.hpp"
#include "CMSIS/Device/STM32F1xx/Include/stm32f1xx.h"

namespace hal::uart_priv
{
constexpr USART_TypeDef *const uart[uart::UART_END] =
{
	USART1, USART2,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	USART3,
#else
	NULL,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	UART4, UART5
#else
	NULL, NULL
#endif
};

constexpr IRQn_Type irqn[uart::UART_END] =
{
	USART1_IRQn, USART2_IRQn,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	USART3_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	UART4_IRQn, UART5_IRQn
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0)
#endif
};

constexpr uint32_t rcc_en[uart::UART_END] =
{
	RCC_APB2ENR_USART1EN, RCC_APB1ENR_USART2EN,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	RCC_APB1ENR_USART3EN,
#else
	0
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	RCC_APB1ENR_UART4EN, RCC_APB1ENR_UART5EN
#else
	0, 0
#endif
};

constexpr volatile uint32_t *rcc_en_reg[uart::UART_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR
};

constexpr rcc_src_t rcc_src[uart::UART_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1
};
};