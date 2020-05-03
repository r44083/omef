#pragma once

#include "rcc/rcc.hpp"
#include "uart.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"

namespace hal::uart_priv
{
constexpr USART_TypeDef *const uart[uart::UART_END] =
{
	USART1, USART2,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	USART3,
#else
	NULL,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	UART4, UART5,
#else
	NULL, NULL,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	USART6
#else
	NULL
#endif
};

constexpr IRQn_Type irqn[uart::UART_END] =
{
	USART1_IRQn, USART2_IRQn,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	USART3_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	UART4_IRQn, UART5_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	USART6_IRQn
#else
	static_cast<IRQn_Type>(0)
#endif
};

constexpr uint32_t rcc_en[uart::UART_END] =
{
	RCC_APB2ENR_USART1EN, RCC_APB1ENR_USART2EN,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1ENR_USART3EN,
#else
	0,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB1ENR_UART4EN, RCC_APB1ENR_UART5EN,
#else
	0, 0,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB2ENR_USART6EN
#else
	0
#endif
};

constexpr uint32_t rcc_rst[uart::UART_END] =
{
	RCC_APB2RSTR_USART1RST, RCC_APB1RSTR_USART2RST,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1RSTR_USART3RST,
#else
	0,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB1RSTR_UART4RST, RCC_APB1RSTR_UART5RST,
#else
	0, 0,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB2RSTR_USART6RST
#else
	0
#endif
};

constexpr volatile uint32_t *rcc_en_reg[uart::UART_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR,
	&RCC->APB2ENR
};

constexpr volatile uint32_t *rcc_rst_reg[uart::UART_END] =
{
	&RCC->APB2RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR,
	&RCC->APB1RSTR, &RCC->APB2RSTR
};

constexpr rcc_src_t rcc_src[uart::UART_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1,
	RCC_SRC_APB2
};

constexpr uint8_t gpio_af_list[uart::UART_END] =
{
	0x07, 0x07, 0x07, 0x08, 0x08, 0x08
};
};