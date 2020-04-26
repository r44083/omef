#pragma once

#include "CMSIS/Device/STM32F0xx/Include/stm32f0xx.h"

namespace hal::uart_priv
{
constexpr auto irq_priority = 6;

constexpr USART_TypeDef *const uart_list[uart::UART_END] =
{
	USART1,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	USART2,
#else
	NULL,
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	USART3, USART4,
#else
	NULL, NULL,
#endif
#if defined(STM32F030xC) || defined(STM32F091xC) || defined(STM32F098xx)
	USART5, USART6,
#else
	NULL, NULL,
#endif
#if defined(STM32F091xC) || defined(STM32F098xx)
	USART7, USART8
#else
	NULL, NULL
#endif
};

constexpr IRQn_Type irq_list[uart::UART_END] =
{
	USART1_IRQn,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	USART2_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F030xC)
	USART3_6_IRQn, USART3_6_IRQn,
#elif defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
	defined(STM32F078xx)
	USART3_4_IRQn, USART3_4_IRQn,
#elif defined(STM32F091xC) || defined(STM32F098xx)
	USART3_8_IRQn, USART3_8_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F030xC)
	USART3_6_IRQn, USART3_6_IRQn,
#elif defined(STM32F091xC) || defined(STM32F098xx)
	USART3_8_IRQn, USART3_8_IRQn,
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F091xC) || defined(STM32F098xx)
	USART3_8_IRQn, USART3_8_IRQn
#else
	static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0)
#endif
};

constexpr uint32_t rcc_list[uart::UART_END] =
{
	RCC_APB2ENR_USART1EN,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	RCC_APB1ENR_USART2EN,
#else
	0,
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	RCC_APB1ENR_USART3EN, RCC_APB1ENR_USART4EN,
#else
	0, 0,
#endif
#if defined(STM32F030xC) || defined(STM32F091xC) || defined(STM32F098xx)
	RCC_APB1ENR_USART5EN, RCC_APB2ENR_USART6EN,
#else
	0, 0,
#endif
#if defined(STM32F091xC) || defined(STM32F098xx)
	RCC_APB2ENR_USART7EN, RCC_APB2ENR_USART8EN
#else
	0, 0
#endif
};

constexpr uint32_t reset_list[uart::UART_END] =
{
	RCC_APB2RSTR_USART1RST,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	RCC_APB1RSTR_USART2RST,
#else
	0,
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
	RCC_APB1RSTR_USART3RST, RCC_APB1RSTR_USART4RST,
#else
	0, 0,
#endif
#if defined(STM32F030xC) || defined(STM32F091xC) || defined(STM32F098xx)
	RCC_APB1RSTR_USART5RST, RCC_APB2RSTR_USART6RST,
#else
	0, 0,
#endif
#if defined(STM32F091xC) || defined(STM32F098xx)
	RCC_APB2RSTR_USART7RST, RCC_APB2RSTR_USART8RST
#else
	0, 0
#endif
};

constexpr volatile uint32_t *rcc_addr_list[uart::UART_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR,
	&RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB2ENR
};

constexpr volatile uint32_t *reset_addr_list[uart::UART_END] =
{
	&RCC->APB2RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR,
	&RCC->APB1RSTR, &RCC->APB2RSTR, &RCC->APB2RSTR, &RCC->APB2RSTR
};

constexpr rcc_src_t rcc_src_list[uart::UART_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1,
	RCC_SRC_APB2, RCC_SRC_APB2, RCC_SRC_APB2
};

/* Get AF index by uart interface and port number:
af = uart2afr[_uart][gpio.port()] */
constexpr uint8_t uart2afr[][PORT_QTY] =
{
	{1, 0},
	{1, 0, 0, 0},
	{0, 4, 1, 0},
	{4, 0, 0}
};

constexpr GPIO_TypeDef *const gpio_list[PORT_QTY] =
{
	GPIOA, GPIOB, GPIOC,
#if defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F030xC) || \
	defined(STM32F051x8) || defined(STM32F058xx) || defined(STM32F070x6) || \
	defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
	defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
	GPIOD,
#else
	NULL,
#endif
#if defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
	defined(STM32F091xC) || defined(STM32F098xx)
	GPIOE,
#else
	NULL,
#endif
	GPIOF
};
};
