#pragma once

#include "rcc/rcc.hpp"
#include "spi.hpp"
#include "CMSIS/Device/STM32F0xx/Include/stm32f0xx.h"

namespace hal::spi_priv
{
constexpr SPI_TypeDef *const spi[spi::SPI_END] =
{
	SPI1,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
	defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
	SPI2,
#else
	NULL,
#endif
};

constexpr IRQn_Type irqn[spi::SPI_END] =
{
	SPI1_IRQn,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
	defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
	SPI2_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
};

constexpr uint32_t rcc_en[spi::SPI_END] =
{
	RCC_APB2ENR_SPI1EN,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
	defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
	RCC_APB1ENR_SPI2EN,
#else
	0,
#endif
};

constexpr uint32_t rcc_rst[spi::SPI_END] =
{
	RCC_APB2RSTR_SPI1RST,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
	defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
	RCC_APB1RSTR_SPI2RST,
#else
	0,
#endif
};

constexpr volatile uint32_t *rcc_en_reg[spi::SPI_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR
};

constexpr volatile uint32_t *rcc_rst_reg[spi::SPI_END] =
{
	&RCC->APB2RSTR, &RCC->APB1RSTR
};

constexpr rcc_src_t rcc_src[spi::SPI_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1
};

/* Get AF index by spi interface and port number:
af = spi2afr[_spi][gpio.port()] */
constexpr uint8_t spi2afr[][gpio::ports] =
{
	{0, 0, 0, 0, 1},
	{0, 0, 1, 1}
};
};