#pragma once

#include "rcc/rcc.hpp"
#include "spi.hpp"
#include "CMSIS/Device/STM32F1xx/Include/stm32f1xx.h"

namespace hal::spi_priv
{
constexpr SPI_TypeDef *const spi[spi::SPI_END] =
{
	SPI1,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	SPI2,
#else
	NULL,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	SPI3
#else
	NULL
#endif
};

constexpr IRQn_Type irqn[spi::SPI_END] =
{
	SPI1_IRQn,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	SPI2_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	SPI3_IRQn
#else
	static_cast<IRQn_Type>(0)
#endif
};

constexpr uint32_t rcc_en[spi::SPI_END] =
{
	RCC_APB2ENR_SPI1EN,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	RCC_APB1ENR_SPI2EN,
#else
	0,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	RCC_APB1ENR_SPI3EN
#else
	0
#endif
};

constexpr uint32_t rcc_rst[spi::SPI_END] =
{
	RCC_APB2RSTR_SPI1RST,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	RCC_APB1RSTR_SPI2RST,
#else
	0,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	RCC_APB1RSTR_SPI3RST
#else
	0
#endif
};

constexpr volatile uint32_t *rcc_en_reg[spi::SPI_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR
};

constexpr volatile uint32_t *rcc_rst_reg[spi::SPI_END] =
{
	&RCC->APB2RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR
};

constexpr rcc_src_t rcc_src[spi::SPI_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1
};
};