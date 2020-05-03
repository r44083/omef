#pragma once

#include "rcc/rcc.hpp"
#include "spi.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"

namespace hal::spi_priv
{
constexpr SPI_TypeDef *const spi[spi::SPI_END] =
{
	SPI1,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	SPI2,
#else
	NULL,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	SPI3,
#else
	NULL,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || \
	defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || \
	defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	SPI4,
#else
	NULL,
#endif
#if defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F411xE) || \
	defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || \
	defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
	SPI5,
#else
	NULL,
#endif
#if defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
	SPI6
#else
	NULL
#endif
};

constexpr IRQn_Type irqn[spi::SPI_END] =
{
	SPI1_IRQn,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	SPI2_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	SPI3_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || \
	defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || \
	defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	SPI4_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F411xE) || \
	defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || \
	defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
	SPI5_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
	SPI6_IRQn
#else
	static_cast<IRQn_Type>(0)
#endif
};

constexpr uint32_t rcc_en[spi::SPI_END] =
{
	RCC_APB2ENR_SPI1EN,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB1ENR_SPI2EN,
#else
	0,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1ENR_SPI3EN,
#else
	0,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || \
	defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || \
	defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB2ENR_SPI4EN,
#else
	0,
#endif
#if defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F411xE) || \
	defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || \
	defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB2ENR_SPI5EN,
#else
	0,
#endif
#if defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB2ENR_SPI6EN
#else
	0
#endif
};

constexpr uint32_t rcc_rst[spi::SPI_END] =
{
	RCC_APB2RSTR_SPI1RST,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB1RSTR_SPI2RST,
#else
	0,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1RSTR_SPI3RST,
#else
	0,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || \
	defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || \
	defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB2RSTR_SPI4RST,
#else
	0,
#endif
#if defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F411xE) || \
	defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || \
	defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB2RSTR_SPI5RST,
#else
	0,
#endif
#if defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB2RSTR_SPI6RST
#else
	0
#endif
};

constexpr volatile uint32_t *rcc_en_reg[spi::SPI_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB2ENR, &RCC->APB2ENR,
	&RCC->APB2ENR
};

constexpr volatile uint32_t *rcc_rst_reg[spi::SPI_END] =
{
	&RCC->APB2RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB2RSTR,
	&RCC->APB2RSTR, &RCC->APB2RSTR
};

constexpr rcc_src_t rcc_src[spi::SPI_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB2, RCC_SRC_APB2,
	RCC_SRC_APB2
};

constexpr uint8_t af[spi::SPI_END] =
{
	0x05, 0x05, 0x06, 0x05, 0x05, 0x05
};
};