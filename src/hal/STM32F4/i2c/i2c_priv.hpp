#pragma once

#include "i2c.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"

namespace hal::i2c_priv
{
constexpr I2C_TypeDef *const i2c[i2c::I2C_END] =
{
	I2C1, I2C2,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	I2C3
#else
	NULL
#endif
};

constexpr IRQn_Type irqn_event[i2c::I2C_END] =
{
	I2C1_EV_IRQn, I2C2_EV_IRQn,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	I2C3_EV_IRQn
#else
	static_cast<IRQn_Type>(0)
#endif
};

constexpr IRQn_Type irqn_error[i2c::I2C_END] =
{
	I2C1_ER_IRQn, I2C2_ER_IRQn,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	I2C3_ER_IRQn
#else
	static_cast<IRQn_Type>(0)
#endif
};

constexpr uint32_t rcc_en[i2c::I2C_END] =
{
	RCC_APB1ENR_I2C1EN, RCC_APB1ENR_I2C2EN,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1ENR_I2C3EN
#else
	0
#endif
};

constexpr uint32_t rcc_rst[i2c::I2C_END] =
{
	RCC_APB1RSTR_I2C1RST, RCC_APB1RSTR_I2C2RST,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1RSTR_I2C3RST
#else
	0
#endif
};

constexpr uint8_t af[i2c::I2C_END] =
{
	0x04,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
	0x09, 0x09
#else
	0x04, 0x04
#endif
};
};