#pragma once

#include "dma.hpp"
#include "CMSIS/Device/STM32F1xx/Include/stm32f1xx.h"

namespace hal::dma_priv
{
constexpr DMA_Channel_TypeDef *const channel[dma::DMA_END][dma::CH_END] =
{
	{
		DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4,
		DMA1_Channel5, DMA1_Channel6, DMA1_Channel7
	},
	{
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
		DMA2_Channel1, DMA2_Channel2, DMA2_Channel3,
#else
		NULL, NULL, NULL,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC) || defined(STM32F105xC) || defined(STM32F107xC)
		DMA2_Channel4, DMA2_Channel5,
#else
		NULL, NULL,
#endif
		// DMA2 doesn't have 6th and 7th channels
		NULL, NULL
	}
};

constexpr IRQn_Type irqn[dma::DMA_END][dma::CH_END] =
{
	{
		DMA1_Channel1_IRQn, DMA1_Channel2_IRQn, DMA1_Channel3_IRQn,
		DMA1_Channel4_IRQn, DMA1_Channel5_IRQn, DMA1_Channel6_IRQn,
		DMA1_Channel7_IRQn
	},
	{
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
		DMA2_Channel1_IRQn, DMA2_Channel2_IRQn, DMA2_Channel3_IRQn,
#else
		static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
		static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG)
		DMA2_Channel4_5_IRQn, DMA2_Channel4_5_IRQn,
#elif defined(STM32F105xC) || defined(STM32F107xC)
		DMA2_Channel4_IRQn, DMA2_Channel5_IRQn,
#else
		static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#endif
		// DMA2 doesn't have 6th and 7th channels
		static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0)
	}
};
};