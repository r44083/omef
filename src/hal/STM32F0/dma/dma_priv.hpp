#pragma once

#include "dma.hpp"
#include "CMSIS/Device/STM32F0xx/Include/stm32f0xx.h"

namespace hal::dma_priv
{
constexpr DMA_Channel_TypeDef *const channel[dma::DMA_END][dma::CH_END] =
{
	{
		DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4,
		DMA1_Channel5,
#if defined(STM32F042x6) || defined(STM32F048xx) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
		DMA1_Channel6, DMA1_Channel7
#else
		NULL, NULL
#endif
	},
	{
#if defined(STM32F091xC) || defined(STM32F098xx)
		DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4,
		DMA2_Channel5
#else
		NULL, NULL, NULL, NULL, NULL
#endif
	}
};

constexpr IRQn_Type irqn[dma::DMA_END][dma::CH_END] =
{
	{
#if defined(STM32F091xC) || defined(STM32F098xx)
		DMA1_Ch1_IRQn, DMA1_Ch2_3_DMA2_Ch1_2_IRQn, DMA1_Ch2_3_DMA2_Ch1_2_IRQn,
#else
		DMA1_Channel1_IRQn, DMA1_Channel2_3_IRQn, DMA1_Channel2_3_IRQn,
#endif
#if defined(STM32F091xC) || defined(STM32F098xx)
		DMA1_Ch4_7_DMA2_Ch3_5_IRQn, DMA1_Ch4_7_DMA2_Ch3_5_IRQn,
#elif defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F030xC) || \
	defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070x6) || defined(STM32F070xB)
		DMA1_Channel4_5_IRQn, DMA1_Channel4_5_IRQn,
#elif defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx)
		DMA1_Channel4_5_6_7_IRQn, DMA1_Channel4_5_6_7_IRQn,
#endif
#if defined(STM32F091xC) || defined(STM32F098xx)
		DMA1_Ch4_7_DMA2_Ch3_5_IRQn, DMA1_Ch4_7_DMA2_Ch3_5_IRQn,
#elif defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx)
		DMA1_Channel4_5_6_7_IRQn, DMA1_Channel4_5_6_7_IRQn
#else
		static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0)
#endif
	},
	{
#if defined(STM32F091xC) || defined(STM32F098xx)
		DMA1_Ch2_3_DMA2_Ch1_2_IRQn, DMA1_Ch2_3_DMA2_Ch1_2_IRQn,
		DMA1_Ch4_7_DMA2_Ch3_5_IRQn, DMA1_Ch4_7_DMA2_Ch3_5_IRQn,
		DMA1_Ch4_7_DMA2_Ch3_5_IRQn
#else
		static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
		static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
		static_cast<IRQn_Type>(0)
#endif
	}
};
};