#pragma once

#include "dma.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"

namespace hal::dma_priv
{
constexpr DMA_Stream_TypeDef *const stream[dma::DMA_END][dma::STREAM_END] =
{
	{
		DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4,
		DMA1_Stream5, DMA1_Stream6, DMA1_Stream7
	},
	{
		DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4,
		DMA2_Stream5, DMA2_Stream6, DMA2_Stream7
	}
};

constexpr IRQn_Type irqn[dma::DMA_END][dma::STREAM_END] =
{
	{
		DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn,
		DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn,
		DMA1_Stream6_IRQn, DMA1_Stream7_IRQn
	},
	{
		DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn,
		DMA2_Stream3_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn,
		DMA2_Stream6_IRQn, DMA2_Stream7_IRQn
	}
};

constexpr volatile uint32_t *const isr[dma::DMA_END][dma::STREAM_END] =
{
	{
		&DMA1->LISR, &DMA1->LISR, &DMA1->LISR, &DMA1->LISR, &DMA1->HISR,
		&DMA1->HISR, &DMA1->HISR, &DMA1->HISR
	},
	{
		&DMA2->LISR, &DMA2->LISR, &DMA2->LISR, &DMA2->LISR, &DMA2->HISR,
		&DMA2->HISR, &DMA2->HISR, &DMA2->HISR
	}
};

constexpr volatile uint32_t *const iclr[dma::DMA_END][dma::STREAM_END] =
{
	{
		&DMA1->LIFCR, &DMA1->LIFCR, &DMA1->LIFCR, &DMA1->LIFCR, &DMA1->HIFCR,
		&DMA1->HIFCR, &DMA1->HIFCR, &DMA1->HIFCR
	},
	{
		&DMA2->LIFCR, &DMA2->LIFCR, &DMA2->LIFCR, &DMA2->LIFCR, &DMA2->HIFCR,
		&DMA2->HIFCR, &DMA2->HIFCR, &DMA2->HIFCR
	}
};

// DMA ISR registers offset for each stream
constexpr uint32_t isr_offset[dma::STREAM_END] =
{
	DMA_LISR_FEIF0_Pos, DMA_LISR_FEIF1_Pos, DMA_LISR_FEIF2_Pos,
	DMA_LISR_FEIF3_Pos, DMA_HISR_FEIF4_Pos, DMA_HISR_FEIF5_Pos,
	DMA_HISR_FEIF6_Pos, DMA_HISR_FEIF7_Pos
};
};