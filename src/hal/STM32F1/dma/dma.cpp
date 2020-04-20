#include <stddef.h>

#include "common/assert.h"
#include "dma.hpp"
#include "CMSIS/Device/STM32F1xx/Include/stm32f1xx.h"
#include "CMSIS/Include/core_cm3.h"

using namespace hal;

#define IRQ_PRIORITY 2

static DMA_Channel_TypeDef *const ch_list[dma::DMA_END][dma::CH_END] =
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
		/* DMA2 doesn't have 6th and 7th channels */
		NULL, NULL
	}
};

static IRQn_Type const irq_list[dma::DMA_END][dma::CH_END] =
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
		/* DMA2 doesn't have 6th and 7th channels */
		static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0)
	}
};

static dma *obj_list[dma::DMA_END][dma::CH_END];

dma::dma(dma_t dma, ch_t ch, dir_t dir, inc_size_t inc_size):
	_dma(dma),
	_ch(ch),
	_dir(dir),
	_inc_size(inc_size),
	_src(0),
	_dst(0),
	_size(0),
	_ctx(NULL),
	_cb(NULL)
{
	ASSERT(dma < DMA_END);
	ASSERT(ch < CH_END);
	ASSERT(dir <= DIR_MEM_TO_MEM);
	/* Only DMA2 is able to perform memory-to-memory transfers */
	ASSERT(dir != DIR_MEM_TO_MEM || dma != DMA_1);
	ASSERT(inc_size <= INC_SIZE_32);
	
	obj_list[_dma][_ch] = this;
	
	if(_dma == DMA_1)
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	else
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
		RCC->AHBENR |= RCC_AHBENR_DMA2EN;
#else
		ASSERT(0);
#endif
	
	DMA_Channel_TypeDef *dma_ch = ch_list[_dma][_ch];
	dma_ch->CCR &= ~DMA_CCR_EN;
	
	if(_dma == DMA_1)
	{
		DMA1->IFCR = (DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 |
			DMA_IFCR_CTEIF1) << (_ch * 4);
	}
	else
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
	{
		DMA2->IFCR = (DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 |
			DMA_IFCR_CTEIF1) << (_ch * 4);
	}
#else
		ASSERT(0);
#endif
	
	/* Setup data direction. Default is peripheral to memory */
	dma_ch->CCR &= ~(DMA_CCR_DIR | DMA_CCR_MEM2MEM);
	if(_dir == DIR_MEM_TO_PERIPH)
		dma_ch->CCR |= DMA_CCR_DIR;
	else if(_dir == DIR_MEM_TO_MEM)
		dma_ch->CCR |= DMA_CCR_MEM2MEM;
	
	/* Setup data size. Default is 8 */
	dma_ch->CCR &= ~(DMA_CCR_MSIZE | DMA_CCR_PSIZE);
	if(_inc_size == INC_SIZE_16)
		dma_ch->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
	else if(_inc_size == INC_SIZE_32)
		dma_ch->CCR |= DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1;
	
	/* Setup incremental mode */
	dma_ch->CCR |= DMA_CCR_MINC;
	if(_dir == DIR_MEM_TO_MEM)
		dma_ch->CCR |= DMA_CCR_PINC;
	else
		dma_ch->CCR &= ~DMA_CCR_PINC;
	
	dma_ch->CCR |= DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE;
	
	NVIC_SetPriority(irq_list[_dma][_ch], IRQ_PRIORITY);
	NVIC_EnableIRQ(irq_list[_dma][_ch]);
}

dma::~dma()
{
	_cb = NULL;
	obj_list[_dma][_ch] = NULL;
	
	NVIC_DisableIRQ(irq_list[_dma][_ch]);
	ch_list[_dma][_ch]->CCR &= ~DMA_CCR_EN;
}

void dma::src(void *src)
{
	ASSERT(src);

	DMA_Channel_TypeDef *dma_ch = ch_list[_dma][_ch];
	/* This action allowed only when DMA is disabled */
	ASSERT(!(dma_ch->CCR & DMA_CCR_EN));
	
	_src = (uint32_t)src;
	if(_dir == DIR_MEM_TO_PERIPH)
		dma_ch->CMAR = _src;
	else
		dma_ch->CPAR = _src;
}

void dma::dst(void *dst)
{
	ASSERT(dst);
	
	DMA_Channel_TypeDef *dma_ch = ch_list[_dma][_ch];
	/* This action allowed only when DMA is disabled */
	ASSERT(!(dma_ch->CCR & DMA_CCR_EN));
	
	_dst = (uint32_t)dst;
	if(_dir == DIR_MEM_TO_PERIPH)
		dma_ch->CPAR = _dst;
	else
		dma_ch->CMAR = _dst;
}

void dma::size(uint16_t size)
{
	ASSERT(size > 0);
	
	DMA_Channel_TypeDef *dma_ch = ch_list[_dma][_ch];
	/* This action allowed only when DMA is disabled */
	ASSERT(!(dma_ch->CCR & DMA_CCR_EN));
	
	_size = size;
	dma_ch->CNDTR = _size;
}

void dma::dir(dir_t dir)
{
	ASSERT(dir <= DIR_MEM_TO_MEM);
	/* Only DMA2 is able to perform memory-to-memory transfers */
	ASSERT(dir != DIR_MEM_TO_MEM || _dma != DMA_1);
	
	DMA_Channel_TypeDef *dma_ch = ch_list[_dma][_ch];
	/* this action allowed only when DMA is disabled */
	ASSERT(!(dma_ch->CCR & DMA_CCR_EN));
	
	_dir = dir;
	/* Setup data direction. Default is peripheral to memory */
	dma_ch->CCR &= ~(DMA_CCR_DIR | DMA_CCR_MEM2MEM);
	if(_dir == DIR_MEM_TO_PERIPH)
		dma_ch->CCR |= DMA_CCR_DIR;
	else if(_dir == DIR_MEM_TO_MEM)
		dma_ch->CCR |= DMA_CCR_MEM2MEM;
	
	/* Setup incremental mode */
	dma_ch->CCR |= DMA_CCR_MINC;
	if(_dir == DIR_MEM_TO_MEM)
		dma_ch->CCR |= DMA_CCR_PINC;
	else
		dma_ch->CCR &= ~DMA_CCR_PINC;
}

void dma::inc_size(inc_size_t inc_size)
{
	ASSERT(inc_size <= INC_SIZE_32);
	
	DMA_Channel_TypeDef *dma_ch = ch_list[_dma][_ch];
	/* This action allowed only when DMA is disabled */
	ASSERT(!(dma_ch->CCR & DMA_CCR_EN));
	
	_inc_size = inc_size;
	/* Setup data size. Default is 8 */
	dma_ch->CCR &= ~(DMA_CCR_MSIZE | DMA_CCR_PSIZE);
	if(_inc_size == INC_SIZE_16)
		dma_ch->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
	else if(_inc_size == INC_SIZE_32)
		dma_ch->CCR |= DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1;
}

uint16_t dma::transfered() const
{
	DMA_Channel_TypeDef *dma_ch = ch_list[_dma][_ch];
	return _size - dma_ch->CNDTR;
}

uint16_t dma::remain() const
{
	DMA_Channel_TypeDef *dma_ch = ch_list[_dma][_ch];
	return (uint16_t)dma_ch->CNDTR;
}

void dma::start_once(cb_t cb, void *ctx)
{
	ASSERT(_size > 0);
	ASSERT(_src);
	ASSERT(_dst);
	
	DMA_Channel_TypeDef *dma_ch = ch_list[_dma][_ch];
	/* This action allowed only when DMA is disabled */
	ASSERT(!(dma_ch->CCR & DMA_CCR_EN));
	
	_ctx = ctx;
	_cb = cb;
	
	/* Disable circular mode */
	dma_ch->CCR &= ~DMA_CCR_CIRC;
	
	/* Clear interrupt flag to prevent transfer complete interrupt */
	if(_dma == DMA_1)
		DMA1->IFCR = DMA_IFCR_CTCIF1 << (_ch * 4);
	else
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
		DMA2->IFCR = DMA_IFCR_CTCIF1 << (_ch * 4);
#else
		ASSERT(0);
#endif
	
	NVIC_EnableIRQ(irq_list[_dma][_ch]);
	dma_ch->CCR |= DMA_CCR_EN;
}

void dma::start_cyclic(cb_t cb, void *ctx)
{
	ASSERT(_size > 0);
	ASSERT(_src);
	ASSERT(_dst);
	
	DMA_Channel_TypeDef *dma_ch = ch_list[_dma][_ch];
	/* This action allowed only when DMA is disabled */
	ASSERT(!(dma_ch->CCR & DMA_CCR_EN));
	
	_ctx = ctx;
	_cb = cb;
	
	/* Enable circular mode */
	dma_ch->CCR |= DMA_CCR_CIRC;
	
	/* Clear interrupt flag to prevent transfer complete interrupt */
	if(_dma == DMA_1)
		DMA1->IFCR = DMA_IFCR_CTCIF1 << (_ch * 4);
	else
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
		DMA2->IFCR = DMA_IFCR_CTCIF1 << (_ch * 4);
#else
		ASSERT(0);
#endif
	
	NVIC_EnableIRQ(irq_list[_dma][_ch]);
	dma_ch->CCR |= DMA_CCR_EN;
}

void dma::stop()
{
	_cb = NULL;
	_ctx = NULL;
	
	DMA_Channel_TypeDef *dma_ch = ch_list[_dma][_ch];
	
	NVIC_DisableIRQ(irq_list[_dma][_ch]);
	dma_ch->CCR &= ~DMA_CCR_EN;
	
	/* Waiting for end of DMA transmission */
	while(dma_ch->CCR & DMA_CCR_EN);
}

bool dma::busy()
{
	DMA_Channel_TypeDef *dma_ch = ch_list[_dma][_ch];
	return (bool)(dma_ch->CCR & DMA_CCR_EN);
}

extern "C" void dma_irq_hndlr(hal::dma *obj)
{
	DMA_Channel_TypeDef *dma_ch = ch_list[obj->_dma][obj->_ch];
	
	uint32_t isr;
	if(obj->_dma == dma::DMA_1)
		isr = DMA1->ISR;
	else
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
		isr = DMA2->ISR;
#else
		ASSERT(0);
#endif
	
	volatile uint32_t *isr_clr_reg;
	if(obj->_dma == dma::DMA_1)
		isr_clr_reg = &DMA1->IFCR;
	else
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
		isr_clr_reg = &DMA2->IFCR;
#else
		ASSERT(0);
#endif
	
	uint8_t shift = obj->_ch * 4;
	
	if((dma_ch->CCR & DMA_CCR_TCIE) && (isr & (DMA_ISR_TCIF1 << shift)))
	{
		*isr_clr_reg = DMA_IFCR_CTCIF1 << shift;
		
		// Do not stop DMA because of it was started in circular mode
		if(!(dma_ch->CCR & DMA_CCR_CIRC))
			dma_ch->CCR &= ~DMA_CCR_EN;
		
		if(obj->_cb)
			obj->_cb(obj, dma::EVENT_CMPLT, obj->_ctx);
	}
	else if((dma_ch->CCR & DMA_CCR_HTIE) && (isr & (DMA_ISR_HTIF1 << shift)))
	{
		*isr_clr_reg = DMA_IFCR_CHTIF1 << shift;
		if(obj->_cb)
			obj->_cb(obj, dma::EVENT_HALF, obj->_ctx);
	}
	else if((dma_ch->CCR & DMA_CCR_TEIE) && (isr & (DMA_ISR_TEIF1 << shift)))
	{
		*isr_clr_reg = DMA_IFCR_CTEIF1 << shift;
		
		// Do not stop DMA because of it was started in circular mode
		if(!(dma_ch->CCR & DMA_CCR_CIRC))
			dma_ch->CCR &= ~DMA_CCR_EN;
		
		if(obj->_cb)
			obj->_cb(obj, dma::EVENT_ERROR, obj->_ctx);
	}
}

extern "C" void DMA1_Channel1_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_1]);
}

extern "C" void DMA1_Channel2_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_2]);
}

extern "C" void DMA1_Channel3_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_3]);
}

extern "C" void DMA1_Channel4_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_4]);
}

extern "C" void DMA1_Channel5_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_5]);
}

extern "C" void DMA1_Channel6_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_6]);
}

extern "C" void DMA1_Channel7_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_7]);
}

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
	defined(STM32F107xC)
extern "C" void DMA2_Channel1_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::CH_1]);
}

extern "C" void DMA2_Channel2_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::CH_2]);
}

extern "C" void DMA2_Channel3_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::CH_3]);
}
#endif

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
	defined(STM32F103xE) || defined(STM32F103xG)
extern "C" void DMA2_Channel4_5_IRQHandler(void)
{
	uint32_t isr = DMA2->ISR;
	if(isr & DMA_ISR_GIF4)
		dma_irq_hndlr(obj_list[dma::DMA_2][dma::CH_4]);
	else if(isr & DMA_ISR_GIF5)
		dma_irq_hndlr(obj_list[dma::DMA_2][dma::CH_5]);
}
#elif defined(STM32F105xC) || defined(STM32F107xC)
extern "C" void DMA2_Channel4_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::CH_4]);
}

extern "C" void DMA2_Channel5_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::CH_5]);
}
#endif
