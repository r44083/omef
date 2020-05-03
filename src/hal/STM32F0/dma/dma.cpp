#include <stddef.h>
#include "common/assert.h"
#include "dma.hpp"
#include "dma_priv.hpp"
#include "CMSIS/Device/STM32F0xx/Include/stm32f0xx.h"
#include "CMSIS/Include/core_cm0.h"

using namespace hal;

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
	ASSERT(dma_priv::channel[dma][ch]);
	ASSERT(dir <= DIR_MEM_TO_MEM);
	// Only DMA2 is able to perform memory-to-memory transfers
	ASSERT(dir != DIR_MEM_TO_MEM || dma != DMA_1);
	ASSERT(inc_size <= INC_SIZE_32);
	
	obj_list[_dma][_ch] = this;
	
	if(_dma == DMA_1)
	{
		RCC->AHBENR |= RCC_AHBENR_DMAEN;
		DMA1->IFCR = DMA_IFCR_CGIF1 << (_ch * DMA_IFCR_CGIF2_Pos);
	}
#if defined(STM32F091xC) || defined(STM32F098xx)
	else
	{
		RCC->AHBENR |= RCC_AHBENR_DMA2EN;
		DMA2->IFCR = DMA_IFCR_CGIF1 << (_ch * DMA_IFCR_CGIF2_Pos);
	}
#endif
	
	DMA_Channel_TypeDef *channel = dma_priv::channel[_dma][_ch];
	
	// Setup data direction
	channel->CCR &= ~(DMA_CCR_DIR | DMA_CCR_MEM2MEM);
	if(_dir == DIR_MEM_TO_PERIPH)
		channel->CCR |= DMA_CCR_DIR;
	else if(_dir == DIR_MEM_TO_MEM)
		channel->CCR |= DMA_CCR_MEM2MEM;
	
	// Setup data size
	channel->CCR &= ~(DMA_CCR_MSIZE | DMA_CCR_PSIZE);
	if(_inc_size == INC_SIZE_16)
		channel->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
	else if(_inc_size == INC_SIZE_32)
		channel->CCR |= DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1;
	
	// Setup incremental mode
	channel->CCR |= DMA_CCR_MINC;
	if(_dir == DIR_MEM_TO_MEM)
		channel->CCR |= DMA_CCR_PINC;
	else
		channel->CCR &= ~DMA_CCR_PINC;
	
	channel->CCR |= DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE;
	
	NVIC_SetPriority(dma_priv::irqn[_dma][_ch], 3);
	NVIC_EnableIRQ(dma_priv::irqn[_dma][_ch]);
}

dma::~dma()
{
	NVIC_DisableIRQ(dma_priv::irqn[_dma][_ch]);
	dma_priv::channel[_dma][_ch]->CCR &= ~DMA_CCR_EN;
	obj_list[_dma][_ch] = NULL;
}

void dma::src(void *src)
{
	ASSERT(src);
	
	DMA_Channel_TypeDef *channel = dma_priv::channel[_dma][_ch];
	// This action allowed only when DMA is disabled
	ASSERT(!(channel->CCR & DMA_CCR_EN));
	
	_src = (uint32_t)src;
	if(_dir == DIR_MEM_TO_PERIPH)
		channel->CMAR = _src;
	else
		channel->CPAR = _src;
}

void dma::dst(void *dst)
{
	ASSERT(dst);
	
	DMA_Channel_TypeDef *channel = dma_priv::channel[_dma][_ch];
	// This action allowed only when DMA is disabled
	ASSERT(!(channel->CCR & DMA_CCR_EN));
	
	_dst = (uint32_t)dst;
	if(_dir == DIR_MEM_TO_PERIPH)
		channel->CPAR = _dst;
	else
		channel->CMAR = _dst;
}

void dma::size(uint16_t size)
{
	ASSERT(size > 0);
	
	DMA_Channel_TypeDef *channel = dma_priv::channel[_dma][_ch];
	// This action allowed only when DMA is disabled
	ASSERT(!(channel->CCR & DMA_CCR_EN));
	
	_size = size;
	channel->CNDTR = _size;
}

void dma::dir(dir_t dir)
{
	ASSERT(dir <= DIR_MEM_TO_MEM);
	// Only DMA2 is able to perform memory-to-memory transfers
	ASSERT(dir != DIR_MEM_TO_MEM || _dma != DMA_1);
	
	DMA_Channel_TypeDef *channel = dma_priv::channel[_dma][_ch];
	// This action allowed only when DMA is disabled
	ASSERT(!(channel->CCR & DMA_CCR_EN));
	
	_dir = dir;
	// Setup data direction
	channel->CCR &= ~(DMA_CCR_DIR | DMA_CCR_MEM2MEM);
	if(_dir == DIR_MEM_TO_PERIPH)
		channel->CCR |= DMA_CCR_DIR;
	else if(_dir == DIR_MEM_TO_MEM)
		channel->CCR |= DMA_CCR_MEM2MEM;
	
	// Setup incremental mode
	channel->CCR |= DMA_CCR_MINC;
	if(_dir == DIR_MEM_TO_MEM)
		channel->CCR |= DMA_CCR_PINC;
	else
		channel->CCR &= ~DMA_CCR_PINC;
}

void dma::inc_size(inc_size_t inc_size)
{
	ASSERT(inc_size <= INC_SIZE_32);
	
	DMA_Channel_TypeDef *channel = dma_priv::channel[_dma][_ch];
	// This action allowed only when DMA is disabled
	ASSERT(!(channel->CCR & DMA_CCR_EN));
	
	_inc_size = inc_size;
	// Setup data size
	channel->CCR &= ~(DMA_CCR_MSIZE | DMA_CCR_PSIZE);
	if(_inc_size == INC_SIZE_16)
		channel->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
	else if(_inc_size == INC_SIZE_32)
		channel->CCR |= DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1;
}

uint16_t dma::transfered() const
{
	return _size - dma_priv::channel[_dma][_ch]->CNDTR;
}

uint16_t dma::remain() const
{
	return dma_priv::channel[_dma][_ch]->CNDTR;
}

void dma::start_once(cb_t cb, void *ctx)
{
	ASSERT(_size > 0);
	ASSERT(_src);
	ASSERT(_dst);
	
	DMA_Channel_TypeDef *channel = dma_priv::channel[_dma][_ch];
	// This action allowed only when DMA is disabled
	ASSERT(!(channel->CCR & DMA_CCR_EN));
	
	_ctx = ctx;
	_cb = cb;
	
	// Disable circular mode
	channel->CCR &= ~DMA_CCR_CIRC;
	
	// Clear interrupt flag to prevent transfer complete interrupt
	if(_dma == DMA_1)
		DMA1->IFCR = DMA_IFCR_CTCIF1 << (_ch * DMA_IFCR_CGIF2_Pos);
#if defined(STM32F091xC) || defined(STM32F098xx)
	else
		DMA2->IFCR = DMA_IFCR_CTCIF1 << (_ch * DMA_IFCR_CGIF2_Pos);
#endif
	
	NVIC_EnableIRQ(dma_priv::irqn[_dma][_ch]);
	channel->CCR |= DMA_CCR_EN;
}

void dma::start_cyclic(cb_t cb, void *ctx)
{
	ASSERT(_size > 0);
	ASSERT(_src);
	ASSERT(_dst);
	
	DMA_Channel_TypeDef *channel = dma_priv::channel[_dma][_ch];
	// This action allowed only when DMA is disabled
	ASSERT(!(channel->CCR & DMA_CCR_EN));
	
	_ctx = ctx;
	_cb = cb;
	
	// Clear interrupt flag to prevent transfer complete interrupt
	if(_dma == DMA_1)
		DMA1->IFCR = DMA_IFCR_CTCIF1 << (_ch * DMA_IFCR_CGIF2_Pos);
#if defined(STM32F091xC) || defined(STM32F098xx)
	else
		DMA2->IFCR = DMA_IFCR_CTCIF1 << (_ch * DMA_IFCR_CGIF2_Pos);
#endif
	
	NVIC_EnableIRQ(dma_priv::irqn[_dma][_ch]);
	channel->CCR |= DMA_CCR_EN | DMA_CCR_CIRC;
}

void dma::stop()
{
	_cb = NULL;
	_ctx = NULL;
	
	NVIC_DisableIRQ(dma_priv::irqn[_dma][_ch]);
	
	DMA_Channel_TypeDef *channel = dma_priv::channel[_dma][_ch];
	channel->CCR &= ~DMA_CCR_EN;
	
	// Waiting for end of DMA transmission
	while(channel->CCR & DMA_CCR_EN);
}

bool dma::busy()
{
	return dma_priv::channel[_dma][_ch]->CCR & DMA_CCR_EN;
}

extern "C" void dma_irq_hndlr(hal::dma *obj)
{
	uint32_t isr;
	volatile uint32_t *iclr;
	if(obj->_dma == dma::DMA_1)
	{
		isr = DMA1->ISR;
		iclr = &DMA1->IFCR;
	}
#if defined(STM32F091xC) || defined(STM32F098xx)
	else
	{
		isr = DMA2->ISR;
		iclr = &DMA2->IFCR;
	}
#endif
	
	DMA_Channel_TypeDef *channel = dma_priv::channel[obj->_dma][obj->_ch];
	uint8_t isr_offset = obj->_ch * DMA_ISR_GIF2_Pos;
	
	if((channel->CCR & DMA_CCR_TCIE) && (isr & (DMA_ISR_TCIF1 << isr_offset)))
	{
		*iclr = DMA_IFCR_CTCIF1 << isr_offset;
		
		// Do not stop DMA because of it was started in circular mode
		if(!(channel->CCR & DMA_CCR_CIRC))
			channel->CCR &= ~DMA_CCR_EN;
		
		if(obj->_cb)
			obj->_cb(obj, dma::EVENT_CMPLT, obj->_ctx);
	}
	else if((channel->CCR & DMA_CCR_HTIE) &&
		(isr & (DMA_ISR_HTIF1 << isr_offset)))
	{
		*iclr = DMA_IFCR_CHTIF1 << isr_offset;
		if(obj->_cb)
			obj->_cb(obj, dma::EVENT_HALF, obj->_ctx);
	}
	else if((channel->CCR & DMA_CCR_TEIE) &&
		(isr & (DMA_ISR_TEIF1 << isr_offset)))
	{
		*iclr = DMA_IFCR_CTEIF1 << isr_offset;
		
		// Do not stop DMA because of it was started in circular mode
		if(!(channel->CCR & DMA_CCR_CIRC))
			channel->CCR &= ~DMA_CCR_EN;
		
		if(obj->_cb)
			obj->_cb(obj, dma::EVENT_ERROR, obj->_ctx);
	}
}

#if defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F030xC) || \
	defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx)
extern "C" void DMA1_Channel1_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_1]);
}
#elif defined(STM32F091xC) || defined(STM32F098xx)
extern "C" void DMA1_Ch1_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_1]);
}
#endif

#if defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F030xC) || \
	defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx)
extern "C" void DMA1_Channel2_3_IRQHandler(void)
{
	uint32_t isr = DMA1->ISR;
	
	if(isr & DMA_ISR_GIF2)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_2]);
	else if(isr & DMA_ISR_GIF3)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_3]);
}
#elif defined(STM32F091xC) || defined(STM32F098xx)
extern "C" void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler(void)
{
	uint32_t dma1_isr = DMA1->ISR;
	uint32_t dma2_isr = DMA2->ISR;
	
	if(dma1_isr & DMA_ISR_GIF2)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_2]);
	else if(dma1_isr & DMA_ISR_GIF3)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_3]);
	else if(dma2_isr & DMA_ISR_GIF1)
		dma_irq_hndlr(obj_list[dma::DMA_2][dma::CH_1]);
	else if(dma2_isr & DMA_ISR_GIF2)
		dma_irq_hndlr(obj_list[dma::DMA_2][dma::CH_2]);
}
#endif

#if defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F030xC) || \
	defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070x6) || defined(STM32F070xB)
extern "C" void DMA1_Channel4_5_IRQHandler(void)
{
	uint32_t isr = DMA1->ISR;
	
	if(isr & DMA_ISR_GIF4)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_4]);
	else if(isr & DMA_ISR_GIF5)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_5]);
}
#elif defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx)
extern "C" void DMA1_Channel4_5_6_7_IRQHandler(void)
{
	uint32_t isr = DMA1->ISR;
	
	if(isr & DMA_ISR_GIF4)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_4]);
	else if(isr & DMA_ISR_GIF5)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_5]);
	else if(isr & DMA_ISR_GIF6)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_6]);
	else if(isr & DMA_ISR_GIF7)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_7]);
}
#elif defined(STM32F091xC) || defined(STM32F098xx)
extern "C" void DMA1_Ch4_7_DMA2_Ch3_5_IRQHandler(void)
{
	uint32_t dma1_isr = DMA1->ISR;
	uint32_t dma2_isr = DMA2->ISR;
	
	if(dma1_isr & DMA_ISR_GIF4)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_4]);
	else if(dma1_isr & DMA_ISR_GIF5)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_5]);
	else if(dma1_isr & DMA_ISR_GIF6)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_6]);
	else if(dma1_isr & DMA_ISR_GIF7)
		dma_irq_hndlr(obj_list[dma::DMA_1][dma::CH_7]);
	else if(dma2_isr & DMA_ISR_GIF3)
		dma_irq_hndlr(obj_list[dma::DMA_2][dma::CH_3]);
	else if(dma2_isr & DMA_ISR_GIF4)
		dma_irq_hndlr(obj_list[dma::DMA_2][dma::CH_4]);
	else if(dma2_isr & DMA_ISR_GIF5)
		dma_irq_hndlr(obj_list[dma::DMA_2][dma::CH_5]);
}
#endif