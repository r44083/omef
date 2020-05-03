#include <stddef.h>
#include "common/assert.h"
#include "dma.hpp"
#include "dma_priv.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"
#include "CMSIS/Include/core_cm4.h"

using namespace hal;

static dma *obj_list[dma::DMA_END][dma::STREAM_END];

dma::dma(dma_t dma, stream_t str, ch_t ch, dir_t dir, inc_size_t inc_size):
	_dma(dma),
	_str(str),
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
	ASSERT(str < STREAM_END);
	ASSERT(ch < CH_END);
	ASSERT(dir <= DIR_MEM_TO_MEM);
	// Only DMA2 is able to perform memory-to-memory transfers
	ASSERT(dir != DIR_MEM_TO_MEM || dma != DMA_1);
	ASSERT(inc_size <= INC_SIZE_32);
	
	obj_list[_dma][_str] = this;
	
	RCC->AHB1ENR |= (_dma == DMA_1) ? RCC_AHB1ENR_DMA1EN : RCC_AHB1ENR_DMA2EN;
	
	DMA_Stream_TypeDef *stream = dma_priv::stream[_dma][_str];
	
	*dma_priv::iclr[_dma][_str] = (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 |
		DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0) <<
		dma_priv::isr_offset[_str];
	
	stream->CR &= ~DMA_SxCR_CHSEL;
	stream->CR |= _ch << DMA_SxCR_CHSEL_Pos;
	
	// Setup data direction
	stream->CR &= ~DMA_SxCR_DIR;
	if(_dir == DIR_MEM_TO_PERIPH)
		stream->CR |= DMA_SxCR_DIR_0;
	else if(_dir == DIR_MEM_TO_MEM)
		stream->CR |= DMA_SxCR_DIR_1;
	
	// Setup data size
	stream->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);
	if(_inc_size == INC_SIZE_16)
		stream->CR |= DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0;
	else if(_inc_size == INC_SIZE_32)
		stream->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;
	
	// Setup incremental mode
	stream->CR |= DMA_SxCR_MINC;
	if(_dir == DIR_MEM_TO_MEM)
		stream->CR |= DMA_SxCR_PINC;
	else
		stream->CR &= ~DMA_SxCR_PINC;
	
	stream->CR |= DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE |
		DMA_SxCR_DMEIE;
	
	NVIC_SetPriority(dma_priv::irqn[_dma][_str], 3);
	NVIC_EnableIRQ(dma_priv::irqn[_dma][_str]);
}

dma::~dma()
{
	NVIC_DisableIRQ(dma_priv::irqn[_dma][_str]);
	dma_priv::stream[_dma][_str]->CR &= ~DMA_SxCR_EN;
	obj_list[_dma][_str] = NULL;
}

void dma::src(void *src)
{
	ASSERT(src);
	
	DMA_Stream_TypeDef *stream = dma_priv::stream[_dma][_str];
	// This action allowed only when DMA is disabled
	ASSERT(!(stream->CR & DMA_SxCR_EN));
	
	_src = (uint32_t)src;
	if(_dir == DIR_MEM_TO_PERIPH)
		stream->M0AR = _src;
	else
		stream->PAR = _src;
}

void dma::dst(void *dst)
{
	ASSERT(dst);
	
	DMA_Stream_TypeDef *stream = dma_priv::stream[_dma][_str];
	// This action allowed only when DMA is disabled
	ASSERT(!(stream->CR & DMA_SxCR_EN));
	
	_dst = (uint32_t)dst;
	if(_dir == DIR_MEM_TO_PERIPH)
		stream->PAR = _dst;
	else
		stream->M0AR = _dst;
}

void dma::size(uint16_t size)
{
	ASSERT(size > 0);
	
	DMA_Stream_TypeDef *stream = dma_priv::stream[_dma][_str];
	// This action allowed only when DMA is disabled
	ASSERT(!(stream->CR & DMA_SxCR_EN));
	
	_size = size;
	stream->NDTR = _size;
}

void dma::dir(dir_t dir)
{
	ASSERT(dir <= DIR_MEM_TO_MEM);
	// Only DMA2 is able to perform memory-to-memory transfers
	ASSERT(dir != DIR_MEM_TO_MEM || _dma != DMA_1);
	
	DMA_Stream_TypeDef *stream = dma_priv::stream[_dma][_str];
	// This action allowed only when DMA is disabled
	ASSERT(!(stream->CR & DMA_SxCR_EN));
	
	_dir = dir;
	// Setup data direction
	stream->CR &= ~DMA_SxCR_DIR;
	if(_dir == DIR_MEM_TO_PERIPH)
		stream->CR |= DMA_SxCR_DIR_0;
	else if(_dir == DIR_MEM_TO_MEM)
		stream->CR |= DMA_SxCR_DIR_1;
	
	// Setup incremental mode
	stream->CR |= DMA_SxCR_MINC;
	if(_dir == DIR_MEM_TO_MEM)
		stream->CR |= DMA_SxCR_PINC;
	else
		stream->CR &= ~DMA_SxCR_PINC;
}

void dma::inc_size(inc_size_t inc_size)
{
	ASSERT(inc_size <= INC_SIZE_32);
	
	DMA_Stream_TypeDef *stream = dma_priv::stream[_dma][_str];
	// This action allowed only when DMA is disabled
	ASSERT(!(stream->CR & DMA_SxCR_EN));
	
	_inc_size = inc_size;
	// Setup data size
	stream->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);
	if(_inc_size == INC_SIZE_16)
		stream->CR |= DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0;
	else if(_inc_size == INC_SIZE_32)
		stream->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;
}

uint16_t dma::transfered() const
{
	return _size - dma_priv::stream[_dma][_str]->NDTR;
}

uint16_t dma::remain() const
{
	return dma_priv::stream[_dma][_str]->NDTR;
}

void dma::start_once(cb_t cb, void *ctx)
{
	ASSERT(_size > 0);
	ASSERT(_src);
	ASSERT(_dst);
	
	DMA_Stream_TypeDef *stream = dma_priv::stream[_dma][_str];
	// This action allowed only when DMA is disabled
	ASSERT(!(stream->CR & DMA_SxCR_EN));
	
	_ctx = ctx;
	_cb = cb;
	
	// Disable circular mode
	stream->CR &= ~DMA_SxCR_CIRC;
	
	// Clear interrupt flag to prevent transfer complete interrupt
	*dma_priv::iclr[_dma][_str] = DMA_LIFCR_CTCIF0 <<
		dma_priv::isr_offset[_str];
	
	NVIC_EnableIRQ(dma_priv::irqn[_dma][_str]);
	stream->CR |= DMA_SxCR_EN;
}

void dma::start_cyclic(cb_t cb, void *ctx)
{
	ASSERT(_size > 0);
	ASSERT(_src);
	ASSERT(_dst);
	
	DMA_Stream_TypeDef *stream = dma_priv::stream[_dma][_str];
	// This action allowed only when DMA is disabled
	ASSERT(!(stream->CR & DMA_SxCR_EN));
	
	_ctx = ctx;
	_cb = cb;
	
	// Clear interrupt flag to prevent transfer complete interrupt
	*dma_priv::iclr[_dma][_str] = DMA_LIFCR_CTCIF0 <<
		dma_priv::isr_offset[_str];
	
	NVIC_EnableIRQ(dma_priv::irqn[_dma][_str]);
	stream->CR |= DMA_SxCR_EN | DMA_SxCR_CIRC;
}

void dma::stop()
{
	_cb = NULL;
	_ctx = NULL;
	
	NVIC_DisableIRQ(dma_priv::irqn[_dma][_str]);
	
	DMA_Stream_TypeDef *stream = dma_priv::stream[_dma][_str];
	stream->CR &= ~DMA_SxCR_EN;
	
	// Waiting for end of DMA transmission
	while(stream->CR & DMA_SxCR_EN);
}

bool dma::busy()
{
	return dma_priv::stream[_dma][_str]->CR & DMA_SxCR_EN;
}

extern "C" void dma_irq_hndlr(hal::dma *obj)
{
	DMA_Stream_TypeDef *stream = dma_priv::stream[obj->_dma][obj->_str];
	uint32_t isr = *dma_priv::isr[obj->_dma][obj->_str];
	volatile uint32_t *iclr = dma_priv::iclr[obj->_dma][obj->_str];
	uint8_t isr_offset = dma_priv::isr_offset[obj->_str];
	
	if((stream->CR & DMA_SxCR_TCIE) && (isr & (DMA_LISR_TCIF0 << isr_offset)))
	{
		*iclr = DMA_LIFCR_CTCIF0 << isr_offset;
		
		// Do not stop DMA because of it was started in circular mode
		if(!(stream->CR & DMA_SxCR_CIRC))
			stream->CR &= ~DMA_SxCR_EN;
		
		if(obj->_cb)
			obj->_cb(obj, dma::EVENT_CMPLT, obj->_ctx);
	}
	else if((stream->CR & DMA_SxCR_HTIE) &&
		(isr & (DMA_LISR_HTIF0 << isr_offset)))
	{
		*iclr = DMA_LIFCR_CHTIF0 << isr_offset;
		if(obj->_cb)
			obj->_cb(obj, dma::EVENT_HALF, obj->_ctx);
	}
	else if((stream->CR & DMA_SxCR_TEIE) &&
		(isr & (DMA_LISR_TEIF0 << isr_offset)))
	{
		*iclr = DMA_LIFCR_CTEIF0 << isr_offset;
		
		// Do not stop DMA because of it was started in circular mode
		if(!(stream->CR & DMA_SxCR_CIRC))
			stream->CR &= ~DMA_SxCR_EN;
		
		if(obj->_cb)
			obj->_cb(obj, dma::EVENT_ERROR, obj->_ctx);
	}
}

extern "C" void DMA1_Stream0_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::STREAM_0]);
}

extern "C" void DMA1_Stream1_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::STREAM_1]);
}

extern "C" void DMA1_Stream2_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::STREAM_2]);
}

extern "C" void DMA1_Stream3_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::STREAM_3]);
}

extern "C" void DMA1_Stream4_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::STREAM_4]);
}

extern "C" void DMA1_Stream5_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::STREAM_5]);
}

extern "C" void DMA1_Stream6_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::STREAM_6]);
}

extern "C" void DMA1_Stream7_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_1][dma::STREAM_7]);
}

extern "C" void DMA2_Stream0_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::STREAM_0]);
}

extern "C" void DMA2_Stream1_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::STREAM_1]);
}

extern "C" void DMA2_Stream2_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::STREAM_2]);
}

extern "C" void DMA2_Stream3_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::STREAM_3]);
}

extern "C" void DMA2_Stream4_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::STREAM_4]);
}

extern "C" void DMA2_Stream5_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::STREAM_5]);
}

extern "C" void DMA2_Stream6_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::STREAM_6]);
}

extern "C" void DMA2_Stream7_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[dma::DMA_2][dma::STREAM_7]);
}