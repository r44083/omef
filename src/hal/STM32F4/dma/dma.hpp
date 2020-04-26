#pragma once

#include <stdint.h>
#include <stdbool.h>

namespace hal { class dma; }
// For internal use only! (called from ISR)
extern "C" void dma_irq_hndlr(hal::dma *obj);

namespace hal
{
class dma
{
	public:
		enum dma_t
		{
			DMA_1,
			DMA_2,
			DMA_END
		};
		
		enum stream_t
		{
			STREAM_0,
			STREAM_1,
			STREAM_2,
			STREAM_3,
			STREAM_4,
			STREAM_5,
			STREAM_6,
			STREAM_7,
			STREAM_END
		};
		
		enum ch_t
		{
			CH_0,
			CH_1,
			CH_2,
			CH_3,
			CH_4,
			CH_5,
			CH_6,
			CH_7,
			CH_END
		};
		
		enum dir_t
		{
			DIR_PERIPH_TO_MEM,
			DIR_MEM_TO_PERIPH,
			DIR_MEM_TO_MEM
		};
		
		enum inc_size_t
		{
			INC_SIZE_8,
			INC_SIZE_16,
			INC_SIZE_32
		};
		
		enum event_t
		{
			EVENT_CMPLT,
			EVENT_HALF,
			EVENT_ERROR
		};
		
		typedef void (*cb_t)(dma *dma, event_t event, void *ctx);
		
		dma(dma_t dma, stream_t stream, ch_t ch, dir_t dir, inc_size_t inc_size);
		~dma();
		
		void src(void *src);
		void dst(void *dst);
		void size(uint16_t size);
		void dir(dir_t dir);
		dir_t dir() const { return _dir; }
		void inc_size(inc_size_t inc_size);
		inc_size_t inc_size() const { return _inc_size; }
		uint16_t transfered() const;
		uint16_t remain() const;
		
		void start_once(cb_t cb, void *ctx);
		void start_cyclic(cb_t cb, void *ctx);
		void stop();
		
		bool busy();
		
		dma_t get_dma() {return _dma;}
		ch_t get_stream() {return _stream;}
		ch_t get_ch() {return _ch;}
		
	private:
		dma_t _dma;
		stream_t _stream;
		ch_t _ch;
		dir_t _dir;
		inc_size_t _inc_size;
		uint32_t _src;
		uint32_t _dst;
		uint16_t _size;
		void *_ctx;
		cb_t _cb;
		friend void ::dma_irq_hndlr(dma *obj);
};
}
