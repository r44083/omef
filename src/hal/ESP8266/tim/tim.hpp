#pragma once

#include <stdint.h>

namespace hal { class tim; }
// For internal use only! (called from ISR)
extern "C" void tim_irq_hndlr(hal::tim *obj);

namespace hal
{
class tim
{
	public:
		enum tim_t
		{
			TIM_1
		};
		
		typedef void (*cb_t)(tim *tim, void *ctx);
		
		tim(tim_t tim);
		~tim();
		
		void cb(cb_t cb, void *ctx);
		
		void us(uint32_t us);
		uint32_t us() const { return _us; }
		
		void start(bool is_cyclic = false);
		void stop();
		
		bool is_expired() const;
	
	private:
		tim_t _tim;
		uint32_t _us;
		void *_ctx;
		cb_t _cb;
		static void calc_clk(tim_t tim, uint32_t us, uint8_t &div,
			uint32_t &load);
		friend void ::tim_irq_hndlr(tim *obj);
};
}