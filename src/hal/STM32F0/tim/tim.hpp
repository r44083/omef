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
			TIM_1,  // Advanced-control timer TIM1
			TIM_2,  // General-purpose timer TIM2
			TIM_3,  // General-purpose timer TIM3
			TIM_4,  // General-purpose timer TIM4
			TIM_5,  // General-purpose timer TIM5
			TIM_6,  // Basic timer TIM6
			TIM_7,  // Basic timer TIM7
			TIM_8,  // Advanced-control timer TIM8
			TIM_9,  // General-purpose timer TIM9
			TIM_10, // General-purpose timer TIM10
			TIM_11, // General-purpose timer TIM11
			TIM_12, // General-purpose timer TIM12
			TIM_13, // General-purpose timer TIM13
			TIM_14, // General-purpose timer TIM14
			TIM_15, // General-purpose timer TIM15
			TIM_16, // General-purpose timer TIM16
			TIM_17, // General-purpose timer TIM17
			TIM_END
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
		static void calc_clk(tim_t tim, uint32_t us, uint16_t &psc,
			uint16_t &arr);
		friend void ::tim_irq_hndlr(tim *obj);
};
}