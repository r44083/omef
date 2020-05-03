#pragma once

#include <stdint.h>
#include "gpio/gpio.hpp"

namespace hal { class exti; }
// For internal use only! (called from ISR)
extern "C" void exti_irq_hndlr(hal::exti *obj);

namespace hal
{
class exti
{
	public:
		enum trigger_t
		{
			TRIGGER_RISING,
			TRIGGER_FALLING,
			TRIGGER_BOTH
		};
		
		typedef void (*cb_t)(exti *exti, void *ctx);
		
		exti(gpio &gpio, trigger_t trigger = TRIGGER_BOTH);
		~exti();
		
		void cb(cb_t cb, void *ctx);
		
		void on();
		void off();
		
		void trigger(trigger_t trigger);
		trigger_t trigger() const { return _trigger; }
		
		exti &operator = (const exti &);
	
	private:
		gpio &_gpio;
		trigger_t _trigger;
		void *_ctx;
		cb_t _cb;
		friend void ::exti_irq_hndlr(exti *obj);
};
}