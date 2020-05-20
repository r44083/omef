#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "gpio/gpio.hpp"

namespace drv
{
class di
{
	public:
		di(hal::gpio &gpio, size_t debounce_timeout, bool default_state);
		~di();
		
		/**
		 * @brief Poll di driver for new event. Should be called with 1 ms
		 * period.
		 * 
		 * @param new_state New gpio state after debouncing.
		 * @return true  New event has happened, check new_state and handle it.
		 * @return false No event has happened.
		 */
		bool poll_event(bool &new_state);
		
		size_t debounce_timeout() const { return _debounce_timeout; }
		void debounce_timeout(size_t debounce_timeout)
		{ _debounce_timeout = debounce_timeout; }
		
	private:
		bool get_filtered_state();
		
		hal::gpio &_gpio;
		size_t _debounce_timeout;
		bool _state;
		size_t _cnt;
};
}
