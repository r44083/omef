#include <stddef.h>

#include "common/assert.h"
#include "di.hpp"

using namespace drv;
using namespace hal;

di::di(gpio &gpio, size_t debounce_timeout, bool default_state):
	_gpio(gpio),
	_debounce_timeout(debounce_timeout),
	_state(default_state),
	_cnt(0)
{
	ASSERT(_gpio.mode() == gpio::mode::DI);
}

di::~di()
{
	
}

bool di::poll_event(bool &new_state)
{
	bool prev_state = _state;
	new_state = get_filtered_state();
	
	return prev_state != new_state;
}

bool di::get_filtered_state()
{
	if(_gpio.get())
	{
		if(_cnt < _debounce_timeout)
			_cnt++;
		else
			_state = true;
	}
	else
	{
		if(_cnt > 0)
			_cnt--;
		else
			_state = false;
	}
	
	return _state;
}