#include <stddef.h>

#include "common/assert.h"
#include "di.hpp"

using namespace drv;
using namespace hal;

di::di(gpio &gpio, uint16_t threshold, bool default_state):
	_gpio(gpio),
	_threshold(threshold),
	_state(default_state),
	_cnt(0),
	_ctx(NULL),
	_cb(NULL)
{
	ASSERT(_gpio.mode() == gpio::mode::DI);
}

di::~di()
{
	
}

void di::poll()
{
	bool tmp_state = _state;
	
	jitter_filter();
	if((tmp_state != _state) && _cb)
		_cb(this, _state, _ctx);
}

void di::jitter_filter()
{
	if(_gpio.get())
	{
		if(_cnt < _threshold)
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
}
