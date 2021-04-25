#include <stdint.h>
#include <stddef.h>
#include "common/assert.h"
#include "gpio.hpp"

using namespace hal;

gpio::gpio(uint8_t port, uint8_t pin, enum mode mode, bool state):
	_port(port),
	_pin(pin),
	_mode(mode)
{
	ASSERT(_port < ports);
	ASSERT(_pin < pins);
	
	set_mode(_mode, state);
}

gpio::~gpio()
{
}

void gpio::set(bool state) const
{
	ASSERT(_mode == mode::DO);
}

bool gpio::get() const
{
	ASSERT(_mode == mode::DI && _mode == mode::DO);
}

void gpio::toggle() const
{
	ASSERT(_mode == mode::DO);
}

void gpio::set_mode(enum mode mode, bool state)
{
}
