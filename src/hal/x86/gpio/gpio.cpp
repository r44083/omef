#include <stdint.h>
#include <stddef.h>
#include "common/assert.h"
#include "gpio.hpp"
#include <iostream>

using namespace hal;

gpio::gpio(uint8_t port, uint8_t pin, enum mode mode, bool state):
	_port(port),
	_pin(pin),
	_mode(mode),
	_state(false)
{
	ASSERT(_port < ports);
	ASSERT(_pin < pins);
	
	gpio::mode(_mode, state);
}

gpio::~gpio()
{
}

void gpio::set(bool state) const
{
	ASSERT(_mode == mode::DO || _mode == mode::OD);
	
	_state = state;
	
	std::cout << "gpio " << +_port << "." << +_pin << " set to " << _state
		<< std::endl;
}

bool gpio::get() const
{
	ASSERT(_mode != mode::AN && _mode != mode::AF);
	
	return _state;
}

void gpio::toggle() const
{
	ASSERT(_mode == mode::DO || _mode == mode::OD);
	
	_state = !_state;
	std::cout << "gpio " << +_port << "." << +_pin << " toggle to " << _state
		<< std::endl;
}

void gpio::mode(enum mode mode, bool state)
{
	_mode = mode;
	
	std::string mode_name;
	switch(_mode)
	{
		case mode::DO: mode_name = "Digital output"; break;
		case mode::OD: mode_name = "Open drain"; break;
		case mode::DI: mode_name = "Digital input"; break;
		case mode::AN: mode_name = "Analog mode"; break;
		case mode::AF: mode_name = "Alternate function"; break;
	}
	
	std::cout << "gpio " << +_port << "." << +_pin << " mode: \""
		<< mode_name << "\" state: " << _state << std::endl;
}
