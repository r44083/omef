#include <stdint.h>
#include <stddef.h>
#include "common/assert.h"
#include "gpio.hpp"
#include "gpio_priv.hpp"

using namespace hal;

gpio::gpio(uint8_t port, uint8_t pin, enum mode mode, bool state):
	_port(port),
	_pin(pin),
	_mode(mode)
{
	ASSERT(_port < ports && gpio_priv::gpio[_port]);
	ASSERT(_pin < pins);
	
	RCC->APB2ENR |= gpio_priv::rcc_en[_port];
	
	gpio::mode(_mode, state);
}

gpio::~gpio()
{
	GPIO_TypeDef *gpio = gpio_priv::gpio[_port];
	volatile uint32_t &cr = _pin < 8 ? gpio->CRL : gpio->CRH;
	
	// No pull-up, no pull-down
	cr &= ~(GPIO_CRL_MODE0 << ((_pin % 8) * 4));
	// Analog mode
	cr &= ~(GPIO_CRL_CNF0 << ((_pin % 8) * 4));
}

void gpio::set(bool state) const
{
	ASSERT(_mode == mode::DO || _mode == mode::OD);
	
	gpio_priv::gpio[_port]->BSRR = 1 << (state ? _pin : _pin +
		GPIO_BSRR_BR0_Pos);
}

bool gpio::get() const
{
	ASSERT(_mode != mode::AN && _mode != mode::AF);
	
	return gpio_priv::gpio[_port]->IDR & (1 << _pin);
}

void gpio::toggle() const
{
	ASSERT(_mode == mode::DO || _mode == mode::OD);
	
	gpio_priv::gpio[_port]->ODR ^= 1 << _pin;
}

void gpio::mode(enum mode mode, bool state)
{
	_mode = mode;
	GPIO_TypeDef *gpio = gpio_priv::gpio[_port];
	volatile uint32_t &cr = _pin < 8 ? gpio->CRL : gpio->CRH;
	
	// No pull-up, no pull-down
	cr &= ~(GPIO_CRL_MODE0 << ((_pin % 8) * 4));
	// Analog mode
	cr &= ~(GPIO_CRL_CNF0 << ((_pin % 8) * 4));
	
	switch(_mode)
	{
		case mode::DO:
			// Digital output push-pull
			cr |= GPIO_CRL_MODE0 << ((_pin % 8) * 4);
			break;
		
		case mode::OD:
			// Digital output
			cr |= GPIO_CRL_MODE0 << ((_pin % 8) * 4);
			// Open drain
			cr |= GPIO_CRL_CNF0_0 << ((_pin % 8) * 4);
			break;
		
		case mode::DI:
			// Digital input
			cr |= GPIO_CRL_CNF0_1 << ((_pin % 8) * 4);
			if(state)
				gpio->ODR |= GPIO_ODR_ODR0 << _pin; // Pull-up
			else
				gpio->ODR &= ~(GPIO_ODR_ODR0 << _pin); // Pull-down
			break;
		
		case mode::AN:
			// Analog mode is already enabled
			break;
		
		case mode::AF:
			cr |= GPIO_CRL_MODE0 << ((_pin % 8) * 4);
			cr |= GPIO_CRL_CNF0_1 << ((_pin % 8) * 4);
			// Modification of AFR register should be done during periph init
			break;
	}
	
	// Setup default state
	if(_mode == mode::DO || _mode == mode::OD)
		gpio->BSRR =  1 << (state ? _pin : _pin + GPIO_BSRR_BR0_Pos);
}