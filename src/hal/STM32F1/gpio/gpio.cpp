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
	ASSERT(_port < ports && gpio_priv::ports[_port]);
	ASSERT(_pin < pins);
	
	RCC->APB2ENR |= gpio_priv::rcc[_port];
	
	gpio::mode(_mode, state);
}

gpio::~gpio()
{
	GPIO_TypeDef *gpio = gpio_priv::ports[_port];
	
	if(_pin < 8)
	{
		/* No pull-up, no pull-down */
		gpio->CRL &= ~(GPIO_CRL_MODE0 << (_pin * 4));
		/* Analog mode */
		gpio->CRL &= ~(GPIO_CRL_CNF0 << (_pin * 4));
	}
	else
	{
		/* No pull-up, no pull-down */
		gpio->CRH &= ~(GPIO_CRL_MODE0 << ((_pin - 8) * 4));
		/* Analog mode */
		gpio->CRH &= ~(GPIO_CRL_CNF0 << ((_pin - 8) * 4));
	}
}

void gpio::set(bool state) const
{
	ASSERT(_mode == mode::DO || _mode == mode::OD);
	
	gpio_priv::ports[_port]->BSRR = 1 << (state ? _pin : _pin + 16);
}

bool gpio::get() const
{
	ASSERT(_mode != mode::AN && _mode != mode::AF);
	
	return gpio_priv::ports[_port]->IDR & (1 << _pin);
}

void gpio::toggle() const
{
	ASSERT(_mode == mode::DO || _mode == mode::OD);
	
	gpio_priv::ports[_port]->ODR ^= 1 << _pin;
}

void gpio::mode(enum mode mode, bool state)
{
	_mode = mode;
	GPIO_TypeDef *gpio = gpio_priv::ports[_port];
	
	if(_pin < 8)
	{
		/* Input mode */
		gpio->CRL &= ~(GPIO_CRL_MODE0 << (_pin * 4));
		/* Analog mode */
		gpio->CRL &= ~(GPIO_CRL_CNF0 << (_pin * 4));
	}
	else
	{
		/* Input mode */
		gpio->CRH &= ~(GPIO_CRL_MODE0 << ((_pin - 8) * 4));
		/* Analog mode */
		gpio->CRH &= ~(GPIO_CRL_CNF0 << ((_pin - 8) * 4));
	}
	
	switch(_mode)
	{
		case mode::DO:
			if(_pin < 8)
				gpio->CRL |= GPIO_CRL_MODE0 << (_pin * 4);
			else
				gpio->CRH |= GPIO_CRL_MODE0 << ((_pin - 8) * 4);
			break;
		
		case mode::OD:
			if(_pin < 8)
			{
				gpio->CRL |= GPIO_CRL_CNF0_0 << (_pin * 4);
				gpio->CRL |= GPIO_CRL_MODE0 << (_pin * 4);
			}
			else
			{
				gpio->CRH |= GPIO_CRL_CNF0_0 << ((_pin - 8) * 4);
				gpio->CRH |= GPIO_CRL_MODE0 << ((_pin - 8) * 4);
			}
			break;
		
		case mode::DI:
			if(_pin < 8)
				gpio->CRL |= GPIO_CRL_CNF0_1 << (_pin * 4);
			else
				gpio->CRH |= GPIO_CRL_CNF0_1 << ((_pin - 8) * 4);
			
			if(state)
				/* Pull-up */
				gpio->ODR |= GPIO_ODR_ODR0 << _pin;
			else
				/* Pull-down */
				gpio->ODR &= ~(GPIO_ODR_ODR0 << _pin);
			break;
		
		case mode::AN:
			/* Analog mode has already enabled */
			break;
		
		case mode::AF:
			if(_pin < 8)
			{
				gpio->CRL |= GPIO_CRL_CNF0_1 << (_pin * 4);
				gpio->CRL |= GPIO_CRL_MODE0 << (_pin * 4);
			}
			else
			{
				gpio->CRH |= GPIO_CRL_CNF0_1 << ((_pin - 8) * 4);
				gpio->CRH |= GPIO_CRL_MODE0 << ((_pin - 8) * 4);
			}
			break;
	}
	
	/* Setup default state */
	if(_mode == mode::DO || _mode == mode::OD)
		gpio->BSRR =  1 << (state ? _pin : _pin + 16);
}
