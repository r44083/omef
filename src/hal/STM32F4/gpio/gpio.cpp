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
	
	RCC->AHB1ENR |= gpio_priv::rcc[_port];
	
	gpio::mode(_mode, state);
}

gpio::~gpio()
{
	GPIO_TypeDef *gpio = gpio_priv::ports[_port];
	/* No pull-up, no pull-down */
	gpio->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (_pin * 2));
	/* Analog mode */
	gpio->MODER |= GPIO_MODER_MODE0 << (_pin * 2);
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
	
	/* Input mode */
	gpio->MODER &= ~(GPIO_MODER_MODE0 << (_pin * 2));
	
	/* No pull-up, no pull-down */
	gpio->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (_pin * 2));
	
	/* Set very high speed */
	gpio->OSPEEDR |= GPIO_OSPEEDR_OSPEED0 << (_pin * 2);
	
	switch(_mode)
	{
		case mode::DO:
			/* Digital output type */
			gpio->MODER |= GPIO_MODER_MODE0_0 << (_pin * 2);
			/* Push-pull type */
			gpio->OTYPER &= ~(GPIO_OTYPER_OT0 << _pin);
			break;
		
		case mode::OD:
			/* Digital output type */
			gpio->MODER |= GPIO_MODER_MODE0_0 << (_pin * 2);
			/* Open drain type */
			gpio->OTYPER |= GPIO_OTYPER_OT0 << _pin;
			break;
		
		case mode::DI:
			/* Pull-up or pull-down */
			if(state)
				gpio->PUPDR |= GPIO_PUPDR_PUPD0_0 << (_pin * 2);
			else
				gpio->PUPDR |= GPIO_PUPDR_PUPD0_1 << (_pin * 2);
			break;
		
		case mode::AN:
			/* Analog mode */
			gpio->MODER |= GPIO_MODER_MODE0 << (_pin * 2);
			break;
		
		case mode::AF:
			/* Alternate function mode */
			gpio->MODER |= GPIO_MODER_MODE0_1 << (_pin * 2);
			/* Modification of AFR register should be done during periph init */
			break;
	}
	
	/* Setup default state */
	if(_mode == mode::DO || _mode == mode::OD)
		gpio->BSRR =  1 << (state ? _pin : _pin + 16);
}
