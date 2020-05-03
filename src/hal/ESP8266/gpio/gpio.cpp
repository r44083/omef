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
	
	if(_pin != gpio_priv::rtc_pin)
	{
		if(state)
			GPIO.out_w1ts |= 1 << _pin;
		else
			GPIO.out_w1tc |= 1 << _pin;
	}
	else
	{
		if(state)
			SET_PERI_REG_MASK(RTC_GPIO_OUT, 1);
		else
			CLEAR_PERI_REG_MASK(RTC_GPIO_OUT, 1);
	}
}

bool gpio::get() const
{
	ASSERT(_mode == mode::DI && _mode == mode::DO);
	
	if(_pin != gpio_priv::rtc_pin)
		return GPIO.in & (1 << _pin);
	else
		return READ_PERI_REG(RTC_GPIO_IN_DATA) & 1;
}

void gpio::toggle() const
{
	ASSERT(_mode == mode::DO);
	
	if(_pin != gpio_priv::rtc_pin)
	{
		if(GPIO.in & (1 << _pin))
			GPIO.out_w1tc |= 1 << _pin;
		else
			GPIO.out_w1ts |= 1 << _pin;
	}
	else
	{
		if(READ_PERI_REG(RTC_GPIO_IN_DATA) & 1)
			CLEAR_PERI_REG_MASK(RTC_GPIO_OUT, 1);
		else
			SET_PERI_REG_MASK(RTC_GPIO_OUT, 1);
	}
}

void gpio::set_mode(enum mode mode, bool state)
{
	if(_pin == gpio_priv::rtc_pin)
	{
		ASSERT(mode != gpio::mode::DI || !state); // GPIO16 has only pulldown input mode
		ASSERT(mode != gpio::mode::OD); // GPIO16 doesn't have open drain mode
	}
	
	_mode = mode;
	switch(_mode)
	{
		case gpio::mode::DO:
			if(_pin != gpio_priv::rtc_pin)
			{
				PIN_FUNC_SELECT(gpio_priv::mux[_pin], gpio_priv::func[_pin][0]);
				GPIO.enable_w1ts |= 1 << _pin;
				GPIO.pin[_pin].driver = 0;
				set(state);
			}
			else
			{
				// mux configuration for XPD_DCDC to output rtc_gpio0
				uint32_t tmp_val = READ_PERI_REG(gpio_priv::mux[_pin]) &
					0xffffffbc;
				WRITE_PERI_REG(gpio_priv::mux[_pin], tmp_val | 1);
				
				CLEAR_PERI_REG_MASK(RTC_GPIO_CONF, 1);
				SET_PERI_REG_MASK(RTC_GPIO_ENABLE, 1);
				set(state);
			}
			break;
		
		case gpio::mode::OD:
			PIN_FUNC_SELECT(gpio_priv::mux[_pin], gpio_priv::func[_pin][0]);
			GPIO.enable_w1ts |= 1 << _pin;
			GPIO.pin[_pin].driver = 1;
			set(state);
			break;
		
		case gpio::mode::DI:
			if(_pin != gpio_priv::rtc_pin)
			{
				PIN_FUNC_SELECT(gpio_priv::mux[_pin], gpio_priv::func[_pin][0]);
				GPIO.enable_w1tc |= 1 << _pin;
				GPIO.pin[_pin].driver = 0;
				
				// GPIO 0-15 has pullup/floating input modes only
				if(state)
					PIN_PULLUP_EN(gpio_priv::mux[_pin]);
				else
					PIN_PULLUP_DIS(gpio_priv::mux[_pin]);
			}
			else
			{
				// mux configuration for XPD_DCDC and rtc_gpio0 connection
				uint32_t tmp_val = READ_PERI_REG(gpio_priv::mux[_pin]) &
					0xffffffbc;
				WRITE_PERI_REG(gpio_priv::mux[_pin], tmp_val | 1);
				
				CLEAR_PERI_REG_MASK(RTC_GPIO_CONF, 1);
				CLEAR_PERI_REG_MASK(RTC_GPIO_ENABLE, 1);
				
				// GPIO 16 has pulldown/floating input modes only
				gpio_pin_reg_t pin_reg = {.val =
					READ_PERI_REG(gpio_priv::mux[_pin])};
				pin_reg.rtc_pin.pulldown = !state;
				WRITE_PERI_REG(gpio_priv::mux[_pin], pin_reg.val);
			}
			break;
		
		case gpio::mode::AF1:
		case gpio::mode::AF2:
		case gpio::mode::AF3:
		case gpio::mode::AF4:
			if(_pin == gpio_priv::rtc_pin)
			{
				// Not implemented yet
				ASSERT(0);
			}
			else
			{
				/* Normalize _mode (- gpio::mode::DI) to access proper row from
				func list */
				PIN_FUNC_SELECT(gpio_priv::mux[_pin], gpio_priv::func[_pin]
					[static_cast<uint8_t>(_mode) - static_cast<uint8_t>
					(gpio::mode::DI)]);
			}
			break;
	}
}