#include "common/assert.h"
#include "gpio.hpp"
#include "nRF5_SDK/modules/nrfx/mdk/nrf.h"
#include "nRF5_SDK/modules/nrfx/mdk/nrf_peripherals.h"

using namespace hal;

gpio::gpio(uint8_t port, uint8_t pin, enum mode mode, bool state):
	_port(port),
	_pin(pin),
	_mode(mode)
{
	ASSERT(_port < GPIO_COUNT);
#if GPIO_COUNT > 1 // NRF52833_XXAA, NRF52840_XXAA
	ASSERT(_port != 1 || _pin < P1_PIN_NUM);
#endif
	
	gpio::mode(_mode, state);
}

gpio::~gpio()
{
	NRF_GPIO_Type *gpio;
#if GPIO_COUNT > 1 // NRF52833_XXAA, NRF52840_XXAA
	if(_port == 1)
		gpio = NRF_P1;
	else
#endif
		gpio = NRF_P0;
	
	// Set default config
	gpio->PIN_CNF[_pin] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
		(GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
		(GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
		(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
		(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

void gpio::set(bool state) const
{
	ASSERT(_mode == mode::DO || _mode == mode::OD);
	
	NRF_GPIO_Type *gpio;
#if GPIO_COUNT > 1 // NRF52833_XXAA, NRF52840_XXAA
	if(_port == 1)
		gpio = NRF_P1;
	else
#endif
		gpio = NRF_P0;
	
	if(state)
		gpio->OUTSET = 1 << _pin;
	else
		gpio->OUTCLR = 1 << _pin;
}

bool gpio::get() const
{
	ASSERT(_mode != mode::AN);
	
	NRF_GPIO_Type *gpio;
#if GPIO_COUNT > 1 // NRF52833_XXAA, NRF52840_XXAA
	if(_port == 1)
		gpio = NRF_P1;
	else
#endif
		gpio = NRF_P0;
	
	if(_mode == mode::DI || _mode == mode::OD)
		return gpio->IN & (1 << _pin);
	else
		return gpio->OUT & (1 << _pin);
}

void gpio::toggle() const
{
	ASSERT(_mode == mode::DO || _mode == mode::OD);
	
	NRF_GPIO_Type *gpio;
#if GPIO_COUNT > 1 // NRF52833_XXAA, NRF52840_XXAA
	if(_port == 1)
		gpio = NRF_P1;
	else
#endif
		gpio = NRF_P0;
	
	if(gpio->OUT & (1 << _pin))
		gpio->OUTCLR = 1 << _pin;
	else
		gpio->OUTSET = 1 << _pin;
}

void gpio::mode(enum mode mode, bool state)
{
	_mode = mode;
	
	NRF_GPIO_Type *gpio;
#if GPIO_COUNT > 1 // NRF52833_XXAA, NRF52840_XXAA
	if(_port == 1)
		gpio = NRF_P1;
	else
#endif
		gpio = NRF_P0;
	
	// Set default config
	gpio->PIN_CNF[_pin] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
		(GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
		(GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
		(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
		(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
	
	switch(_mode)
	{
		case mode::DO:
			gpio->PIN_CNF[_pin] |=
				(GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
			gpio->DIRSET = 1 << _pin;
			break;
		
		case mode::OD:
			// TODO
			gpio->DIRSET = 1 << _pin;
			break;
		
		case mode::DI:
			gpio->DIRCLR = 1 << _pin;
			if(state)
			{
				gpio->PIN_CNF[_pin] |=
					(GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
			}
			else
			{
				gpio->PIN_CNF[_pin] |=
					(GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);
			}
			break;
		
		case mode::AN:
			// TODO
			gpio->DIRCLR = 1 << _pin;
			break;
	}
	
	// Setup default state
	if(_mode == mode::DO || _mode == mode::OD)
	{
		if(state)
			gpio->OUTSET = 1 << _pin;
		else
			gpio->OUTCLR = 1 << _pin;
	}
}