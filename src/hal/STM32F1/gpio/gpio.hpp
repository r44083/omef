#pragma once

#include <stdint.h>

namespace hal
{
class gpio
{
	public:
		enum class mode
		{
			DO, /**< Digital output */
			OD, /**< Open drain */
			DI, /**< Digital input */
			AN, /**< Analog mode */
			AF  /**< Alternate function */
		};
		static constexpr auto ports = 7; // GPIOG
		static constexpr auto pins = 16;
		
		gpio(uint8_t port, uint8_t pin, mode mode, bool state = false);
		~gpio();
		
		void set(bool state) const;
		bool get() const;
		void toggle() const;
		void mode(mode mode, bool state = false);
		
		enum mode mode() const { return _mode; }
		uint8_t port() const { return _port; }
		uint8_t pin() const { return _pin; }
	
	private:
		uint8_t _port;
		uint8_t _pin;
		enum mode _mode;
};
}