#pragma once

#include <stdint.h>

namespace hal
{
class gpio
{
	public:
		enum class mode
		{
			DO,  /**< Digital output */
			OD,  /**< Open drain */
			DI,  /**< Digital input */
			AF1, /**< Alternate function 1 */
			AF2, /**< Alternate function 2 */
			AF3, /**< Alternate function 3 */
			AF4  /**< Alternate function 4 */
		};
		static constexpr auto ports = 1;
		static constexpr auto pins = 17;
		
		gpio(uint8_t port, uint8_t pin, mode mode, bool state = false);
		~gpio();
		
		void set(bool state) const;
		bool get() const;
		void toggle() const;
		void set_mode(mode mode, bool state = false);
		
		enum mode get_mode() const { return _mode; }
		uint8_t port() const { return _port; }
		uint8_t pin() const { return _pin; }
	
	private:
		uint8_t _port;
		uint8_t _pin;
		enum mode _mode;
};
}
