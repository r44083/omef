#pragma once

#include <stdint.h>

/* List of ESP8266 pin functions according to this hal:
| DO,OD,DI |    AF1   |    AF2     |    AF3     |    AF4      |
| -------- | -------- | ---------- | ---------- | ----------- |
| GPIO0    | SPICS2   | -          | -          | CLK_OUT     |
| GPIO1    | U0TXD    | SPICS1     | -          | CLK_RTC_BK  |
| GPIO2    | I2SO_WS  | U1TXD_BK   | -          | U0TXD_BK    |
| GPIO3    | U0RXD    | I2SO_DATA  | -          | CLK_XTAL_BK |
| GPIO4    | CLK_XTAL | -          | -          | -           |
| GPIO5    | CLK_RTC  | -          | -          | -           |
| GPIO6*   | SDCLK    | SPICLK     | -          | U1CTS       |
| GPIO7*   | SDDATA0  | SPIQ_MISO  | -          | U1TXD       |
| GPIO8*   | SDDATA1  | SPID_MOSI  | -          | U1RXD       |
| GPIO9*   | SDDATA2  | SPIHD      | -          | HSPIHD      |
| GPIO10*  | SDDATA3  | SPIWP      | -          | HSPIWP      |
| GPIO11*  | SDCMD    | SPICS0     | -          | U1RTS       |
| GPIO12   | MTDI     | I2SI_DATA  | HSPIQ_MISO | U0DTR       |
| GPIO13   | MTCK     | I2SI_BCK   | HSPID_MOSI | U0CTS       |
| GPIO14   | MTMS     | I2SI_WS    | HSPI_CLK   | U0DSR       |
| GPIO15   | MTDO     | I2SO_BCK   | HSPI_CS    | U0RTS       |
| GPIO16   | XPD_DCDC | EXT_WAKEUP | DEEPSLEEP  | BT_XTAL_EN  |
| -------- | -------- | ---------- | ---------- | ----------- |
* - pins used for accessing external flash memory */

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