#pragma once

#include "gpio.hpp"
#include "ESP8266_RTOS_SDK/components/esp8266/include/esp8266/pin_mux_register.h"
#include "ESP8266_RTOS_SDK/components/esp8266/include/esp8266/gpio_register.h"
#include "ESP8266_RTOS_SDK/components/esp8266/include/esp8266/gpio_struct.h"

namespace hal
{
namespace gpio_priv
{
constexpr auto rtc_pin = 16;

constexpr uint32_t mux[gpio::pins] =
{
	PERIPHS_IO_MUX_GPIO0_U, PERIPHS_IO_MUX_U0TXD_U, PERIPHS_IO_MUX_GPIO2_U,
	PERIPHS_IO_MUX_U0RXD_U, PERIPHS_IO_MUX_GPIO4_U, PERIPHS_IO_MUX_GPIO5_U,
	PERIPHS_IO_MUX_SD_CLK_U, PERIPHS_IO_MUX_SD_DATA0_U, PERIPHS_IO_MUX_SD_DATA1_U,
	PERIPHS_IO_MUX_SD_DATA2_U, PERIPHS_IO_MUX_SD_DATA3_U, PERIPHS_IO_MUX_SD_CMD_U,
	PERIPHS_IO_MUX_MTDI_U, PERIPHS_IO_MUX_MTCK_U, PERIPHS_IO_MUX_MTMS_U,
	PERIPHS_IO_MUX_MTDO_U, PAD_XPD_DCDC_CONF
};

// RTC pin functions definitions
constexpr auto FUNC_RTC_XPD_DCDC = 0;
constexpr auto FUNC_RTC_GPIO0 = 1;
constexpr auto FUNC_EXT_WAKEUP = 2;
constexpr auto FUNC_DEEPSLEEP = 3;
constexpr auto FUNC_BT_XTAL_EN = 4;

constexpr uint32_t func[gpio::pins][5] =
{
//  DO/DI            AF1,               AF2,             AF3,             AF4
	{FUNC_GPIO0,     FUNC_SPICS2,       0,               0,               FUNC_CLK_OUT},
	{FUNC_GPIO1,     FUNC_U0TXD,        FUNC_SPICS1,     0,               FUNC_CLK_RTC_BK},
	{FUNC_GPIO2,     FUNC_I2SO_WS,      FUNC_U1TXD_BK,   0,               FUNC_U0TXD_BK},
	{FUNC_GPIO3,     FUNC_U0RXD,        FUNC_I2SO_DATA,  0,               FUNC_CLK_XTAL_BK},
	{FUNC_GPIO4,     FUNC_CLK_XTAL,     0,               0,               0},
	{FUNC_GPIO5,     FUNC_CLK_RTC,      0,               0,               0},
	{FUNC_GPIO6,     FUNC_SDCLK,        FUNC_SPICLK,     0,               UART1_CTS},
	{FUNC_GPIO7,     FUNC_SDDATA0,      FUNC_SPIQ_MISO,  0,               FUNC_U1TXD},
	{FUNC_GPIO8,     FUNC_SDDATA1,      FUNC_SPID_MOSI,  0,               FUNC_U1RXD},
	{FUNC_GPIO9,     FUNC_SDDATA2,      FUNC_SPIHD,      0,               UFNC_HSPIHD},
	{FUNC_GPIO10,    FUNC_SDDATA3,      FUNC_SPIWP,      0,               FUNC_HSPIWP},
	{FUNC_GPIO11,    FUNC_SDCMD,        FUNC_SPICS0,     0,               U1RTS},
	{FUNC_GPIO12,    FUNC_MTDI,         FUNC_I2SI_DATA,  FUNC_HSPIQ_MISO, FUNC_UART0_DTR},
	{FUNC_GPIO13,    FUNC_MTCK,         FUNC_I2SI_BCK,   FUNC_HSPID_MOSI, FUNC_UART0_CTS},
	{FUNC_GPIO14,    FUNC_MTMS,         FUNC_I2SI_WS,    FUNC_HSPI_CLK,   FUNC_UART0_DSR},
	{FUNC_GPIO15,    FUNC_MTDO,         FUNC_I2SO_BCK,   FUNC_HSPI_CS0,   FUNC_U0RTS},
	{FUNC_RTC_GPIO0, FUNC_RTC_XPD_DCDC, FUNC_EXT_WAKEUP, FUNC_DEEPSLEEP,  FUNC_BT_XTAL_EN}
};
};
};