#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "gpio/gpio.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

namespace hal { class uart; }
// For internal use only! (called from ISR)
extern "C" void uart_irq_hndlr(hal::uart *obj);

namespace hal
{
class uart
{
	public:
		enum uart_t
		{
			UART_0,
			UART_1
		};

		enum stopbit_t
		{
			STOPBIT_1,
			STOPBIT_1_5,
			STOPBIT_2
		};

		enum parity_t
		{
			PARITY_NONE,
			PARITY_EVEN,
			PARITY_ODD
		};

		enum res_t
		{
			RES_OK         =  0,
			RES_RX_TIMEOUT = -1,
			RES_TX_FAIL    = -2,
			RES_RX_FAIL    = -3
		};
		
		uart(uart_t uart, uint32_t baud, stopbit_t stopbit, parity_t parity,
			gpio *gpio_tx, gpio *gpio_rx);
		~uart();
		
		void baud(uint32_t baud);
		uint32_t baud() const { return _baud; }
		int8_t write(const void *buff, size_t size);
		int8_t read(void *buff, size_t *size, uint32_t timeout);
		int8_t exch(const void *tx_buff, size_t tx_size, void *rx_buff,
			size_t *rx_size, uint32_t timeout);
	
	private:
		uart_t _uart;
		uint32_t _baud;
		stopbit_t _stopbit;
		parity_t _parity;
		struct
		{
			hal::gpio *gpio;
			uint8_t *buff;
			size_t remain;
			int8_t irq_res;
		} tx;
		struct
		{
			hal::gpio *gpio;
			uint8_t *buff;
			size_t remain;
			int8_t irq_res;
		} rx;
		SemaphoreHandle_t api_lock;
		TaskHandle_t task;
		
		void tx_irq_hndlr();
		void rx_irq_hndlr(bool is_timeout);
		void err_irq_hndlr();
		friend void ::uart_irq_hndlr(uart *obj);
};
}