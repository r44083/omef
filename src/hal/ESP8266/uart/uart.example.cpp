#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio/gpio.hpp"
#include "uart/uart.hpp"

static void main_task(void *pvParameters)
{
	hal::uart &_uart = *((hal::uart *)pvParameters);
	
	while(1)
	{
		_uart.write("uart test\n", sizeof("uart test\n") - 1);
		
		/* Example of reading:
		uint8_t buff[100];
		size_t size;
		memset(buff, 0, sizeof(buff));
		size = sizeof(buff);
		_uart.read(buff, &size, portMAX_DELAY); */
		
		/* Example of full duplex transferring:
		uint8_t buff[100];
		size_t size;
		memset(buff, 0, sizeof(buff));
		size = sizeof(buff);
		_uart.exch("mytest", sizeof("mytest") - 1, buff, &size, portMAX_DELAY); */
		
		vTaskDelay(500);
	}
}

extern "C" void app_main(void)
{
	/* To be able to use UART0, don't forget to set CONFIG_CONSOLE_UART_NUM to 1
	   in sdkconfig.h. To be able to use printf() from SDK - initialize uart1_tx
	   (GPIO2) like in example below.
	*/

	static hal::gpio uart0_tx(0, 1, hal::gpio::mode::AF1);
	static hal::gpio uart0_rx(0, 3, hal::gpio::mode::AF1);
	static hal::uart uart0(hal::uart::UART_0, 115200, hal::uart::STOPBIT_1,
		hal::uart::PARITY_NONE, &uart0_tx, &uart0_rx);
	
	/* Initializing UART1:
	static hal::gpio uart1_tx(0, 2, hal::gpio::mode::AF2);
	static hal::uart uart1(hal::uart::UART_1, 115200, hal::uart::STOPBIT_1,
		hal::uart::PARITY_NONE, &uart1_tx, NULL);*/
	
	xTaskCreate(main_task, "main", 500, &uart0, tskIDLE_PRIORITY + 1, NULL);
}
