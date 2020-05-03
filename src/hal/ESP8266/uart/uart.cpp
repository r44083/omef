#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include "common/assert.h"
#include "uart.hpp"
#include "ESP8266_RTOS_SDK/components/esp8266/include/esp8266/uart_register.h"
#include "ESP8266_RTOS_SDK/components/esp8266/include/esp8266/uart_struct.h"
#include "ESP8266_RTOS_SDK/components/esp8266/include/rom/ets_sys.h"

using namespace hal;

constexpr uint8_t fifo_size = 128;
constexpr uint32_t max_uart_div = 0xFFFFF;

constexpr uint32_t rx_ena_bits = (UART_RXFIFO_FULL_INT_ENA | UART_RXFIFO_TOUT_INT_ENA);
constexpr uint32_t rx_clr_bits = (UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
constexpr uint32_t tx_ena_bits = UART_TXFIFO_EMPTY_INT_ENA;
constexpr uint32_t tx_clr_bits = UART_TXFIFO_EMPTY_INT_CLR;
constexpr uint32_t err_ena_bits = (UART_RXFIFO_OVF_INT_ENA | UART_PARITY_ERR_INT_ENA |
	UART_FRM_ERR_INT_ENA);
constexpr uint32_t err_clr_bits = (UART_RXFIFO_OVF_INT_CLR | UART_PARITY_ERR_INT_CLR |
	UART_FRM_ERR_INT_CLR);

static DRAM_ATTR uart_dev_t *const uart_devs[] = {&uart0, &uart1};

static void uart_hw_cb(void *arg);

uart::uart(uart_t uart, uint32_t baud, stopbit_t stopbit, parity_t parity,
	gpio *gpio_tx, gpio *gpio_rx):
	_uart(uart),
	_baud(baud),
	_stopbit(stopbit),
	_parity(parity),
	tx{.gpio = gpio_tx, .buff = NULL, .remain = 0},
	rx{.gpio = gpio_rx, .buff = NULL, .remain = 0}
{
	ASSERT(_uart <= UART_1);
	ASSERT(_baud > 0);
	ASSERT(_stopbit <= STOPBIT_2);
	ASSERT(_parity <= PARITY_ODD);
	ASSERT(tx.gpio || rx.gpio); // Should be at least tx or rx
	
	ASSERT(api_lock = xSemaphoreCreateMutex());
	
	uart_dev_t &uart_dev = *uart_devs[_uart];
	
	enum {DATABITS_5, DATABITS_6, DATABITS_7, DATABITS_8};
	uart_dev.conf0.bit_num = DATABITS_8;
	
	uart_dev.conf0.stop_bit_num = _stopbit + 1;
	
	enum {ESP8266_PARITY_EVEN, ESP8266_PARITY_ODD};
	switch(_parity)
	{
		case PARITY_NONE:
			uart_dev.conf0.parity_en = 0;
			break;
		case PARITY_EVEN:
			uart_dev.conf0.parity_en = 1;
			uart_dev.conf0.parity = ESP8266_PARITY_EVEN;
			break;
		case PARITY_ODD:
			uart_dev.conf0.parity_en = 1;
			uart_dev.conf0.parity = ESP8266_PARITY_ODD;
			break;
	}
	
	uint32_t div = APB_CLK_FREQ / _baud;
	ASSERT(div <= max_uart_div);
	uart_dev.clk_div.val = div;
	
	// IDLE timeout for UART receiver
	uart_dev.conf1.rx_tout_thrhd = 2;
	uart_dev.conf1.rx_tout_en = 1;
	
	// Generate tx interrupt when fifo is completely empty
	uart_dev.conf1.txfifo_empty_thrhd = 0;
	
	uart_dev.int_ena.val &= ~(rx_ena_bits | tx_ena_bits | err_ena_bits);
	uart_dev.int_clr.val = rx_ena_bits | tx_ena_bits | err_ena_bits;
	
	_xt_isr_attach(ETS_UART_INUM, uart_hw_cb, this);
	_xt_isr_unmask(1 << ETS_UART_INUM);
}

uart::~uart()
{
	_xt_isr_mask(1 << ETS_UART_INUM);
	_xt_isr_attach(ETS_UART_INUM, NULL, NULL);
	uart_dev_t &uart_dev = *uart_devs[_uart];
	uart_dev.int_ena.val &= ~(rx_ena_bits | tx_ena_bits | err_ena_bits);
	uart_dev.conf0.val |= UART_TXFIFO_RST | UART_RXFIFO_RST;
	uart_dev.conf0.val &= ~(UART_TXFIFO_RST | UART_RXFIFO_RST);
	xSemaphoreGive(api_lock);
	vSemaphoreDelete(api_lock);
}

void uart::baud(uint32_t baud)
{
	ASSERT(baud > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_baud = baud;
	uint32_t div = APB_CLK_FREQ / _baud;
	ASSERT(div <= max_uart_div);
	uart_devs[_uart]->clk_div.val = div;
	
	xSemaphoreGive(api_lock);
}

int8_t uart::write(const void *buff, size_t size)
{
	ASSERT(buff);
	ASSERT(size > 0);
	ASSERT(tx.gpio);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	task = xTaskGetCurrentTaskHandle();
	
	tx.buff = (uint8_t *)buff;
	uint8_t payload_size = std::min<size_t>(size, fifo_size);
	tx.remain = size - payload_size;
	
	uart_dev_t &uart_dev = *uart_devs[_uart];
	uart_dev.conf0.txfifo_rst = 1;
	uart_dev.conf0.txfifo_rst = 0;
	
	while(payload_size--)
		uart_dev.fifo.rw_byte = *(tx.buff++);
	
	uart_dev.int_clr.val = tx_clr_bits;
	uart_dev.int_ena.val |= tx_ena_bits;
	
	// Task will be unlocked later from isr
	ulTaskNotifyTake(true, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	
	return tx.irq_res;
}

int8_t uart::read(void *buff, size_t *size, uint32_t timeout)
{
	ASSERT(buff);
	ASSERT(size);
	ASSERT(*size > 0);
	ASSERT(rx.gpio);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	task = xTaskGetCurrentTaskHandle();
	
	rx.buff = (uint8_t *)buff;
	rx.remain = *size;
	
	uart_dev_t &uart_dev = *uart_devs[_uart];
	uart_dev.conf0.rxfifo_rst = 1;
	uart_dev.conf0.rxfifo_rst = 0;
	uart_dev.conf1.rxfifo_full_thrhd = std::min<size_t>(*size, fifo_size - 1);
	
	uart_dev.int_clr.val = rx_clr_bits | err_clr_bits;
	uart_dev.int_ena.val |= rx_ena_bits | err_ena_bits;
	
	// Task will be unlocked later from isr
	if(!ulTaskNotifyTake(true, timeout))
	{
		uart_dev.int_ena.val &= ~(rx_ena_bits | err_ena_bits);
		rx.irq_res = RES_RX_TIMEOUT;
	}
	*size -= rx.remain;
	
	xSemaphoreGive(api_lock);
	
	return rx.irq_res;
}

int8_t uart::exch(const void *tx_buff, size_t tx_size, void *rx_buff,
	size_t *rx_size, uint32_t timeout)
{
	ASSERT(tx_buff);
	ASSERT(rx_buff);
	ASSERT(tx_size > 0);
	ASSERT(rx_size);
	ASSERT(*rx_size > 0);
	ASSERT(tx.gpio && rx.gpio);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	task = xTaskGetCurrentTaskHandle();
	
	rx.buff = (uint8_t *)rx_buff;
	rx.remain = *rx_size;
	
	tx.buff = (uint8_t *)tx_buff;
	uint8_t payload_size = std::min<size_t>(tx_size, fifo_size);
	tx.remain = tx_size - payload_size;
	
	uart_dev_t &uart_dev = *uart_devs[_uart];
	uart_dev.conf0.val |= UART_TXFIFO_RST | UART_RXFIFO_RST;
	uart_dev.conf0.val &= ~(UART_TXFIFO_RST | UART_RXFIFO_RST);
	uart_dev.conf1.rxfifo_full_thrhd = std::min<size_t>(*rx_size, fifo_size - 1);
	
	while(payload_size--)
		uart_dev.fifo.rw_byte = *(tx.buff++);
	
	uart_dev.int_clr.val = rx_clr_bits | tx_clr_bits | err_clr_bits;
	uart_dev.int_ena.val |= rx_ena_bits | tx_ena_bits | err_ena_bits;
	
	// Task will be unlocked later from isr
	if(!ulTaskNotifyTake(true, timeout))
	{
		uart_dev.int_ena.val &= ~(rx_ena_bits | tx_ena_bits | err_ena_bits);
		rx.irq_res = RES_RX_TIMEOUT;
	}
	*rx_size -= rx.remain;
	
	xSemaphoreGive(api_lock);
	
	return tx.irq_res != RES_OK ? tx.irq_res : rx.irq_res;
}

void uart::tx_irq_hndlr()
{
	uart_dev_t &uart_dev = *uart_devs[_uart];
	
	if(tx.remain)
	{
		uint8_t payload_size = std::min<size_t>(tx.remain, fifo_size);
		tx.remain -= payload_size;
		
		while(payload_size--)
			uart_dev.fifo.rw_byte = *(tx.buff++);
		return;
	}
	tx.irq_res = uart::RES_OK;
	uart_dev.int_ena.val &= ~tx_ena_bits;
	
	if(uart_dev.int_ena.val & rx_ena_bits)
		return; // Wait for rx operation
	
	BaseType_t hi_task_woken = 0;
	vTaskNotifyGiveFromISR(task, &hi_task_woken);
	if(hi_task_woken)
		taskYIELD();
}

void uart::rx_irq_hndlr(bool is_timeout)
{
	uart_dev_t &uart_dev = *uart_devs[_uart];
	
	uint8_t received = uart_dev.status.rxfifo_cnt;
	rx.remain -= received;
	
	while(received--)
		*(rx.buff++) = uart_dev.fifo.rw_byte;
	
	if(rx.remain && !is_timeout)
	{
		uart_dev.conf1.rxfifo_full_thrhd = std::min<size_t>(rx.remain,
			fifo_size - 1);
		return;
	}
	rx.irq_res = uart::RES_OK;
	uart_dev.int_ena.val &= ~(rx_ena_bits | err_ena_bits);
	
	if(uart_dev.int_ena.val & tx_ena_bits)
		return; // Wait for tx operation
	
	BaseType_t hi_task_woken = 0;
	vTaskNotifyGiveFromISR(task, &hi_task_woken);
	if(hi_task_woken)
		taskYIELD();
}

void uart::err_irq_hndlr()
{
	uart_dev_t &uart_dev = *uart_devs[_uart];
	uart_dev.int_ena.val &= ~(rx_ena_bits | tx_ena_bits | err_ena_bits);
	uart_dev.conf0.val |= UART_TXFIFO_RST | UART_RXFIFO_RST;
	uart_dev.conf0.val &= ~(UART_TXFIFO_RST | UART_RXFIFO_RST);
	
	rx.irq_res = uart::RES_RX_FAIL;
	
	BaseType_t hi_task_woken = 0;
	vTaskNotifyGiveFromISR(task, &hi_task_woken);
	if(hi_task_woken)
		taskYIELD();
}

extern "C" void uart_irq_hndlr(uart *obj)
{
	uart_dev_t &uart_dev = *uart_devs[obj->_uart];
	uint32_t status = uart_dev.int_st.val;
	
	if((uart_dev.int_ena.rxfifo_tout && (status & UART_RXFIFO_TOUT_INT_ST)) ||
		(uart_dev.int_ena.rxfifo_full && (status & UART_RXFIFO_FULL_INT_ST)))
	{
		obj->rx_irq_hndlr(status & UART_RXFIFO_TOUT_INT_ST);
		uart_dev.int_clr.val = rx_clr_bits;
	}
	else if(uart_dev.int_ena.txfifo_empty && (status & UART_TXFIFO_EMPTY_INT_ST))
	{
		obj->tx_irq_hndlr();
		uart_dev.int_clr.val = tx_clr_bits;
	}
	else if((uart_dev.int_ena.rxfifo_ovf && (status & UART_RXFIFO_OVF_INT_ST)) ||
		(uart_dev.int_ena.frm_err && (status & UART_FRM_ERR_INT_ST)) ||
		(uart_dev.int_ena.parity_err && (status & UART_PARITY_ERR_INT_ST)))
	{
		obj->err_irq_hndlr();
		uart_dev.int_clr.val = err_clr_bits;
	}
}

static void uart_hw_cb(void *arg)
{
	uart_irq_hndlr((uart *)arg);
}