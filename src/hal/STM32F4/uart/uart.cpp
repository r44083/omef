#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "common/assert.h"
#include "uart.hpp"
#include "rcc/rcc.hpp"
#include "gpio/gpio_priv.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"
#include "CMSIS/Include/core_cm4.h"

using namespace hal;

#define IRQ_PRIORITY 6
#define MAX_BRR_VAL 0xFFFF

static USART_TypeDef *const uart_list[uart::UART_END] =
{
	USART1,
	USART2,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	USART3,
#else
	NULL,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	UART4,
	UART5,
#else
	NULL,
	NULL,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	USART6
#else
	NULL
#endif
};

static IRQn_Type const irq_list[uart::UART_END] =
{
	USART1_IRQn,
	USART2_IRQn,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	USART3_IRQn,
#else
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	UART4_IRQn,
	UART5_IRQn,
#else
	static_cast<IRQn_Type>(0),
	static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	USART6_IRQn
#else
	static_cast<IRQn_Type>(0)
#endif
};

static uint32_t const rcc_list[uart::UART_END] =
{
	RCC_APB2ENR_USART1EN,
	RCC_APB1ENR_USART2EN,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1ENR_USART3EN,
#else
	0,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB1ENR_UART4EN,
	RCC_APB1ENR_UART5EN,
#else
	0,
	0,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB2ENR_USART6EN
#else
	0
#endif
};

static uint32_t const reset_list[uart::UART_END] =
{
	RCC_APB2RSTR_USART1RST,
	RCC_APB1RSTR_USART2RST,
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1RSTR_USART3RST,
#else
	0,
#endif
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB1RSTR_UART4RST,
	RCC_APB1RSTR_UART5RST,
#else
	0,
	0,
#endif
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
	RCC_APB2RSTR_USART6RST
#else
	0
#endif
};

static volatile uint32_t *rcc_addr_list[uart::UART_END] =
{
	&RCC->APB2ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB2ENR
};

static volatile uint32_t *reset_addr_list[uart::UART_END] =
{
	&RCC->APB2RSTR,
	&RCC->APB1RSTR,
	&RCC->APB1RSTR,
	&RCC->APB1RSTR,
	&RCC->APB1RSTR,
	&RCC->APB2RSTR
};

static rcc_src_t const rcc_src_list[uart::UART_END] =
{
	RCC_SRC_APB2,
	RCC_SRC_APB1,
	RCC_SRC_APB1,
	RCC_SRC_APB1,
	RCC_SRC_APB1,
	RCC_SRC_APB2
};

static uint8_t const gpio_af_list[uart::UART_END] =
{
	0x07,
	0x07,
	0x07,
	0x08,
	0x08,
	0x08
};

static uart *obj_list[uart::UART_END];

static void gpio_af_init(uart::uart_t uart, gpio &gpio);

#if configUSE_TRACE_FACILITY
static traceHandle isr_dma_tx, isr_dma_rx, isr_uart;
#endif

uart::uart(uart_t uart, uint32_t baud, stopbit_t stopbit, parity_t parity,
	dma &dma_tx, dma &dma_rx, gpio &gpio_tx, gpio &gpio_rx):
	_uart(uart),
	_baud(baud),
	_stopbit(stopbit),
	_parity(parity),
	tx_dma(dma_tx),
	tx_gpio(gpio_tx),
	tx_irq_res(RES_OK),
	rx_dma(dma_rx),
	rx_gpio(gpio_rx),
	rx_cnt(NULL),
	rx_irq_res(RES_OK)
{
	ASSERT(_uart < UART_END && uart_list[_uart]);
	ASSERT(_baud > 0);
	ASSERT(_stopbit <= STOPBIT_2);
	ASSERT(_parity <= PARITY_ODD);
	ASSERT(tx_dma.dir() == dma::DIR_MEM_TO_PERIPH);
	ASSERT(tx_dma.inc_size() == dma::INC_SIZE_8);
	ASSERT(rx_dma.dir() == dma::DIR_PERIPH_TO_MEM);
	ASSERT(rx_dma.inc_size() == dma::INC_SIZE_8);
	ASSERT(tx_gpio.mode() == gpio::mode::AF);
	ASSERT(rx_gpio.mode() == gpio::mode::AF);
	
	ASSERT(api_lock = xSemaphoreCreateMutex());
	
#if configUSE_TRACE_FACILITY
	vTraceSetMutexName((void *)api_lock, "uart_api_lock");
	isr_dma_tx = xTraceSetISRProperties("ISR_dma_uart_tx", 1);
	isr_dma_rx = xTraceSetISRProperties("ISR_dma_uart_rx", 1);
	isr_uart = xTraceSetISRProperties("ISR_uart", 1);
#endif
	
	obj_list[_uart] = this;
	
	*rcc_addr_list[_uart] |= rcc_list[_uart];
	*reset_addr_list[_uart] |= reset_list[_uart];
	*reset_addr_list[_uart] &= ~reset_list[_uart];
	
	gpio_af_init(_uart, tx_gpio);
	gpio_af_init(_uart, rx_gpio);
	
	USART_TypeDef *uart_base = uart_list[_uart];
	
	switch(_stopbit)
	{
		case STOPBIT_0_5: uart_base->CR2 |= USART_CR2_STOP_0; break;
		case STOPBIT_1: uart_base->CR2 &= ~USART_CR2_STOP; break;
		case STOPBIT_1_5: uart_base->CR2 |= USART_CR2_STOP; break;
		case STOPBIT_2: uart_base->CR2 |= USART_CR2_STOP_1; break;
	}
	
	switch(_parity)
	{
		case PARITY_NONE:
			uart_base->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
			break;
		case PARITY_EVEN:
			uart_base->CR1 |= USART_CR1_PCE;
			uart_base->CR1 &= ~USART_CR1_PS;
			break;
		case PARITY_ODD:
			uart_base->CR1 |= USART_CR1_PCE | USART_CR1_PS;
			break;
	}
	
	/* Calculate UART prescaller */
	uint32_t div = rcc_get_freq(rcc_src_list[_uart]) / _baud;
	/* Baud rate is too low or too high */
	ASSERT(div > 0 && div <= MAX_BRR_VAL);
	uart_base->BRR = (uint16_t)div;
	
	tx_dma.dst((uint8_t *)&uart_base->DR);
	rx_dma.src((uint8_t *)&uart_base->DR);
	
	uart_base->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_IDLEIE |
		USART_CR1_PEIE;
	uart_base->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT | USART_CR3_EIE |
		USART_CR3_ONEBIT;
	
	NVIC_ClearPendingIRQ(irq_list[_uart]);
	NVIC_SetPriority(irq_list[_uart], IRQ_PRIORITY);
	NVIC_EnableIRQ(irq_list[_uart]);
}

uart::~uart()
{
	*reset_addr_list[_uart] |= reset_list[_uart];
	*reset_addr_list[_uart] &= ~reset_list[_uart];
	*rcc_addr_list[_uart] &= ~rcc_list[_uart];
}

void uart::baud(uint32_t baud)
{
	ASSERT(baud > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_baud = baud;
	USART_TypeDef *uart = uart_list[_uart];
	uart->CR1 &= ~USART_CR1_UE;
	uint32_t div = rcc_get_freq(rcc_src_list[_uart]) / _baud;
	/* Baud rate is too low or too high */
	ASSERT(div > 0 && div <= MAX_BRR_VAL);
	
	uart->BRR = (uint16_t)div;
	uart->CR1 |= USART_CR1_UE;
	
	xSemaphoreGive(api_lock);
}

int8_t uart::write(const uint8_t *buff, uint16_t size)
{
	ASSERT(buff);
	ASSERT(size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	task = xTaskGetCurrentTaskHandle();
	tx_dma.src((uint8_t*)buff);
	tx_dma.size(size);
	tx_dma.start_once(on_dma_tx, this);
	
	// Task will be unlocked later from isr
	ulTaskNotifyTake(true, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	
	return tx_irq_res;
}

int8_t uart::read(uint8_t *buff, uint16_t *size, uint32_t timeout)
{
	ASSERT(buff);
	ASSERT(size);
	ASSERT(*size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	rx_dma.dst(buff);
	rx_dma.size(*size);
	*size = 0;
	rx_cnt = size;
	
	task = xTaskGetCurrentTaskHandle();
	USART_TypeDef *uart = uart_list[_uart];
	uart->CR1 |= USART_CR1_RE;
	rx_dma.start_once(on_dma_rx, this);
	
	// Task will be unlocked later from isr
	if(!ulTaskNotifyTake(true, timeout))
	{
		vPortEnterCritical();
		/* Prevent common (non-DMA) UART IRQ */
		uart->CR1 &= ~USART_CR1_RE;
		uint32_t sr = uart->SR;
		uint32_t dr = uart->DR;
		NVIC_ClearPendingIRQ(irq_list[_uart]);
		/* Prevent DMA IRQ */
		rx_dma.stop();
		rx_irq_res = RES_RX_TIMEOUT;
		vPortExitCritical();
	}
	xSemaphoreGive(api_lock);
	
	return rx_irq_res;
}

int8_t uart::exch(uint8_t *tx_buff, uint16_t tx_size, uint8_t *rx_buff,
	uint16_t *rx_size, uint32_t timeout)
{
	ASSERT(tx_buff);
	ASSERT(rx_buff);
	ASSERT(tx_size > 0);
	ASSERT(rx_size);
	ASSERT(*rx_size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	/* Prepare tx */
	tx_dma.src((uint8_t *)tx_buff);
	tx_dma.size(tx_size);
	
	/* Prepare rx */
	rx_dma.dst(rx_buff);
	rx_dma.size(*rx_size);
	*rx_size = 0;
	rx_cnt = rx_size;
	
	task = xTaskGetCurrentTaskHandle();
	/* Start rx */
	USART_TypeDef *uart = uart_list[_uart];
	uart->CR1 |= USART_CR1_RE;
	rx_dma.start_once(on_dma_rx, this);
	/* Start tx */
	tx_dma.start_once(on_dma_tx, this);
	
	// Task will be unlocked later from isr
	if(!ulTaskNotifyTake(true, timeout))
	{
		vPortEnterCritical();
		/* Prevent common (non-DMA) UART IRQ */
		uart->CR1 &= ~USART_CR1_RE;
		uint32_t sr = uart->SR;
		uint32_t dr = uart->DR;
		NVIC_ClearPendingIRQ(irq_list[_uart]);
		/* Prevent DMA IRQ */
		rx_dma.stop();
		rx_irq_res = RES_RX_TIMEOUT;
		vPortExitCritical();
	}
	
	xSemaphoreGive(api_lock);
	
	return tx_irq_res != RES_OK ? tx_irq_res : rx_irq_res;
}

static void gpio_af_init(uart::uart_t uart, gpio &gpio)
{
	GPIO_TypeDef *gpio_reg = gpio_priv::ports[gpio.port()];
	
	uint8_t pin = gpio.pin();
	/* Push-pull type */
	gpio_reg->OTYPER &= ~(GPIO_OTYPER_OT0 << pin);
	
	/* Configure alternate function */
	if(pin < 8)
	{
		gpio_reg->AFR[0] &= ~(0x0F << (pin * 4));
		gpio_reg->AFR[0] |= gpio_af_list[uart] << (pin * 4);
	}
	else
	{
		gpio_reg->AFR[1] &= ~(0x0F << ((pin - 8) * 4));
		gpio_reg->AFR[1] |= gpio_af_list[uart] << ((pin - 8) * 4);
	}
}

void uart::on_dma_tx(dma *dma, dma::event_t event, void *ctx)
{
	if(event == dma::EVENT_HALF)
		return;
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_dma_tx);
#endif
	uart *obj = static_cast<uart *>(ctx);
	
	if(event == dma::EVENT_CMPLT)
		obj->tx_irq_res = RES_OK;
	else if(event == dma::EVENT_ERROR)
		obj->tx_irq_res = RES_TX_FAIL;
	
	if(obj->rx_dma.busy())
	{
		// Wait for rx operation
#if configUSE_TRACE_FACILITY
		vTraceStoreISREnd(0);
#endif
		return;
	}
	
	BaseType_t hi_task_woken = 0;
	vTaskNotifyGiveFromISR(obj->task, &hi_task_woken);
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(hi_task_woken);
#endif
	portYIELD_FROM_ISR(hi_task_woken);
}

void uart::on_dma_rx(dma *dma, dma::event_t event, void *ctx)
{
	if(event == dma::EVENT_HALF)
		return;
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_dma_rx);
#endif
	uart *obj = static_cast<uart *>(ctx);
	USART_TypeDef *uart = uart_list[obj->_uart];
	
	/* Prevent common (non-DMA) UART IRQ */
	uart->CR1 &= ~USART_CR1_RE;
	uint32_t sr = uart->SR;
	uint32_t dr = uart->DR;
	NVIC_ClearPendingIRQ(irq_list[obj->_uart]);
	
	if(event == dma::EVENT_CMPLT)
		obj->rx_irq_res = RES_OK;
	else if(event == dma::EVENT_ERROR)
		obj->rx_irq_res = RES_RX_FAIL;
	/* Rx buffer has partly filled (package has received) or Rx buffer has
	totally filled */
	if(obj->rx_cnt)
		*obj->rx_cnt = obj->rx_dma.transfered();
	
	if(obj->tx_dma.busy())
	{
		// Wait for tx operation
#if configUSE_TRACE_FACILITY
		vTraceStoreISREnd(0);
#endif
		return;
	}
	
	BaseType_t hi_task_woken = 0;
	vTaskNotifyGiveFromISR(obj->task, &hi_task_woken);
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(hi_task_woken);
#endif
	portYIELD_FROM_ISR(hi_task_woken);
}

extern "C" void uart_irq_hndlr(hal::uart *obj)
{
	USART_TypeDef *uart = uart_list[obj->_uart];
	uint32_t sr = uart->SR;
	uint32_t dr = uart->DR;
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_uart);
#endif
	if((uart->CR1 & USART_CR1_IDLEIE) && (sr & USART_SR_IDLE))
	{
		/* IDLE event has happened (package has received) */
		obj->rx_irq_res = uart::RES_OK;
	}
	else if((uart->CR3 & USART_CR3_EIE) && (sr & (USART_SR_PE | USART_SR_FE |
		USART_SR_NE | USART_SR_ORE)))
	{
		/* Error event has happened */
		obj->rx_irq_res = uart::RES_RX_FAIL;
	}
	else
	{
#if configUSE_TRACE_FACILITY
		vTraceStoreISREnd(0);
#endif
		return;
	}
	
	/* Prevent DMA IRQ */
	obj->rx_dma.stop();
	
	uart->CR1 &= ~USART_CR1_RE;
	if(obj->rx_cnt)
		*obj->rx_cnt = obj->rx_dma.transfered();
	
	if(obj->tx_dma.busy())
	{
		// Wait for tx operation
#if configUSE_TRACE_FACILITY
		vTraceStoreISREnd(0);
#endif
		return;
	}
	
	BaseType_t hi_task_woken = 0;
	vTaskNotifyGiveFromISR(obj->task, &hi_task_woken);
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(hi_task_woken);
#endif
	portYIELD_FROM_ISR(hi_task_woken);
}

extern "C" void USART1_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[uart::UART_1]);
}

extern "C" void USART2_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[uart::UART_2]);
}

#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
extern "C" void USART3_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[uart::UART_3]);
}
#endif

#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
extern "C" void UART4_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[uart::UART_4]);
}

extern "C" void UART5_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[uart::UART_5]);
}
#endif

#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
	defined(STM32F411xE) || defined(STM32F412Cx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F412Zx) || defined(STM32F413xx) || \
	defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F423xx) || \
	defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || \
	defined(STM32F439xx) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx)
extern "C" void USART6_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[uart::UART_6]);
}
#endif
