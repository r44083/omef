#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <limits>
#include "common/assert.h"
#include "uart.hpp"
#include "uart_priv.hpp"
#include "rcc/rcc.hpp"
#include "CMSIS/Device/STM32F0xx/Include/stm32f0xx.h"
#include "CMSIS/Include/core_cm0.h"

using namespace hal;

constexpr auto isr_err_flags = USART_ISR_PE | USART_ISR_FE | USART_ISR_NE |
	USART_ISR_ORE;

static uart *obj_list[uart::UART_END];

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
	ASSERT(_uart < UART_END && uart_priv::uart[_uart]);
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
	
	*uart_priv::rcc_en_reg[_uart] |= uart_priv::rcc_en[_uart];
	*uart_priv::rcc_rst_reg[_uart] |= uart_priv::rcc_rst[_uart];
	*uart_priv::rcc_rst_reg[_uart] &= ~uart_priv::rcc_rst[_uart];
	
	gpio_af_init(_uart, tx_gpio);
	gpio_af_init(_uart, rx_gpio);
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	remap_dma(_uart, tx_dma);
	remap_dma(_uart, rx_dma);
	
	USART_TypeDef *uart_reg = uart_priv::uart[_uart];
	
	switch(_stopbit)
	{
		case STOPBIT_0_5: uart_reg->CR2 |= USART_CR2_STOP_0; break;
		case STOPBIT_1: uart_reg->CR2 &= ~USART_CR2_STOP; break;
		case STOPBIT_1_5: uart_reg->CR2 |= USART_CR2_STOP; break;
		case STOPBIT_2: uart_reg->CR2 |= USART_CR2_STOP_1; break;
	}
	
	switch(_parity)
	{
		case PARITY_NONE:
			uart_reg->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
			break;
		case PARITY_EVEN:
			uart_reg->CR1 |= USART_CR1_PCE;
			uart_reg->CR1 &= ~USART_CR1_PS;
			break;
		case PARITY_ODD:
			uart_reg->CR1 |= USART_CR1_PCE | USART_CR1_PS;
			break;
	}
	
	// Calculate UART prescaller
	uint32_t div = rcc_get_freq(uart_priv::rcc_src[_uart]) / _baud;
	
	const auto brr_max = std::numeric_limits<uint16_t>::max();
	ASSERT(div > 0 && div <= brr_max); // Baud rate is too low or too high
	uart_reg->BRR = div;
	
	tx_dma.dst((void *)&uart_reg->TDR);
	rx_dma.src((void *)&uart_reg->RDR);
	
	uart_reg->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_IDLEIE |
		USART_CR1_PEIE;
	uart_reg->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT | USART_CR3_EIE |
		USART_CR3_ONEBIT;
	
	NVIC_ClearPendingIRQ(uart_priv::irqn[_uart]);
	NVIC_SetPriority(uart_priv::irqn[_uart], 6);
	NVIC_EnableIRQ(uart_priv::irqn[_uart]);
}

uart::~uart()
{
	NVIC_DisableIRQ(uart_priv::irqn[_uart]);
	uart_priv::uart[_uart]->CR1 &= ~USART_CR1_UE;
	*uart_priv::rcc_en_reg[_uart] &= ~uart_priv::rcc_en[_uart];
	xSemaphoreGive(api_lock);
	vSemaphoreDelete(api_lock);
	obj_list[_uart] = NULL;
}

void uart::baud(uint32_t baud)
{
	ASSERT(baud > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_baud = baud;
	USART_TypeDef *uart = uart_priv::uart[_uart];
	uart->CR1 &= ~USART_CR1_UE;
	uint32_t div = rcc_get_freq(uart_priv::rcc_src[_uart]) / _baud;
	
	const auto brr_max = std::numeric_limits<uint16_t>::max();
	ASSERT(div > 0 && div <= brr_max); // Baud rate is too low or too high
	
	uart->BRR = div;
	uart->CR1 |= USART_CR1_UE;
	
	xSemaphoreGive(api_lock);
}

int8_t uart::write(void *buff, uint16_t size)
{
	ASSERT(buff);
	ASSERT(size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	task = xTaskGetCurrentTaskHandle();
	tx_dma.src(buff);
	tx_dma.size(size);
	tx_dma.start_once(on_dma_tx, this);
	
	// Task will be unlocked later from isr
	ulTaskNotifyTake(true, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	return tx_irq_res;
}

int8_t uart::read(void *buff, uint16_t *size, uint32_t timeout)
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
	USART_TypeDef *uart = uart_priv::uart[_uart];
	uart->CR1 |= USART_CR1_RE;
	rx_dma.start_once(on_dma_rx, this);
	
	// Task will be unlocked later from isr
	if(!ulTaskNotifyTake(true, timeout))
	{
		vPortEnterCritical();
		// Prevent common (non-DMA) UART IRQ
		uart->CR1 &= ~USART_CR1_RE;
		uint32_t sr = uart->ISR;
		uint32_t dr = uart->RDR;
		NVIC_ClearPendingIRQ(uart_priv::irqn[_uart]);
		// Prevent DMA IRQ
		rx_dma.stop();
		rx_irq_res = RES_RX_TIMEOUT;
		vPortExitCritical();
	}
	xSemaphoreGive(api_lock);
	return rx_irq_res;
}

int8_t uart::exch(void *tx_buff, uint16_t tx_size, void *rx_buff,
	uint16_t *rx_size, uint32_t timeout)
{
	ASSERT(tx_buff);
	ASSERT(rx_buff);
	ASSERT(tx_size > 0);
	ASSERT(rx_size);
	ASSERT(*rx_size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	// Prepare tx
	tx_dma.src(tx_buff);
	tx_dma.size(tx_size);
	
	// Prepare rx
	rx_dma.dst(rx_buff);
	rx_dma.size(*rx_size);
	*rx_size = 0;
	rx_cnt = rx_size;
	
	task = xTaskGetCurrentTaskHandle();
	// Start rx
	USART_TypeDef *uart = uart_priv::uart[_uart];
	uart->CR1 |= USART_CR1_RE;
	rx_dma.start_once(on_dma_rx, this);
	// Start tx
	tx_dma.start_once(on_dma_tx, this);
	
	// Task will be unlocked later from isr
	if(!ulTaskNotifyTake(true, timeout))
	{
		vPortEnterCritical();
		// Prevent common (non-DMA) UART IRQ
		uart->CR1 &= ~USART_CR1_RE;
		uint32_t sr = uart->ISR;
		uint32_t dr = uart->RDR;
		NVIC_ClearPendingIRQ(uart_priv::irqn[_uart]);
		// Prevent DMA IRQ
		rx_dma.stop();
		rx_irq_res = RES_RX_TIMEOUT;
		vPortExitCritical();
	}
	
	xSemaphoreGive(api_lock);
	return tx_irq_res != RES_OK ? tx_irq_res : rx_irq_res;
}

void uart::remap_dma(uart_t uart, dma &dma)
{
	dma::ch_t ch = dma.get_ch();
	
#if defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx)
	switch(ch)
	{
		case dma::CH_2:
			if(uart == uart::UART_1)
				SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART1TX_DMA_RMP;
			else if(uart == uart::UART_3)
				SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART3_DMA_RMP;
			break;
		
		case dma::CH_3:
			if(uart == uart::UART_1)
				SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART1RX_DMA_RMP;
			else if(uart == uart::UART_3)
				SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART3_DMA_RMP;
			break;
		
		case dma::CH_4:
			if(uart == uart::UART_1)
				SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;
			else if(uart == uart::UART_2)
				SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART2_DMA_RMP;
			break;
		
		case dma::CH_5:
			if(uart == uart::UART_1)
				SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1RX_DMA_RMP;
			else if(uart == uart::UART_2)
				SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART2_DMA_RMP;
			break;
		
		case dma::CH_6:
		case dma::CH_7:
			if(uart == uart::UART_2)
				SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART2_DMA_RMP;
			else if(uart == uart::UART_3)
				SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART3_DMA_RMP;
			break;
			
		default: break;
	}
#elif defined(STM32F091xC) || defined(STM32F098xx)
#error Not implemented. Need to change DMA1_CSELR: "DMAx channel selection registers"
#else
	switch(ch)
	{
		case dma::CH_2:
			if(uart == uart::UART_1)
				SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART1TX_DMA_RMP;
			break;
		
		case dma::CH_3:
			if(uart == uart::UART_1)
				SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART1RX_DMA_RMP;
			break;
		
		case dma::CH_4:
			if(uart == uart::UART_1)
				SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;
			break;
		
		case dma::CH_5:
			if(uart == uart::UART_1)
				SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1RX_DMA_RMP;
			break;
		
		default: break;
	}
#endif
}

void uart::gpio_af_init(uart_t uart, gpio &gpio)
{
	GPIO_TypeDef *gpio_reg = gpio_priv::gpio[gpio.port()];
	uint8_t pin = gpio.pin();
	
	// Push-pull
	gpio_reg->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin);
	
	// Configure alternate function
	gpio_reg->AFR[pin / 8] &= ~(GPIO_AFRL_AFSEL0 << ((pin % 8) * 4));
	gpio_reg->AFR[pin / 8] |= uart_priv::uart2afr[uart][gpio.port()] <<
		((pin % 8) * 4);
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
	USART_TypeDef *uart = uart_priv::uart[obj->_uart];
	
	// Prevent common (non-DMA) UART IRQ
	uart->CR1 &= ~USART_CR1_RE;
	uint32_t sr = uart->ISR;
	uint32_t dr = uart->RDR;
	NVIC_ClearPendingIRQ(uart_priv::irqn[obj->_uart]);
	
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
	USART_TypeDef *uart = uart_priv::uart[obj->_uart];
	uint32_t sr = uart->ISR;
	uint32_t dr = uart->RDR;
	
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_uart);
#endif
	if((uart->CR1 & USART_CR1_IDLEIE) && (sr & USART_ISR_IDLE))
	{
		// IDLE event has happened (package has been received)
		obj->rx_irq_res = uart::RES_OK;
	}
	else if((uart->CR3 & USART_CR3_EIE) && (sr & isr_err_flags))
	{
		// Error event has happened
		obj->rx_irq_res = uart::RES_RX_FAIL;
	}
	else
	{
#if configUSE_TRACE_FACILITY
		vTraceStoreISREnd(0);
#endif
		return;
	}
	
	// Prevent DMA IRQ
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

#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
	defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
	defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
extern "C" void USART2_IRQHandler(void)
{
	uart_irq_hndlr(obj_list[uart::UART_2]);
}
#endif

#if defined(STM32F030xC)
extern "C" void USART3_6_IRQHandler(void)
{
	for(uint8_t i = uart::UART_3; i <= uart::UART_6; i++)
	{
		USART_TypeDef *uart = uart_priv::uart[i];
		uint32_t sr = uart->ISR;
		
		if((uart->CR1 & USART_CR1_UE) &&
			(((uart->CR1 & USART_CR1_IDLEIE) && (sr & USART_ISR_IDLE)) ||
			((uart->CR3 & USART_CR3_EIE) && (sr & isr_err_flags))))
		{
			uart_irq_hndlr(obj_list[i]);
			break;
		}
	}
}
#elif defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
	defined(STM32F078xx)
extern "C" void USART3_4_IRQHandler(void)
{
	for(uint8_t i = uart::UART_3; i <= uart::UART_4; i++)
	{
		USART_TypeDef *uart = uart_priv::uart[i];
		uint32_t sr = uart->ISR;
		
		if((uart->CR1 & USART_CR1_UE) &&
			(((uart->CR1 & USART_CR1_IDLEIE) && (sr & USART_ISR_IDLE)) ||
			((uart->CR3 & USART_CR3_EIE) && (sr & isr_err_flags))))
		{
			uart_irq_hndlr(obj_list[i]);
			break;
		}
	}
}
#elif defined(STM32F091xC) || defined(STM32F098xx)
extern "C" void USART3_8_IRQHandler(void)
{
	for(uint8_t i = uart::UART_3; i <= uart::UART_8; i++)
	{
		USART_TypeDef *uart = uart_priv::uart[i];
		uint32_t sr = uart->ISR;
		
		if((uart->CR1 & USART_CR1_UE) &&
			(((uart->CR1 & USART_CR1_IDLEIE) && (sr & USART_ISR_IDLE)) ||
			((uart->CR3 & USART_CR3_EIE) && (sr & isr_err_flags))))
		{
			uart_irq_hndlr(obj_list[i]);
			break;
		}
	}
}
#endif