#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "common/assert.h"
#include "i2c.hpp"
#include "rcc/rcc.hpp"
#include "gpio/gpio_priv.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"
#include "CMSIS/Include/core_cm4.h"

using namespace hal;

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a):(b))
#endif

#define IRQ_PRIORITY 3
#define STANDARD_I2C_MAX_SPEED 100000 /* bps */
#define FAST_I2C_MAX_SPEED 400000 /* bps */

static I2C_TypeDef *const i2c_list[i2c::I2C_END] =
{
	I2C1, I2C2,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	I2C3
#else
	NULL
#endif
};

static IRQn_Type const event_irq_list[i2c::I2C_END] =
{
	I2C1_EV_IRQn, I2C2_EV_IRQn,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	I2C3_EV_IRQn
#else
	static_cast<IRQn_Type>(0)
#endif
};

static IRQn_Type const err_irq_list[i2c::I2C_END] =
{
	I2C1_ER_IRQn, I2C2_ER_IRQn,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	I2C3_ER_IRQn
#else
	static_cast<IRQn_Type>(0)
#endif
};

static uint32_t const rcc_list[i2c::I2C_END] =
{
	RCC_APB1ENR_I2C1EN, RCC_APB1ENR_I2C2EN,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1ENR_I2C3EN
#else
	0
#endif
};

static uint32_t const reset_list[i2c::I2C_END] =
{
	RCC_APB1RSTR_I2C1RST, RCC_APB1RSTR_I2C2RST,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
	RCC_APB1RSTR_I2C3RST
#else
	0
#endif
};

static uint8_t const gpio_af_list[i2c::I2C_END] =
{
	0x04,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
	0x09, 0x09
#else
	0x04, 0x04
#endif
};

static i2c *obj_list[i2c::I2C_END];

static void gpio_af_init(i2c::i2c_t i2c, gpio &gpio);
static void calc_clk(i2c::i2c_t i2c, uint32_t baud, uint8_t *freq, uint8_t *trise,
	uint32_t *ccr);

#if configUSE_TRACE_FACILITY
static traceHandle isr_dma_tx, isr_dma_rx, isr_i2c_event, isr_i2c_error;
#endif

#if configUSE_TRACE_FACILITY
static traceString ch1;
#endif

i2c::i2c(i2c_t i2c, uint32_t baud, dma &dma_tx, dma &dma_rx, gpio &sda,
	gpio &scl):
	_i2c(i2c),
	_baud(baud),
	irq_res(RES_OK),
	_sda(sda),
	_scl(scl),
	tx_dma(dma_tx),
	tx_buff(NULL),
	tx_size(0),
	rx_dma(dma_rx),
	rx_buff(NULL),
	rx_size(0)
{
	ASSERT(_i2c < I2C_END && i2c_list[_i2c]);
	ASSERT(_baud > 0 && _baud <= FAST_I2C_MAX_SPEED);
	ASSERT(tx_dma.dir() == dma::DIR_MEM_TO_PERIPH);
	ASSERT(tx_dma.inc_size() == dma::INC_SIZE_8);
	ASSERT(rx_dma.dir() == dma::DIR_PERIPH_TO_MEM);
	ASSERT(rx_dma.inc_size() == dma::INC_SIZE_8);
	ASSERT(_sda.mode() == gpio::mode::AF);
	ASSERT(_scl.mode() == gpio::mode::AF);
	
	ASSERT(api_lock = xSemaphoreCreateMutex());
	
#if configUSE_TRACE_FACILITY
	vTraceSetMutexName((void *)api_lock, "i2c_api_lock");
	isr_dma_tx = xTraceSetISRProperties("ISR_dma_i2c_tx", 1);
	isr_dma_rx = xTraceSetISRProperties("ISR_dma_i2c_rx", 1);
	isr_i2c_event = xTraceSetISRProperties("ISR_i2c_event", 1);
	isr_i2c_error = xTraceSetISRProperties("ISR_i2c_error", 1);
#endif
	
	obj_list[_i2c] = this;
	
	RCC->APB1ENR |= rcc_list[_i2c];
	RCC->APB1RSTR |= reset_list[_i2c];
	RCC->APB1RSTR &= ~reset_list[_i2c];
	
	gpio_af_init(_i2c, _sda);
	gpio_af_init(_i2c, _scl);
	
	I2C_TypeDef *i2c_base = i2c_list[_i2c];
	
	i2c_base->CR1 |= I2C_CR1_SWRST;
	i2c_base->CR1 &= ~I2C_CR1_SWRST;
	
	/* Setup i2c speed */
	uint8_t freq = 0, trise = 0;
	uint32_t ccr = 0;
	calc_clk(_i2c, _baud, &freq, &trise, &ccr);
	i2c_base->CR2 |= (freq & I2C_CR2_FREQ);
	i2c_base->TRISE = trise;
	i2c_base->CCR = ccr;
	//i2c_base->TRISE = 11;
	//i2c_base->CCR = 30;
	
	/* Enable DMA support */
	i2c_base->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;
	
	/* Enable interrupts */
	i2c_base->CR2 |= (I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
	
	i2c_base->CR1 |= I2C_CR1_PE;
	
	tx_dma.dst((uint8_t *)&i2c_base->DR);
	rx_dma.src((uint8_t *)&i2c_base->DR);
	
	NVIC_ClearPendingIRQ(event_irq_list[_i2c]);
	NVIC_ClearPendingIRQ(err_irq_list[_i2c]);
	NVIC_SetPriority(event_irq_list[_i2c], IRQ_PRIORITY);
	NVIC_SetPriority(err_irq_list[_i2c], IRQ_PRIORITY);
	NVIC_EnableIRQ(event_irq_list[_i2c]);
	NVIC_EnableIRQ(err_irq_list[_i2c]);
}

i2c::~i2c()
{
	RCC->APB1RSTR |= reset_list[_i2c];
	RCC->APB1RSTR &= ~reset_list[_i2c];
	RCC->APB1ENR &= ~rcc_list[_i2c];
}

void i2c::baud(uint32_t baud)
{
	ASSERT(baud > 0 && baud <= FAST_I2C_MAX_SPEED);
	
	uint8_t freq = 0, trise = 0;
	uint32_t ccr = 0;
	calc_clk(_i2c, baud, &freq, &trise, &ccr);
	
	I2C_TypeDef *i2c_base = i2c_list[_i2c];
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_baud = baud;
	
	i2c_base->CR1 &= ~I2C_CR1_PE;
	
	i2c_base->CR2 &= ~I2C_CR2_FREQ;
	i2c_base->CR2 |= (freq & I2C_CR2_FREQ);
	i2c_base->TRISE = trise;
	i2c_base->CCR = ccr;
	
	i2c_base->CR1 |= I2C_CR1_PE;
	
	xSemaphoreGive(api_lock);
}

#if 0
int8_t i2c::tx(uint16_t addr, void *buff, uint16_t size)
{
	
}

int8_t i2c::rx(uint16_t addr, void *buff, uint16_t size)
{
	
}

int8_t spi::tx(void *buff, uint16_t size, gpio *cs)
{
	ASSERT(buff);
	ASSERT(size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cs = cs;
	if(_cs)
		_cs->set(0);
	
	tx_buff = buff;
	tx_dma.src((uint8_t*)tx_buff);
	tx_dma.size(size);
	tx_dma.start_once(on_dma_tx, this);
	spi_list[_spi]->CR2 |= SPI_CR2_TXDMAEN;
	
	/* Task will be blocked during spi tx operation */
	/* irq_lock will be given later from irq handler */
	xSemaphoreTake(irq_lock, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	
	return irq_res;
}

int8_t spi::rx(void *buff, uint16_t size, gpio *cs)
{
	ASSERT(buff);
	ASSERT(size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_cs = cs;
	if(_cs)
		_cs->set(0);
	
	rx_buff = buff;
	rx_dma.src((uint8_t*)rx_buff);
	rx_dma.size(size);
	rx_dma.start_once(on_dma_rx, this);
	
	/* Setup tx for reception */
	tx_buff = rx_buff;
	tx_dma.src((uint8_t*)tx_buff);
	tx_dma.size(size);
	tx_dma.start_once(on_dma_tx, this);
	spi_list[_spi]->DR;
	spi_list[_spi]->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
	
	/* Task will be blocked during spi rx operation */
	/* irq_lock will be given later from irq handler */
	xSemaphoreTake(irq_lock, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	
	return irq_res;
}
#endif

int8_t i2c::exch(uint16_t addr, void *buff_tx, uint16_t size_tx, void *buff_rx,
	uint16_t size_rx)
{
	ASSERT(buff_tx);
	ASSERT(size_tx > 0);
	ASSERT(buff_rx);
	ASSERT(size_rx > 0);
	
	/* 10-bit addresses haven't supported yet */
	ASSERT(addr <= 127);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	_addr = addr;
	tx_buff = buff_tx;
	tx_size = size_tx;
	rx_buff = buff_rx;
	rx_size = size_rx;
	
	task = xTaskGetCurrentTaskHandle();
	i2c_list[_i2c]->CR1 |= I2C_CR1_START;
	
	// Task will be unlocked later from isr
	ulTaskNotifyTake(true, portMAX_DELAY);
	
	xSemaphoreGive(api_lock);
	
	return irq_res;
}

static void gpio_af_init(i2c::i2c_t i2c, gpio &gpio)
{
	GPIO_TypeDef *gpio_reg = gpio_priv::ports[gpio.port()];
	
	uint8_t pin = gpio.pin();
	/* Open drain type */
	gpio_reg->OTYPER |= GPIO_OTYPER_OT0 << pin;
	
	/* Configure alternate function */
	if(pin < 8)
	{
		gpio_reg->AFR[0] &= ~(0x0F << (pin * 4));
		gpio_reg->AFR[0] |= gpio_af_list[i2c] << (pin * 4);
	}
	else
	{
		gpio_reg->AFR[1] &= ~(0x0F << ((pin - 8) * 4));
		gpio_reg->AFR[1] |= gpio_af_list[i2c] << ((pin - 8) * 4);
	}
}

static void calc_clk(i2c::i2c_t i2c, uint32_t baud, uint8_t *freq,
	uint8_t *trise, uint32_t *ccr)
{
	uint32_t apb1_freq = rcc_get_freq(RCC_SRC_APB1);
	*freq = apb1_freq / 1000000;
	/* According to RM0090 page 853:
	"Bits 5:0 FREQ[5:0]: Peripheral clock frequency"
	2 Mhz is min and allowed and 50 Mhz is max allowed*/
	ASSERT(*freq >= 2 && *freq <= 50);
	
	/* According to RM0090 page 860:
	fPCLK1 must be at least 2 MHz to achieve Sm mode I2C frequencies. It must be
	at least 4 MHz to achieve Fm mode I2C frequencies. It must be a multiple of
	10MHz to reach the 400 kHz maximum I2C Fm mode clock */
	ASSERT(baud <= STANDARD_I2C_MAX_SPEED || *freq >= 2);
	ASSERT(baud < STANDARD_I2C_MAX_SPEED || *freq >= 4);
	
	if(baud <= STANDARD_I2C_MAX_SPEED)
	{
		*ccr = apb1_freq / (baud << 1);
		*ccr = MAX(*ccr, 4);
		*ccr &= ~(I2C_CCR_FS | I2C_CCR_DUTY);
		*trise = *freq + 1;
	}
	else
	{
		*ccr = apb1_freq / (baud * 25);
		*ccr = MAX(*ccr, 1);
		*ccr |= I2C_CCR_FS | I2C_CCR_DUTY;
		*trise = ((*freq * 300) / 1000) + 1;
	}
}

void i2c::on_dma_tx(dma *dma, dma::event_t event, void *ctx)
{
	if(event == dma::EVENT_HALF)
		return;
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_dma_tx);
#endif
	i2c *obj = static_cast<i2c *>(ctx);
	BaseType_t hi_task_woken = 0;
	
	if(event == dma::EVENT_CMPLT)
	{
#if configUSE_TRACE_FACILITY
		vTracePrint(ch1, "tx_hndlr");
#endif
		
		tx_hndlr(obj, &hi_task_woken);
	}
	else if(event == dma::EVENT_ERROR)
	{
#if configUSE_TRACE_FACILITY
		vTracePrint(ch1, "err_hndlr");
#endif
		
		err_hndlr(obj, RES_TX_FAIL, &hi_task_woken);
	}
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(hi_task_woken);
#endif
}

void i2c::on_dma_rx(dma *dma, dma::event_t event, void *ctx)
{
	if(event == dma::EVENT_HALF)
		return;
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_dma_rx);
#endif
	i2c *obj = static_cast<i2c *>(ctx);
	BaseType_t hi_task_woken = 0;
	
	if(event == dma::EVENT_CMPLT)
	{
#if configUSE_TRACE_FACILITY
		vTracePrint(ch1, "rx_hndlr");
#endif
		
		rx_hndlr(obj, &hi_task_woken);
	}
	else if(event == dma::EVENT_ERROR)
	{
#if configUSE_TRACE_FACILITY
		vTracePrint(ch1, "err_hndlr");
#endif
		
		err_hndlr(obj, RES_RX_FAIL, &hi_task_woken);
	}
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(hi_task_woken);
#endif
}

extern "C" void tx_hndlr(i2c *obj, BaseType_t *hi_task_woken)
{
	*hi_task_woken = 0;
	obj->tx_buff = NULL;
	obj->tx_size = 0;
	
	I2C_TypeDef *i2c_base = i2c_list[obj->_i2c];
	
	if(obj->rx_buff && obj->rx_size > 0)
	{
		/* Generate the second start to read the data */
		i2c_base->CR1 |= I2C_CR1_START;
		return;
	}
	/* Wait for last byte transmitting */
	while(!(i2c_base->SR1 & I2C_SR1_TXE));
	
	i2c_base->CR1 |= I2C_CR1_STOP;
	
	obj->irq_res = i2c::RES_OK;
	
	vTaskNotifyGiveFromISR(obj->task, hi_task_woken);
	portYIELD_FROM_ISR(*hi_task_woken);
}

extern "C" void rx_hndlr(i2c *obj, BaseType_t *hi_task_woken)
{
	*hi_task_woken = 0;
	obj->rx_buff = NULL;
	obj->rx_size = 0;
	
	i2c_list[obj->_i2c]->CR2 &= ~I2C_CR2_LAST;
	i2c_list[obj->_i2c]->CR1 |= I2C_CR1_STOP;
	
	obj->irq_res = i2c::RES_OK;
	
	vTaskNotifyGiveFromISR(obj->task, hi_task_woken);
	portYIELD_FROM_ISR(*hi_task_woken);
}

extern "C" void err_hndlr(i2c *obj, int8_t err, BaseType_t *hi_task_woken)
{
	*hi_task_woken = 0;
	
	obj->tx_dma.stop();
	obj->rx_dma.stop();
	
	obj->tx_buff = NULL;
	obj->tx_size = 0;
	obj->rx_buff = NULL;
	obj->rx_size = 0;
	
	//i2c_list[obj->_i2c]->CR2 &= ~I2C_CR2_LAST;
	i2c_list[obj->_i2c]->CR1 |= I2C_CR1_STOP;
	
	obj->irq_res = (i2c::res_t)err;
	
	vTaskNotifyGiveFromISR(obj->task, hi_task_woken);
	portYIELD_FROM_ISR(*hi_task_woken);
}

extern "C" void i2c_event_irq_hndlr(i2c *obj)
{
	I2C_TypeDef *i2c_base = i2c_list[obj->_i2c];
	uint16_t sr1 = (uint16_t)i2c_list[obj->_i2c]->SR1;
	
#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_i2c_event);
#endif
	
	if(sr1 & I2C_SR1_SB)
	{
#if configUSE_TRACE_FACILITY
		vTracePrint(ch1, "send address");
#endif
		
		/* Start condition is sent. Need to send device address */
		i2c_base->DR = (obj->_addr << 1) | (obj->tx_buff ? 0 : 1);
	}
	else if(sr1 & (I2C_SR1_ADDR | I2C_SR1_ADD10))
	{
		uint16_t sr2 = (uint16_t)i2c_base->SR2;
		/* Device address is sent. Need to send/receive data */
		if(obj->tx_buff)
		{
#if configUSE_TRACE_FACILITY
			vTracePrint(ch1, "start tx dma");
#endif
			
			obj->tx_dma.src(obj->tx_buff);
			obj->tx_dma.size(obj->tx_size);
			obj->tx_dma.start_once(obj->on_dma_tx, obj);
		}
		else if(obj->rx_buff)
		{
			if(obj->rx_size > 1)
				i2c_base->CR1 |= I2C_CR1_ACK;
			else
				i2c_base->CR1 &= ~I2C_CR1_ACK;
			
#if configUSE_TRACE_FACILITY
			vTracePrint(ch1, "start rx dma");
#endif
			
			obj->rx_dma.dst(obj->rx_buff);
			obj->rx_dma.size(obj->rx_size);
			obj->rx_dma.start_once(obj->on_dma_rx, obj);
		}
	}
	else if(sr1 & I2C_SR1_BTF)
	{
		/* Clear BTF flag */
		uint32_t dr = i2c_base->DR;
	}
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(0);
#endif
}

extern "C" void i2c_error_irq_hndlr(i2c *obj)
{
	I2C_TypeDef *i2c_base = i2c_list[obj->_i2c];
	
	uint32_t sr1 = i2c_base->SR1;
	uint32_t sr2 = i2c_base->SR2;
	uint32_t dr = i2c_base->DR;

#if configUSE_TRACE_FACILITY
	vTraceStoreISRBegin(isr_i2c_error);
#endif
	BaseType_t hi_task_woken = 0;
	
	
#if configUSE_TRACE_FACILITY
	vTracePrint(ch1, "Error irq");
#endif
	
	if(sr1 & I2C_SR1_AF)
	{
#if configUSE_TRACE_FACILITY
		vTracePrint(ch1, "AF irq");
#endif
		
		/* Error: no ACK from device */
		i2c_base->SR1 &= ~I2C_SR1_AF;
		err_hndlr(obj, i2c::RES_NO_ACK, &hi_task_woken);
	}
	else if(sr1 & I2C_SR1_OVR)
	{
		/* Error: overrun/underrun has happened */
		i2c_base->SR1 &= ~I2C_SR1_OVR;
		err_hndlr(obj, i2c::RES_RX_FAIL, &hi_task_woken);
	}
	else if(sr1 & I2C_SR1_ARLO)
	{
		/* Error: arbitration lost is detected */
		i2c_base->SR1 &= ~I2C_SR1_ARLO;
		err_hndlr(obj, i2c::RES_TX_FAIL, &hi_task_woken);
	}
	else if(sr1 & I2C_SR1_BERR)
	{
		/* Error: bus error is detected (misplaced start or stop condition) */
		i2c_base->SR1 &= ~I2C_SR1_BERR;
		err_hndlr(obj, i2c::RES_TX_FAIL, &hi_task_woken);
	}
#if configUSE_TRACE_FACILITY
	vTraceStoreISREnd(hi_task_woken);
#endif
}

extern "C" void I2C1_EV_IRQHandler(void)
{
	i2c_event_irq_hndlr(obj_list[i2c::I2C_1]);
}

extern "C" void I2C1_ER_IRQHandler(void)
{
	i2c_error_irq_hndlr(obj_list[i2c::I2C_1]);
}

extern "C" void I2C2_EV_IRQHandler(void)
{
	i2c_event_irq_hndlr(obj_list[i2c::I2C_2]);
}

extern "C" void I2C2_ER_IRQHandler(void)
{
	i2c_error_irq_hndlr(obj_list[i2c::I2C_2]);
}

#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
	defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
	defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
	defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
	defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
	defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
	defined(STM32F469xx) || defined(STM32F479xx)
extern "C" void I2C3_EV_IRQHandler(void)
{
	i2c_event_irq_hndlr(obj_list[i2c::I2C_3]);
}

extern "C" void I2C3_ER_IRQHandler(void)
{
	i2c_error_irq_hndlr(obj_list[i2c::I2C_3]);
}
#endif
