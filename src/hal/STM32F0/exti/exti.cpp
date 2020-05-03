#include <stddef.h>
#include "common/assert.h"
#include "exti.hpp"
#include "gpio/gpio.hpp"
#include "CMSIS/Device/STM32F0xx/Include/stm32f0xx.h"
#include "CMSIS/Include/core_cm0.h"

using namespace hal;

constexpr IRQn_Type irqn[gpio::pins] =
{
	EXTI0_1_IRQn, EXTI0_1_IRQn, EXTI2_3_IRQn, EXTI2_3_IRQn, EXTI4_15_IRQn,
	EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn,
	EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn,
	EXTI4_15_IRQn
};

static exti *obj_list[gpio::pins];

exti::exti(gpio &gpio, trigger_t trigger):
	_gpio(gpio),
	_trigger(trigger),
	_ctx(NULL),
	_cb(NULL)
{
	ASSERT(_trigger <= TRIGGER_BOTH);
	ASSERT(_gpio.mode() == gpio::mode::DI);
	
	// Enable clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	
	uint8_t pin = _gpio.pin();
	
	// Setup EXTI line source
	uint8_t exti_offset = (pin % SYSCFG_EXTICR1_EXTI1_Pos) *
		SYSCFG_EXTICR1_EXTI1_Pos;
	SYSCFG->EXTICR[pin / SYSCFG_EXTICR1_EXTI1_Pos] &= ~(SYSCFG_EXTICR1_EXTI0 <<
		exti_offset);
	SYSCFG->EXTICR[pin / SYSCFG_EXTICR1_EXTI1_Pos] |= _gpio.port() <<
		exti_offset;
	
	uint32_t line_bit = 1 << pin;
	// Setup EXTI mask regs
	EXTI->EMR &= ~line_bit;
	EXTI->IMR &= ~line_bit;
	
	// Setup EXTI trigger
	EXTI->RTSR |= line_bit;
	EXTI->FTSR |= line_bit;
	if(_trigger == TRIGGER_RISING)
		EXTI->FTSR &= ~line_bit;
	else if(_trigger == TRIGGER_FALLING)
		EXTI->RTSR &= ~line_bit;
	
	obj_list[pin] = this;
	
	NVIC_ClearPendingIRQ(irqn[pin]);
	NVIC_SetPriority(irqn[pin], 2);
	NVIC_EnableIRQ(irqn[pin]);
}

exti::~exti()
{
	uint8_t pin = _gpio.pin();
	
	NVIC_DisableIRQ(irqn[pin]);
	EXTI->IMR &= ~(1 << _gpio.pin());
	
	uint8_t exti_offset = (pin % SYSCFG_EXTICR1_EXTI1_Pos) *
		SYSCFG_EXTICR1_EXTI1_Pos;
	SYSCFG->EXTICR[pin / SYSCFG_EXTICR1_EXTI1_Pos] &= ~(SYSCFG_EXTICR1_EXTI0 <<
		exti_offset);
	
	obj_list[pin] = NULL;
}

void exti::cb(cb_t cb, void *ctx)
{
	_cb = cb;
	_ctx = ctx;
}

void exti::on()
{
	ASSERT(_cb);
	
	uint8_t pin = _gpio.pin();
	
	EXTI->PR |= 1 << pin;
	EXTI->IMR |= 1 << pin;
	
	NVIC_ClearPendingIRQ(irqn[pin]);
}

void exti::off()
{
	EXTI->IMR &= ~(1 << _gpio.pin());
}

void exti::trigger(trigger_t trigger)
{
	ASSERT(trigger <= TRIGGER_BOTH);
	
	_trigger = trigger;
	uint32_t line_bit = 1 << _gpio.pin();
	
	EXTI->RTSR |= line_bit;
	EXTI->FTSR |= line_bit;
	if(_trigger == TRIGGER_RISING)
		EXTI->FTSR &= ~line_bit;
	else if(_trigger == TRIGGER_FALLING)
		EXTI->RTSR &= ~line_bit;
}

extern "C" void exti_irq_hndlr(hal::exti *obj)
{
	EXTI->PR = 1 << obj->_gpio.pin();
	
	if(obj->_cb)
		obj->_cb(obj, obj->_ctx);
}

extern "C" void EXTI0_1_IRQHandler(void)
{
	uint32_t pr = EXTI->PR;
	
	if(pr & EXTI_PR_PR0)
		exti_irq_hndlr(obj_list[EXTI_PR_PR0_Pos]);
	else if(pr & EXTI_PR_PR1)
		exti_irq_hndlr(obj_list[EXTI_PR_PR1_Pos]);
}

extern "C" void EXTI2_3_IRQHandler(void)
{
	uint32_t pr = EXTI->PR;
	
	if(pr & EXTI_PR_PR2)
		exti_irq_hndlr(obj_list[EXTI_PR_PR2_Pos]);
	else if(pr & EXTI_PR_PR3)
		exti_irq_hndlr(obj_list[EXTI_PR_PR3_Pos]);
}

extern "C" void EXTI4_15_IRQHandler(void)
{
	uint32_t pr = EXTI->PR;
	
	for(uint8_t i = EXTI_PR_PR4_Pos; i < EXTI_PR_PR16_Pos; i++)
	{
		if(pr & (1 << i))
		{
			exti_irq_hndlr(obj_list[i]);
			break;
		}
	}
}