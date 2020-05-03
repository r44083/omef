#include "systick.hpp"
#include "rcc/rcc.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"
#include "CMSIS/Include/core_cm4.h"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

void hal::systick_init(void)
{
	uint32_t systick_freq = rcc_get_freq(RCC_SRC_AHB);
	SysTick_Config(systick_freq / configTICK_RATE_HZ);
}

uint32_t hal::systick_get_ms(void)
{
	return xTaskGetTickCount();
}

uint32_t hal::systick_get_past(uint32_t start)
{
	return xTaskGetTickCount() - start;
}

/* Implemented in third_party/FreeRTOS/port.c
extern "C" void SysTick_Handler(void)
{
	systick_cnt++;
}
*/