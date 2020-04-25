#pragma once

#include "FreeRTOS.h"
#include "semphr.h"

namespace freertos
{
// Take semaphore and release automatically in destructor
class semaphore_take
{
	public:
		semaphore_take(const SemaphoreHandle_t &semaphore,
			TickType_t ticks_to_wait):
			_semaphore(semaphore)
		{
			is_taken = xSemaphoreTake(_semaphore, ticks_to_wait) == pdTRUE;
		}
		
		~semaphore_take()
		{
			xSemaphoreGive(_semaphore);
		}
		
		bool is_taken;
	
	private:
		const SemaphoreHandle_t &_semaphore;
};
}
