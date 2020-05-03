#include "common/assert.h"
#include "adc/adc.hpp"
#include "dma/dma.hpp"
#include "gpio/gpio.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

static void main_task(void *pvParameters)
{
	gpio *green_led = (gpio *)pvParameters;
	while(1)
	{
		green_led->toggle();
		vTaskDelay(500);
	}
}

void adc::adc_clbk(adc *adc, adc::adc_ch_t ch, float val, void *ctx)
{
	// Handle adc value here
}

int main(void)
{
	static gpio green_led(2, 9, gpio::mode::DO, 0);
	static gpio adc_gpio(0, 0, gpio::mode::AN);
	
	static dma adc_dma(dma::DMA_1, dma::DMA_CH_1,
		dma::DMA_DIR_PERIPH_TO_MEM, dma::DMA_INC_SIZE_16);
	
	adc::adc_ch_t adc_chnls[] = {adc::ADC_CH_0};
	
	/* Need to capture 32 samples per each 10 ms (100 Hz). So need to start ADC
	conversations with 3200 Hz frequency */
	static adc adc_1(adc::ADC_1, adc_chnls,
		sizeof(adc_chnls) / sizeof(adc_chnls[0]), adc::ADC_TIM_3, adc_dma,
		adc::ADC_RESOL_12BIT, 3200, 32);
	
	adc_1.add_clbk(adc_chnls[0], adc_clbk, NULL);
	adc_1.enable(true);
	
	ASSERT(xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE * 1,
		&green_led, tskIDLE_PRIORITY + 1, NULL) == pdPASS);
	
	vTaskStartScheduler();
}