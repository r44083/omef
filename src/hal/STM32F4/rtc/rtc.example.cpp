// Example for STM32F4DISCOVERY development board

#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "rtc/rtc.hpp"
#include "systick/systick.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

static void rtc_cb(struct tm time, void *ctx);

static void heartbeat_task(void *pvParameters)
{
	gpio *green_led = (gpio *)pvParameters;
	while(1)
	{
		green_led->toggle();
		
		struct tm time = rtc::get();
		
		vTaskDelay(500);
	}
}

static void rtc_cb(struct tm time, void *ctx)
{
	// Handle rtc events here
}

int main(void)
{
	systick::init();
	static gpio green_led(3, 12, gpio::mode::DO, 0);
	
	rtc::init(rtc::CLK_LSI);
	struct tm time = {};
	time.tm_year = 119;
	time.tm_mday = 1;
	rtc::set(time);
	
	rtc::set_alarm_cb(rtc_cb, NULL);
	hal::rtc::set_alarm({.tm_sec = 1});
	
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE,
		&green_led, 1, NULL);
	
	vTaskStartScheduler();
}