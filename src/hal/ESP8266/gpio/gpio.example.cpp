#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio/gpio.hpp"
#include "esp_event_loop.h"
#include "esp_wifi.h"

static void main_task(void *pvParameters)
{
	hal::gpio *led = static_cast<hal::gpio *>(pvParameters);
	
	while(1)
	{
		led->toggle();
		vTaskDelay(500);
	}
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	printf("event_handler() event_id=%d\n", event->event_id);
	switch(event->event_id)
	{
		case SYSTEM_EVENT_AP_STACONNECTED:
			printf("STA %02x:%02x:%02x:%02x:%02x:%02x has connected, aid = %d",
				MAC2STR(event->event_info.sta_connected.mac),
				event->event_info.sta_connected.aid);
			break;
		case SYSTEM_EVENT_AP_STADISCONNECTED:
			printf("STA %02x:%02x:%02x:%02x:%02x:%02x has disconnected, aid = %d",
				MAC2STR(event->event_info.sta_disconnected.mac),
				event->event_info.sta_disconnected.aid);
			break;
		default: return ESP_OK;
	}
	return ESP_OK;
}

static void wifi_init_softap(void)
{
	tcpip_adapter_init();
	esp_event_loop_init(event_handler, NULL);
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&wifi_init_config);
	esp_wifi_set_storage(WIFI_STORAGE_RAM);
	
	wifi_config_t wifi_config = {};
	wifi_config.ap.max_connection = 5;
	strcpy((char *)wifi_config.ap.ssid, "esp8266-ap");
	wifi_config.ap.ssid_len = strlen("esp8266-ap");
	strcpy((char *)wifi_config.ap.password, "mytestpass");
	//wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
	wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	
	esp_wifi_set_mode(WIFI_MODE_AP);
	esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
	esp_wifi_start();
}

extern "C" void app_main(void)
{
	static hal::gpio blue_led(0, 2, hal::gpio::mode::DO, 1);
	
	wifi_init_softap();
	
	xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE * 1, &blue_led,
		tskIDLE_PRIORITY + 1, NULL);
}
