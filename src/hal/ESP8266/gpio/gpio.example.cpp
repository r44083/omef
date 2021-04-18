// Example for ESP8266

// Temporary workaround to fix build error: https://github.com/espressif/ESP8266_RTOS_SDK/issues/1083
extern "C" void pthread_cond_init(void) {}
extern "C" void pthread_cond_broadcast(void) {}
extern "C" void pthread_cond_wait(void) {}

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "gpio/gpio.hpp"

static void main_task(void *pvParameters)
{
    hal::gpio *led = static_cast<hal::gpio *>(pvParameters);
    
    while(1)
    {
        led->toggle();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

static void wifi_event_handler(void *ctx, esp_event_base_t event_base,
    int32_t event_id, void* event_data)
{
    printf("wifi_event_handler() event_id=%d\n", event_id);
    
    switch(event_id)
    {
        case WIFI_EVENT_AP_STACONNECTED:
        {
            wifi_event_ap_staconnected_t *event =
                static_cast<wifi_event_ap_staconnected_t *>(event_data);
            printf("station " MACSTR " join, AID=%d\n", MAC2STR(event->mac), event->aid);
        }
        break;
        
        case WIFI_EVENT_AP_STADISCONNECTED:
        {
            wifi_event_ap_stadisconnected_t *event =
                static_cast<wifi_event_ap_stadisconnected_t *>(event_data);
            printf("station " MACSTR " leave, AID=%d\n", MAC2STR(event->mac), event->aid);
        }
        break;
    }
}

static void wifi_init_softap(void)
{
    tcpip_adapter_init();
    
    esp_event_loop_create_default();
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    
    wifi_config_t wifi_cfg = {};
    wifi_cfg.ap.max_connection = 4;
    strcpy((char *)wifi_cfg.ap.ssid, "esp8266-ap");
    wifi_cfg.ap.ssid_len = sizeof("esp8266-ap") - 1;
    strcpy((char *)wifi_cfg.ap.password, "mytestpass");
    //wifi_cfg.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;
    
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_cfg);
    esp_wifi_start();
}

// Baud rate is 74880 or 115200
extern "C" void app_main()
{
    static hal::gpio blue_led(0, 2, hal::gpio::mode::DO, 1);
    
    wifi_init_softap();
    
    xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE, &blue_led,
        tskIDLE_PRIORITY + 1, NULL);
}
