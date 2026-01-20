#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"

typedef struct {
    int x;
    int y;
} joystick_data_t;


//SENDER CODE
// uint8_t receiver_mac[6] = {0x44, 0x1D, 0x64, 0xF6, 0x4C, 0x40};

// static const char *TAG = "ESP_NOW_SENDER";

// #define JOYSTICK_X_CHAN ADC_CHANNEL_6
// #define JOYSTICK_Y_CHAN ADC_CHANNEL_7

// adc_oneshot_unit_handle_t adc1_handle;

// void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//     ESP_LOGI(TAG, "Last packet sent status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
// }

// void joystick_send_task(void *args) {
//     joystick_data_t data;

//     while(1) {
//         ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK_X_CHAN, &data.x));
//         ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK_Y_CHAN, &data.y));

//         esp_err_t result = esp_now_send(receiver_mac, (uint8_t*)&data, sizeof(data));

//         if (result == ESP_OK) {
//             ESP_LOGI(TAG, "Data sent: X=%d, Y=%d", data.x, data.y);
//         } else {
//             ESP_LOGE(TAG, "Error sending data: %s", esp_err_to_name(result));
//         }

//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }

// void app_main() {
//     adc_oneshot_unit_init_cfg_t init_cfg = {
//         .unit_id = ADC_UNIT_1,
//     };
//     ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

//     adc_oneshot_chan_cfg_t chan_cfg  = {
//         .bitwidth = ADC_BITWIDTH_DEFAULT,
//         .atten = ADC_ATTEN_DB_12,
//     };

//     ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_X_CHAN, &chan_cfg));
//     ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_Y_CHAN, &chan_cfg));

//     esp_err_t ret = nvs_flash_init();

//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
//     ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
//     ESP_ERROR_CHECK(esp_wifi_start());

//     ESP_ERROR_CHECK(esp_now_init());
//     ESP_ERROR_CHECK(esp_now_register_send_cb((esp_now_send_cb_t)on_data_sent));

//     esp_now_peer_info_t peer_info = {};
//     memcpy(peer_info.peer_addr, receiver_mac, 6);
//     peer_info.channel = 1;
//     peer_info.encrypt = false;

//     if (esp_now_add_peer(&peer_info) != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to add peer");
//         return;
//     }

//     xTaskCreate(joystick_send_task, "joystick_send_task", 4096, NULL, 5, NULL);
// }




//===================================================================




//RECEIVER CODE
static const char *TAG = "ESP_NOW_RECEIVER";

void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    joystick_data_t *joystick_data = (joystick_data_t*)data;

    ESP_LOGI(TAG, "Data received from " MACSTR, MAC2STR(recv_info->src_addr));

    if (data_len == sizeof(joystick_data_t)) {
        ESP_LOGI(TAG, "Joystick Data - X: %d, Y: %d", joystick_data->x, joystick_data->y);
    } else {
        ESP_LOGE(TAG, "Received data length mismatch");
    }
}

void app_main() {
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    
    ESP_ERROR_CHECK(esp_now_register_recv_cb((esp_now_recv_cb_t)on_data_recv));
}