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
#include "driver/ledc.h"
#include "driver/gpio.h"

#ifndef ABS
#define ABS(x) ((x) > 0) ? (x) : -(x)
#endif

#define IN1_GPIO 25
#define IN2_GPIO 26
#define IN3_GPIO 32
#define IN4_GPIO 33
#define ENA_GPIO 27
#define ENA_CHANNEL LEDC_CHANNEL_0
#define ENB_GPIO 14
#define ENB_CHANNEL LEDC_CHANNEL_1

#define LOWER_ZERO_THRESHHOLD 1700
#define UPPER_ZERO_THRESHHOLD 1970

static const char *TAG = "ESP_NOW_RECEIVER";

QueueHandle_t data_queue;

void motors_control_task(void *args) {
    joystick_data_t data;

    while (1) {
        if (xQueueReceive(data_queue, &data, portMAX_DELAY)) {
            int Vx = 0, Vy = 0, Vl = 0, Vr = 0;
            bool is_forward_l = true, is_forward_r = true;

            if (data.x < LOWER_ZERO_THRESHHOLD) {
                Vx = (ABS(data.x - UPPER_ZERO_THRESHHOLD) * 255) / UPPER_ZERO_THRESHHOLD;
                is_forward_l = false;
                is_forward_r = false;
            } else if (data.x > UPPER_ZERO_THRESHHOLD) {
                Vx = ((data.x - UPPER_ZERO_THRESHHOLD) * 255) / (4095 - UPPER_ZERO_THRESHHOLD);
                is_forward_l = true;
                is_forward_r = true;
            }

            if (data.y < LOWER_ZERO_THRESHHOLD) {
                Vy = (ABS(data.y - LOWER_ZERO_THRESHHOLD) * 255) / LOWER_ZERO_THRESHHOLD;
                
                Vl = Vx - Vy / 2;
                
                if (Vl < 0) {
                    Vl = -1 * Vl;
                    is_forward_l = !is_forward_l;
                }

                Vr = Vx + Vy / 2;

                if (Vr > 255) {
                    Vr = 255;
                }
            } else if (data.y > UPPER_ZERO_THRESHHOLD) {
                Vy = ((data.y - UPPER_ZERO_THRESHHOLD) * 255) / (4095 - UPPER_ZERO_THRESHHOLD);

                Vr = Vx - Vy / 2;

                if (Vr < 0) {
                    Vr = -1 * Vr;
                    is_forward_r = !is_forward_r;
                }

                Vl = Vx + Vy / 2;

                if (Vl > 255) {
                    Vl = 255;
                }
            }

            if (Vl && is_forward_l) {
                gpio_set_level(IN1_GPIO, 1);
                gpio_set_level(IN2_GPIO, 0);
            } else {
                gpio_set_level(IN1_GPIO, 0);
                gpio_set_level(IN2_GPIO, 1);
            }

            if (Vr && is_forward_r) {
                gpio_set_level(IN3_GPIO, 1);
                gpio_set_level(IN4_GPIO, 0);
            } else {
                gpio_set_level(IN3_GPIO, 0);
                gpio_set_level(IN4_GPIO, 1);
            }

            ledc_set_duty(LEDC_LOW_SPEED_MODE, ENA_CHANNEL, Vl);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, ENA_CHANNEL);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, ENB_CHANNEL, Vr);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, ENB_CHANNEL);
        }
    }

}

void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    if (data_len == sizeof(joystick_data_t)) {
        joystick_data_t temp_data;
        memcpy(&temp_data, data, sizeof(joystick_data_t));
        
        if (xQueueSend(data_queue, &temp_data, 0) != pdTRUE) {
            ESP_LOGE(TAG, "The queue is full");
        } else {
            ESP_LOGI(TAG, "Joystick Data - X: %d, Y: %d", temp_data.x, temp_data.y);
        }

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

    data_queue = xQueueCreate(15, sizeof(joystick_data_t));
    if (data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create data queue");
        return;
    }

    xTaskCreate(motors_control_task, "motors_control_task", 4096, NULL, 5, NULL);

    gpio_reset_pin(IN1_GPIO);
    gpio_reset_pin(IN2_GPIO);
    gpio_reset_pin(IN3_GPIO);
    gpio_reset_pin(IN4_GPIO);

    gpio_set_direction(IN1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN3_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN4_GPIO, GPIO_MODE_OUTPUT);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel_ENA = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ENA_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ENA_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel_ENA);

    ledc_channel_config_t ledc_channel_ENB = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ENB_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ENB_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel_ENB);

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