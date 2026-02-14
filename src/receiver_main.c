#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "joystick_data.h"

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

            Vl = Vx;
            Vr = Vx;

            if (data.y < LOWER_ZERO_THRESHHOLD) {
                Vy = (ABS(data.y - LOWER_ZERO_THRESHHOLD) * 255) / LOWER_ZERO_THRESHHOLD;
                
                Vl -= Vy / 2;
                
                if (Vl < 0) {
                    Vl = -1 * Vl;
                    is_forward_l = !is_forward_l;
                }

                Vr += Vy / 2;

                if (Vr > 255) {
                    Vr = 255;
                }
            } else if (data.y > UPPER_ZERO_THRESHHOLD) {
                Vy = ((data.y - UPPER_ZERO_THRESHHOLD) * 255) / (4095 - UPPER_ZERO_THRESHHOLD);

                Vr -= Vy / 2;

                if (Vr < 0) {
                    Vr = -1 * Vr;
                    is_forward_r = !is_forward_r;
                }

                Vl += Vy / 2;

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