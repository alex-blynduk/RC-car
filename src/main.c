#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "JOYSTICK_TASK";

#define JOYSTICK_X_CHAN ADC_CHANNEL_6
#define JOYSTICK_Y_CHAN ADC_CHANNEL_7

adc_oneshot_unit_handle_t adc1_handle;

void joystick_task(void *args) {
    int x_raw, y_raw;

    while(1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK_X_CHAN, &x_raw));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK_Y_CHAN, &y_raw));

        ESP_LOGI(TAG, "Joystick X: %d, Y: %d", x_raw, y_raw);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main() {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_cfg  = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_X_CHAN, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_Y_CHAN, &chan_cfg));


    xTaskCreate(joystick_task, "joystick_task", 4096, NULL, 5, NULL);
}
