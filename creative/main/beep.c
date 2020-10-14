#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include <driver/dac.h>
#include <math.h>

void play_beep() {
    // for (int volt = 0; volt < 256; volt+=32) {
        // dac_output_voltage(DAC_CHANNEL_1, volt);
        // vTaskDelay(50 / portTICK_PERIOD_MS);
    // }
    for (int time = 0; time < 50; time++) {
        for(int i=0; i < 256; i++)
            dac_output_voltage(DAC_CHANNEL_1, i);
        for(int i=254; i > 0; i--)
            dac_output_voltage(DAC_CHANNEL_1, i);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

esp_err_t beep_init(void) {
    return dac_output_enable(DAC_CHANNEL_1);
}
