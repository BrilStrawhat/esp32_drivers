/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include <driver/dac.h>
#include <math.h>

void play_cool_bit() {
    while (true) {
        for (int volt = 0; volt < 256; volt+=32) {
            dac_output_voltage(DAC_CHANNEL_1, volt);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        for (int time = 0; time < 50; time++) {
            for(int i=0; i < 256; i++)
                dac_output_voltage(DAC_CHANNEL_1, i);
            for(int i=254; i > 0; i--)
                dac_output_voltage(DAC_CHANNEL_1, i);
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }
}

void app_main(void)
{
    esp_err_t rc = ESP_OK;

    if ((rc = dac_output_enable(DAC_CHANNEL_1)) != ESP_OK) {
        printf("line %d: %d\n", (__LINE__ - 1), rc);
        return;
    }
    play_cool_bit();
}
