// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define LED1 27
#define LED2 26
#define LED3 33
#define GPIO_OUTPUT_PIN_SEL ((1ULL << LED1) |\
                             (1ULL << LED2) |\
                             (1ULL << LED3))

void app_main(void)
{
    gpio_config_t io_conf;

    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);

    while(1) {
        gpio_set_level(LED1, 0);
        gpio_set_level(LED2, 0);
        gpio_set_level(LED3, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED1, 1);
        gpio_set_level(LED2, 1);
        gpio_set_level(LED3, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
