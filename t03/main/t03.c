#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#define GPIO_OUTPUT_IO_0        27
#define GPIO_OUTPUT_IO_1        26
#define GPIO_OUTPUT_PIN_SEL     ((1ULL<<GPIO_OUTPUT_IO_0)\
                                | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0         39
#define GPIO_INPUT_IO_1         18
#define GPIO_INPUT_PIN_SEL      ((1ULL<<GPIO_INPUT_IO_0)\
                                | (1ULL<<GPIO_INPUT_IO_1))

#define BUF_LEN                 100

// idk, from guide https://www.lucadentella.it/en/2017/02/25/esp32-12-io-e-interrupts/
#define ESP_INTR_FLAG_DEFAULT   0 



static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t) arg;

    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void button_task(void *arg) {
    uint32_t io_num;
    bool led_status1 = false;
    bool led_status2 = false;

    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (io_num == GPIO_INPUT_IO_0) {
                led_status1 = !led_status1;
                gpio_set_level(GPIO_OUTPUT_IO_0, led_status1);
            }
            else {
                led_status2 = !led_status2;
                gpio_set_level(GPIO_OUTPUT_IO_1, led_status2);
            }
        }
    }
}

void app_main(void) {
    esp_err_t rc = ESP_OK;
    char buf[BUF_LEN];
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    if ((rc = gpio_config(&io_conf)) != ESP_OK) {
        printf("line %d: %d, %s\n", (__LINE__ - 1), rc, esp_err_to_name_r(rc, buf, BUF_LEN));
        return;
    }

    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    if ((rc = gpio_config(&io_conf)) != ESP_OK) {
        printf("line %d: %d, %s\n", (__LINE__ - 1), rc, esp_err_to_name_r(rc, buf, BUF_LEN));
        return;
    }
    if ((rc = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT)) != ESP_OK) {
        printf("line %d: %d, %s\n", (__LINE__ - 1), rc, esp_err_to_name_r(rc, buf, BUF_LEN));
        return;
    }
    if ((gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t))) == NULL) {
        printf("line %d: Not enought heap\n", (__LINE__ - 1));
        return;
    }
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    if ((rc = gpio_isr_handler_add(GPIO_INPUT_IO_0,
                                   gpio_isr_handler,
                                   (void*) GPIO_INPUT_IO_0)) != ESP_OK) {
        printf("line %d: %d, %s\n", (__LINE__ - 1), rc, esp_err_to_name_r(rc, buf, BUF_LEN));
        return;
    }
    if ((rc = gpio_isr_handler_add(GPIO_INPUT_IO_1,
                                   gpio_isr_handler,
                                   (void*) GPIO_INPUT_IO_1)) != ESP_OK) {
        printf("line %d: %d, %s\n", (__LINE__ - 1), rc, esp_err_to_name_r(rc, buf, BUF_LEN));
        return;
    }
}

