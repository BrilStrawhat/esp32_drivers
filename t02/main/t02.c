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
#include "driver/ledc.h"
#include <driver/dac.h>

#define LED1                    27u
#define LED2                    26u
#define GPIO_OUTPUT_PIN_SEL     ((1ULL << LED1) |\
                                 (1ULL << LED2))
#define MAX_FADE_TIME_MS        1000u
#define FREQ_HZ                 5000u
#define DUTY                    0xff

static void pwm_led_pulse_task(void *pvParameters) {
    esp_err_t rc = ESP_OK;
    ledc_channel_config_t *channel_conf = (ledc_channel_config_t*)pvParameters;

    while (true) {
        if ((rc = ledc_set_fade_with_time(channel_conf->speed_mode, channel_conf->channel,
                                          0, MAX_FADE_TIME_MS)) != ESP_OK) {
            printf("line %d: %d\n", (__LINE__ - 2), rc);
            return;
        }
        if ((rc = ledc_fade_start(channel_conf->speed_mode, channel_conf->channel,
                                  LEDC_FADE_WAIT_DONE)) != ESP_OK) {
            printf("line %d: %d\n", (__LINE__ - 2), rc);
            return;
        }
        if ((rc = ledc_set_fade_with_time(channel_conf->speed_mode, channel_conf->channel,
                                          DUTY, MAX_FADE_TIME_MS)) != ESP_OK) {
            printf("line %d: %d\n", (__LINE__ - 2), rc);
            return;
        }
        if ((rc = ledc_fade_start(channel_conf->speed_mode, channel_conf->channel,
                                  LEDC_FADE_WAIT_DONE)) != ESP_OK) {
            printf("line %d: %d\n", (__LINE__ - 2), rc);
            return;
        }
    }
}

static void dac_led_pulse_task() {
    esp_err_t rc = ESP_OK;

    while (true) {
        for (uint8_t volt = 120; volt < 200; volt++) {
            if ((rc = dac_output_voltage(DAC_CHANNEL_2, volt)) != ESP_OK) {
                printf("line %d: %d\n", (__LINE__ - 1), rc);
                return;
            }
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        for (uint8_t volt = 200; volt > 120; volt--) {
            if ((rc = dac_output_voltage(DAC_CHANNEL_2, volt)) != ESP_OK) {
                printf("line %d: %d\n", (__LINE__ - 1), rc);
                return;
            }
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
}

void app_main(void) {
    esp_err_t rc = ESP_OK;
    ledc_timer_config_t timer_conf;
    ledc_channel_config_t channel_conf;

    // Config the timer
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.duty_resolution = LEDC_TIMER_8_BIT;
    timer_conf.timer_num = LEDC_TIMER_0;
    timer_conf.freq_hz = FREQ_HZ;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    if ((rc = ledc_timer_config(&timer_conf)) != ESP_OK) {
        printf("line %d: %d\n", (__LINE__ - 1), rc);
        return;
    }
    channel_conf.gpio_num = LED1;
    channel_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_conf.channel = LEDC_CHANNEL_0;
    channel_conf.intr_type = LEDC_INTR_FADE_END;
    channel_conf.timer_sel = LEDC_TIMER_0; 
    channel_conf.duty = DUTY; 
    channel_conf.hpoint = 0; // idk, from example;
    if ((rc = ledc_channel_config(&channel_conf)) != ESP_OK) {
        printf("line %d: %d\n", (__LINE__ - 1), rc);
        return;
    }

    ledc_fade_func_install(0);

    if ((rc = dac_output_enable(DAC_CHANNEL_2)) != ESP_OK) {
        printf("line %d: %d\n", (__LINE__ - 1), rc);
        return;
    }

    xTaskCreate(dac_led_pulse_task, "dac task", 2048u, NULL, 1, 0);
    xTaskCreate(pwm_led_pulse_task, "pwm task", 2048u, &channel_conf, 1, 0);
}
