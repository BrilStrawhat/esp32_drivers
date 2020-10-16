#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "sh1106.h"
#include "font6x8.h"

#define I2C_SDA 21
#define I2C_SCL 22
#define EN_OLED 32
#define I2C_ADDR SH1106_DEFAULT_ADDR
#define I2C_PORT SH1106_DEFAULT_PORT

bool i2c_address_scanner(uint8_t i2c_port, i2c_config_t i2c_conf) {
    bool address_found = false;
    esp_err_t rc = ESP_OK;

    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, I2C_MODE_MASTER,
                                       0, 0, 0));
    for (int i = 0; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, i, true));
        ESP_ERROR_CHECK(i2c_master_stop(cmd));
        if ((rc = i2c_master_cmd_begin(i2c_port, cmd,
                                 10 / portTICK_PERIOD_MS)) == ESP_OK)
        {
            address_found = true;
            printf("address = %#04x\n", i);
        }
        else if (rc != ESP_ERR_TIMEOUT && rc != ESP_FAIL)
            printf("error number %#04x\n", rc);
        i2c_cmd_link_delete(cmd);
    }
    i2c_driver_delete(i2c_port);
    if (address_found == true)
        return true;
    else {
        printf("No address found\n");
        return false;
    }
}

static void OLED_power_on(void) {
    gpio_set_direction(EN_OLED, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_OLED, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void init_i2c() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000
    };
    i2c_param_config(I2C_PORT, &i2c_config);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static void put_char_in_disply(char chr, sh1106_t *display,
                               uint8_t page, uint8_t *cursor) {
    if (chr < 32 || chr > 126) {
        printf("Invalid char in string for OLED: '%c' ASCII code: %d\n", chr, chr);
        exit(-1);
    }
    for (int i = 0; i < 6; i++) {
        display->pages[page][(*cursor)++] = font6x8[(chr - 32) * 6 + i];
    }
}

static void str_in_display(sh1106_t *display, char *str) {
    uint8_t page = 0;
    uint8_t cursor = 0; 

    for (int i = 0; str[i] != '\0'; i++, cursor++) {
        if (cursor >= 122) {
            cursor = 0;
            page++;
            if (page == 7) {
                printf("Too long str to display it on OLED, max size of str is 142");
                exit(-1);
            }
        }
        put_char_in_disply(str[i], display, page, &cursor);
    }
    display->changes = 0xff;
} 

void sh1106_send_cmd(sh1106_t *display, uint8_t cmd_byte) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (display->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // command stream
    i2c_master_write_byte(cmd, cmd_byte, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(display->port, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// /* stack overflow error */
// static void wtf(sh1106_t *display) {
    // printf("%d\n", display->addr);
// }

// void app_main(void) {


    // sh1106_t display;
    // sh1106_t display_temp;

    // display.addr = I2C_ADDR;
    // display.port = I2C_PORT;

    // display_temp.addr = 1;
    // // display_temp.port = I2C_PORT;
    // init_i2c();
    // sh1106_init(&display);
    // wtf(&display);
    // wtf(&display_temp);

    // printf("Done!\n");
// }

void app_main(void) {
    OLED_power_on();
    sh1106_t display;

    display.addr = I2C_ADDR;
    display.port = I2C_PORT;
    init_i2c();
    sh1106_init(&display);

    sh1106_clear(&display);
    str_in_display(&display, "Hello world!");
    sh1106_update(&display);
}
