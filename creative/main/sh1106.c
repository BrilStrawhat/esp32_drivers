#include "driver/i2c.h"
#include "sh1106.h"

void sh1106_init(sh1106_t *display) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (display->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // command stream
    i2c_master_write_byte(cmd, 0xAE, true); // off
    i2c_master_write_byte(cmd, 0xD5, true); // clock div
    i2c_master_write_byte(cmd, 0x80, true);
    i2c_master_write_byte(cmd, 0xA8, true); // multiplex
    i2c_master_write_byte(cmd, 0x3f, true);
    i2c_master_write_byte(cmd, 0x8D, true); // charge pump
    i2c_master_write_byte(cmd, 0x14, true);
    i2c_master_write_byte(cmd, 0x10, true); // high column
    i2c_master_write_byte(cmd, 0xB0, true);
    i2c_master_write_byte(cmd, 0xC8, true);
    i2c_master_write_byte(cmd, 0x00, true); // low column
    i2c_master_write_byte(cmd, 0x10, true);
    i2c_master_write_byte(cmd, 0x40, true);
    i2c_master_write_byte(cmd, 0xA1, true); // segment remap
    i2c_master_write_byte(cmd, 0xA6, true);
    i2c_master_write_byte(cmd, 0x81, true); // contrast
    i2c_master_write_byte(cmd, 0xFF, true);
    i2c_master_write_byte(cmd, 0xAF, true); // on
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(display->port, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    for (uint8_t i = 0; i < 8; i++) {
        for (uint8_t j = 0; j < 128; j++) {
            display->clear_pages[i][j] = 0x00;
        }
    }
}

void sh1106_write_page(sh1106_t *display, uint8_t page, bool buffer_type) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (display->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x80, true); // single command
    i2c_master_write_byte(cmd, 0xB0 + page, true);
    i2c_master_write_byte(cmd, 0x40, true); // data stream
    if (buffer_type == 0)
        i2c_master_write(cmd, display->pages[page], 128, true);
    else 
        i2c_master_write(cmd, display->clear_pages[page], 128, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(display->port, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void sh1106_update(sh1106_t *display) {
    for (uint8_t i = 0; i < 8; i++)
        sh1106_write_page(display, i, 0);
}

void sh1106_clear_OLED(sh1106_t *display) {
    for (uint8_t i = 0; i < 8; i++)
        sh1106_write_page(display, i, 1);
}

void sh1106_clear(sh1106_t *display) {
    for (uint8_t i = 0; i < 8; i++) {
        for (uint8_t j = 0; j < 128; j++) {
            display->pages[i][j] = 0x00;
        }
    }
}

void sh1106_fill(sh1106_t *display) {
    for (uint8_t i = 0; i < 8; i++) {
        for (uint8_t j = 0; j < 128; j++) {
            display->pages[i][j] = 0xff;
        }
    }
}

void sh1106_set(sh1106_t *display, uint8_t x, uint8_t y, bool s) {
    const uint8_t i = y / 8;
    if (s) {
        display->pages[i][x] |= (1 << (y % 8));
    } else {
        display->pages[i][x] &= ~(1 << (y % 8));
    }
}


bool sh1106_get(sh1106_t *display, uint8_t x, uint8_t y) {
    return display->pages[y / 8][x] & (1 << (y % 8));
}

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

