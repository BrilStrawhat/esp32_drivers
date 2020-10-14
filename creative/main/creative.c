#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"
#include "sh1106.h"
#include "font6x8.h"
#include "dht11.h"
#include "beep.h"
#include "adxl345.h"

#define I2C_SDA 21
#define I2C_SCL 22
#define EN_OLED 32
#define I2C_ADDR SH1106_DEFAULT_ADDR
#define I2C_PORT SH1106_DEFAULT_PORT

#define GPIO_INPUT_IO_0         39
#define GPIO_INPUT_IO_1         18
#define GPIO_INPUT_PIN_SEL      ((1ULL<<GPIO_INPUT_IO_0)\
                                | (1ULL<<GPIO_INPUT_IO_1))
// idk, from guide https://www.lucadentella.it/en/2017/02/25/esp32-12-io-e-interrupts/
#define ESP_INTR_FLAG_DEFAULT   0 

static xQueueHandle gpio_evt_queue = NULL;
static TaskHandle_t show_temperature_handle = NULL;
static TaskHandle_t show_humidity_handle = NULL;
static bool is_reversed = 0;
static sh1106_t display;

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

static void show_temperature_task(void *display) {
    static uint8_t temp = 0;
    char *str = NULL;

    while (true) {
        read_tmp_hmd(&temp, NULL);
        asprintf(&str, "Temperature: %dC", temp);
        sh1106_clear(display);
        sh1106_str_in_display(display, str);
        sh1106_update(display);
        free(str);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static void show_humidity_task(void *display) {
    static uint8_t hmd = 0;
    char *str = NULL;

    while (true) {
        read_tmp_hmd(NULL, &hmd);
        asprintf(&str, "Humidity: %d%%", hmd);
        sh1106_clear(display);
        sh1106_str_in_display(display, str);
        sh1106_update(display);
        free(str);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static void show_temperature(sh1106_t *display) {
    vTaskSuspend(show_humidity_handle);
    vTaskResume(show_temperature_handle);
    play_beep();
}

static void show_humidity(sh1106_t *display) {
    vTaskSuspend(show_temperature_handle);
    vTaskResume(show_humidity_handle);
    play_beep();
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t) arg;

    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void button_task(void *display) {
    uint32_t io_num;

    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (io_num == GPIO_INPUT_IO_0) {
                show_temperature((sh1106_t*)display);
            }
            else {
                show_humidity((sh1106_t*)display);
            }
        }
    }
}

void buttons_init(sh1106_t *display) {
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(button_task, "button_task", 2048, (void*)display, 1, NULL);
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler,
                         (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler,
                         (void*) GPIO_INPUT_IO_1);
    xTaskCreate(show_temperature_task, "show_temperature_task", 2048,
                (void*)display, 3, &show_temperature_handle);
    vTaskSuspend(show_temperature_handle);
    xTaskCreate(show_humidity_task, "show_humidity_task", 2048,
                (void*)display, 3, &show_humidity_handle);
    vTaskSuspend(show_humidity_handle);
}

void adxl345_acceleration_reverse_task(void *spi) {
	int16_t accs[3];

	while (1) {
		adxl345_read_acceleration(spi, accs);
        if (accs[1] >= 200) {
            if (is_reversed == 0)
                sh1106_reverse(&display);

        }
        if (accs[1] <= -200) {
            if (is_reversed == 1)
                sh1106_reverse(&display);
        }
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

}

static void OLED_power_on(void) {
    gpio_set_direction(EN_OLED, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_OLED, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

static void sh1106_put_char_in_disply(char chr, sh1106_t *display,
                               uint8_t page, uint8_t *cursor) {
    if (chr < 32 || chr > 126) {
        printf("Invalid char in string for OLED: '%c' ASCII code: %d\n", chr, chr);
        exit(-1);
    }
    for (int i = 0; i < 6; i++) {
        display->pages[page][(*cursor)++] = font6x8[(chr - 32) * 6 + i];
    }
}

static void sh1106_str_in_display(sh1106_t *display, char *str) {
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
        sh1106_put_char_in_disply(str[i], display, page, &cursor);
    }
} 

/* You should clean display and write on it again
* Did not find better solution because of bug, could not create another 
* sh1106_t variable, for demonstation run code from 110 to 133 line */
void sh1106_reverse(sh1106_t *display) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // command link initialize.

    i2c_master_start(cmd); // start bit.
    i2c_master_write_byte(cmd, (display->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // command stream
    if (is_reversed == 0) {
        i2c_master_write_byte(cmd, 0xC0, true);
        i2c_master_write_byte(cmd, 0xA0, true);
        is_reversed = 1;
    }
    else {
        i2c_master_write_byte(cmd, 0xC8, true);
        i2c_master_write_byte(cmd, 0xA1, true);
        is_reversed = 0;
    }
    i2c_master_stop(cmd); // stop bit.
    ESP_ERROR_CHECK(i2c_master_cmd_begin(display->port, cmd, 10 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    sh1106_update(display);
}

void app_main(void) {
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_device_interface_config_t devcfg = {
		.clock_speed_hz = 1000000, // 1Mbps. nearest clock will be chosen.
        .mode = 3, // CPOL=1, CPHA=1
        .spics_io_num = PIN_NUM_CS,
		.command_bits = 8, // ADXL345 always takes 1+7 bit command (address).
        .queue_size = 1 
    };

    ret = spi_bus_initialize(ADXL_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(ADXL_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    display.addr = I2C_ADDR;
    display.port = I2C_PORT;
    OLED_power_on();
    init_i2c();
    sh1106_init(&display);
    dht_init();
    buttons_init(&display);
    beep_init();
	adxl345_start(spi);

	xTaskCreate(adxl345_acceleration_reverse_task, "adxl345_acceleration_reverse_task",
		2048u, (void*)spi, 1, 0);
    sh1106_clear(&display);
    sh1106_update(&display);
    sh1106_str_in_display(&display, "Usage: press the  right button to   see the temperatu-re,"
            " press the leftbutton to see the humidity");
    sh1106_update(&display);
}
