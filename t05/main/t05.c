#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/uart.h"

#define UART_TX 17
#define UART_RX 16
#define BUF_SIZE 1024

void app_main(void) {
    char *msg = "\e[41mRED\e[0m \e[42mGREEN\e[0m \e[44mBLUE\e[0m \e[0mDEFAULT\r\n";
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_TX, UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, BUF_SIZE, 0, 0, NULL, 0);
    uart_write_bytes(UART_NUM_1, msg, strlen(msg));
}
