#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"


#define I2C_SLAVE_SCL_IO           19
#define I2C_SLAVE_SDA_IO           18
#define DATA_SEND_LEN               2
#define DATA_RECEIVE_LEN            6

#define I2C_SLAVE_NUM              I2C_NUM_0

static const char *TAG = "I2C_MASTER";


i2c_master_dev_handle_t dev_handle;

i2c_master_bus_handle_t bus_handle;

void i2c_master_init() {
    i2c_master_bus_config_t i2c_mst_config_1 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_SLAVE_NUM,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    i2c_new_master_bus(&i2c_mst_config_1, &bus_handle);
}

void i2c_slave_device_init() {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x58,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
}

void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 1024 * 2, 0, 0, NULL, 0);
}

void app_main () {
    uint8_t data_wr[DATA_SEND_LEN] = {0x27, 0x9F};
    uint8_t buffer[DATA_RECEIVE_LEN] = {0, 0, 0, 0, 0, 0};
    int trys = 0;
    int32_t temperature = 0;
    uint8_t temperature_value[2];

    uart_init();
    i2c_master_init();
    i2c_slave_device_init();

    while(1) {
        while (trys <= 3) {
            if (i2c_master_transmit(dev_handle, data_wr, DATA_SEND_LEN, 500) == ESP_OK) {
                vTaskDelay(55/ portTICK_PERIOD_MS);
                if (i2c_master_receive(dev_handle, buffer, DATA_RECEIVE_LEN, 500) == ESP_OK) {
                    temperature = (buffer[2] << 24) | (buffer[3] << 16) | (buffer[4] << 8) | buffer[5];
  
                    uart_write_bytes(UART_NUM_0, "Temperatura: ", 13);
                    
                    temperature_value[0] = temperature/1000 + 48;
                    temperature_value[1] = (temperature/100) % 10 + 48;
                    uart_write_bytes(UART_NUM_0, temperature_value, 2);
                    uart_write_bytes(UART_NUM_0, "\n", 1);
                    break;

                }
                else {
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    trys++;
                }
            }
            else {
                if (trys == 3) {
                    uart_write_bytes(UART_NUM_0, "Comunicacion terminada, el periferico no responde\n", 49);
                    while(1) {
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }
                else {
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    trys++;
                }
                
            }
        }
        trys = 0;
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}