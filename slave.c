#include "driver/i2c_slave.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

//Los pines para esp en modo esclavo
#define I2C_SLAVE_SCL_IO           19
#define I2C_SLAVE_SDA_IO           18

//los pines para esp en modo maestro
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21

//los numeros de los buses i2c
#define I2C_SLAVE_NUM              I2C_NUM_1
#define I2C_MASTER_NUM              I2C_NUM_0

//bits de temperatura ready o de temperatura solicitada
#define I2C_GET_TEMPERATURE BIT0
#define I2C_TEMPERATURE_READY BIT1

#define DATA_LENGTH 2

QueueHandle_t s_receive_queue;

//Para controlar cuando se deba hacer una medicion de temperatura
EventGroupHandle_t i2c_event_group;

//El que controla el esp32 como slave, la comunicacion entre esp's 
i2c_slave_dev_handle_t slave_handle;

i2c_master_bus_handle_t bus_handle;


i2c_master_dev_handle_t dev_handle;

static const char *TAG = "I2C_SLAVE";

int32_t temperature = 0;

long signed int DIG_T1 = 27698;
long signed int DIG_T2= -410;
long signed int DIG_T3= 189;

static IRAM_ATTR bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel, const i2c_slave_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);

    return high_task_wakeup == pdTRUE;
}

void i2c_master_init() {
    i2c_master_bus_config_t i2c_mst_config_1 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        .i2c_port = I2C_MASTER_NUM,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config_1, &bus_handle));
    
}

void i2c_slaves_devices_init() {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x76,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}

void esp_slave_init () {
    i2c_slave_config_t i2c_slv_config = {
        .i2c_port = I2C_SLAVE_NUM,
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_SLAVE_NUM,
        .send_buf_depth = 256,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .slave_addr = 0x58,
        .flags = {
            #if SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE
                    .stretch_en = 1,  // Activar estiramiento de reloj
            #endif
            #if SOC_I2C_SLAVE_SUPPORT_BROADCAST
                    .broadcast_en = 0,
            #endif
            #if SOC_I2C_SLAVE_SUPPORT_I2CRAM_ACCESS
                    .access_ram_en = 0,
            #endif
            #if SOC_I2C_SLAVE_SUPPORT_SLAVE_UNMATCH
                    .slave_unmatch_en = 0,
            #endif
        },
    };

    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config, &slave_handle));

    s_receive_queue = xQueueCreate(1, sizeof(i2c_slave_rx_done_event_data_t));
    i2c_slave_event_callbacks_t cbs = {
        .on_recv_done = i2c_slave_rx_done_callback,
    };
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(slave_handle, &cbs, s_receive_queue));
}


long signed int BME280_compensate_T_double(long signed int adc_T) {
    long signed int var1, var2;
    var1 = ((((adc_T>>3) - ((long signed int)DIG_T1<<1))) * ((long signed int)DIG_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((long signed int)DIG_T1)) * ((adc_T>>4) - ((long signed int)DIG_T1))) >> 12) * ((long signed int)DIG_T3)) >> 14;
    return var1 + var2;
}

void get_compensation_values() {
    uint8_t r_buffer[6] = {0 , 0 , 0, 0 , 0 , 0 };
    uint8_t data= 0x88;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &data, 1, r_buffer, 6, -1));
    DIG_T1 = r_buffer[0] | r_buffer[1] << 8;
    DIG_T2 = r_buffer[2] | r_buffer[3] << 8;
    DIG_T3 = r_buffer[4] | r_buffer[5] << 8;
}

long signed int get_temperature() {
    //instrucciones necesarias para solo obtener la medicion de temperatura, direccion/valor 
    uint8_t data_wr[8] = {0xF5, 0x00, 0xF2, 0x00, 0xF4, 0xA1};
    uint8_t r_buffer[6] = {0 , 0 , 0, 0 , 0 , 0 };
    
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data_wr, 6, -1));

    data_wr[0]= 0xF3;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, data_wr, 1, r_buffer, 1, -1));

    while (r_buffer[0] & (1<<0|1<<3)) {
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, data_wr, 1, r_buffer, 1, -1));
    }

    data_wr[0]= 0xFA;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, data_wr, 1, r_buffer, 3, -1));

    long signed int adc_T = 0;
    adc_T = r_buffer[0] << 12 | r_buffer[1] << 4 | r_buffer[2] >> 4;

    long signed int t_fine = BME280_compensate_T_double(adc_T);

    long signed int T = (t_fine * 5 + 128) >> 8;

    return T;
}


void get_temperature_task(void *args) {
    while (1) {
        xEventGroupWaitBits(i2c_event_group, I2C_GET_TEMPERATURE, pdTRUE, pdTRUE, portMAX_DELAY);
        temperature = get_temperature();
        xEventGroupSetBits(i2c_event_group, I2C_TEMPERATURE_READY);
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
}

void read_as_slave_task() {
    uint8_t *data_rd = (uint8_t *) malloc(DATA_LENGTH);
    uint8_t *data_wr = (uint8_t *) malloc(4);
    i2c_slave_rx_done_event_data_t rx_data;

    while (1) {
        ESP_ERROR_CHECK(i2c_slave_receive(slave_handle, data_rd, DATA_LENGTH));
        xQueueReceive(s_receive_queue, &rx_data, pdMS_TO_TICKS(portMAX_DELAY));
        if (data_rd[0] == 0x27 && data_rd[1] == 0x9F) {
                xEventGroupSetBits(i2c_event_group, I2C_GET_TEMPERATURE);
                xEventGroupWaitBits(i2c_event_group, I2C_TEMPERATURE_READY, pdTRUE, pdTRUE, portMAX_DELAY);
                
                data_wr[0] = 0x27;
                data_wr[1] = 0x9F;
                
                data_wr[2] = (temperature >> 24) & 0xFF;
                data_wr[3] = (temperature >> 16) & 0xFF;
                data_wr[4] = (temperature >> 8) & 0xFF;
                data_wr[5] = temperature & 0xFF;

                i2c_slave_transmit(slave_handle, data_wr, 6, -1);
                ESP_LOGI(TAG , " Enviando temperatura: %d" , (int) temperature);
            }
        vTaskDelay(20/portTICK_PERIOD_MS);
    }

}

void app_main () {
    
    esp_slave_init();
    i2c_master_init();
    i2c_slaves_devices_init();

    get_compensation_values();

    i2c_event_group = xEventGroupCreate();
    xTaskCreate(read_as_slave_task, "read_as_slave_task", 2048, NULL, 10, NULL);
    xTaskCreate(get_temperature_task, "get_temperature_task", 2048, NULL, 10, NULL);
}