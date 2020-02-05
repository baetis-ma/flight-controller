//#include <stdio.h>
//#include <stdlib.h>
//#include <stdint.h>
//#include <string.h>
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "esp_system.h"
//#include "soc/gpio_struct.h"
//#include "esp_log.h"
//#include "esp_err.h"
//#include "esp_heap_caps.h"
//#include "sdkconfig.h"
//#include <math.h>

#include "driver/spi_common.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

esp_err_t writeBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data);
esp_err_t writeByte(spi_device_handle_t handle, uint8_t regAddr, uint8_t data) ;
esp_err_t readByte(spi_device_handle_t handle, uint8_t regAddr, uint8_t *data) ;
esp_err_t readBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data) ;

spi_device_handle_t spi;

void spi_init()
{
    esp_err_t ret;
    vTaskDelay(10);  //needs to be here ?
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCLK_PIN,
        .quadwp_io_num = -1,  // -1 not used
        .quadhd_io_num = -1,  // -1 not used
        .max_transfer_sz = 1024 
    };
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);  // 0 DMA not used 1= dma used
    ESP_ERROR_CHECK(ret);
    vTaskDelay(10);  //needs to be here ?

    spi_device_interface_config_t devcfg={
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .mode = 0, //dma mode = 0
        .duty_cycle_pos = 128,  // default 128 = 50%/50% duty
        .cs_ena_pretrans = 0,  // 0 not used
        .cs_ena_posttrans = 0,  // 0 not used
        .clock_speed_hz = SPI_CLOCK,
        .spics_io_num = CS_PIN,
        .flags = 0,  // 0 not used
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL
    };
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

esp_err_t readByte(spi_device_handle_t handle, uint8_t regAddr, uint8_t *data) {
    return readBytes(handle, regAddr, 1, data);
}

esp_err_t readBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data) {
    if(length == 0) return ESP_ERR_INVALID_SIZE;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr | 0x80;
    transaction.length = length * 8;
    transaction.rxlength = length * 8;
    transaction.user = NULL;
    transaction.tx_buffer = NULL;
    transaction.rx_buffer = data;
    esp_err_t err = spi_device_transmit(handle, &transaction);
    assert (err == ESP_OK);
        if (!err) {
            char str[length*5+1];
            if(length > 10)length = 10; //don't print too much
            for(size_t i = 0; i < length; i++)
                 sprintf(str+i*5, "0x%s%x ", (data[i] < 0x10 ? "0" : ""), data[i]);
            //printf("read  reg 0x%x, data: %s\n", regAddr, str);
        }
    return err;
}

esp_err_t writeByte(spi_device_handle_t handle, uint8_t regAddr, uint8_t data) {
    return writeBytes(handle, regAddr, 1, &data);
}

esp_err_t writeBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data) {
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr & 0x7f;
    transaction.length = length * 8;
    transaction.rxlength = 0;
    transaction.user = NULL;
    transaction.tx_buffer = data;
    transaction.rx_buffer = NULL;
    esp_err_t err = spi_device_transmit(handle, &transaction);
    assert ( err == ESP_OK );
        if (!err) { 
            char str[length*5+1];
            for(size_t i = 0; i < length; i++) 
                sprintf(str+i*5, "0x%s%x ", (data[i] < 0x10 ? "0" : ""), data[i]);
            printf("write reg 0x%x, data: %s\n", regAddr, str);
        }
    return err;
}

esp_err_t removeDevice(spi_device_handle_t handle) {
    return spi_bus_remove_device(handle);
}

