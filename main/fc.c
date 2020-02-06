#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "soc/gpio_struct.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/spi_common.h"

int   throttle=1000, yaw, pitch, roll;
int   cal_cnt = 0, astate = 0, calib = 0;
char  col = 0;
char  blackbox_str[256];

//requirements for mcpwm
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#define GPIO_MCPWM0A   32       //motor1
#define GPIO_MCPWM0B   25       //motor2
#define GPIO_MCPWM1A   35       //motor3
#define GPIO_MCPWM1B   34       //motor4

//requirements for hspi attached to nrf24l01
#define HSPI_MISO_PIN  14       //ad0
#define HSPI_MOSI_PIN  27       //sda
#define HSPI_SCLK_PIN  12       //scl
#define HSPI_CS_PIN    26       //ncs
#define HSPI_SPI_CLOCK 1000000  //1MHz 
#include "./include/hspi.h"

#define NRF24L01_CE    13
#include "./include/nrf24l01.h"

//requirements for vspi attached to mpu9250
#define VSPI_MISO_PIN  17       //ad0
#define VSPI_MOSI_PIN  5        //sda
#define VSPI_SCLK_PIN  23       //scl
#define VSPI_CS_PIN    16       //ncs
#define VSPI_SPI_CLOCK 10000000  // 10 MHz - must be >> 1M for 8ksamp/sec
#include "./include/vspi.h"

//requirements for imu
//imu globals
float xAccl, yAccl, zAccl, xGyro, yGyro, zGyro;
//float xAccl_Int=0.0, yAccl_Int=0.0, zAccl_Int=0.0;
float xAccl_LP, yAccl_LP, zAccl_LP;
float zGyro_Int=0;
float xFusion, yFusion, zGyro_Int_HP;
float gmag, theta, phi;
int   nsamp = 0;
//pid globals
int   clearInts = 0;
float xPIDout, yPIDout, zPIDout, aPIDout;
float xSig, xErr, xPgain = 0.50, xIgain = 0.033, xDgain = 60, xInt = 0, xDer, xLast = 0; //pitch
float ySig, yErr, yPgain = 0.50, yIgain = 0.033, yDgain = 60, yInt = 0, yDer, yLast = 0; //roll
float zSig, zErr, zPgain = 0.00, zIgain = 0.000, zDgain = 60, zInt = 0, zDer, zLast = 0; //yaw
float aSig, aErr, aPgain = 0.00, aIgain = 0.000, aDgain =  0, aInt = 0, aDer, aLast = 0; //altitude
#include "./include/imu.h"

void app_main() {
    nvs_flash_init();
    
    vspi_init();
    imu_init (vspi); 
    vTaskDelay(50/portTICK_RATE_MS);  

    hspi_init();
    nrf24_gpio_init();

    uint8_t data[32];
    for(int a=0; a<0x1d; a++){
        printf("reg 0x%02x  content 0x%02x\n",a, spiReadByte(hspi, a, data));
        vTaskDelay(10);
    }

    xTaskCreatePinnedToCore (imu_read, "imu_read", 8096, NULL, 5, NULL, 1);
    vTaskDelay(1);
    //xTaskCreatePinnedToCore (printcrap, "printcrap", 2048, NULL, 4, NULL, 0);

//void printcrap() {
    while(1){
       vTaskDelay(10);
       scanf("%c", &col);
       if (col == 'c') cal_cnt = 0;
       col = 0;
       vTaskDelay(10);
       printf("accelxyz %7.3f %7.3f %7.3f       theta=%7.2f   phi=%7.2f\n", 
           xAccl, yAccl, zAccl, -57.3*theta, 57.3*phi);

       vTaskDelay(40);
    }
//}

    removeDevice(vspi);
    removeDevice(hspi);
    vTaskDelay(1);
}
