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
#define GPIO_MCPWM0A         18 //motor1
#define GPIO_MCPWM0B         19 //motor2
#define GPIO_MCPWM1A         21 //motor3
#define GPIO_MCPWM1B         22 //motor4

//requirements for spi
#define MISO_PIN  17       //ad0
#define MOSI_PIN  5        //sda
#define SCLK_PIN  23       //scl
#define CS_PIN    16       //ncs
#define SPI_CLOCK 10000000  // 10 MHz - must be >> 1M for 8ksamp/sec
#include "./include/spi.h"


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

void printcrap() {
    while(1){
       printf("accelxyz %7.3f %7.3f %7.3f       theta=%7.2f   phi=%7.2f\n", 
           xAccl, yAccl, zAccl, -57.3*theta, 57.3*phi);

       scanf("%c", &col);
       if (col == 'c') cal_cnt = 0;
       col = 0;

       vTaskDelay(50);
    }
}


void app_main() {
    esp_err_t ret = nvs_flash_init();

    spi_init();
    imu_init (spi); 
    vTaskDelay(50/portTICK_RATE_MS);  

    xTaskCreatePinnedToCore (imu_read, "imu_read", 8096, NULL, 5, NULL, 0);
    vTaskDelay(5);
    xTaskCreatePinnedToCore (printcrap, "printcrap", 4096, NULL, 4, NULL, 1);

    removeDevice(spi);
    vTaskDelay(1);
}
