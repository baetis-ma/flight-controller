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

int   seq, throttle=1000, yaw, pitch, roll, state;
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

void attitude_control() {
     //setup pwm
     while (1) {
         //update pids
         //update motors
         vTaskDelay(30);
     }
}

void app_main() {
    signed char data[32];
    nvs_flash_init();
    
    vspi_init();
    imu_init (vspi); 
    vTaskDelay(50/portTICK_RATE_MS);  

    hspi_init();
    nrf24_gpio_init();

    xTaskCreatePinnedToCore (imu_read, "imu_read", 8096, NULL, 5, NULL, 1);
    vTaskDelay(10);
    xTaskCreatePinnedToCore (attitude_control, "attitude_control", 4096, NULL, 4, NULL, 0);

    int cnt = 0;
    int motor1=1000, motor2=1000, motor3=1000, motor4=1000;
    float height = 3.72;
    float xdisp = -1.6;
    float ydisp = -0.1;
    float voltage = 12.1;
    //bookkepp these elsewhere share globally
    int8_t blackbox_str[32];
    blackbox_str[2] = 10*height; 
    blackbox_str[3] = 10*xdisp; 
    blackbox_str[4] = 10*ydisp; 
    blackbox_str[5] = motor1/256; data[6] = motor1%256;
	blackbox_str[7] = motor2/256; data[8] = motor2%256;
	blackbox_str[9] = motor3/256; data[10] = motor3%256;
	blackbox_str[11] = motor4/256; data[12] = motor4%256;
	blackbox_str[13] = (int8_t)10.0*(voltage-10.0);

    int timeout = 30;
    int waitcnt;
    while(1){
       ++cnt;
       //wait for packet from transmitter - if no packets over 10 timeouts land
       waitcnt = nrf24_receive_pkt ((uint8_t*)data, timeout);
       if (waitcnt < timeout){
          printf("%8.4f   waited %3dmsec   ",
              (float)esp_timer_get_time()/1000000, 10*waitcnt);
          sscanf((char*)data,"%d,%d,%d,%d,%d,%d",&seq, &throttle, &yaw, &pitch, &roll, &state);
          printf("seq = %d, remote %d %d %d %d, state = %d\n", seq,throttle,yaw,pitch,roll,state);
       } 
       else {
          printf("no packet = timed out at %dmsec\n", 10*waitcnt);
       }
       //send blackbox data to transmitter
       if (waitcnt < timeout){
           blackbox_str[0] = cnt/256; blackbox_str[1] = cnt%256;

           nrf24_transmit_pkt ((uint8_t*)blackbox_str, 32);
       }

       //printf("accelxyz %7.3f %7.3f %7.3f       theta=%7.2f   phi=%7.2f\n", 
       //    xAccl, yAccl, zAccl, -57.3*theta, 57.3*phi);

       //debug interface
       scanf("%c", &col);
       if (col == 'c') cal_cnt = 0;
       col = 0;

       vTaskDelay(1);
    }

    removeDevice(vspi);
    removeDevice(hspi);
    vTaskDelay(1);
}
