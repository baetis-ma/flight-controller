#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "soc/gpio_struct.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/spi_common.h"

//requirements for espnow
#include "esp_wifi.h"
#include "esp_now.h"
//mac for attached transmitter
static uint8_t s_tx_mac[ESP_NOW_ETH_ALEN] = { 0xde, 0x4f, 0x22, 0x17, 0xf5, 0x6a };
#define ESPNNOW_SEND_COUNT  1000
#define ESPNOW_SEND_DELAY   100
#define ESPNOW_SEND_LEN     200
#define CONFIG_ESPNOW_CHANNEL  1
#define CONFIG_ESPNOW_PMK   "pmk1234567890123"
#define CONFIG_ESPNOW_SEND_LEN 200
#include "rom/crc.h"
int   throttle, yaw, pitch, roll;
char  blackbox_str[200];
int   cal_cnt = 0, astate = 0, calib = 0;
#include "./include/espnow.h"

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
#define SPI_CLOCK 1000000  // 1 MHz
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
int   throttle = 1000, yaw, pitch, roll;
#include "./include/imu.h"

void app_main() {
    esp_err_t ret = nvs_flash_init();
    wifi_init();
    vTaskDelay(10);
    espnow_init();  //starts task after init

    spi_init();
    imu_init (spi); 
    vTaskDelay(50/portTICK_RATE_MS);  

    xTaskCreatePinnedToCore (imu_read, "imu_read", 4096, NULL, 5, NULL, 1);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 475;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    pwm_config.frequency = 475;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_MCPWM0A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_MCPWM0B);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, GPIO_MCPWM1A);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, GPIO_MCPWM1B);
 
    char axis = 'x', col = 0, par = 'p';
    int cntp =0, cal=0;
    float amt = 0.1;
    int Motor1, Motor2, Motor3, Motor4; 
    int lastsamp = 0;
    while (1) {
     
        if(isnan(xFusion)) xErr = 0; else xErr = xFusion;
        xInt = xInt + xErr;
        xDer = xErr - xLast;
        xPIDout = xPgain * xErr + xIgain * xInt + xDgain * xDer;
        xLast = xErr;

        if(isnan(yFusion)) yErr = 0; else yErr = yFusion;
        yInt = yInt + yErr;
        yDer = yErr - yLast;
        yPIDout = yPgain * yErr + yIgain * yInt + yDgain * yDer;
        yLast = yErr;

        if(isnan(zGyro_Int)) zErr = 0; else zErr = zGyro_Int / 8;
        zInt = zInt + zErr;
        zDer = zErr - zLast;
        zPIDout = zPgain * zErr + zIgain * zInt + zDgain * zDer;
        zLast = zErr;

        // X configuration quadcopter motor speeds
        Motor1 = (int)(throttle - xPIDout - yPIDout - zPIDout - (1500-yaw)/10 + (1500-pitch)/10 - (1500-roll)/10);
        Motor2 = (int)(throttle - xPIDout + yPIDout + zPIDout + (1500-yaw)/10 + (1500-pitch)/10 + (1500-roll)/10);
        Motor3 = (int)(throttle + xPIDout + yPIDout - zPIDout - (1500-yaw)/10 - (1500-pitch)/10 + (1500-roll)/10);
        Motor4 = (int)(throttle + xPIDout - yPIDout + zPIDout + (1500-yaw)/10 - (1500-pitch)/10 - (1500-roll)/10);

        //limits
        //if (throttle < 1200) { xInt = 0; yInt = 0; xAccl_Int=0; yAccl_Int=0; zAccl_Int=0; }
        if (abs(xInt) > 10000) xInt = 0; if (abs(yInt) > 10000) yInt = 0;
        if (Motor1 < 1000) Motor1 = 1000; if (Motor1 > 1700) Motor1 = 1700;
        if (Motor2 < 1000) Motor2 = 1000; if (Motor2 > 1700) Motor2 = 1700;
        if (Motor3 < 1000) Motor3 = 1000; if (Motor3 > 1700) Motor3 = 1700;
        if (Motor4 < 1000) Motor4 = 1000; if (Motor4 > 1700) Motor4 = 1700;

        //cal and arming
        if (cal == 1) { Motor1 = 2000; Motor2 = 2000; Motor3 = 2000; Motor4 = 2000; }
        if (astate == 0){ Motor1 = 1000; Motor2 = 1000; Motor3 = 1000; Motor4 = 1000; xInt = 0; yInt = 0; }
        //if (calib == 1) { xAccl_Int=0.0; yAccl_Int=0.0; zAccl_Int=0.0; cal_cnt = 0; xInt = 0; yInt = 0; }
        if (calib == 1) { cal_cnt = 0; xInt = 0; yInt = 0; }

        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Motor1);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, Motor2);
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, Motor3);
        mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, Motor4);

        //sprintf(blackbox_str, "%8.2f   %6.2f %6.2f %6.2f   %4d %4d %4d %4d   %3d %3d %3d    %7.2f %7.2f %7.2f;", 
        //   0.001*esp_log_timestamp(), xFusion, yFusion, zErr, Motor1, Motor2, Motor3, Motor4, 
        //   (int)xPIDout, (int)yPIDout, (int)zPIDout, xAccl_Int, yAccl_Int, zAccl_Int);
        sprintf(blackbox_str, "%8.2f   %6.2f %6.2f %6.2f   %4d %4d %4d %4d   %3d %3d %3d;", 
           0.001*esp_log_timestamp(), xFusion, yFusion, zErr, Motor1, Motor2, Motor3, Motor4, 
           (int)xPIDout, (int)yPIDout, (int)zPIDout);
        cntp++;
        if((cntp%10)==1){
        printf("%7.2fs %3d tune-> %c,%c/%6.3f  ", 0.001*esp_log_timestamp(), nsamp - lastsamp, axis, par, amt);  
        if(axis == 'x') printf("PID gain %3.2f %3.2f %3.0f   I=%8.1f D=%5.2f  FusX=%6.2f xPID= %4d %4d %4d", 
           xPgain,xIgain,xDgain,xInt,xDer,xFusion,(int)(xPgain*xErr),(int)(xIgain*xInt),(int)(xDgain*xDer));
        if(axis == 'y') printf("PID gain %3.2f %3.2f %3.0f   I=%8.1f D=%5.2f  FusY=%6.2f yPID= %4d %4d %4d", 
           yPgain,yIgain,yDgain,yInt,yDer,yFusion,(int)(yPgain*yErr),(int)(yIgain*yInt),(int)(yDgain*yDer));
        if(axis == 'z') printf("PID gain %3.2f %3.2f %3.0f   I=%8.1f D=%5.2f  yawZ=%6.2f zPID= %4d %4d %4d", 
           zPgain,zIgain,zDgain,zInt,zDer,zErr,   (int)(zPgain*zErr),(int)(zIgain*zInt),(int)(zDgain*zDer));
        printf("  %4dt %4dy %4dp %4dr\n", throttle, yaw, pitch, roll);
        lastsamp = nsamp;
        }

        //tuning interface
        scanf("%c\n", &col);
        if(col!=0){
            if (col == 'M') amt = amt * 10; 
            if (col == 'm') amt = amt / 10; 
            if (col == 'a') { axis = axis + 1; if (axis > 'z') axis = 'x'; } 
            if (col == 'p') par = col; 
            if (col == 'i') {par = col; xInt=0; yInt=0; zInt=0; zGyro_Int=0; } 
             
            if (col == 'd') par = col; 
            if (col == 't') par = col; 
            if (col == 'T') par = col; 
            if (col == 'r') {throttle = 1000; xInt=0; xIgain=0; xPgain=0; xDgain=0;amt=0.1;
                             yInt=0; yIgain=0; yPgain=0; yDgain=0; zInt=0; zGyro_Int=0; zPgain=0; zDgain=0; zIgain=0;}
            if (par == 't' && col == '+') throttle = throttle + 1;
            if (par == 'T' && col == '+') throttle = throttle + 50;
            if (par == 't' && col == '-') throttle = throttle - 1;
            if (par == 'T' && col == '-') throttle = throttle - 50;
            if (axis == 'x' && par == 'p' && col == '+') xPgain = xPgain + amt; 
            if (axis == 'x' && par == 'i' && col == '+') {xIgain = xIgain + amt; xInt=0; yInt=0; }
            if (axis == 'x' && par == 'd' && col == '+') xDgain = xDgain + amt; 
            if (axis == 'x' && par == 'p' && col == '-') xPgain = xPgain - amt;
            if (axis == 'x' && par == 'i' && col == '-') {xIgain = xIgain - amt; xInt=0; yInt=0; }
            if (axis == 'x' && par == 'd' && col == '-') xDgain = xDgain - amt;
            if (axis == 'y' && par == 'p' && col == '+') yPgain = yPgain + amt; 
            if (axis == 'y' && par == 'i' && col == '+') {yIgain = yIgain + amt; xInt=0; yInt=0; }
            if (axis == 'y' && par == 'd' && col == '+') yDgain = yDgain + amt; 
            if (axis == 'y' && par == 'p' && col == '-') yPgain = yPgain - amt;
            if (axis == 'y' && par == 'i' && col == '-') {yIgain = yIgain - amt; xInt=0; yInt=0; }
            if (axis == 'y' && par == 'd' && col == '-') yDgain = yDgain - amt;
            if (axis == 'z' && par == 'p' && col == '+') {zPgain = zPgain + amt;  zGyro_Int = 0; }
            if (axis == 'z' && par == 'i' && col == '+') zIgain = 0; //zIgain + amt; 
            if (axis == 'z' && par == 'd' && col == '+') {zDgain = zDgain + amt;  zGyro_Int = 0; }
            if (axis == 'z' && par == 'p' && col == '-') {zPgain = zPgain - amt; zGyro_Int = 0; }
            if (axis == 'z' && par == 'i' && col == '-') zIgain = 0; //zIgain - amt;
            if (axis == 'z' && par == 'd' && col == '-') {zDgain = zDgain - amt; zGyro_Int = 0; }
            if (col == 's' ){ xPgain = 1.0; xIgain = 0.05; xDgain = 60; yPgain = 1.0; yIgain = 0.05; yDgain = 60; zPgain = 3; xInt=0; yInt=0; zGyro_Int=0;}
            if (col == 'C' ) cal = 1;
            if (col == 'c' ) {cal = 0; Motor1 = 1000;  Motor2 = 1000;  Motor3 = 1000;  Motor4 = 1000; }
            if (col == 'h' || col == '?'){
                 printf("Options \n");
                 printf("h or ?   - this message\n");
                 printf("p, i, d  - select parameter to tune p i d \n");
                 printf("M or m   - increase/decrease amt by factor of 10\n");
                 printf("+ or -   - increase/decrease parmameter by amt \n");
                 printf("T or t   - select throttle adjment T +/- 50, t +/- 1 \n");
                 printf("C or c   - select for esc cal C max throt c = min \n");
                 printf("s        - load stanard default values \n");
                 printf("r        - reset throttle to 1000 \n");
            }
        }
        col = 0;
        zInt = 0; //never use
        vTaskDelay(20/portTICK_RATE_MS);
    }

    removeDevice(spi);
    vTaskDelay(portMAX_DELAY);
}
