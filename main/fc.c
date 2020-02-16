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
#include "driver/i2c.h"
#include <math.h>
#include <driver/adc.h>

#define QMC5883L_I2C_ADDR     0x0d //hmc5883 i2c address
#define i2c_frequency       500000 // max frequency of i2c clk
#define i2c_port                 0 //i2c channel on ESP-WROOM-32 ESP32S
#define i2c_gpio_scl            19 //D19 on ESP-WROOM-32 ESP32S
#define i2c_gpio_sda            18 //D18 on ESP-WROOM-32 ESP32S
#include "./i2c-bus/i2c.h"
#include "./i2c-bus/qmc5883l.h"

#define BMP280_I2C_ADDR       0x76
int digT1, digT2, digT3;
int digP1, digP2, digP3, digP4, digP5, digP6, digP7, digP8, digP9;
#include "./i2c-bus//bmp280.h"

//requirements for mcpwm
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#define GPIO_MCPWM0A   21       //Motor1
#define GPIO_MCPWM0B   25       //Motor2
#define GPIO_MCPWM1A   32       //Motor3
#define GPIO_MCPWM1B   33       //Motor4
#include "./include/esp_perif.h"

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
float xAccl_LP, yAccl_LP, zAccl_LP;
float zGyro_Int=0;
float xFusion, yFusion, zGyro_Int_HP;
float gmag, theta, phi;
int   seq, throttle=1000, yaw, pitch, roll, state;
int   cal_cnt = 0, astate = 0, calib = 0, nsamp = 0;
//pid globals
int   clearInts = 0;
float xPIDout, yPIDout, zPIDout, aPIDout;
float xSig, xErr, xPgain = 0.50, xIgain = 0.033, xDgain = 60, xInt = 0, xDer, xLast = 0; //pitch
float ySig, yErr, yPgain = 0.50, yIgain = 0.033, yDgain = 60, yInt = 0, yDer, yLast = 0; //roll
float zSig, zErr, zPgain = 0.00, zIgain = 0.000, zDgain = 60, zInt = 0, zDer, zLast = 0; //yaw
float aSig, aErr, aPgain = 0.00, aIgain = 0.000, aDgain =  0, aInt = 0, aDer, aLast = 0; //altitude
#include "./include/imu.h"
int Motor1=1000, Motor2=1000, Motor3=1000, Motor4=1000;
char col = 0; int cntp =0; int cal=0; char par = 'p'; float amt = 0.1; char axis = 'x'; int lastsamp = 0;
char  blackbox_str[256];
#include "./include/debug_interface.h"

float height = 3.72; float heightprog= -5.3; float height_cal = 0;
float heading= 90.5; float headingprog= 91.5;
float xdisp = -1.6; float ydisp = -0.1;
int cnt = 0;
void blackbox_string() {
    int temp_int;
    cnt++;
    blackbox_str[0] = cnt/256; blackbox_str[1] = cnt%256;
    blackbox_str[2] = (uint8_t)128+height;
    blackbox_str[3] = (uint8_t)128+heightprog;
    blackbox_str[4] = (uint8_t)heading/2;
    blackbox_str[5] = (uint8_t)headingprog/2;
    blackbox_str[6] = (uint8_t)128+xdisp; 
    blackbox_str[7] = (uint8_t)128+ydisp; 
    temp_int = (int)(100 * (180 + -57.3*theta));
    blackbox_str[8] = (int8_t)(temp_int/256);
    blackbox_str[9] = (int8_t)(temp_int%256);
    temp_int = (int)(100 * (180 + 57.3*phi));
    blackbox_str[10] = (int8_t)(temp_int/256);
    blackbox_str[11] = (int8_t)(temp_int%256);
    blackbox_str[12] = Motor1/256; blackbox_str[13] = Motor1%256;
    blackbox_str[14] = Motor2/256; blackbox_str[15] = Motor2%256;
    blackbox_str[16] = Motor3/256; blackbox_str[17] = Motor3%256;
    blackbox_str[18] = Motor4/256; blackbox_str[19] = Motor4%256;
    blackbox_str[20] = (int8_t)10.0*(meas_battery() - 10);
}

void attitude_control() {
     while (1) {
         //update pids
         xErr = xFusion;
         xInt = xInt + xErr;
         xDer = xErr - xLast;
         xLast = xErr;
         xPIDout = xPgain * xErr + xIgain * xInt + xDgain * xDer;
         //update Motors
         Motor1 = (int)(throttle - xPIDout - yPIDout - zPIDout );
         Motor2 = (int)(throttle - xPIDout + yPIDout + zPIDout );
         Motor3 = (int)(throttle + xPIDout + yPIDout - zPIDout );
         Motor4 = (int)(throttle + xPIDout - yPIDout + zPIDout );

         if (Motor1 < 1000) Motor1 = 1000; if (Motor1 > 1500) Motor1 = 1500;
         if (Motor2 < 1000) Motor2 = 1000; if (Motor2 > 1500) Motor2 = 1500;
         if (Motor3 < 1000) Motor3 = 1000; if (Motor3 > 1500) Motor3 = 1500;
         if (Motor4 < 1000) Motor4 = 1000; if (Motor4 > 1500) Motor4 = 1500;
         //if (cal == 1) { Motor1 = 2000; Motor2 = 2000; Motor3 = 2000; Motor4 = 2000; }

         mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Motor1);
         mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, Motor2);
         mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, Motor3);
         mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, Motor4);
 
         vTaskDelay(20/portTICK_RATE_MS); //should run about 50Hz min 
     }
}

void transmitter_comms () {
    //wait for packet from transmitter - if no packets over 10 timeouts land
    int timeout = 10; int waitcnt;
    signed char data[32];
    while(1) {
        waitcnt = nrf24_receive_pkt ((uint8_t*)data, timeout);
        if (waitcnt < timeout){
           printf("%8.4f   waited %3dmsec   ", (float)esp_timer_get_time()/1000000, 10*waitcnt);
           sscanf((char*)data,"%d,%d,%d,%d,%d,%d",&seq, &throttle, &yaw, &pitch, &roll, &state);
           printf("seq = %d, remote %d %d %d %d, state = %d\n", seq,throttle,yaw,pitch,roll,state);
        } 
        else { printf("no packet = timed out at %dmsec\n", 10*waitcnt); }

        //send blackbox data to transmitter
        if (waitcnt < timeout){ 
               blackbox_string(); nrf24_transmit_pkt ((uint8_t*)blackbox_str, 32); }

        //good a place as any to update i2c bus devices whe they get put in
        //if (cnt == 10) height_cal = bmp280_read();
        //printf("heading = %3d degrees   ", qmc5883_read());
        //printf("altitude = %6.2lf", 28.4 * (height_cal - bmp280_read()));
        //printf("\n");

        vTaskDelay(20/portTICK_RATE_MS); //should keep up with tranmitter commands 
    }
}

void app_main() {
    nvs_flash_init();
     
    vspi_init();         //spi for imu
    imu_init (vspi); 
    vTaskDelay(50/portTICK_RATE_MS);  

    hspi_init();         //spi for radio
    nrf24_gpio_init();

    escint_init();       //pwm servo for esc to motors

    //i2cdetect();
    //bmp280_cal();       //air pressure altitude measurements
    //qmc5883_init();     //magnetometer heading measurements

    //start tasks
    xTaskCreatePinnedToCore (imu_read, "imu_read", 8096, NULL, 5, NULL, 1);
    vTaskDelay(1);
    xTaskCreatePinnedToCore (attitude_control, "attitude_control", 4096, NULL, 4, NULL, 0);
    vTaskDelay(1);
    //xTaskCreatePinnedToCore (transmitter_comms, "transmitter_comms", 4096, NULL, 4, NULL, 0);

    while(1){
       //printf("accelxyz %7.3f %7.3f %7.3f       theta=%7.2f   phi=%7.2f\n", 
       //    xAccl, yAccl, zAccl, -57.3*theta, 57.3*phi);

       debug_interface();
       vTaskDelay(1000/portTICK_RATE_MS);  
    }

    removeDevice(vspi);
    removeDevice(hspi);
    vTaskDelay(1);
}
