# esp32_imu Test

Support of testing and PID tuning of Base IMU functionality 
for quadcopter drone project

- MPU9250 (not using magnetometer)
- ESP32 module SPI interface 
- reading 8K gyroscope samples per second
- reading 1K accelerometer samples per second
- simple one-pole low pass on accelerometer x, y and z axes
- simple one-pole high pass filter on gyroscope z axis (yaw)
- trig functions to convert accelerometer data into pitch
  and roll data in degrees
- Complementary filter fusion of gyroscope and accelerometer 
  x and y axes (pitch and roll) (output is angle in degrees)
- start-up calibration
- Inner control loop error estimaters and PID controllers
  (pitch, roll and yaw)
- imu_test.c supports one axis/two motor drone development and pid tuning
