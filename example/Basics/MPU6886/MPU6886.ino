/*
*******************************************************************************
* Copyright (c) 2022 by M5Stack
*                  Equipped with M5Station sample source code
*                          配套  M5Station 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/station_bat
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/station_bat
*
* Describe: MPU6886 example.  惯性传感器
* Date: 2022/8/4
*******************************************************************************
*/
#include <M5Station.h>

float accX = 0.0F;  // Define variables for storing inertial sensor data
float accY = 0.0F;  //定义存储惯性传感器相关数据的相关变量
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

float temp = 0.0F;

/* After M5Station is started or reset
the program in the setUp () function will be run, and this part will only be run
once. 在 M5Station
启动或者复位后，即会开始执行setup()函数中的程序，该部分只会执行一次。 */
void setup() {
    M5.begin();              // Init M5Station.  初始化 M5Station
    while (M5.IMU.Init()) {  // Init IMU sensor.  初始化惯性传感器
        Serial.println("IMU init failed");
        delay(1000);
    }
}

void loop() {
    // Stores the triaxial gyroscope data of the inertial sensor to the relevant
    // variable. 将惯性传感器的三轴陀螺仪数据存储至相关变量
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    M5.IMU.getAccelData(
        &accX, &accY,
        &accZ);  // Stores the triaxial accelerometer.  存储三轴加速度计数据
    M5.IMU.getAhrsData(
        &pitch, &roll,
        &yaw);  // Stores the inertial sensor attitude.  存储惯性传感器的姿态
    M5.IMU.getTempData(&temp);  // Stores the inertial sensor temperature to
                                // temp.  存储惯性传感器的温度
    // gyroscope output related.
    //陀螺仪输出相关
    Serial.printf("gyroX:%6.2f gyroY:%6.2f gyroZ:%6.2f o/s\n", gyroX, gyroY,
                  gyroZ);

    // Accelerometer output is related
    //加速度计输出相关
    Serial.printf("accX:%5.2f accY:%5.2f accZ:%5.2f G\n", accX, accY, accZ);

    // Pose output is related
    //姿态输出相关
    Serial.printf("pitch:%5.2f roll:%5.2f yaw:%5.2f deg\n", pitch, roll, yaw);

    // Inertial sensor temperature output related
    //惯性传感器温度输出相关
    Serial.printf("Temperature:%.2f C\n\n", temp);

    delay(100);  // Delay 100ms.  延迟100ms
}