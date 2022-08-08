/*
*******************************************************************************
* Copyright (c) 2022 by M5Stack
*                  Equipped with M5Station sample source code
*                          配套  M5Station 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/station_485
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/station_485
*
* Describe: Current and voltage meters.  电流电压计
* Date: 2022/8/5
*******************************************************************************
Measuring port currents and voltages
测量端口电流及电压
*/
#include <M5Station.h>
/* After M5Station is started or reset
  the program in the setUp () function will be run, and this part will only be
  run once. 在 M5Station
  启动或者复位后，即会开始执行setup()函数中的程序，该部分只会执行一次。 */
void setup() {
    M5.begin();  // Init M5Station.  初始化 M5Station
    M5.Lcd.setTextColor(ORANGE);
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(35, 0);
    M5.Lcd.print("Current and voltage meters");
}

/* After the program in setup() runs, it runs the program in loop()
The loop() function is an infinite loop in which the program runs repeatedly
在setup()函数中的程序执行完后，会接着执行loop()函数中的程序
loop()函数是一个死循环，其中的程序会不断的重复运行 */
void loop() {
    M5.Lcd.fillRect(0, 30, 240, 95, BLACK);
    M5.Lcd.setCursor(25, 40);
    M5.Lcd.printf("A1%3.0f ma %1.1fV  A2%3.0f ma %1.1fV\r",
                  M5.INA1.getCurrent(INA3221_CH1) * 1000,
                  M5.INA1.getVoltage(INA3221_CH1),
                  M5.INA1.getCurrent(INA3221_CH2) * 1000,
                  M5.INA1.getVoltage(INA3221_CH2));
    M5.Lcd.setCursor(25, 55);
    M5.Lcd.printf("B1%3.0f ma %1.1fV  B2%3.0f ma %1.1fV\r",
                  M5.INA1.getCurrent(INA3221_CH3) * 1000,
                  M5.INA1.getVoltage(INA3221_CH3),
                  M5.INA2.getCurrent(INA3221_CH1) * 1000,
                  M5.INA2.getVoltage(INA3221_CH1));
    M5.Lcd.setCursor(25, 70);
    M5.Lcd.printf("C1%3.0f ma %1.1fV  C2%3.0f ma %1.1fV\r",
                  M5.INA2.getCurrent(INA3221_CH2) * 1000,
                  M5.INA2.getVoltage(INA3221_CH2),
                  M5.INA2.getCurrent(INA3221_CH3) * 1000,
                  M5.INA2.getVoltage(INA3221_CH3));

    delay(1000);
}
