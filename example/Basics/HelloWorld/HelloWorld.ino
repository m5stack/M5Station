/*
*******************************************************************************
* Copyright (c) 2023 by M5Stack
*                  Equipped with M5Station sample source code
*                          配套  M5Station 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/station_485
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/station_485
*
* Describe: Hello World.
* Date: 2022/8/4
*******************************************************************************
*/
#include <M5Station.h>

/* After M5Station is started or reset the program in the setUp ()
 function will be run, and this part will only be run once.
  在 M5Station
 启动或者复位后，即会开始执行setup()函数中的程序，该部分只会执行一次。 */
void setup() {
    M5.begin();             // Initialize M5Station.  初始化 M5Station
    M5.Lcd.setTextSize(3);  // Set font size.  设置字体大小
    // LCD display.  Lcd显示
    M5.Lcd.print("Hello World");
    Serial.print("Hello World");
}

/* After the program in setup() runs, it runs the program in loop()
The loop() function is an infinite loop in which the program runs repeatedly
在setup()函数中的程序执行完后，会接着执行loop()函数中的程序
loop()函数是一个死循环，其中的程序会不断的重复运行 */
void loop() {
}