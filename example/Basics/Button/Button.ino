/*
*******************************************************************************
* Copyright (c) 2022 by M5Stack
*                  Equipped with M5Station sample source code
*                          配套  M5Station 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/station_485
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/station_485
*
* Describe: Button example.  按键示例
* Date: 2022/8/4
*******************************************************************************
Press button A/B/C to display the corresponding output on the screen and serial.
按下按键A/B/C，在屏幕和串口上显示相应输出
*/
#include <M5Station.h>
/* After M5Station is started or reset
  the program in the setUp () function will be run, and this part will only be
  run once. 在 M5Station
  启动或者复位后，即会开始执行setup()函数中的程序，该部分只会执行一次。 */
void setup() {
    M5.begin();  // Init M5Station.  初始化 M5Station
    M5.Lcd.setCursor(80, 0);
    M5.Lcd.setTextColor(ORANGE);
    M5.Lcd.println("Pls press A/B/C");
}

/* After the program in setup() runs, it runs the program in loop()
The loop() function is an infinite loop in which the program runs repeatedly
在setup()函数中的程序执行完后，会接着执行loop()函数中的程序
loop()函数是一个死循环，其中的程序会不断的重复运行 */
void loop() {
    M5.update();  // Read the press state of the key.  读取按键 A, B, C 的状态
    if (M5.BtnA.wasReleased() || M5.BtnA.pressedFor(1000)) {
        Serial.print('A');
        M5.Lcd.print('A');
    } else if (M5.BtnB.wasReleased() || M5.BtnB.pressedFor(1000)) {
        Serial.print('B');
        M5.Lcd.print('B');
    } else if (M5.BtnC.wasReleased() || M5.BtnC.pressedFor(1000)) {
        Serial.print('C');
        M5.Lcd.print('C');
    }
}
