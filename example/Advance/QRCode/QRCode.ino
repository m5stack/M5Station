/*
*******************************************************************************
* Copyright (c) 2023 by M5Stack
*                  Equipped with M5Station sample source code
*                          配套  M5Station 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/station_bat
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/station_bat
*
* Describe: QRcode.  创建二维码
* Date: 2022/8/30
*******************************************************************************
*/
#include <M5Station.h>

void setup() {
    M5.begin();
    M5.Lcd.qrcode("http://www.m5stack.com", 50, 0,
                  135);  // Create a QR code with a width of 135 QR code  at (0,
                         // 0).  在(0,0)处创建一个宽为135二维码
    // Please select the appropriate QR code version according to the number of
    // characters.  请根据字符数量选择合适的二维码版本
}

void loop() {
}