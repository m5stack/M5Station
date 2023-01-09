// Copyright (c) M5Station. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

#include "M5Station.h"

M5Station::M5Station() : isInited(0) {
}

void M5Station::begin(bool LCDEnable, bool SerialEnable, bool I2CEnable,
                      mbus_mode_t mode) {
    // Correct init once
    if (isInited) {
        return;
    } else {
        isInited = true;
    }

    // UART
    if (SerialEnable) {
        Serial.begin(115200);
        Serial.flush();
        delay(50);
        Serial.print("M5Station initializing...\n");
    }

    // I2C init
    if (I2CEnable) {
        Wire.begin(32, 33);
    }

    Axp.begin(mode);

    // LCD INIT
    if (LCDEnable) {
        Lcd.begin();
    }
    Rtc.begin();

    INA1.begin(&Wire1);
    INA1.reset();
    INA1.setShuntRes(10, 10, 10);
    INA2.begin(&Wire1, INA3221_ADDR41_VCC);
    INA2.reset();
    INA2.setShuntRes(10, 10, 10);

    BtnA.begin();
    BtnB.begin();
    BtnC.begin();

    if (SerialEnable) {
        Serial.println("OK");
    }
}

void M5Station::update() {
    BtnA.read();
    BtnB.read();
    BtnC.read();
}

void M5Station::shutdown() {
    Axp.PowerOff();
}
int M5Station::shutdown(int seconds) {
    Rtc.clearIRQ();
    Rtc.SetAlarmIRQ(seconds);
    delay(10);
    Axp.PowerOff();
    return 0;
}
int M5Station::shutdown(const RTC_TimeTypeDef &RTC_TimeStruct) {
    Rtc.clearIRQ();
    Rtc.SetAlarmIRQ(RTC_TimeStruct);
    delay(10);
    Axp.PowerOff();
    return 0;
}
int M5Station::shutdown(const RTC_DateTypeDef &RTC_DateStruct,
                        const RTC_TimeTypeDef &RTC_TimeStruct) {
    Rtc.clearIRQ();
    Rtc.SetAlarmIRQ(RTC_DateStruct, RTC_TimeStruct);
    delay(10);
    Axp.PowerOff();
    return 0;
}

M5Station M5;
