#ifndef _M5STATION_H_
#define _M5STATION_H_

#if defined(ESP32)

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "FS.h"
#include "SD.h"

#include "M5Display.h"
#include "utility/Button.h"
#include "utility/Config.h"
// #include "utility/CommUtil.h"
#include "utility/MPU6886.h"
#include "utility/INA3221.h"
#include "AXP192.h"
#include "RTC.h"

class M5Station {
   public:
    M5Station();
    void begin(bool LCDEnable = true, bool SerialEnable = true,
               bool I2CEnable = false, mbus_mode_t mode = kMBusModeOutput);
    void update();

    void shutdown();
    int shutdown(int seconds);
    int shutdown(const RTC_TimeTypeDef &RTC_TimeStruct);
    int shutdown(const RTC_DateTypeDef &RTC_DateStruct,
                 const RTC_TimeTypeDef &RTC_TimeStruct);

    AXP192 Axp;

    M5Display Lcd = M5Display();

    MPU6886 IMU;

    RTC Rtc;

    INA3221 INA1{INA3221_ADDR40_GND};
    INA3221 INA2{INA3221_ADDR41_VCC};

    // Button API
#define DEBOUNCE_MS 25
    Button BtnA = Button(BUTTON_A_PIN, DEBOUNCE_MS, true, true);
    Button BtnB = Button(BUTTON_B_PIN, DEBOUNCE_MS, true, true);
    Button BtnC = Button(BUTTON_C_PIN, DEBOUNCE_MS, true, true);

    // // I2C
    // CommUtil I2C;

    /**
     * Functions have been moved to Power class for compatibility.
     * These will be removed in a future release.
     */
    // void setPowerBoostKeepOn(bool en) __attribute__((deprecated));
    // void setWakeupButton(uint8_t button) __attribute__((deprecated));
    // void powerOFF() __attribute__((deprecated));

   private:
    bool isInited;
};

extern M5Station M5;
#define m5  M5
#define lcd Lcd
#define imu IMU
#define Imu IMU

#else
#error "This library only supports boards with ESP32 processor."
#endif
#endif
