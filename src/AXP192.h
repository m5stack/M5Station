#ifndef __AXP192_H__
#define __AXP192_H__

#include <Wire.h>
#include <Arduino.h>

#define SLEEP_MSEC(us) (((uint64_t)us) * 1000L)
#define SLEEP_SEC(us)  (((uint64_t)us) * 1000000L)
#define SLEEP_MIN(us)  (((uint64_t)us) * 60L * 1000000L)
#define SLEEP_HR(us)   (((uint64_t)us) * 60L * 60L * 1000000L)

#define AXP_ADDR 0X34

#define PowerOff(x) SetSleep(x)

typedef enum {
    kMBusModeOutput = 0,  // powered by USB or Battery
    kMBusModeInput  = 1   // powered by outside input
} mbus_mode_t;

class AXP192 {
   public:
    enum CHGCurrent {
        kCHG_100mA = 0,
        kCHG_190mA,
        kCHG_280mA,
        kCHG_360mA,
        kCHG_450mA,
        kCHG_550mA,
        kCHG_630mA,
        kCHG_700mA,
        kCHG_780mA,
        kCHG_880mA,
        kCHG_960mA,
        kCHG_1000mA,
        kCHG_1080mA,
        kCHG_1160mA,
        kCHG_1240mA,
        kCHG_1320mA,
    };

    enum GroovePort {
        kGroovePort_A1_A2,
        kGroovePort_B1,
        kGroovePort_B2,
        kGroovePort_C1,
        kGroovePort_C2,
        kGroovePort_ALL,
        kGroovePort_MAX,
    };

    enum GroovePower {
        kGroovePower_OFF,
        kGroovePower_ON,
        kGroovePower_MAX,
    };

    AXP192();
    void begin();
    // Will be deprecated
    void begin(mbus_mode_t mode);
    void ScreenBreath(int brightness);
    bool GetBatState();

    void EnableCoulombcounter(void);
    void DisableCoulombcounter(void);
    void StopCoulombcounter(void);
    void ClearCoulombcounter(void);
    uint32_t GetCoulombchargeData(void);
    uint32_t GetCoulombdischargeData(void);
    float GetCoulombData(void);
    float GetBatteryLevel(void);
    void PowerOff(void);
    void SetAdcState(bool state);
    // -- sleep
    void PrepareToSleep(void);
    void RestoreFromLightSleep(void);
    void DeepSleep(uint64_t time_in_us = 0);
    void LightSleep(uint64_t time_in_us = 0);
    uint8_t GetWarningLeve(void);

    // void SetChargeVoltage( uint8_t );
    // void SetChargeCurrent( uint8_t );
    float GetBatVoltage();
    float GetBatCurrent();
    float GetVinVoltage();
    float GetVinCurrent();
    float GetVBusVoltage();
    float GetVBusCurrent();
    float GetTempInAXP192();
    float GetBatPower();
    float GetBatChargeCurrent();
    float GetAPSVoltage();
    float GetBatCoulombInput();
    float GetBatCoulombOut();
    uint8_t GetWarningLevel(void);
    void SetCoulombClear();
    void SetLDO3(bool State);

    uint8_t AXPInState();
    bool isACIN();
    bool isCharging();
    bool isVBUS();

    void SetLDOVoltage(uint8_t number, uint16_t voltage);
    void SetDCVoltage(uint8_t number, uint16_t voltage);
    void SetESPVoltage(uint16_t voltage);
    void SetLcdVoltage(uint16_t voltage);
    void SetLDOEnable(uint8_t number, bool state);
    // void SetLCDRSet(bool state);
    void SetBusPowerMode(uint8_t state);
    // void SetLed(uint8_t state);
    void SetCHGCurrent(uint8_t state);
    void SetGPIOOutput(uint8_t state);
    uint8_t GetGPIOOutput();
    void SetGroovePower(GroovePort port, GroovePower state);

   private:
    void Write1Byte(uint8_t Addr, uint8_t Data);
    uint8_t Read8bit(uint8_t Addr);
    uint16_t Read12Bit(uint8_t Addr);
    uint16_t Read13Bit(uint8_t Addr);
    uint16_t Read16bit(uint8_t Addr);
    uint32_t Read24bit(uint8_t Addr);
    uint32_t Read32bit(uint8_t Addr);
    void ReadBuff(uint8_t Addr, uint8_t Size, uint8_t *Buff);
};

#endif
