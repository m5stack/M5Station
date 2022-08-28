#include "AXP192.h"

AXP192::AXP192() {
}

// Will be deprecated
void AXP192::begin(mbus_mode_t mode) {
    begin();
}

void AXP192::begin() {
    Wire1.begin(21, 22);
    Wire1.setClock(400000);

    // AXP192 30H
    Write1Byte(0x30, (Read8bit(0x30) & 0x04) | 0X02);
    Serial.printf("axp: vbus limit off\n");

    // Disable DCDC2,Enable EXTEN,LDO2,3,DCDC1
    Write1Byte(0x12, Read8bit(0x12) | 0x4D);

    // Set LDO3 TFT_LED 2.8V
    SetLcdVoltage(2800);
    Serial.printf("axp: lcd backlight voltage was set to 2.80v\n");

    SetESPVoltage(3350);
    Serial.printf("axp: esp32 power voltage was set to 3.35v\n");

    // // AXP192 GPIO0:OD OUTPUT
    // Write1Byte(0x90, Read8bit(0x92) & 0xf8);
    // Serial.printf("axp: gpio1 init\n");

    // // AXP192 GPIO1:OD OUTPUT
    // Write1Byte(0x92, Read8bit(0x92) & 0xf8);
    // Serial.printf("axp: gpio1 init\n");

    // // AXP192 GPIO2:OD OUTPUT
    // Write1Byte(0x93, Read8bit(0x93) & 0xf8);
    // Serial.printf("axp: gpio2 init\n");

    // GPIO0-4 are used to control Groove port power
    // GPIO0: select NMOS open drain mode
    Write1Byte(0x90, (Read8bit(0x90) & 0b11111000) | 0b00000000);
    // GPIO1: select NMOS open drain mode
    Write1Byte(0x92, (Read8bit(0x92) & 0b11111000) | 0b00000000);
    // GPIO2: select NMOS open drain mode
    Write1Byte(0x93, (Read8bit(0x93) & 0b11111000) | 0b00000000);
    // GPIO4/GPIO3: select NMOS open drain mode
    // Note: MSB must be set to enable GPIO functionality (info not in all datasheets)
    Write1Byte(0x95, (Read8bit(0x95) & 0b01110000) | 0b10000101);
    // Turn power on for all Groove ports
    SetGroovePower(kGroovePort_ALL, kGroovePower_ON);

    // Enable bat detection
    Write1Byte(0x32, 0x46);

    // Set temperature protection
    Write1Byte(0x39, 0xfc);

    // 512ms power on time,4s power off time,1s long keystroke time
    // Auto shutdown when key time exceeds boot time
    Write1Byte(0X36, 0X4C);

    // SetLDOEnable(2, true);

    // SetCHGCurrent(kCHG_100mA);
    // SetAxpPriphPower(1);
    // Serial.printf("axp: lcd_logic and sdcard power enabled\n\n");

    // pinMode(39, INPUT_PULLUP);

    // AXP192 GPIO4
    // Write1Byte(0X95, (Read8bit(0x95) & 0x72) | 0X84);

    // Enable Bat,ACIN,VBUS,APS adc
    Write1Byte(0x82, 0xff);

    // SetLCDRSet(0);
    // delay(100);
    // SetLCDRSet(1);
    // delay(100);
    // I2C_WriteByteDataAt(0X15,0XFE,0XFF);

    // axp: check v-bus status
    if (Read8bit(0x00) & 0x08) {
        Write1Byte(0x30, Read8bit(0x30) | 0x80);
        // if v-bus can use, disable M-Bus 5V output to input
        SetBusPowerMode(kMBusModeInput);
    } else {
        // if not, enable M-Bus 5V output
        SetBusPowerMode(kMBusModeOutput);
    }
}

void AXP192::Write1Byte(uint8_t Addr, uint8_t Data) {
    Wire1.beginTransmission(AXP_ADDR);
    Wire1.write(Addr);
    Wire1.write(Data);
    Wire1.endTransmission();
}

uint8_t AXP192::Read8bit(uint8_t Addr) {
    Wire1.beginTransmission(AXP_ADDR);
    Wire1.write(Addr);
    Wire1.endTransmission();
    Wire1.requestFrom(AXP_ADDR, 1);
    return Wire1.read();
}

uint16_t AXP192::Read12Bit(uint8_t Addr) {
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr, 2, buf);
    Data = ((buf[0] << 4) + buf[1]);  //
    return Data;
}

uint16_t AXP192::Read13Bit(uint8_t Addr) {
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr, 2, buf);
    Data = ((buf[0] << 5) + buf[1]);  //
    return Data;
}

uint16_t AXP192::Read16bit(uint8_t Addr) {
    uint16_t ReData = 0;
    Wire1.beginTransmission(AXP_ADDR);
    Wire1.write(Addr);
    Wire1.endTransmission();
    Wire1.requestFrom(AXP_ADDR, 2);
    for (int i = 0; i < 2; i++) {
        ReData <<= 8;
        ReData |= Wire1.read();
    }
    return ReData;
}

uint32_t AXP192::Read24bit(uint8_t Addr) {
    uint32_t ReData = 0;
    Wire1.beginTransmission(AXP_ADDR);
    Wire1.write(Addr);
    Wire1.endTransmission();
    Wire1.requestFrom(AXP_ADDR, 3);
    for (int i = 0; i < 3; i++) {
        ReData <<= 8;
        ReData |= Wire1.read();
    }
    return ReData;
}

uint32_t AXP192::Read32bit(uint8_t Addr) {
    uint32_t ReData = 0;
    Wire1.beginTransmission(AXP_ADDR);
    Wire1.write(Addr);
    Wire1.endTransmission();
    Wire1.requestFrom(AXP_ADDR, 4);
    for (int i = 0; i < 4; i++) {
        ReData <<= 8;
        ReData |= Wire1.read();
    }
    return ReData;
}

void AXP192::ReadBuff(uint8_t Addr, uint8_t Size, uint8_t *Buff) {
    Wire1.beginTransmission(AXP_ADDR);
    Wire1.write(Addr);
    Wire1.endTransmission();
    Wire1.requestFrom(AXP_ADDR, (int)Size);
    for (int i = 0; i < Size; i++) {
        *(Buff + i) = Wire1.read();
    }
}

void AXP192::ScreenBreath(int brightness) {
    int vol = map(brightness, 0, 100, 2400, 3300);
    // Serial.printf("brightness:%d\n", brightness);

    // Serial.printf("vol:%d\n", vol);
    // Serial.printf("vol:%u\n", vol);

    SetLcdVoltage((uint16_t)vol);
    // delay(10);
    // uint8_t buf = Read8bit(0x27);
    // Serial.printf("brightness:%hhu\n", brightness);
    // Serial.printf("brightness:%d\n", brightness);
    // Serial.printf("brightness:%x\n", brightness);

    // Serial.printf("buf:%hhu\n", buf);
    // Serial.printf("buf:%d\n", buf);
    // Serial.printf("buf:%x\n", buf);

    // Serial.printf("result:%hhu\n", ((buf & 0x0f) | (brightness << 4)));
    // Serial.printf("result:%d\n", ((buf & 0x0f) | (brightness << 4)));
    // Serial.printf("result:%x\n", ((buf & 0x0f) | (brightness << 4)));

    // Write1Byte(0x27, ((buf & 0x0f) | (brightness << 4)));
}

bool AXP192::GetBatState() {
    return (Read8bit(0x01) & 0x20) ? true : false;
}
//---------coulombcounter_from_here---------
// enable: void EnableCoulombcounter(void);
// disable: void DisableCOulombcounter(void);
// stop: void StopCoulombcounter(void);
// clear: void ClearCoulombcounter(void);
// get charge data: uint32_t GetCoulombchargeData(void);
// get discharge data: uint32_t GetCoulombdischargeData(void);
// get coulomb val affter calculation: float GetCoulombData(void);
//------------------------------------------
void AXP192::EnableCoulombcounter(void) {
    Write1Byte(0xB8, 0x80);
}

void AXP192::DisableCoulombcounter(void) {
    Write1Byte(0xB8, 0x00);
}

void AXP192::StopCoulombcounter(void) {
    Write1Byte(0xB8, 0xC0);
}

void AXP192::ClearCoulombcounter(void) {
    Write1Byte(0xB8, 0xA0);
}

uint32_t AXP192::GetCoulombchargeData(void) {
    return Read32bit(0xB0);  // Read from B0 to B4
}

uint32_t AXP192::GetCoulombdischargeData(void) {
    return Read32bit(0xB4);  // Read from B4 to B7
}

float AXP192::GetCoulombData(void) {
    uint32_t coin  = 0;
    uint32_t coout = 0;

    coin  = GetCoulombchargeData();
    coout = GetCoulombdischargeData();

    // c = 65536 * current_LSB * (coin - coout) / 3600 / ADC rate
    // Adc rate can be read from 84H ,change this variable if you change the ADC
    // reate
    float ccc = 65536 * 0.5 * (int32_t)(coin - coout) / 3600.0 / 25.0;
    return ccc;
}

// Cut all power, except for LDO1 (RTC)
void AXP192::PowerOff(void) {
    Write1Byte(0x32, Read8bit(0x32) | 0x80);
}

void AXP192::SetAdcState(bool state) {
    // Enable / Disable all ADCs
    Write1Byte(0x82, state ? 0xff : 0x00);
}

void AXP192::PrepareToSleep(void) {
    // Disable ADCs
    SetAdcState(false);
    // Turn LCD backlight off
    SetLDO3(false);
}

void AXP192::RestoreFromLightSleep(void) {
    // Turn LCD backlight on
    SetLDO3(true);
    // Enable ADCs
    SetAdcState(true);
}

// -- sleep
void AXP192::DeepSleep(uint64_t time_in_us) {
    PrepareToSleep();

    if (time_in_us > 0) {
        esp_sleep_enable_timer_wakeup(time_in_us);
    } else {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    (time_in_us == 0) ? esp_deep_sleep_start() : esp_deep_sleep(time_in_us);

    // Never reached - after deep sleep ESP32 restarts
}

void AXP192::LightSleep(uint64_t time_in_us) {
    PrepareToSleep();

    if (time_in_us > 0) {
        esp_sleep_enable_timer_wakeup(time_in_us);
    } else {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    esp_light_sleep_start();

    RestoreFromLightSleep();
}

// Get current battery level
float AXP192::GetBatteryLevel(void) {
    const float batVoltage = GetBatVoltage();
    const float batPercentage =
        (batVoltage < 3.248088) ? (0) : (batVoltage - 3.120712) * 100;
    return (batPercentage <= 100) ? batPercentage : 100;
}

uint8_t AXP192::GetWarningLeve(void) {
    Wire1.beginTransmission(AXP_ADDR);
    Wire1.write(0x47);
    Wire1.endTransmission();
    Wire1.requestFrom(AXP_ADDR, 1);
    uint8_t buf = Wire1.read();
    return (buf & 0x01);
}

uint8_t AXP192::GetWarningLevel(void) {
    return Read8bit(0x47) & 0x01;
}

float AXP192::GetBatVoltage() {
    float ADCLSB    = 1.1 / 1000.0;
    uint16_t ReData = Read12Bit(0x78);
    return ReData * ADCLSB;
}

float AXP192::GetBatCurrent() {
    float ADCLSB        = 0.5;
    uint16_t CurrentIn  = Read13Bit(0x7A);
    uint16_t CurrentOut = Read13Bit(0x7C);
    return (CurrentIn - CurrentOut) * ADCLSB;
}

float AXP192::GetVinVoltage() {
    float ADCLSB    = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit(0x56);
    return ReData * ADCLSB;
}

float AXP192::GetVinCurrent() {
    float ADCLSB    = 0.625;
    uint16_t ReData = Read12Bit(0x58);
    return ReData * ADCLSB;
}

float AXP192::GetVBusVoltage() {
    float ADCLSB    = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit(0x5A);
    return ReData * ADCLSB;
}

float AXP192::GetVBusCurrent() {
    float ADCLSB    = 0.375;
    uint16_t ReData = Read12Bit(0x5C);
    return ReData * ADCLSB;
}

float AXP192::GetTempInAXP192() {
    float ADCLSB             = 0.1;
    const float OFFSET_DEG_C = -144.7;
    uint16_t ReData          = Read12Bit(0x5E);
    return OFFSET_DEG_C + ReData * ADCLSB;
}

float AXP192::GetBatPower() {
    float VoltageLSB = 1.1;
    float CurrentLCS = 0.5;
    uint32_t ReData  = Read24bit(0x70);
    return VoltageLSB * CurrentLCS * ReData / 1000.0;
}

float AXP192::GetBatChargeCurrent() {
    float ADCLSB    = 0.5;
    uint16_t ReData = Read12Bit(0x7A);
    return ReData * ADCLSB;
}
float AXP192::GetAPSVoltage() {
    float ADCLSB    = 1.4 / 1000.0;
    uint16_t ReData = Read12Bit(0x7E);
    return ReData * ADCLSB;
}

float AXP192::GetBatCoulombInput() {
    uint32_t ReData = Read32bit(0xB0);
    return ReData * 65536 * 0.5 / 3600 / 25.0;
}

float AXP192::GetBatCoulombOut() {
    uint32_t ReData = Read32bit(0xB4);
    return ReData * 65536 * 0.5 / 3600 / 25.0;
}

void AXP192::SetCoulombClear() {
    Write1Byte(0xB8, 0x20);
}

void AXP192::SetLDO3(bool State) {
    uint8_t buf = Read8bit(0x12);
    if (State == true)
        buf = (1 << 3) | buf;
    else
        buf = ~(1 << 3) & buf;
    Write1Byte(0x12, buf);
}

uint8_t AXP192::AXPInState() {
    return Read8bit(0x00);
}
bool AXP192::isACIN() {
    return (Read8bit(0x00) & 0x80) ? true : false;
}
bool AXP192::isCharging() {
    return (Read8bit(0x00) & 0x04) ? true : false;
}
bool AXP192::isVBUS() {
    return (Read8bit(0x00) & 0x20) ? true : false;
}

void AXP192::SetLDOVoltage(uint8_t number, uint16_t voltage) {
    voltage = (voltage > 3300) ? 15 : (voltage / 100) - 18;
    switch (number) {
        // uint8_t reg, data;
        case 2:  //清零高四位,保留第四位,左移四位写入数据
            Write1Byte(0x28, (Read8bit(0x28) & 0X0F) | (voltage << 4));
            break;
        case 3:  //清零第四位,保留高四位,写入数据
            Write1Byte(0x28, (Read8bit(0x28) & 0XF0) | voltage);
            break;
    }
}

/// @param number 0=DCDC1 / 1=DCDC2 / 2=DCDC3
void AXP192::SetDCVoltage(uint8_t number, uint16_t voltage) {
    uint8_t addr;
    if (number > 2) return;
    voltage = (voltage < 700) ? 0 : (voltage - 700) / 25;
    switch (number) {
        case 0:
            addr = 0x26;
            break;
        case 1:
            addr = 0x23;
            break;
        case 2:
            addr = 0x27;
            break;
    }
    // Serial.printf("result:%hhu\n", (Read8bit(addr) & 0X80) | (voltage &
    // 0X7F)); Serial.printf("result:%d\n", (Read8bit(addr) & 0X80) | (voltage &
    // 0X7F)); Serial.printf("result:%x\n", (Read8bit(addr) & 0X80) | (voltage &
    // 0X7F));
    Write1Byte(addr, (Read8bit(addr) & 0X80) | (voltage & 0X7F));
}

void AXP192::SetESPVoltage(uint16_t voltage) {
    if (voltage >= 3000 && voltage <= 3400) {
        SetDCVoltage(0, voltage);  // DCDC1
    }
}

void AXP192::SetLcdVoltage(uint16_t voltage) {
    if (voltage >= 2500 && voltage <= 3300) {
        SetLDOVoltage(3, voltage);  // LDO3
    }
}

void AXP192::SetLDOEnable(uint8_t number, bool state) {
    uint8_t mark = 0x01;
    if ((number < 2) || (number > 3)) return;

    mark <<= number;
    if (state) {
        Write1Byte(0x12, (Read8bit(0x12) | mark));
    } else {
        Write1Byte(0x12, (Read8bit(0x12) & (~mark)));
    }
}


// void AXP192::SetLCDRSet(bool state) {
//     uint8_t reg_addr = 0x96;
//     uint8_t gpio_bit = 0x02;
//     uint8_t data;
//     data = Read8bit(reg_addr);

//     if (state) {
//         data |= gpio_bit;
//     } else {
//         data &= ~gpio_bit;
//     }

//     Write1Byte(reg_addr, data);
// }

// Select source for BUS_5V
// 0 : use internal boost
// 1 : powered externally
void AXP192::SetBusPowerMode(uint8_t state) {
    uint8_t data;
    if (state == 0) {
        // Set EXTEN to enable 5v boost
        data = Read8bit(0x10);
        Write1Byte(0x10, data | 0x04);
    } else {
        // Set EXTEN to disable 5v boost
        data = Read8bit(0x10);
        Write1Byte(0x10, data & ~0x04);
    }
}

// void AXP192::SetLed(uint8_t state) {
//     uint8_t reg_addr = 0x94;
//     uint8_t data;
//     data = Read8bit(reg_addr);

//     if (state) {
//         data = data & 0XFD;
//     } else {
//         data |= 0X02;
//     }

//     Write1Byte(reg_addr, data);
// }

// set led state(GPIO high active,set 1 to enable amplifier)

void AXP192::SetCHGCurrent(uint8_t state) {
    uint8_t data = Read8bit(0x33);
    data &= 0xf0;
    data = data | (state & 0x0f);
    Write1Byte(0x33, data);
}

// GPIO0-4 are used to control Groove port power
// GPIO0: Groove ports A1 & A2
// GPIO1: Groove port B1
// GPIO2: Groove port B2
// GPIO3: Groove port C1
// GPIO4: Groove port C2
void AXP192::SetGPIOOutput(uint8_t bitfield) {
    // GPIO2 / GPIO1 / GPIO0
    uint8_t data1 = Read8bit(0x94);
    data1 &= 0b11111000;
    data1 |= (bitfield & 0b00000111);
    Write1Byte(0x94, data1);
    // GPIO4 / GPIO3
    uint8_t data2 = Read8bit(0x96);
    data2 &= 0b11111100;
    data2 |= (bitfield & 0b00011000) >> 3;
    Write1Byte(0x96, data2);
}

uint8_t AXP192::GetGPIOOutput()
{
    // GPIO2 / GPIO1 / GPIO0
    uint8_t data1 = Read8bit(0x94);
    data1 &= 0b00000111;
    // GPIO4 / GPIO3
    uint8_t data2 = Read8bit(0x96);
    data2 &= 0b00000011;

    return (data2 << 3) | data1;
}

// Sample call: M5.Axp.SetGroovePower(AXP192::kGroovePort_A1_A2, AXP192::kGroovePower_ON);
void AXP192::SetGroovePower(GroovePort port, GroovePower power)
{
    if(port >= kGroovePort_MAX) return;
    if(power >= kGroovePower_MAX) return;

    uint8_t state = GetGPIOOutput();
    uint8_t mask;

    if(port == kGroovePort_ALL) {
        mask = 0b00011111;
    } else {
        mask = 0b00000001 << port;
    }
    state &= 0b00011111;
    if(power == kGroovePower_OFF) {
        SetGPIOOutput(state & ~mask);
    } else {
        SetGPIOOutput(state | mask);
    }
}
