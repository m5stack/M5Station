
#include "factory.h"

// PORT口电源管理
// PORTA1跟PORTA2电源相通
void port_power_control(port_index_type_t index, bool value) {
    switch (index) {
        case PORT_A1:
        case PORT_A2:
            // AXP192 GPIO0 控制
            if (value) {
                M5.Power.Axp192.writeRegister8(0x90, 0x06);
            } else {
                M5.Power.Axp192.writeRegister8(0x90, 0x05);
            }
            break;
        case PORT_B1:
            // AXP192 GPIO1 控制
            if (value) {
                M5.Power.Axp192.writeRegister8(0x92, 0x06);
            } else {
                M5.Power.Axp192.writeRegister8(0x92, 0x05);
            }
            break;
        case PORT_C1:
            // AXP192 GPIO3 控制
            {
                uint8_t data = M5.Power.Axp192.readRegister8(0x96);
                if (value) {
                    data |= 0x01;
                } else {
                    data &= ~0x01;
                }
                M5.Power.Axp192.writeRegister8(0x96, data);
            }
            break;
        case PORT_B2:
            // AXP192 GPIO2 控制
            if (value) {
                M5.Power.Axp192.writeRegister8(0x93, 0x06);
            } else {
                M5.Power.Axp192.writeRegister8(0x93, 0x05);
            }
            break;
        case PORT_C2:
            // AXP192 GPIO4 控制
            {
                uint8_t data = M5.Power.Axp192.readRegister8(0x96);
                if (value) {
                    data |= 0x02;
                } else {
                    data &= ~0x02;
                }
                M5.Power.Axp192.writeRegister8(0x96, data);
            }
            break;
        case PORT_USB:
            // ESP32 GPIO12 控制
            {
                if (value) {
                    digitalWrite(PORTUSB_PWR_PIN, HIGH);
                } else {
                    digitalWrite(PORTUSB_PWR_PIN, LOW);
                }
            }
        default:
            break;
    }
}

void axp192_init() {
    M5.Power.Axp192.writeRegister8(0x90,
                                   0b00000000);  // AXP192 GPIO0 开漏输出
    M5.Power.Axp192.writeRegister8(0x92,
                                   0b00000000);  // AXP192 GPIO1 开漏输出
    M5.Power.Axp192.writeRegister8(0x93,
                                   0b00000000);  // AXP192 GPIO2 开漏输出
    M5.Power.Axp192.writeRegister8(0x95,
                                   0b10000101);  // AXP192 GPIO3&4 开漏输出
    M5.Power.Axp192.writeRegister8(0x33,
                                   0b11000111);  // AXP192 充电 4.2V 700mA
    M5.Power.Axp192.setEXTEN(1);
}

void current_measure_init() {
    ina_0.begin(&Wire);
    ina_0.reset();
    ina_0.setShuntRes(10, 10, 10);
    ina_1.begin(&Wire);
    ina_1.reset();
    ina_1.setShuntRes(10, 10, 10);
}

extern int get_battery_voltage(void);
extern float usb_vref;

void dis_current() {
    canvas.setTextSize((float)canvas.width() / 150);  // Use small font size
    canvas.fillRect(0, 0, canvas.width(), 60, 0);
    canvas.setTextColor(TFT_GREEN);

    canvas.setCursor(3, 0);
    canvas.setTextColor(TFT_GREEN);
    canvas.printf("CHN  I   V   CHN  I   V");
    canvas.setTextColor(TFT_WHITE);
    canvas.setCursor(3, 15);
    canvas.setTextColor(TFT_WHITE);
    canvas.printf(
        "A1%3.0fma %1.1fV A2%3.0fma %1.1fV\r\n",
        ina_0.getCurrent(INA3221_CH1) * 1000, ina_0.getVoltage(INA3221_CH1),
        ina_0.getCurrent(INA3221_CH2) * 1000, ina_0.getVoltage(INA3221_CH2));
    canvas.setCursor(3, 28);
    canvas.printf(
        "B1%3.0fma %1.1fV B2%3.0fma %1.1fV\r\n",
        ina_0.getCurrent(INA3221_CH3) * 1000, ina_0.getVoltage(INA3221_CH3),
        ina_1.getCurrent(INA3221_CH1) * 1000, ina_1.getVoltage(INA3221_CH1));
    canvas.setCursor(3, 41);
    canvas.printf(
        "C1%3.0fma %1.1fV C2%3.0fma %1.1fV\r\n",
        ina_1.getCurrent(INA3221_CH2) * 1000, ina_1.getVoltage(INA3221_CH2),
        ina_1.getCurrent(INA3221_CH3) * 1000, ina_1.getVoltage(INA3221_CH3));

    canvas.setCursor(55, 60);
    canvas.fillRect(60, 60, 120, 40, TFT_BLACK);
    float usb_vol = get_battery_voltage() / 1000.0f;
    Serial.printf("vol: %f\r\n", usb_vol);
    float usb_current = ((usb_vol - usb_vref) / 50.0f / 0.01f);
    canvas.printf("  USB: %0.03fA  ",
                  abs(usb_current) < 0.010 ? 0.0 : usb_current);

    canvas.setCursor(80, 80);
    Wire.beginTransmission(0x34);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(0x34, 1);
    bool status = Wire.read() & 0x04;
    canvas.println(status ? "Charging" : "Not Charge");
    canvas.setCursor(0, 120);
    canvas.printf("WiFi   %s     Power",
                  has_imu_flag ? "     IMU " : "    RS485");
}

void rs485_echo() {
    xSemaphoreTake(tMutex, portMAX_DELAY);
    canvas.fillSprite(BLACK);
    canvas.setTextSize(3);
    canvas.setCursor(35, 40);
    canvas.print("RS485 TEST");
    canvas.pushSprite(0, 0);
    while (1) {
        if (Serial.available()) {
            char data = Serial.read();
            digitalWrite(PORT485_DE_RE, HIGH);  // DE&RE 高电平发送数据
            delayMicroseconds(75);
            Serial.write(data);
            delayMicroseconds(75);
            digitalWrite(PORT485_DE_RE, LOW);  // DE&RE
        }
        if (mode != last_mode) {
            last_mode = mode;
            canvas.fillSprite(BLACK);
            break;
        }
    }
    xSemaphoreGive(tMutex);
}

void draw_port_io_status(const int &color) {
    canvas.fillRect(10, 60, 15, 15, color);  // PORTA_SDA_PIN
    canvas.fillRect(30, 60, 15, 15, color);  // PORTA_SCL_PIN

    canvas.fillRect(10, 80, 15, 15, color);  // PORTB1_IO1_PIN

    canvas.fillRect(10, 100, 15, 15, color);  // PORTC1_TXD_PIN
    canvas.fillRect(30, 100, 15, 15, color);  // PORTC1_RXD_PIN

    canvas.fillRect(195, 60, 15, 15, color);  // PORTA_SDA_PIN
    canvas.fillRect(215, 60, 15, 15, color);  // PORTA_SCL_PIN

    canvas.fillRect(215, 80, 15, 15, color);  // PORTB2_IO1_PIN

    canvas.fillRect(195, 100, 15, 15, color);  // PORTC2_TXD_PIN
    canvas.fillRect(215, 100, 15, 15, color);  // PORTC2_RXD_PIN
}

void gpio_test() {
    if (((millis() - reverse_io_lasttime) > 1500) && port_power_flag) {
        canvas.fillSprite(BLACK);
        reverse_io_lasttime = millis();
        port_pin_flag       = !port_pin_flag;
        // Grove口IO翻转
        for (size_t i = 0; i < 8; i++) {
            digitalWrite(port_pin_table[i], port_pin_flag);
        }
        // 显示颜色
        if (port_pin_flag) {
            draw_port_io_status(TFT_BLUE);
        } else {
            draw_port_io_status(TFT_LIGHTGRAY);
        }
        canvas.setCursor(57, 100);
        canvas.setTextSize((float)canvas.width() / 150);  // Use small font size
        if (wifi_scan_flag == 0) {
            canvas.print("WiFi Not Start");
        } else if (wifi_scan_flag == 1) {
            canvas.printf(" WiFi Scan...");
        } else if (wifi_scan_flag == 2) {
            canvas.setTextColor(TFT_RED);
            canvas.printf("  Not Found :( ");
            canvas.setTextColor(TFT_WHITE);
        } else if (wifi_scan_flag == 3){
            canvas.setTextColor(TFT_WHITE);
            canvas.printf("  %s:%02d", ssid, rssi);
        } else {
            canvas.printf(">>>M5STATION<<<");
        }
    }

    // 6s
    if ((millis() - power_control_lasttime) > 6000) {
        // 开启/关闭电源
        power_control_lasttime = millis();
        port_power_flag        = !port_power_flag;
        for (size_t i = 0; i < 8; i++) {
            port_power_control(port_def_table[i], port_power_flag);
        }
    }

    if (!port_power_flag && (millis() - power_control_lasttime) > 1000) {
        port_power_flag = !port_power_flag;
        for (size_t i = 0; i < 8; i++) {
            port_power_control(port_def_table[i], port_power_flag);
        }
    }

    // PortB 输入管脚
    if (digitalRead(PORTB1_IO2_PIN) == HIGH) {
        canvas.fillRect(30, 80, 15, 15, TFT_BLUE);  // PORTB1_IO1_PIN
    } else {
        canvas.fillRect(30, 80, 15, 15, TFT_LIGHTGRAY);  // PORTB1_IO1_PIN
    }

    if (digitalRead(PORTB2_IO2_PIN) == HIGH) {
        canvas.fillRect(195, 80, 15, 15, TFT_BLUE);  // PORTB2_IO1_PIN
    } else {
        canvas.fillRect(195, 80, 15, 15, TFT_LIGHTGRAY);  // PORTB2_IO1_PIN
    }

    // 显示每个Grove口的电流电压 ==
    canvas.fillRect(0, 0, canvas.width(), 60, 0);
    dis_current();
}

void rtc_test() {
    for (size_t i = 0; i < 8; i++) {
        port_power_control(port_def_table[i], false);
    }
    canvas.setTextSize(2);
    canvas.setCursor(10, 10);
    canvas.setTextColor(TFT_RED);
    canvas.println("Power shutdown...");
    canvas.setCursor(10, 40);
    canvas.setTextColor(TFT_WHITE);
    canvas.println("Will wake up after   5s :)");
    canvas.pushSprite(0, 0);
    delay(3000);
    M5.Rtc.clearIRQ();
    M5.Rtc.setAlarmIRQ(5);
    delay(10);
    M5.Power.Axp192.powerOff();
}

void led_brithe() {
    for (size_t i = 100; i > 0; i--) {
        FastLED.setBrightness(i);
        FastLED.show();
    }
    for (size_t i = 0; i < 100; i++) {
        FastLED.setBrightness(i);
        FastLED.show();
    }
}

void set_led_color(int color) {
    for (size_t i = 0; i < 7; i++) {
        NeoPixel[i] = color;
    }
    FastLED.show();
}

void btn_task(void *arg) {
    while (1) {
        if (!digitalRead(BTN_A_PIN)) {
            mode           = GPIO_TEST;
            wifi_scan_flag = 1;
        } else if (!digitalRead(BTN_B_PIN)) {
            if (has_imu_flag) {
                mode = IMU_TEST;
            } else {
                mode = RS485_TEST;
            }
        } else if (!digitalRead(BTN_C_PIN)) {
            mode = RTC_TEST;
        }
        vTaskDelay(200);
    }
}

void wifi_task(void *arg) {
    WiFi.mode(WIFI_STA);

    while (1) {
        xSemaphoreTake(tMutex, portMAX_DELAY);
        if (wifi_scan_flag == 1) {
            wifi_scan_num = WiFi.scanNetworks(false, false, false, 50);
            // return the number of networks
            // found. 返回发现的网络数
            if (wifi_scan_num == 0) {
                wifi_scan_flag = 2;
                Serial.printf("Not Found APs\r\n");
            } else {
                // If have network is found.  找到网络
                wifi_scan_flag = 3;
                for (int i = 0; i < wifi_scan_num; ++i) {
                    // Print SSID and RSSI for each network found.
                    // 打印每个找到的网络的SSID和信号强度
                    Serial.printf("%d:", i + 1);
                    Serial.print(WiFi.SSID(i));
                    Serial.printf("(%d)\r\n", WiFi.RSSI(i));
                }

                if (WiFi.SSID(0).length() > 7) {
                    ssid = WiFi.SSID(0).substring(0, 6);
                } else {
                    ssid = WiFi.SSID(0);
                }
                rssi = WiFi.RSSI(0);
            }
        }
        xSemaphoreGive(tMutex);
        vTaskDelay(100);
    }
}

typedef struct {
    double x;
    double y;
    double z;
} point_3d_t;

typedef struct {
    point_3d_t start_point;
    point_3d_t end_point;
} line_3d_t;

typedef struct {
    double x;
    double y;
} point_2d_t;

double r_rand = PI / 180;

double r_alpha = 19.47 * PI / 180;
double r_gamma = 20.7 * PI / 180;

double sin_alpha = sin(19.47 * PI / 180);
double cos_alpha = cos(19.47 * PI / 180);
double sin_gamma = sin(20.7 * PI / 180);
double cos_gamma = cos(20.7 * PI / 180);

void RotatePoint(point_3d_t *point, double x, double y, double z) {
    if (x != 0) {
        point->y = point->y * cos(x * r_rand) - point->z * sin(x * r_rand);
        point->z = point->y * sin(x * r_rand) + point->z * cos(x * r_rand);
    }

    if (y != 0) {
        point->x = point->z * sin(y * r_rand) + point->x * cos(y * r_rand);
        point->z = point->z * cos(y * r_rand) - point->x * sin(y * r_rand);
    }

    if (z != 0) {
        point->x = point->x * cos(z * r_rand) - point->y * sin(z * r_rand);
        point->y = point->x * sin(z * r_rand) + point->y * cos(z * r_rand);
    }
}

void RotatePoint(point_3d_t *point, point_3d_t *point_new, double x, double y,
                 double z) {
    if (x != 0) {
        point_new->y = point->y * cos(x * r_rand) - point->z * sin(x * r_rand);
        point_new->z = point->y * sin(x * r_rand) + point->z * cos(x * r_rand);
    }

    if (y != 0) {
        point_new->x = point->z * sin(y * r_rand) + point->x * cos(y * r_rand);
        point_new->z = point->z * cos(y * r_rand) - point->x * sin(y * r_rand);
    }

    if (z != 0) {
        point_new->x = point->x * cos(z * r_rand) - point->y * sin(z * r_rand);
        point_new->y = point->x * sin(z * r_rand) + point->y * cos(z * r_rand);
    }
}

bool point3Dto2D(point_3d_t *source, point_2d_t *point) {
    point->x = (source->x * cos_gamma) - (source->y * sin_gamma);
    point->y = -(source->x * sin_gamma * sin_alpha) -
               (source->y * cos_gamma * sin_alpha) + (source->z * cos_alpha);
    return true;
}

bool point2DToDisPoint(point_2d_t *point, uint8_t *x, uint8_t *y) {
    *x = point->x + 80;
    *y = 40 - point->y;
    return true;
}

bool printLine3D(M5Canvas *display, line_3d_t *line, uint32_t color) {
    uint8_t start_x = 0, start_y = 0, end_x = 0, end_y = 0;
    point_2d_t point;
    point3Dto2D(&line->start_point, &point);
    point2DToDisPoint(&point, &start_x, &start_y);
    point3Dto2D(&line->end_point, &point);
    point2DToDisPoint(&point, &end_x, &end_y);
    display->drawLine(start_x + 35, start_y + 35, end_x + 35, end_y + 35,
                      color);

    return true;
}

line_3d_t rect[12] = {
    {.start_point = {-1, -1, 1}, .end_point = {1, -1, 1}},
    {.start_point = {1, -1, 1}, .end_point = {1, 1, 1}},
    {.start_point = {1, 1, 1}, .end_point = {-1, 1, 1}},
    {.start_point = {-1, 1, 1}, .end_point = {-1, -1, 1}},
    {
        .start_point = {-1, -1, 1},
        .end_point   = {-1, -1, -1},
    },
    {
        .start_point = {1, -1, 1},
        .end_point   = {1, -1, -1},
    },
    {
        .start_point = {1, 1, 1},
        .end_point   = {1, 1, -1},
    },
    {
        .start_point = {-1, 1, 1},
        .end_point   = {-1, 1, -1},
    },
    {.start_point = {-1, -1, -1}, .end_point = {1, -1, -1}},
    {.start_point = {1, -1, -1}, .end_point = {1, 1, -1}},
    {.start_point = {1, 1, -1}, .end_point = {-1, 1, -1}},
    {.start_point = {-1, 1, -1}, .end_point = {-1, -1, -1}},
};

void imu_test() {
    float accX = 0;
    float accY = 0;
    float accZ = 0;

    double theta = 0, last_theta = 0;
    double phi = 0, last_phi = 0;
    double alpha = 0.2;

    line_3d_t x = {.start_point = {0, 0, 0}, .end_point = {0, 0, 0}};
    line_3d_t y = {.start_point = {0, 0, 0}, .end_point = {0, 0, 0}};
    line_3d_t z = {.start_point = {0, 0, 0}, .end_point = {0, 0, 22}};

    line_3d_t rect_source[12];
    line_3d_t rect_dis;

    for (int n = 0; n < 12; n++) {
        rect_source[n].start_point.x = rect[n].start_point.x * 22;
        rect_source[n].start_point.y = rect[n].start_point.y * 22;
        rect_source[n].start_point.z = rect[n].start_point.z * 22;
        rect_source[n].end_point.x   = rect[n].end_point.x * 22;
        rect_source[n].end_point.y   = rect[n].end_point.y * 22;
        rect_source[n].end_point.z   = rect[n].end_point.z * 22;
    }

    while (1) {
        M5.Imu.Mpu6886.getAccel(&accX, &accY, &accZ);
        if ((accX < 1) && (accX > -1)) {
            theta = asin(-accX) * 57.295;
        }
        // accZ = accZ - 0.1;
        if (accZ != 0) {
            phi = atan(accY / accZ) * 57.295;
        }

        theta = alpha * theta + (1 - alpha) * last_theta;
        phi   = alpha * phi + (1 - alpha) * last_phi;

        canvas.fillSprite(BLACK);
        canvas.setTextSize(1);
        canvas.setCursor(10, 115);
        canvas.printf("%.2f", theta);
        canvas.setCursor(10, 125);
        canvas.printf("%.2f", phi);

        canvas.setCursor(55, 125);
        canvas.printf("ax:%.1f  ay:%.1f  az:%.1f\r\n", accX, accY, accZ);
        delay(20);

        z.end_point.x = 0;
        z.end_point.y = 0;
        z.end_point.z = 60;
        RotatePoint(&z.end_point, theta, phi, 0);
        RotatePoint(&z.end_point, &x.end_point, -90, 0, 0);
        RotatePoint(&z.end_point, &y.end_point, 0, 90, 0);
        for (int n = 0; n < 12; n++) {
            RotatePoint(&rect_source[n].start_point, &rect_dis.start_point,
                        theta, phi, (double)0);
            RotatePoint(&rect_source[n].end_point, &rect_dis.end_point, theta,
                        phi, (double)0);
            printLine3D(&canvas, &rect_dis, TFT_WHITE);
        }
        // canvas.fillRect(0,0,160,80,BLACK);
        printLine3D(&canvas, &x, TFT_RED);
        printLine3D(&canvas, &y, TFT_GREEN);
        printLine3D(&canvas, &z, TFT_BLUE);
        canvas.pushSprite(0, 0);
        last_theta = theta;
        last_phi   = phi;
        if (mode != last_mode) {
            last_mode = mode;
            canvas.fillSprite(BLACK);
            break;
        }
    }
}