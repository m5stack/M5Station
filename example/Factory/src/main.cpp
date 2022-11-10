#include <M5GFX.h>
#include <M5Unified.h>
#include <FastLED.h>
#include <Beastdevices_INA3221.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "m5station.h"
#include "WiFi.h"
#include "ArduinoModbus.h"
#include "factory.h"

M5GFX display;
M5Canvas canvas(&display);


bool factory_mode = true; // 正常运行模式，只显示LOGO，呼吸灯

xSemaphoreHandle tMutex = NULL;
test_mode_t mode = GPIO_TEST;
test_mode_t last_mode = GPIO_TEST;
time_t reverse_io_lasttime    = 0;
time_t power_control_lasttime = 0;
bool port_pin_flag            = 0;
bool port_power_flag          = 0;
bool has_imu_flag             = 0;
int rssi                      = 0;
String ssid;
uint8_t wifi_scan_flag        = 0;
uint8_t wifi_scan_num         = 0;

uint8_t port_pin_table[] = {PORTA_SDA_PIN,  PORTA_SCL_PIN,  PORTB1_IO1_PIN,
                            PORTB2_IO1_PIN, PORTC1_TXD_PIN, PORTC1_RXD_PIN,
                            PORTC2_TXD_PIN, PORTC2_RXD_PIN};

port_index_type_t port_def_table[] = {PORT_A1, PORT_B1, PORT_C1, PORT_USB,
                                      PORT_C2, PORT_B2, PORT_A2, PORT_MAX};

Beastdevices_INA3221 ina_0(INA3221_ADDR40_GND);
Beastdevices_INA3221 ina_1(INA3221_ADDR41_VCC);

CRGB NeoPixel[7];

esp_adc_cal_characteristics_t *adc_chars;
float usb_vref;

int get_battery_voltage(void) {
    uint32_t adc_reading = 0;
    // Multisampling
    for (int i = 0; i < 64; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)ADC1_CHANNEL_6);
    }
    adc_reading /= 64;
    // Convert adc_reading to voltage in mV
    uint32_t voltage =
        (uint32_t)(esp_adc_cal_raw_to_voltage(adc_reading, adc_chars));
    Serial.printf("Raw: %d\tVoltage: %dmV\r\n", adc_reading, voltage);
    return voltage;
}

void setup() {
    M5.begin();
    // 电源监控
    // AXP192 GPIO init for power control
    tMutex = xSemaphoreCreateMutex();
    current_measure_init();

    M5.Power.Axp192.setEXTEN(1);

    axp192_init();

    pinMode(PORT485_DE_RE, OUTPUT);
    Serial.flush();

    // Neopixel
    FastLED.addLeds<SK6812, NEO_PIXEL_PIN, GRB>(NeoPixel, NEO_PIXEL_NUM);
    digitalWrite(PORT485_DE_RE, LOW);  // DE&RE

    // Grove口管脚初始化，共10个管脚，8个I/O，2个only input
    for (size_t i = 0; i < 8; i++) {
        pinMode(port_pin_table[i], OUTPUT);
    }
    // PortB1/2 有个管脚只能输入
    pinMode(PORTB1_IO2_PIN, INPUT);
    pinMode(PORTB2_IO2_PIN, INPUT);

    // USB口电源控制和ADC采样(电流)
    pinMode(PORTUSB_PWR_PIN, OUTPUT);
    digitalWrite(PORTUSB_PWR_PIN, HIGH);
    pinMode(PORTUSB_ADC_PIN, INPUT);

    // // 按键管脚初始化
    pinMode(BTN_A_PIN, INPUT_PULLUP);
    pinMode(BTN_B_PIN, INPUT_PULLUP);
    pinMode(BTN_C_PIN, INPUT_PULLUP);

    display.begin();

    if (display.width() < display.height()) {
        display.setRotation(display.getRotation() ^ 0);
    }

    // 开机画面
    canvas.createSprite(240, 135);
    canvas.setTextSize((float)canvas.width() / 150);  // Use small font size
    canvas.setTextSize(3);
    display.drawBitmap(0, 0, 240, 135, img_startup);

    for (size_t i = 0; i < 7; i++) {
        NeoPixel[i]     = CRGB(255, 0, 0);
        FastLED.show();
        delay(50);
    }
    for (size_t i = 0; i < 7; i++) {
        NeoPixel[i]     = CRGB(0, 255, 0);
        FastLED.show();
        delay(50);
    }
    for (size_t i = 0; i < 7; i++) {
        NeoPixel[i]     = CRGB(0, 0, 255);
        FastLED.show();
        delay(50);
    }
    FastLED.setBrightness(60);
    set_led_color(0x0000ff);

    // 产测模式
    xTaskCreate(btn_task,    /* Task function. */
                "btn_task",  /* String with name of task. */
                8096,        /* Stack size in bytes. */
                NULL,        /* Parameter passed as input of the task */
                1,           /* Priority of the task. */
                NULL);       /* Task handle. */
    xTaskCreate(wifi_task,   /* Task function. */
                "wifi_task", /* String with name of task. */
                8096,        /* Stack size in bytes. */
                NULL,        /* Parameter passed as input of the task */
                1,           /* Priority of the task. */
                NULL);       /* Task handle. */

    if (!M5.Rtc.isEnabled()) {
        Serial.println("RTC not found.");
        canvas.fillSprite(BLACK);
        canvas.drawString("RTC ERROR.", 25, 80);
        canvas.pushSprite(0, 0);
        for (;;) {
            vTaskDelay(500);
        }
    }

    if (!M5.Imu.begin()) {
        Serial.println("RTC not found.");
    } else {
        has_imu_flag = true;
    }
    canvas.fillSprite(BLACK);
    delay(200);

    for (size_t i = 0; i < 8; i++) {
        digitalWrite(port_pin_table[i], 1);
    }

    draw_port_io_status(TFT_RED);

    for (size_t i = 0; i < 8; i++) {
        port_power_control(port_def_table[i], 1);
    }

    // ADC初始化
    gpio_pad_select_gpio(PORTUSB_ADC_PIN);
    gpio_set_direction((gpio_num_t)PORTUSB_ADC_PIN, GPIO_MODE_INPUT);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc_chars = (esp_adc_cal_characteristics_t *)calloc(
        1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
                            3300, adc_chars);

    // usb_vref = get_battery_voltage();
    for (size_t i = 0; i < 5; i++) {
        usb_vref = usb_vref + get_battery_voltage();
    }

    usb_vref = usb_vref / 5.0f / 1000.0f;

}

static int color = 0;
void loop() {

    if (mode != last_mode) {
        last_mode = mode;
        canvas.fillSprite(BLACK);
    }
    switch (mode) {
        case GPIO_TEST:
            gpio_test();
            break;
        case IMU_TEST:
            imu_test();
            break;
        case RS485_TEST:
            rs485_echo();
            break;
        case RTC_TEST:
            rtc_test();
            break;
    }
    canvas.pushSprite(0, 0);
}
