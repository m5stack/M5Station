#include <M5GFX.h>
#include <M5Unified.h>
#include <FastLED.h>
#include <Beastdevices_INA3221.h>
#include "m5station.h"
#include "WiFi.h"
#include "ArduinoModbus.h"

typedef enum { GPIO_TEST = 0, RS485_TEST, IMU_TEST, RTC_TEST } test_mode_t;

extern test_mode_t mode;
extern test_mode_t last_mode;
extern M5Canvas canvas;

extern time_t reverse_io_lasttime;
extern time_t power_control_lasttime;
extern bool port_pin_flag;
extern bool port_power_flag;
extern bool has_imu_flag;
extern uint8_t wifi_scan_flag;
extern uint8_t wifi_scan_num;
extern uint8_t port_pin_table[];
extern port_index_type_t port_def_table[];
extern int rssi;
extern String ssid;
extern Beastdevices_INA3221 ina_0;
extern Beastdevices_INA3221 ina_1;
extern xSemaphoreHandle tMutex;
extern CRGB NeoPixel[];

void port_power_control(port_index_type_t index, bool value);
void axp192_init();
void dis_current();
void current_measure_init();
void rs485_echo();
void gpio_test();
void rtc_test();
void btn_task(void* arg);
void wifi_task(void* arg);
void led_brithe();
void set_led_color(int color);
void imu_test();
void draw_port_io_status(const int &color);