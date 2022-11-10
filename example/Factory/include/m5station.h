#ifndef __M5STATION_H__
#define __M5STATION_H__

// Pin MAP
#define BTN_A_PIN 37
#define BTN_B_PIN 38
#define BTN_C_PIN 39

#define NEO_PIXEL_PIN 4
#define NEO_PIXEL_NUM 7

#define PORTA_I2C_NUM I2C_NUM_1
#define PORTA_SDA_PIN 32
#define PORTA_SCL_PIN 33

#define PORTB1_IO1_PIN 25
#define PORTB1_IO2_PIN 35

#define PORTB2_IO1_PIN 26
#define PORTB2_IO2_PIN 36

#define PORTC1_TXD_PIN 14
#define PORTC1_RXD_PIN 13
#define PortC1_Serial  Serial1

#define PORTC2_TXD_PIN 17
#define PORTC2_RXD_PIN 16
#define PortC2_Serial  Serial2

#define PORTUSB_ADC_PIN 34
#define PORTUSB_PWR_PIN 12

#define PORT485_Serial Serial
#define PORT485_DE_RE  2

// Use SIM7020 Modem
#define TINY_GSM_MODEM_SIM7020

/* Define the serial console for debug prints, if needed */
#define TINY_GSM_DEBUG        Serial  // debug message dispaly on screen
#define DEBUG_DUMP_AT_COMMAND 0

// MQTT Setting
#define MQTT_BROKER   "mqtt.m5stack.com"
#define MQTT_PORT     1883
#define MQTT_USERNAME "A-DTU-NB-"

#define SINGLE_UP_TOPIC   "$m5things/v1/M5%sM5/up/single/%d"
#define DATA_UP_TOPIC     "$m5things/v1/M5%sM5/up/data/"
#define COMMAND_UP_TOPIC  "$m5things/v1/M5%sM5/up/command/"
#define EVENT_UP_TOPIC    "$m5things/v1/M5%sM5/up/event/"
#define CONFIG_UP_TOPIC   "$m5things/v1/M5%sM5/up/config/"
#define RAW_DATA_UP_TOPIC "$m5things/v1/M5%sM5/up/raw/"

#define SINGLE_DOWN_TOPIC   "$m5things/v1/M5%sM5/down/single/#"
#define DATA_DOWN_TOPIC     "$m5things/v1/M5%sM5/down/data/"
#define COMMAND_DOWN_TOPIC  "$m5things/v1/M5%sM5/down/command/"
#define EVENT_DOWN_TOPIC    "$m5things/v1/M5%sM5/down/event/"
#define CONFIG_DOWN_TOPIC   "$m5things/v1/M5%sM5/down/config/"
#define RAW_DATA_DOWN_TOPIC "$m5things/v1/M5%sM5/down/raw/"

#define MACADDRSTR "%02X%02X%02X%02X%02X%02X"

typedef enum {
    PORT_A1_NEO,
    PORT_B1_NEO,
    PORT_C1_NEO,
    BTN_PWR_NEO,
    PORT_C2_NEO,
    PORT_B2_NEO,
    PORT_A2_NEO,
    NEO_MAX
} neopixel_index_type_t;

typedef enum {
    PORT_A1,
    PORT_B1,
    PORT_C1,
    PORT_USB,
    PORT_C2,
    PORT_B2,
    PORT_A2,
    PORT_MAX
} port_index_type_t;

typedef enum {
    PAGE_MAIN,
    PAGE_POWER_CTRL,
    PAGE_I2C_CTRL,
    PAGE_GPIO_CTRL,
    PAGE_RS485_CTRL,
    PAGE_PAGE_MAX
} page_index_type_t;

// images resources
extern const unsigned char img_startup[64800];
extern const unsigned char img_background[64800];

#endif  // ! __M5STATION_H__