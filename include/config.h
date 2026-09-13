#pragma once

// =============================================================
//  config.h - Dalek ESP32 unified firmware
//
//  Edit pin assignments, thresholds and motor flags here.
//
//  WiFi credentials AND API token live in secrets.ini
//  (never commit that file).
// =============================================================

// =============================================================
//  DEBUG
// =============================================================

#define DEBUG

#ifdef DEBUG
  #define DBG(x)   Serial.print(x)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

// =============================================================
//  SECURITY
// =============================================================

#ifndef API_TOKEN
  #define API_TOKEN "change_me_dalek_token"
#endif

#ifndef OTA_PASSWORD_HASH
  #define OTA_PASSWORD_HASH "d41d8cd98f00b204e9800998ecf8427e"
#endif

// =============================================================
//  PIN ASSIGNMENTS
//  ESP32 DevKitC V4 - ESP32-WROOM-32U, 38 pin
// =============================================================

// -- Ultrasonic sensors (Maxbotix EZ1, PWM output) ------------
//    GPIO32 = common trigger -> all three sensors RX
//    GPIO34 = right sensor PWM input
//    GPIO35 = center sensor PWM input
//    GPIO33 = left sensor PWM input
//
//    Add 10k pulldown resistors on GPIO34/35/33 to prevent
//    floating inputs when sensors are disconnected.

#define PIN_SONIC_TRIGGER   32
#define PIN_SONIC_RIGHT     34
#define PIN_SONIC_CENTER    35
#define PIN_SONIC_LEFT      33

// -- Stepper motors (Big Easy Driver: STEP + DIR) -------------
#define PIN_LEFT_STEP       25
#define PIN_LEFT_DIR        26
#define PIN_RIGHT_STEP      27
#define PIN_RIGHT_DIR       14

// -- Motor direction inversion --------------------------------
#define INVERT_LEFT_MOTOR   false
#define INVERT_RIGHT_MOTOR  false

// -- DFPlayer Mini (hardware UART2) ----------------------------
#define PIN_DFPLAYER_RX     16
#define PIN_DFPLAYER_TX     17

// -- FastLED WS2811 eyestalk ----------------------------------
#define PIN_LED_DATA         4
#define NUM_LEDS             1
#define LED_CHIPSET         WS2811
#define LED_COLOR_ORDER     RGB

// =============================================================
//  SENSOR THRESHOLDS (centimetres)
// =============================================================

#define SONIC_MIN_CM         30
#define SONIC_MID_CM         50
#define SONIC_MAX_CM        300
#define SONIC_HYSTERESIS_CM   8

// Continuous reverse: after this many consecutive reverse
// decision cycles, choose the more open side instead.
#define MOTOR_REVERSE_ESCAPE_LIMIT   3

#define SONIC_PULSE_TIMEOUT_US       20000UL

// =============================================================
//  MOTOR SETTINGS
// =============================================================

#define MOTOR_MAX_SPEED              2000
#define MOTOR_ACCEL                  2000
#define MOTOR_TURN_SLOW_DIV          5

// =============================================================
//  SOUND / LED SETTINGS
// =============================================================

#define DEFAULT_VOLUME               25
#define SND_EXTERMINATE_VOLUME       30
#define BORED_COUNT_MAX              3
#define BORED_INTERVAL_MS            900000UL
#define PULSE_INTERVAL_MS            10000UL
#define BOOT_DELAY_MS                3000UL

// =============================================================
//  DFPLAYER FILES
// =============================================================

#define SND_FOLDER                   10
#define SND_EXTERMINATE              1
#define SND_MOAN                     3
#define SND_STAY_AWAY                4
#define SND_REALLY_BORED            10
