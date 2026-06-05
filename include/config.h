#pragma once

// =============================================================
//  config.h  -  Dalek ESP32 unified firmware
//  Edit pin assignments, thresholds and motor flags here.
//  WiFi credentials AND API token live in secrets.ini
//  (never commit that file).
// =============================================================

// =============================================================
//  DEBUG
//  Comment out the #define to disable all serial output.
//  When disabled: no Serial.begin(), zero runtime overhead,
//  and the serial monitor will show nothing.
//
//  TIP: for production builds leave this commented out, or
//  set it via secrets.ini:
//    build_flags = ... -DDEBUG
// =============================================================
// #define DEBUG

#ifdef DEBUG
  #define DBG(x)   Serial.print(x)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

// =============================================================
//  SECURITY
//  API_TOKEN is injected via secrets.ini as a build flag:
//    -DAPI_TOKEN=\"your_secret_token_here\"
//  Every HTTP request must include the header:
//    X-Token: <your_secret_token_here>
//  The web UI sends it automatically.
//  If not set in secrets.ini this fallback is used — change it!
// =============================================================
#ifndef API_TOKEN
  #define API_TOKEN "change_me_dalek_token"
#endif

// OTA password hash (MD5 of your password).
// Generate with: echo -n "your_password" | md5sum
// Inject via secrets.ini:
//   -DOTA_PASSWORD_HASH=\"your_md5_hash\"
#ifndef OTA_PASSWORD_HASH
  #define OTA_PASSWORD_HASH "d41d8cd98f00b204e9800998ecf8427e"
#endif

// =============================================================
//  PIN ASSIGNMENTS  (ESP32 DevKitC V4 - WROOM-32U, 38 pin)
//
//  AVOID:
//    6-11   internal flash - never use
//    34-39  input-only (no pull-up/drive) - read only
//    0,2,12,15  strapping pins - avoid driving at boot
// =============================================================

// -- Ultrasonic sensors (Maxbotix EZ1, PWM output) ------------
//    GPIO 32 is a safe unrestricted output pin.
//    Pins 34-36 are input-only - fine for pulseIn reads.
//    Add 10k pull-down resistors on pins 34/35/36 in hardware
//    to prevent floating inputs when sensors are disconnected.
#define PIN_SONIC_TRIGGER   32   // common trigger -> all three sensors RX
#define PIN_SONIC_RIGHT     34   // PWM output of right sensor
#define PIN_SONIC_CENTER    35   // PWM output of center sensor
#define PIN_SONIC_LEFT      36   // PWM output of left sensor

// -- Stepper motors (Big Easy Driver: STEP + DIR per motor) ---
//    FastAccelStepper uses the ESP32 RMT peripheral on STEP pins.
//    Any output-capable GPIO works for DIR.
#define PIN_LEFT_STEP       25
#define PIN_LEFT_DIR        26
#define PIN_RIGHT_STEP      27
#define PIN_RIGHT_DIR       14

// -- Motor invert flags (flip if a motor runs the wrong way) --
//    Set to true to reverse that motor's direction in software.
#define INVERT_LEFT_MOTOR   false
#define INVERT_RIGHT_MOTOR  false

// -- DFPlayer Mini (hardware UART2) ---------------------------
#define PIN_DFPLAYER_RX     16   // ESP32 RX2  <- DFPlayer TX
#define PIN_DFPLAYER_TX     17   // ESP32 TX2  -> DFPlayer RX

// -- FastLED  WS2811 eyestalk ---------------------------------
#define PIN_LED_DATA         4
#define NUM_LEDS             1
#define LED_CHIPSET         WS2811
#define LED_COLOR_ORDER     RGB

// =============================================================
//  SENSOR THRESHOLDS  (centimetres)
// =============================================================
#define SONIC_MIN_CM   30    // inner limit  -> Exterminate
#define SONIC_MID_CM   50    // outer limit  -> Stay Away
#define SONIC_MAX_CM  300    // treat readings beyond this as clear

// Timeout for pulseIn per sensor (microseconds).
// 300 cm round-trip at 343 m/s ~ 17500 us. Add margin.
#define SONIC_PULSE_TIMEOUT_US  20000UL

// =============================================================
//  MOTOR SETTINGS
// =============================================================
#define MOTOR_MAX_SPEED      2000   // steps/sec
#define MOTOR_ACCEL          2000   // steps/sec^2
#define MOTOR_REVERSE_STEPS   200   // steps to back up when blocked
#define MOTOR_TURN_SLOW_DIV     5   // inner wheel speed = max / this

// =============================================================
//  SOUND / LED
// =============================================================
#define DEFAULT_VOLUME        25          // 0-30 (was 30; slightly less aggressive)
#define SND_EXTERMINATE_VOLUME 30         // full volume for exterminate only
#define BORED_COUNT_MAX         3         // moans before "really bored" palette
#define BORED_INTERVAL_MS  900000UL       // 15 minutes between bored events
#define PULSE_INTERVAL_MS   10000UL       // eyestalk pulse period
#define BOOT_DELAY_MS        3000UL       // startup animation duration

// Sound folder / track numbers (match your SD card layout)
#define SND_FOLDER          10
#define SND_EXTERMINATE      1
#define SND_MOAN             3
#define SND_STAY_AWAY        4
#define SND_REALLY_BORED    10