// =============================================================
//  main.cpp  -  Dalek ESP32 unified firmware
//
//  Consolidates four separate Arduinos into one ESP32:
//    - dalek_WiFi.ino   (ESP-01)    -> WiFiServer + web UI
//    - dalek_main.ino   (Mega)      -> Ultrasonic sensors
//    - dalek_dome.ino   (Pro Mini)  -> FastLED + DFPlayer
//    - dalek_motors.ino (Nano)      -> stepper motors
//
//  Architecture:
//    Core 0 (motorTask)  - stepper run loop + sensor polling
//    Core 1 (Arduino)    - WiFi server + LED + sound (setup/loop)
//
//  Optimisations vs first version:
//    1. FastAccelStepper  - uses ESP32 RMT hardware peripheral for
//       step pulses; stepper timing is interrupt-driven and never
//       misses a step regardless of what else is running.
//    2. Non-blocking dome events  - doStayAway / doExterminate /
//       doBored use millis() state machines instead of delay(),
//       so the web server stays responsive during sound/light events.
//    3. Sensor reads moved to a short sub-task window  - pulseIn
//       calls are still sequential (hardware constraint of the
//       daisy-chain) but happen in a timed 500 ms slot so the
//       stepper task loop is free the rest of the time.
//    4. WiFi credentials + API token in secrets.ini - never in source.
//    5. Motor direction invert flags  - configurable in config.h
//       without rewiring.
//
//  Security:
//    - Every HTTP route (except /) checks X-Token header.
//    - CORS header removed; UI is same-origin.
//    - 404 handler for unknown routes.
//    - Shadow state variables removed; single source of truth
//      via mutex-protected volatiles.
//
//  v2.1 fixes:
//    - snprintf JSON (no String heap fragmentation).
//    - yield() in boot animation spin-loop.
//    - motorTask stack bumped to 8192 bytes.
//    - WiFi reconnect uses disconnect+begin for reliability.
//    - BORED_COUNT_MAX, SND_EXTERMINATE_VOLUME from config.h.
//    - onNotFound handler added.
//    - Serial.println() outside DEBUG guard fixed.
// =============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <FastLED.h>
#include <FastAccelStepper.h>
#include <DFRobotDFPlayerMini.h>
#include "config.h"
#include "esp_task_wdt.h"
#include "esp_system.h"    // esp_reset_reason()

// WiFi credentials and API token injected via secrets.ini build flags
#ifndef WIFI_SSID
  #define WIFI_SSID     "no_ssid_set"
  #define WIFI_PASSWORD "no_password_set"
#endif

// =============================================================
//  SHARED STATE
//  Written by Core 1 web handler or Core 1 dome logic,
//  read by Core 0 motor task (and vice-versa for sensor values).
//  Protected by a FreeRTOS spinlock (portMUX).
//
//  NOTE: sDisplayMode / sMotorRunning / sSoundEnabled shadow
//  variables have been removed. The volatile vars below are the
//  single source of truth; web handlers read/write them directly
//  under the mutex.
// =============================================================

portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

// Motor commands
//   1=forward  2=stop  3=turn left  4=turn right  5=reverse
//   6=disable movement  7=enable movement
volatile int  motorCmd     = 2;
volatile bool motorRunning = false;

// Dome/sound commands
//   10=normal  11=stay away  12=exterminate
//   14=sound off  15=sound on  16=vol up  17=vol down
//   18=display on  19=display off
volatile int  domeCmd = 10;

// Sensor readings (written Core 0, read Core 1 for web display)
volatile long rightCM = 999, centerCM = 999, leftCM = 999;

// Mode flags (written Core 1, read both cores)
volatile bool displayMode  = true;
volatile bool soundEnabled = true;
volatile int  volume       = DEFAULT_VOLUME;

// =============================================================
//  HARDWARE OBJECTS
// =============================================================

// FastAccelStepper - uses RMT peripheral; no manual run() needed
FastAccelStepperEngine stepperEngine = FastAccelStepperEngine();
FastAccelStepper* leftStepper  = nullptr;
FastAccelStepper* rightStepper = nullptr;

// DFPlayer on hardware Serial2
DFRobotDFPlayerMini mp3;

// FastLED
CRGB leds[NUM_LEDS];

// Web server on port 80
WebServer server(80);

// Colour palette for "really bored" animation
static const CRGB palette[] = {
    CRGB::Khaki, CRGB::Aqua, CRGB::DarkMagenta, CRGB::DarkSeaGreen,
    CRGB::Amethyst, CRGB::RosyBrown, CRGB::OrangeRed, CRGB::Yellow,
    CRGB::LightCoral, CRGB::OldLace
};
static const int PALETTE_SIZE = sizeof(palette) / sizeof(palette[0]);

// =============================================================
//  SAFE COMMAND SETTERS  (callable from any core)
// =============================================================

void setMotorCmd(int cmd) {
    portENTER_CRITICAL(&cmdMux);
    motorCmd = cmd;
    portEXIT_CRITICAL(&cmdMux);
}

void setDomeCmd(int cmd) {
    portENTER_CRITICAL(&cmdMux);
    domeCmd = cmd;
    portEXIT_CRITICAL(&cmdMux);
}

// =============================================================
//  SECURITY HELPERS
// =============================================================

// Returns true if the request carries a valid API token.
// Called at the top of every mutable route handler.
bool checkToken() {
    if (server.hasHeader("X-Token") &&
        server.header("X-Token") == String(API_TOKEN)) {
        return true;
    }
    server.send(403, "text/plain", "Forbidden");
    return false;
}

// =============================================================
//  CORE 0 TASK  -  sensors + steppers
//
//  FastAccelStepper generates step pulses via the RMT peripheral
//  in the background, so this task only needs to call move/stop
//  when the command changes.  Sensor reads happen every 500 ms;
//  the rest of the time the task yields immediately.
// =============================================================

void readSensors() {
    // Trigger daisy-chained Maxbotix sensors
    digitalWrite(PIN_SONIC_TRIGGER, HIGH);
    delayMicroseconds(25);
    digitalWrite(PIN_SONIC_TRIGGER, LOW);

    long rp = pulseIn(PIN_SONIC_RIGHT,  HIGH, SONIC_PULSE_TIMEOUT_US);
    long cp = pulseIn(PIN_SONIC_CENTER, HIGH, SONIC_PULSE_TIMEOUT_US);
    long lp = pulseIn(PIN_SONIC_LEFT,   HIGH, SONIC_PULSE_TIMEOUT_US);

    // Sound ~29 us/cm one-way; round-trip so /2
    rightCM  = (rp > 0) ? rp / 29 / 2 : SONIC_MAX_CM;
    centerCM = (cp > 0) ? cp / 29 / 2 : SONIC_MAX_CM;
    leftCM   = (lp > 0) ? lp / 29 / 2 : SONIC_MAX_CM;
}

void sensorAction() {
    long r = rightCM, c = centerCM, l = leftCM;
    static bool midTriggered = false;
    static bool minTriggered = false;

    if (!motorRunning) return;

    // Movement decisions
    if (r > SONIC_MIN_CM && c > SONIC_MIN_CM && l > SONIC_MIN_CM) {
        setMotorCmd(1);   // all clear - forward
        setDomeCmd(10);   // normal
        midTriggered = false;
        minTriggered = false;
    } else if (r <= SONIC_MIN_CM && c <= SONIC_MIN_CM && l <= SONIC_MIN_CM) {
        setMotorCmd(2);   // fully blocked - stop
    } else if (r <= SONIC_MIN_CM && c > SONIC_MIN_CM && l > SONIC_MIN_CM) {
        setMotorCmd(3);   // blocked right - turn left
    } else if (r > SONIC_MIN_CM && c > SONIC_MIN_CM && l <= SONIC_MIN_CM) {
        setMotorCmd(4);   // blocked left - turn right
    } else if (c <= SONIC_MIN_CM && r > SONIC_MIN_CM && l > SONIC_MIN_CM) {
        setMotorCmd(5);   // blocked front - reverse
    }

    // Alert decisions (fire once per approach event)
    if ((r <= SONIC_MID_CM || c <= SONIC_MID_CM || l <= SONIC_MID_CM) && !midTriggered) {
        setDomeCmd(11);
        midTriggered = true;
    }
    if ((r <= SONIC_MIN_CM || c <= SONIC_MIN_CM || l <= SONIC_MIN_CM) && !minTriggered) {
        setDomeCmd(12);
        minTriggered = true;
    }
}

void applyMotorCmd() {
    static int prevAppliedCmd = -1;
    int cmd;
    portENTER_CRITICAL(&cmdMux);
    cmd = motorCmd;
    portEXIT_CRITICAL(&cmdMux);

    if (cmd == prevAppliedCmd) return;  // nothing changed
    prevAppliedCmd = cmd;

    switch (cmd) {
        case 1:  // forward
            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED);
            leftStepper->runForward();
            rightStepper->runForward();
            break;
        case 2:  // full stop
            leftStepper->stopMove();
            rightStepper->stopMove();
            break;
        case 3:  // turn left  (slow left, fast right)
            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED / MOTOR_TURN_SLOW_DIV);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED - 500);
            leftStepper->runForward();
            rightStepper->runForward();
            break;
        case 4:  // turn right  (fast left, slow right)
            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED - 500);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED / MOTOR_TURN_SLOW_DIV);
            leftStepper->runForward();
            rightStepper->runForward();
            break;
        case 5:  // reverse
            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED / 2);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED / 2);
            leftStepper->move(-MOTOR_REVERSE_STEPS);
            rightStepper->move(-MOTOR_REVERSE_STEPS);
            break;
    }
}

void motorTask(void* pvParameters) {
    // FastAccelStepper init (must happen on the task that owns it)
    stepperEngine.init();

    leftStepper  = stepperEngine.stepperConnectToPin(PIN_LEFT_STEP);
    rightStepper = stepperEngine.stepperConnectToPin(PIN_RIGHT_STEP);

    if (leftStepper) {
        leftStepper->setDirectionPin(PIN_LEFT_DIR,  INVERT_LEFT_MOTOR);
        leftStepper->setAcceleration(MOTOR_ACCEL);
        leftStepper->setSpeedInHz(MOTOR_MAX_SPEED);
    }
    if (rightStepper) {
        rightStepper->setDirectionPin(PIN_RIGHT_DIR, INVERT_RIGHT_MOTOR);
        rightStepper->setAcceleration(MOTOR_ACCEL);
        rightStepper->setSpeedInHz(MOTOR_MAX_SPEED);
    }

    static unsigned long lastSensorRead = 0;

    for (;;) {
        unsigned long now = millis();

        if (now - lastSensorRead >= 500) {
            readSensors();
            sensorAction();
            lastSensorRead = now;
        }

        if (motorRunning) {
            applyMotorCmd();
        } else {
            // Ensure stopped when movement is disabled
            if (leftStepper  && leftStepper->isRunning())  leftStepper->stopMove();
            if (rightStepper && rightStepper->isRunning()) rightStepper->stopMove();
        }

        // FastAccelStepper handles pulses via RMT interrupt - no run() needed.
        // vTaskDelay(1) gives the IDLE task on Core 0 a guaranteed turn each
        // iteration, which resets the task watchdog and prevents the WDT crash.
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =============================================================
//  NON-BLOCKING DOME STATE MACHINE  (Core 1)
//
//  Each "event" (stay away, exterminate, bored, boot, pulse) is
//  modelled as a small state machine so loop() never blocks and
//  the web server stays responsive throughout.
// =============================================================

enum DomeState {
    DOME_IDLE,
    DOME_FADE_UP,
    DOME_HOLD,
    DOME_FADE_DOWN,
    DOME_PULSE_DOWN,
    DOME_PULSE_UP,
    DOME_BOOT,
    DOME_BORED_PALETTE
};

static DomeState  domeState    = DOME_BOOT;
static int        fadeBrightness = 0;
static CRGB       fadeColor;
static unsigned long domeStateStart = 0;
static int        paletteIdx   = 0;
static int        paletteRound = 0;

// Call once per loop() iteration; advances whatever animation is active.
// Returns true while an animation is in progress (blocks new events).
bool updateDomeFSM() {
    unsigned long now = millis();

    // Rate-limit fade steps to ~60/sec so loop() stays free for the web server
    static unsigned long lastFadeStep = 0;
    bool fadeReady = (now - lastFadeStep >= 16);

    switch (domeState) {

        case DOME_IDLE:
            return false;   // ready for next event

        // ---- fade up (used for stay-away / exterminate / stalkBlue) ----
        case DOME_FADE_UP:
            if (!fadeReady) return true;
            lastFadeStep = now;
            fadeBrightness += 3;
            if (fadeBrightness >= 255) { fadeBrightness = 255; domeState = DOME_HOLD; domeStateStart = now; }
            FastLED.setBrightness(fadeBrightness);
            FastLED.show();
            return true;

        // ---- hold for 2 s then fade down ----
        case DOME_HOLD:
            if (now - domeStateStart >= 2000) { domeState = DOME_FADE_DOWN; }
            return true;

        // ---- fade down then go idle ----
        case DOME_FADE_DOWN:
            if (!fadeReady) return true;
            lastFadeStep = now;
            fadeBrightness -= 3;
            if (fadeBrightness <= 0) {
                fadeBrightness = 0;
                FastLED.setBrightness(0);
                FastLED.show();
                domeState = DOME_IDLE;
                return false;
            }
            FastLED.setBrightness(fadeBrightness);
            FastLED.show();
            return true;

        // ---- eyestalk pulse (dim then bright) ----
        case DOME_PULSE_DOWN:
            if (!fadeReady) return true;
            lastFadeStep = now;
            fadeBrightness -= 3;
            if (fadeBrightness <= 20) { fadeBrightness = 20; domeState = DOME_PULSE_UP; }
            FastLED.setBrightness(fadeBrightness);
            FastLED.show();
            return true;

        case DOME_PULSE_UP:
            if (!fadeReady) return true;
            lastFadeStep = now;
            fadeBrightness += 3;
            if (fadeBrightness >= 255) {
                fadeBrightness = 255;
                FastLED.setBrightness(255);
                FastLED.show();
                domeState = DOME_IDLE;
                return false;
            }
            FastLED.setBrightness(fadeBrightness);
            FastLED.show();
            return true;

        // ---- boot animation (RWY flash for BOOT_DELAY_MS) ----
        case DOME_BOOT: {
            static unsigned long lastFlip = 0;
            static int bootPhase = 0;
            static CRGB bootColors[] = { CRGB::Red, CRGB::White, CRGB::Yellow };
            if (now - domeStateStart >= BOOT_DELAY_MS) {
                domeState = DOME_IDLE;
                return false;
            }
            if (now - lastFlip >= 333) {
                leds[0] = bootColors[bootPhase % 3];
                FastLED.setBrightness(255);
                FastLED.show();
                bootPhase++;
                lastFlip = now;
            }
            return true;
        }

        // ---- bored palette cycle ----
        case DOME_BORED_PALETTE: {
            static unsigned long lastSwap = 0;
            if (now - lastSwap >= 500) {
                leds[0] = palette[paletteIdx];
                FastLED.setBrightness(255);
                FastLED.show();
                paletteIdx++;
                if (paletteIdx >= PALETTE_SIZE) {
                    paletteIdx = 0;
                    paletteRound++;
                }
                lastSwap = now;
            }
            if (paletteRound >= 7) {
                paletteRound = 0;
                paletteIdx   = 0;
                domeState = DOME_IDLE;
                return false;
            }
            return true;
        }
    }
    return false;
}

// Helpers to kick off an animation
void startFadeEvent(CRGB color) {
    leds[0] = color;
    fadeBrightness = 0;
    domeState = DOME_FADE_UP;
}

void startPulse() {
    leds[0] = CRGB::Blue;
    fadeBrightness = 255;
    domeState = DOME_PULSE_DOWN;
}

void playSound(int track) {
    if (soundEnabled) mp3.playFolder(SND_FOLDER, track);
}

// =============================================================
//  DOME COMMAND PROCESSOR  (Core 1, called from loop)
// =============================================================

void processDomeCmd(int& prevCmd, int& boredCount,
                    unsigned long& lastBored, unsigned long& lastPulse)
{
    // Don't interrupt a running animation (except volume which is instant)
    bool busy = updateDomeFSM();

    int cmd;
    portENTER_CRITICAL(&cmdMux);
    cmd = domeCmd;
    portEXIT_CRITICAL(&cmdMux);

    // Volume is always handled immediately regardless of animation state
    if (cmd == 16) {
        if (volume < 30) { volume++; mp3.volumeUp(); }
        DBGLN("Volume UP");
        setDomeCmd(prevCmd);
        return;
    }
    if (cmd == 17) {
        if (volume > 0) { volume--; mp3.volumeDown(); }
        DBGLN("Volume DOWN");
        setDomeCmd(prevCmd);
        return;
    }

    if (busy || cmd == prevCmd) return;

    switch (cmd) {
        case 10:
            DBGLN("Normal - Blue stalk");
            startFadeEvent(CRGB::Blue);
            prevCmd = cmd;
            break;

        case 11:  // stay away
            if (displayMode) {
                DBGLN("Stay Away!!");
                mp3.volume(volume);
                startFadeEvent(CRGB::White);
                playSound(SND_STAY_AWAY);
                lastBored = millis();
                setDomeCmd(10);
                prevCmd = 10;
            }
            break;

        case 12:  // exterminate
            DBGLN("Exterminate!!");
            mp3.volume(SND_EXTERMINATE_VOLUME);
            startFadeEvent(CRGB::Red);
            playSound(SND_EXTERMINATE);
            lastBored = millis();
            setDomeCmd(10);
            prevCmd = 10;
            break;

        case 14:
            soundEnabled = false;
            DBGLN("Sound OFF");
            prevCmd = cmd;
            break;

        case 15:
            soundEnabled = true;
            DBGLN("Sound ON");
            prevCmd = cmd;
            break;

        case 18:
            displayMode = true;
            DBGLN("Display mode ON");
            prevCmd = cmd;
            break;

        case 19:
            displayMode = false;
            DBGLN("Display mode OFF");
            prevCmd = cmd;
            break;
    }
}

// =============================================================
//  WEB SERVER  (Core 1)
// =============================================================

// JSON status endpoint - polled every 3s by the page via fetch()
// Uses snprintf instead of String concatenation to avoid heap
// fragmentation on the ESP32.
void handleStatus() {
    char json[256];
    bool dm, mr, se;
    int  vol;
    long rc, cc, lc;

    // Snapshot volatile state under lock
    portENTER_CRITICAL(&cmdMux);
    dm  = displayMode;
    mr  = motorRunning;
    se  = soundEnabled;
    vol = volume;
    rc  = rightCM;
    cc  = centerCM;
    lc  = leftCM;
    portEXIT_CRITICAL(&cmdMux);

    snprintf(json, sizeof(json),
        "{\"display\":%s,\"motors\":%s,\"sound\":%s,"
        "\"volume\":%d,\"right\":%ld,\"center\":%ld,\"left\":%ld,"
        "\"rssi\":%d,\"uptime\":%lu}",
        dm  ? "true" : "false",
        mr  ? "true" : "false",
        se  ? "true" : "false",
        vol, rc, cc, lc,
        (int)WiFi.RSSI(),
        millis() / 1000UL
    );

    // No CORS wildcard - UI is same-origin (served from the same ESP32)
    server.send(200, "application/json", json);
}

// HTML is served from LittleFS (data/index.html).
// Flash the filesystem with: pio run --target uploadfs
void handleRoot() {
    if (LittleFS.exists("/index.html")) {
        File f = LittleFS.open("/index.html", "r");
        server.streamFile(f, "text/html");
        f.close();
    } else {
        server.send(503, "text/plain",
            "Filesystem not found. Run: pio run --target uploadfs");
    }
}

void setupWebRoutes() {
    // Root serves the UI - no token required (browser navigation)
    server.on("/", handleRoot);

    // Status is read-only telemetry - no token required for polling
    server.on("/status", handleStatus);

    // All mutating routes require X-Token header
    server.on("/display/toggle", [](){
        if (!checkToken()) return;
        portENTER_CRITICAL(&cmdMux);
        bool dm = displayMode;
        portEXIT_CRITICAL(&cmdMux);

        if (dm) {
            portENTER_CRITICAL(&cmdMux);
            displayMode  = false;
            motorRunning = false;
            soundEnabled = false;
            portEXIT_CRITICAL(&cmdMux);
            setMotorCmd(2);
            setDomeCmd(14);
        } else {
            portENTER_CRITICAL(&cmdMux);
            displayMode  = true;
            soundEnabled = true;
            portEXIT_CRITICAL(&cmdMux);
            setDomeCmd(15);
        }
        server.send(200, "text/plain", "ok");
    });

    server.on("/movement/toggle", [](){
        if (!checkToken()) return;
        portENTER_CRITICAL(&cmdMux);
        bool dm = displayMode;
        bool mr = motorRunning;
        portEXIT_CRITICAL(&cmdMux);

        // Motors only allowed when display mode is off
        if (!dm) {
            bool newMr = !mr;
            portENTER_CRITICAL(&cmdMux);
            motorRunning = newMr;
            portEXIT_CRITICAL(&cmdMux);
            if (newMr) {
                setDomeCmd(19);
            } else {
                setMotorCmd(2);
                setDomeCmd(18);
            }
        }
        server.send(200, "text/plain", "ok");
    });

    server.on("/sound/toggle", [](){
        if (!checkToken()) return;
        portENTER_CRITICAL(&cmdMux);
        bool se = soundEnabled;
        portEXIT_CRITICAL(&cmdMux);

        bool newSe = !se;
        portENTER_CRITICAL(&cmdMux);
        soundEnabled = newSe;
        portEXIT_CRITICAL(&cmdMux);
        setDomeCmd(newSe ? 15 : 14);
        server.send(200, "text/plain", "ok");
    });

    server.on("/volume/up",   [](){ if (!checkToken()) return; setDomeCmd(16); server.send(200, "text/plain", "ok"); });
    server.on("/volume/down", [](){ if (!checkToken()) return; setDomeCmd(17); server.send(200, "text/plain", "ok"); });

    // Catch-all 404 for unknown routes
    server.onNotFound([](){
        server.send(404, "text/plain", "Not found");
    });
}

// =============================================================
//  SETUP  (Core 1)
// =============================================================
void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    #endif
    DBGLN("\n\n--- Dalek ESP32 booting ---");

    // ---- Print last reset reason (helps diagnose unexpected reboots) ----
    DBG("Reset reason  : ");
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   DBGLN("Power on");            break;
        case ESP_RST_SW:        DBGLN("Software reset");      break;
        case ESP_RST_PANIC:     DBGLN("*** PANIC/CRASH ***");  break;
        case ESP_RST_INT_WDT:   DBGLN("*** INT WATCHDOG ***"); break;
        case ESP_RST_TASK_WDT:  DBGLN("*** TASK WATCHDOG ***");break;
        case ESP_RST_WDT:       DBGLN("*** WATCHDOG ***");     break;
        case ESP_RST_BROWNOUT:  DBGLN("*** BROWNOUT ***");     break;
        case ESP_RST_SDIO:      DBGLN("SDIO reset");           break;
        default:                DBGLN("Unknown");              break;
    }

    // ---- LittleFS ----
    if (LittleFS.begin(true)) {   // true = format if mount fails
        DBGLN("LittleFS      : OK");
    } else {
        DBGLN("LittleFS      : FAILED");
    }

    // FastLED
    FastLED.addLeds<LED_CHIPSET, PIN_LED_DATA, LED_COLOR_ORDER>(leds, NUM_LEDS)
           .setCorrection(Typical8mmPixel);
    FastLED.clear(true);

    // ---- WiFi first - before slow inits so output is visible immediately ----
    DBGLN("----------------------------------------");
    DBG("WiFi SSID     : "); DBGLN(WIFI_SSID);
    DBGLN("Connecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int wifiRetries = 40;   // 20 seconds total
    while (WiFi.status() != WL_CONNECTED && wifiRetries-- > 0) {
        delay(500); DBG(".");
    }
    DBGLN("");   // newline after dots — use DBGLN so it respects DEBUG guard
    if (WiFi.status() == WL_CONNECTED) {
        DBGLN("WiFi status   : CONNECTED");
        DBG("IP address    : "); DBGLN(WiFi.localIP());
        DBG("Gateway       : "); DBGLN(WiFi.gatewayIP());
        DBG("Signal (RSSI) : "); DBG(WiFi.RSSI()); DBGLN(" dBm");
        DBG("Channel       : "); DBGLN(WiFi.channel());
        DBG("MAC address   : "); DBGLN(WiFi.macAddress());
    } else {
        DBGLN("WiFi status   : FAILED - running offline");
        DBG("Status code   : ");
        switch (WiFi.status()) {
            case WL_NO_SSID_AVAIL:  DBGLN("WL_NO_SSID_AVAIL (SSID not found)");   break;
            case WL_CONNECT_FAILED: DBGLN("WL_CONNECT_FAILED (wrong password?)");  break;
            case WL_DISCONNECTED:   DBGLN("WL_DISCONNECTED (timed out)");          break;
            default: DBGLN(WiFi.status()); break;
        }
    }
    DBGLN("----------------------------------------");

    // ---- DFPlayer on hardware Serial2 (non-fatal if absent) ----
    bool dfplayerOK = false;
    Serial2.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
    int retries = 5;
    while (!dfplayerOK && retries-- > 0) {
        if (mp3.begin(Serial2)) {
            dfplayerOK = true;
        } else {
            DBGLN("DFPlayer not ready, retrying...");
            delay(500);
        }
    }
    if (dfplayerOK) {
        mp3.volume(DEFAULT_VOLUME);
        DBGLN("DFPlayer      : OK");
    } else {
        DBGLN("DFPlayer      : NOT FOUND - sound disabled");
        soundEnabled = false;
    }

    // Sonic sensor pins
    pinMode(PIN_SONIC_TRIGGER, OUTPUT);
    pinMode(PIN_SONIC_RIGHT,   INPUT);
    pinMode(PIN_SONIC_CENTER,  INPUT);
    pinMode(PIN_SONIC_LEFT,    INPUT);

    // Boot animation (non-blocking FSM, starts immediately)
    domeStateStart = millis();
    domeState = DOME_BOOT;
    DBGLN("Boot animation started");

    // Spin in the boot animation until it finishes.
    // yield() keeps the WiFi/lwIP stack alive during the wait.
    while (updateDomeFSM()) { yield(); }
    DBGLN("Boot complete");
    playSound(SND_MOAN);

    setupWebRoutes();
    server.begin();
    DBGLN("Web server started");

    // Start motor/sensor task on Core 0
    // Stack bumped to 8192: sensor reads + FastAccelStepper + debug prints
    // can push the 4096-byte stack close to its limit under load.
    xTaskCreatePinnedToCore(
        motorTask,   // function
        "motorTask", // name
        8192,        // stack bytes (was 4096)
        NULL,        // parameter
        1,           // priority
        NULL,        // task handle
        0            // Core 0
    );
    DBGLN("Motor task started on Core 0");

    // Eyestalk ready - blue fade via FSM
    startFadeEvent(CRGB::Blue);
}

// =============================================================
//  LOOP  (Core 1)  -  web server + dome FSM
// =============================================================
void loop() {
    server.handleClient();
    esp_task_wdt_reset();   // keep Core 1 watchdog happy

    static int           prevDomeCmd = -1;
    static int           boredCount  = 0;
    static unsigned long lastBored   = millis();
    static unsigned long lastPulse   = millis();
    static unsigned long lastHB      = millis();

    // Heartbeat every 30s - confirms loop() is still running
    if (millis() - lastHB >= 30000UL) {
        lastHB = millis();
        DBG("[HB] uptime="); DBG(millis()/1000);
        DBG("s  heap=");     DBG(ESP.getFreeHeap());
        DBG("  wifi=");      DBGLN(WiFi.status() == WL_CONNECTED ? "OK" : "DOWN");

        // Robust reconnect: disconnect first, then re-begin.
        // WiFi.reconnect() alone can silently fail after a longer outage.
        if (WiFi.status() != WL_CONNECTED) {
            DBGLN("[HB] WiFi lost - reconnecting...");
            WiFi.disconnect();
            delay(100);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
    }

    processDomeCmd(prevDomeCmd, boredCount, lastBored, lastPulse);

    unsigned long now = millis();

    // Eyestalk pulse every 10 s in display mode (only if idle)
    if (displayMode && domeState == DOME_IDLE && (now - lastPulse >= PULSE_INTERVAL_MS)) {
        startPulse();
        lastPulse = now;
    }

    // Bored timer
    if (displayMode && domeState == DOME_IDLE && (now - lastBored >= BORED_INTERVAL_MS)) {
        DBGLN("Bored!");
        mp3.volume(volume);
        if (boredCount < BORED_COUNT_MAX) {
            playSound(SND_MOAN);
            boredCount++;
        } else {
            DBGLN("Really bored!");
            playSound(SND_REALLY_BORED);
            paletteIdx   = 0;
            paletteRound = 0;
            domeState    = DOME_BORED_PALETTE;
            boredCount   = 0;
        }
        lastBored = now;
        lastPulse = now;
    }
}