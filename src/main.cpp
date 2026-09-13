// =============================================================================
//  main.cpp — Dalek ESP32 unified firmware
//  VERSION: V0.51
//
//  NSD-Dalek
//
//  Hardware:
//    - ESP32-WROOM-32U / DevKitC V4
//    - 2x NEMA17 + Big Easy Driver
//    - 3x Maxbotix EZ1 ultrasonic sensors
//    - DFPlayer Mini
//    - 1x WS2811 eye LED
//
//  Architecture:
//    Core 0:
//      - ultrasonic sensors
//      - motor navigation
//      - FastAccelStepper command handling
//
//    Core 1:
//      - WiFi
//      - WebServer
//      - OTA
//      - DFPlayer
//      - dome/eye FSM
//
//  V0.51 changes:
//    - Fixed /movement/toggle web route
//    - Removed obsolete sDisplayMode / sMovementState
//    - Real sensor hysteresis
//    - All 8 sensor combinations handled
//    - 2-sensor blockage => reverse
//    - Reverse escape after MOTOR_REVERSE_ESCAPE_LIMIT cycles
//    - 3-sensor blockage => stop
//    - Thread-safe motor/display/sound state access
//    - Non-blocking WiFi reconnect
//    - OTA starts after later WiFi reconnect
//    - DFPlayer availability tracked
//    - Display/sound/motor controls separated
//
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <FastLED.h>
#include <FastAccelStepper.h>
#include <DFRobotDFPlayerMini.h>
#include <ArduinoOTA.h>

#include "config.h"
#include "esp_task_wdt.h"
#include "esp_system.h"

// =============================================================================
//  VERSION
// =============================================================================

static const char* FIRMWARE_VERSION = "V0.51";

// =============================================================================
//  SHARED STATE
// =============================================================================

portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

// Motor commands:
//
//   1 = forward
//   2 = stop
//   3 = turn left
//   4 = turn right
//   5 = reverse
//
// 6/7 were used by older versions and are no longer required.

volatile int  motorCmd     = 2;
volatile bool motorRunning = false;

// Dome commands:
//
//   10 = normal blue eye
//   11 = Stay Away
//   12 = Exterminate
//   14 = sound off
//   15 = sound on
//   16 = volume up
//   17 = volume down
//   18 = display on
//   19 = display off

volatile int domeCmd = 10;

// Sensor values.
//
// Written by Core 0.
// Read by Core 1.
//
// 999 is used as "no obstacle / invalid reading".

volatile long rightCM  = 999;
volatile long centerCM = 999;
volatile long leftCM   = 999;

// Display / sound state.

volatile bool displayMode = true;
volatile bool soundEnabled = true;
volatile int  volume = DEFAULT_VOLUME;

// DFPlayer availability.

volatile bool dfplayerAvailable = false;

// =============================================================================
//  HARDWARE OBJECTS
// =============================================================================

FastAccelStepperEngine stepperEngine = FastAccelStepperEngine();

FastAccelStepper* leftStepper  = nullptr;
FastAccelStepper* rightStepper = nullptr;

DFRobotDFPlayerMini mp3;

CRGB leds[NUM_LEDS];

WebServer server(80);

// =============================================================================
//  PALETTE
// =============================================================================

static const CRGB palette[] = {
    CRGB::Khaki,
    CRGB::Aqua,
    CRGB::DarkMagenta,
    CRGB::DarkSeaGreen,
    CRGB::Amethyst,
    CRGB::RosyBrown,
    CRGB::OrangeRed,
    CRGB::Yellow,
    CRGB::LightCoral,
    CRGB::OldLace
};

static const int PALETTE_SIZE =
    sizeof(palette) / sizeof(palette[0]);

// =============================================================================
//  THREAD-SAFE STATE HELPERS
// =============================================================================

void setMotorCmd(int cmd)
{
    portENTER_CRITICAL(&cmdMux);
    motorCmd = cmd;
    portEXIT_CRITICAL(&cmdMux);
}

int getMotorCmd()
{
    int cmd;

    portENTER_CRITICAL(&cmdMux);
    cmd = motorCmd;
    portEXIT_CRITICAL(&cmdMux);

    return cmd;
}

void setMotorRunning(bool state)
{
    portENTER_CRITICAL(&cmdMux);
    motorRunning = state;
    portEXIT_CRITICAL(&cmdMux);
}

bool getMotorRunning()
{
    bool state;

    portENTER_CRITICAL(&cmdMux);
    state = motorRunning;
    portEXIT_CRITICAL(&cmdMux);

    return state;
}

void setDomeCmd(int cmd)
{
    portENTER_CRITICAL(&cmdMux);
    domeCmd = cmd;
    portEXIT_CRITICAL(&cmdMux);
}

int getDomeCmd()
{
    int cmd;

    portENTER_CRITICAL(&cmdMux);
    cmd = domeCmd;
    portEXIT_CRITICAL(&cmdMux);

    return cmd;
}

bool getDisplayMode()
{
    bool state;

    portENTER_CRITICAL(&cmdMux);
    state = displayMode;
    portEXIT_CRITICAL(&cmdMux);

    return state;
}

void setDisplayMode(bool state)
{
    portENTER_CRITICAL(&cmdMux);
    displayMode = state;
    portEXIT_CRITICAL(&cmdMux);
}

bool getSoundEnabled()
{
    bool state;

    portENTER_CRITICAL(&cmdMux);
    state = soundEnabled;
    portEXIT_CRITICAL(&cmdMux);

    return state;
}

void setSoundEnabled(bool state)
{
    portENTER_CRITICAL(&cmdMux);
    soundEnabled = state;
    portEXIT_CRITICAL(&cmdMux);
}

int getVolume()
{
    int value;

    portENTER_CRITICAL(&cmdMux);
    value = volume;
    portEXIT_CRITICAL(&cmdMux);

    return value;
}

void setVolume(int value)
{
    value = constrain(value, 0, 30);

    portENTER_CRITICAL(&cmdMux);
    volume = value;
    portEXIT_CRITICAL(&cmdMux);
}

void setDFPlayerAvailable(bool state)
{
    portENTER_CRITICAL(&cmdMux);
    dfplayerAvailable = state;
    portEXIT_CRITICAL(&cmdMux);
}

bool getDFPlayerAvailable()
{
    bool state;

    portENTER_CRITICAL(&cmdMux);
    state = dfplayerAvailable;
    portEXIT_CRITICAL(&cmdMux);

    return state;
}

// =============================================================================
//  SECURITY
// =============================================================================

bool checkToken()
{
    if (server.hasHeader("X-Token") &&
        server.header("X-Token") == String(API_TOKEN)) {
        return true;
    }

    server.send(403, "text/plain", "Forbidden");
    return false;
}

// =============================================================================
//  SENSOR NAVIGATION STATE
// =============================================================================

// Hysteresis state.

static bool blockedRight  = false;
static bool blockedCenter = false;
static bool blockedLeft   = false;

// Number of consecutive sensor cycles in which the Dalek has been forced
// to reverse.

static int reverseEscapeCounter = 0;

// Alarm states.

static bool midTriggered = false;
static bool minTriggered = false;

// =============================================================================
//  SENSOR READING
// =============================================================================

long pulseToCentimeters(unsigned long pulse)
{
    if (pulse == 0) {
        return SONIC_MAX_CM;
    }

    // Maxbotix PWM:
    // approximately 58 us per cm.
    //
    // The previous firmware used /29/2, which is mathematically identical.

    long distance = pulse / 58;

    if (distance < 0) {
        distance = 0;
    }

    if (distance > SONIC_MAX_CM) {
        distance = SONIC_MAX_CM;
    }

    return distance;
}

void readSensors()
{
    // Common trigger for all three Maxbotix sensors.

    digitalWrite(PIN_SONIC_TRIGGER, HIGH);
    delayMicroseconds(25);
    digitalWrite(PIN_SONIC_TRIGGER, LOW);

    unsigned long rp =
        pulseIn(PIN_SONIC_RIGHT,
                HIGH,
                SONIC_PULSE_TIMEOUT_US);

    unsigned long cp =
        pulseIn(PIN_SONIC_CENTER,
                HIGH,
                SONIC_PULSE_TIMEOUT_US);

    unsigned long lp =
        pulseIn(PIN_SONIC_LEFT,
                HIGH,
                SONIC_PULSE_TIMEOUT_US);

    long r = pulseToCentimeters(rp);
    long c = pulseToCentimeters(cp);
    long l = pulseToCentimeters(lp);

    portENTER_CRITICAL(&cmdMux);

    rightCM  = r;
    centerCM = c;
    leftCM   = l;

    portEXIT_CRITICAL(&cmdMux);
}

// =============================================================================
//  HYSTERESIS
// =============================================================================

static inline void updateBlocked(bool& blocked, long distanceCM)
{
    if (!blocked) {

        if (distanceCM <= SONIC_MIN_CM) {
            blocked = true;
        }

    } else {

        if (distanceCM >
            SONIC_MIN_CM + SONIC_HYSTERESIS_CM) {

            blocked = false;
        }
    }
}

// =============================================================================
//  SENSOR NAVIGATION
// =============================================================================
//
// Sensor layout:
//
//                 FRONT
//
//       LEFT       CENTER       RIGHT
//
//
//
// Navigation:
//
//   0 blocked:
//       forward
//
//   1 blocked:
//       turn away from obstacle
//
//   2 blocked:
//       reverse
//
//   3 blocked:
//       stop
//
// Special case:
//   If reverse has been required for MOTOR_REVERSE_ESCAPE_LIMIT consecutive
//   cycles, choose the more open side and turn instead.
//
// =============================================================================

void sensorAction()
{
    if (!getMotorRunning()) {
        return;
    }

    long r;
    long c;
    long l;

    portENTER_CRITICAL(&cmdMux);

    r = rightCM;
    c = centerCM;
    l = leftCM;

    portEXIT_CRITICAL(&cmdMux);

    // Update hysteresis state.

    updateBlocked(blockedRight,  r);
    updateBlocked(blockedCenter, c);
    updateBlocked(blockedLeft,   l);

    // Count blocked sensors.

    int blockedCount =
        (blockedRight  ? 1 : 0) +
        (blockedCenter ? 1 : 0) +
        (blockedLeft   ? 1 : 0);

    // -------------------------------------------------------------------------
    // Alarm handling
    // -------------------------------------------------------------------------

    bool withinMid =
        (r <= SONIC_MID_CM ||
         c <= SONIC_MID_CM ||
         l <= SONIC_MID_CM);

    bool withinMin =
        (r <= SONIC_MIN_CM ||
         c <= SONIC_MIN_CM ||
         l <= SONIC_MIN_CM);

    // Reset the alarm latch only after the Dalek is clearly away again.

    bool allClearForAlarmReset =
        (r > SONIC_MID_CM + SONIC_HYSTERESIS_CM &&
         c > SONIC_MID_CM + SONIC_HYSTERESIS_CM &&
         l > SONIC_MID_CM + SONIC_HYSTERESIS_CM);

    if (allClearForAlarmReset) {
        midTriggered = false;
        minTriggered = false;
    }

    if (withinMid && !midTriggered) {

        setDomeCmd(11);
        midTriggered = true;

        DBGLN("[SONIC] Stay Away trigger");
    }

    if (withinMin && !minTriggered) {

        setDomeCmd(12);
        minTriggered = true;

        DBGLN("[SONIC] Exterminate trigger");
    }

    // -------------------------------------------------------------------------
    // Navigation
    // -------------------------------------------------------------------------

    switch (blockedCount) {

        // ---------------------------------------------------------------------
        // 0 / 3 sensors blocked
        // ---------------------------------------------------------------------

        case 0:

            reverseEscapeCounter = 0;

            setMotorCmd(1);

            break;

        // ---------------------------------------------------------------------
        // Exactly one sensor blocked
        // ---------------------------------------------------------------------

        case 1:

            reverseEscapeCounter = 0;

            if (blockedRight && !blockedCenter && !blockedLeft) {

                // Obstacle on right -> turn left.

                setMotorCmd(3);
            }
            else if (!blockedRight &&
                     !blockedCenter &&
                     blockedLeft) {

                // Obstacle on left -> turn right.

                setMotorCmd(4);
            }
            else if (!blockedRight &&
                     blockedCenter &&
                     !blockedLeft) {

                // Obstacle directly ahead.

                setMotorCmd(5);
                reverseEscapeCounter = 1;
            }

            break;

        // ---------------------------------------------------------------------
        // Exactly two sensors blocked
        // ---------------------------------------------------------------------

        case 2:

            reverseEscapeCounter++;

            // Reverse for a limited number of cycles.

            if (reverseEscapeCounter <
                MOTOR_REVERSE_ESCAPE_LIMIT) {

                setMotorCmd(5);
            }

            else {

                // We have been reversing long enough.
                //
                // Choose the side with the greatest available distance.

                if (l > r) {

                    DBGLN("[SONIC] Escape -> LEFT");

                    setMotorCmd(3);

                } else {

                    DBGLN("[SONIC] Escape -> RIGHT");

                    setMotorCmd(4);
                }

                reverseEscapeCounter = 0;
            }

            break;

        // ---------------------------------------------------------------------
        // All three sensors blocked
        // ---------------------------------------------------------------------

        case 3:

            reverseEscapeCounter = 0;

            setMotorCmd(2);

            DBGLN("[SONIC] ALL BLOCKED -> STOP");

            break;
    }
}

// =============================================================================
//  MOTOR COMMAND APPLICATION
// =============================================================================

void applyMotorCmd()
{
    static int previousCommand = -1;

    int cmd = getMotorCmd();

    if (cmd == previousCommand) {
        return;
    }

    previousCommand = cmd;

    if (!leftStepper || !rightStepper) {
        return;
    }

    switch (cmd) {

        // ---------------------------------------------------------------------
        // Forward
        // ---------------------------------------------------------------------

        case 1:

            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED);

            leftStepper->runForward();
            rightStepper->runForward();

            break;

        // ---------------------------------------------------------------------
        // Stop
        // ---------------------------------------------------------------------

        case 2:

            leftStepper->stopMove();
            rightStepper->stopMove();

            break;

        // ---------------------------------------------------------------------
        // Turn left
        // ---------------------------------------------------------------------

        case 3:

            leftStepper->setSpeedInHz(
                MOTOR_MAX_SPEED / MOTOR_TURN_SLOW_DIV);

            rightStepper->setSpeedInHz(
                MOTOR_MAX_SPEED);

            leftStepper->runForward();
            rightStepper->runForward();

            break;

        // ---------------------------------------------------------------------
        // Turn right
        // ---------------------------------------------------------------------

        case 4:

            leftStepper->setSpeedInHz(
                MOTOR_MAX_SPEED);

            rightStepper->setSpeedInHz(
                MOTOR_MAX_SPEED / MOTOR_TURN_SLOW_DIV);

            leftStepper->runForward();
            rightStepper->runForward();

            break;

        // ---------------------------------------------------------------------
        // Reverse
        // ---------------------------------------------------------------------

        case 5:

            leftStepper->setSpeedInHz(
                MOTOR_MAX_SPEED / 2);

            rightStepper->setSpeedInHz(
                MOTOR_MAX_SPEED / 2);

            leftStepper->runBackward();
            rightStepper->runBackward();

            break;

        default:

            leftStepper->stopMove();
            rightStepper->stopMove();

            break;
    }
}

// =============================================================================
//  MOTOR TASK — CORE 0
// =============================================================================

void motorTask(void* pvParameters)
{
    // Register this task with the Task Watchdog.

    esp_task_wdt_add(NULL);

    DBGLN("[MOTOR] Initializing FastAccelStepper");

    stepperEngine.init();

    leftStepper =
        stepperEngine.stepperConnectToPin(PIN_LEFT_STEP);

    rightStepper =
        stepperEngine.stepperConnectToPin(PIN_RIGHT_STEP);

    // -------------------------------------------------------------------------
    // Left motor
    // -------------------------------------------------------------------------

    if (leftStepper) {

        leftStepper->setDirectionPin(
            PIN_LEFT_DIR,
            INVERT_LEFT_MOTOR);

        leftStepper->setAcceleration(
            MOTOR_ACCEL);

        leftStepper->setSpeedInHz(
            MOTOR_MAX_SPEED);

        DBGLN("[MOTOR] Left stepper OK");

    } else {

        DBGLN("[MOTOR] ERROR: left stepper unavailable");
    }

    // -------------------------------------------------------------------------
    // Right motor
    // -------------------------------------------------------------------------

    if (rightStepper) {

        rightStepper->setDirectionPin(
            PIN_RIGHT_DIR,
            INVERT_RIGHT_MOTOR);

        rightStepper->setAcceleration(
            MOTOR_ACCEL);

        rightStepper->setSpeedInHz(
            MOTOR_MAX_SPEED);

        DBGLN("[MOTOR] Right stepper OK");

    } else {

        DBGLN("[MOTOR] ERROR: right stepper unavailable");
    }

    unsigned long lastSensorRead = millis();

    // -------------------------------------------------------------------------
    // Main motor loop
    // -------------------------------------------------------------------------

    for (;;) {

        unsigned long now = millis();

        // Sensor update every 500 ms.

        if (now - lastSensorRead >= 500UL) {

            readSensors();

            sensorAction();

            lastSensorRead = now;
        }

        // ---------------------------------------------------------------------
        // Motor enable state
        // ---------------------------------------------------------------------

        if (getMotorRunning()) {

            applyMotorCmd();

        } else {

            // Movement disabled -> force stop.

            if (leftStepper &&
                leftStepper->isRunning()) {

                leftStepper->stopMove();
            }

            if (rightStepper &&
                rightStepper->isRunning()) {

                rightStepper->stopMove();
            }
        }

        // FastAccelStepper generates the actual step pulses through RMT.

        esp_task_wdt_reset();

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =============================================================================
//  DOME / EYE STATE MACHINE
// =============================================================================

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

static DomeState domeState = DOME_BOOT;

static int fadeBrightness = 0;

static unsigned long domeStateStart = 0;

static int paletteIdx   = 0;
static int paletteRound = 0;

// =============================================================================
//  DOME FSM
// =============================================================================

bool updateDomeFSM()
{
    unsigned long now = millis();

    static unsigned long lastFadeStep = 0;

    bool fadeReady =
        (now - lastFadeStep >= 16UL);

    switch (domeState) {

        // ---------------------------------------------------------------------
        // Idle
        // ---------------------------------------------------------------------

        case DOME_IDLE:

            return false;

        // ---------------------------------------------------------------------
        // Fade up
        // ---------------------------------------------------------------------

        case DOME_FADE_UP:

            if (!fadeReady) {
                return true;
            }

            lastFadeStep = now;

            fadeBrightness += 3;

            if (fadeBrightness >= 255) {

                fadeBrightness = 255;

                domeState = DOME_HOLD;

                domeStateStart = now;
            }

            FastLED.setBrightness(
                fadeBrightness);

            FastLED.show();

            return true;

        // ---------------------------------------------------------------------
        // Hold
        // ---------------------------------------------------------------------

        case DOME_HOLD:

            if (now - domeStateStart >= 2000UL) {

                domeState = DOME_FADE_DOWN;
            }

            return true;

        // ---------------------------------------------------------------------
        // Fade down
        // ---------------------------------------------------------------------

        case DOME_FADE_DOWN:

            if (!fadeReady) {
                return true;
            }

            lastFadeStep = now;

            fadeBrightness -= 3;

            if (fadeBrightness <= 0) {

                fadeBrightness = 0;

                FastLED.setBrightness(0);
                FastLED.show();

                domeState = DOME_IDLE;

                return false;
            }

            FastLED.setBrightness(
                fadeBrightness);

            FastLED.show();

            return true;

        // ---------------------------------------------------------------------
        // Pulse down
        // ---------------------------------------------------------------------

        case DOME_PULSE_DOWN:

            if (!fadeReady) {
                return true;
            }

            lastFadeStep = now;

            fadeBrightness -= 3;

            if (fadeBrightness <= 20) {

                fadeBrightness = 20;

                domeState = DOME_PULSE_UP;
            }

            FastLED.setBrightness(
                fadeBrightness);

            FastLED.show();

            return true;

        // ---------------------------------------------------------------------
        // Pulse up
        // ---------------------------------------------------------------------

        case DOME_PULSE_UP:

            if (!fadeReady) {
                return true;
            }

            lastFadeStep = now;

            fadeBrightness += 3;

            if (fadeBrightness >= 255) {

                fadeBrightness = 255;

                FastLED.setBrightness(255);
                FastLED.show();

                domeState = DOME_IDLE;

                return false;
            }

            FastLED.setBrightness(
                fadeBrightness);

            FastLED.show();

            return true;

        // ---------------------------------------------------------------------
        // Boot animation
        // ---------------------------------------------------------------------

        case DOME_BOOT: {

            static unsigned long lastFlip = 0;

            static int bootPhase = 0;

            static CRGB bootColors[] = {
                CRGB::Red,
                CRGB::White,
                CRGB::Yellow
            };

            if (now - domeStateStart >=
                BOOT_DELAY_MS) {

                domeState = DOME_IDLE;

                return false;
            }

            if (now - lastFlip >= 333UL) {

                leds[0] =
                    bootColors[bootPhase % 3];

                FastLED.setBrightness(255);
                FastLED.show();

                bootPhase++;

                lastFlip = now;
            }

            return true;
        }

        // ---------------------------------------------------------------------
        // Bored palette
        // ---------------------------------------------------------------------

        case DOME_BORED_PALETTE: {

            static unsigned long lastSwap = 0;

            if (now - lastSwap >= 500UL) {

                leds[0] =
                    palette[paletteIdx];

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
                paletteIdx = 0;

                domeState = DOME_IDLE;

                return false;
            }

            return true;
        }
    }

    return false;
}

// =============================================================================
//  DOME ANIMATION HELPERS
// =============================================================================

void startFadeEvent(CRGB color)
{
    if (!getDisplayMode()) {
        return;
    }

    leds[0] = color;

    fadeBrightness = 0;

    domeState = DOME_FADE_UP;
}

void startPulse()
{
    if (!getDisplayMode()) {
        return;
    }

    leds[0] = CRGB::Blue;

    fadeBrightness = 255;

    domeState = DOME_PULSE_DOWN;
}

void playSound(int track)
{
    if (!getDFPlayerAvailable()) {
        return;
    }

    if (!getSoundEnabled()) {
        return;
    }

    mp3.playFolder(
        SND_FOLDER,
        track);
}

// =============================================================================
//  DOME COMMAND PROCESSOR
// =============================================================================

void processDomeCmd(
    int& previousCommand,
    int& boredCount,
    unsigned long& lastBored,
    unsigned long& lastPulse)
{
    bool busy =
        updateDomeFSM();

    int cmd =
        getDomeCmd();

    // -------------------------------------------------------------------------
    // Volume up
    // -------------------------------------------------------------------------

    if (cmd == 16) {

        if (getDFPlayerAvailable()) {

            int currentVolume = getVolume();

            if (currentVolume < 30) {

                setVolume(currentVolume + 1);

                mp3.volumeUp();
            }
        }

        DBGLN("[DOME] Volume UP");

        setDomeCmd(previousCommand);

        return;
    }

    // -------------------------------------------------------------------------
    // Volume down
    // -------------------------------------------------------------------------

    if (cmd == 17) {

        if (getDFPlayerAvailable()) {

            int currentVolume = getVolume();

            if (currentVolume > 0) {

                setVolume(currentVolume - 1);

                mp3.volumeDown();
            }
        }

        DBGLN("[DOME] Volume DOWN");

        setDomeCmd(previousCommand);

        return;
    }

    // Don't interrupt an active animation.

    if (busy ||
        cmd == previousCommand) {

        return;
    }

    // -------------------------------------------------------------------------
    // Normal blue eye
    // -------------------------------------------------------------------------

    if (cmd == 10) {

        if (getDisplayMode()) {

            DBGLN("[DOME] Normal");

            startFadeEvent(
                CRGB::Blue);
        }

        previousCommand = cmd;

        return;
    }

    // -------------------------------------------------------------------------
    // Stay Away
    // -------------------------------------------------------------------------

    if (cmd == 11) {

        bool display = getDisplayMode();

        int currentVolume =
            getVolume();

        if (display) {

            DBGLN("[DOME] STAY AWAY");

            if (getDFPlayerAvailable()) {
                mp3.volume(currentVolume);
            }

            startFadeEvent(
                CRGB::White);
        }

        playSound(SND_STAY_AWAY);

        lastBored = millis();
        boredCount = 0;

        setDomeCmd(10);

        previousCommand = 10;

        return;
    }

    // -------------------------------------------------------------------------
    // Exterminate
    // -------------------------------------------------------------------------

    if (cmd == 12) {

        DBGLN("[DOME] EXTERMINATE");

        if (getDFPlayerAvailable()) {

            mp3.volume(
                SND_EXTERMINATE_VOLUME);
        }

        if (getDisplayMode()) {

            startFadeEvent(
                CRGB::Red);
        }

        playSound(SND_EXTERMINATE);

        lastBored = millis();
        boredCount = 0;

        setDomeCmd(10);

        previousCommand = 10;

        return;
    }

    // -------------------------------------------------------------------------
    // Sound OFF
    // -------------------------------------------------------------------------

    if (cmd == 14) {

        setSoundEnabled(false);

        DBGLN("[DOME] Sound OFF");

        previousCommand = cmd;

        return;
    }

    // -------------------------------------------------------------------------
    // Sound ON
    // -------------------------------------------------------------------------

    if (cmd == 15) {

        setSoundEnabled(true);

        DBGLN("[DOME] Sound ON");

        previousCommand = cmd;

        return;
    }

    // -------------------------------------------------------------------------
    // Display ON
    // -------------------------------------------------------------------------

    if (cmd == 18) {

        setDisplayMode(true);

        DBGLN("[DOME] Display ON");

        previousCommand = cmd;

        return;
    }

    // -------------------------------------------------------------------------
    // Display OFF
    // -------------------------------------------------------------------------

    if (cmd == 19) {

        setDisplayMode(false);

        fadeBrightness = 0;

        FastLED.setBrightness(0);
        FastLED.clear(true);

        DBGLN("[DOME] Display OFF");

        previousCommand = cmd;

        return;
    }
}

// =============================================================================
//  WEB SERVER — STATUS
// =============================================================================

void handleStatus()
{
    char json[320];

    bool dm;
    bool mr;
    bool se;
    bool df;

    int vol;

    long rc;
    long cc;
    long lc;

    portENTER_CRITICAL(&cmdMux);

    dm  = displayMode;
    mr  = motorRunning;
    se  = soundEnabled;
    df  = dfplayerAvailable;

    vol = volume;

    rc  = rightCM;
    cc  = centerCM;
    lc  = leftCM;

    portEXIT_CRITICAL(&cmdMux);

    int wifiRSSI =
        (WiFi.status() == WL_CONNECTED)
        ? WiFi.RSSI()
        : 0;

    snprintf(
        json,
        sizeof(json),

        "{"
        "\"version\":\"%s\","
        "\"display\":%s,"
        "\"motors\":%s,"
        "\"sound\":%s,"
        "\"dfplayer\":%s,"
        "\"volume\":%d,"
        "\"right\":%ld,"
        "\"center\":%ld,"
        "\"left\":%ld,"
        "\"rssi\":%d,"
        "\"uptime\":%lu"
        "}",

        FIRMWARE_VERSION,

        dm ? "true" : "false",
        mr ? "true" : "false",
        se ? "true" : "false",
        df ? "true" : "false",

        vol,

        rc,
        cc,
        lc,

        wifiRSSI,

        millis() / 1000UL
    );

    server.send(
        200,
        "application/json",
        json);
}

// =============================================================================
//  WEB SERVER — ROOT
// =============================================================================

void handleRoot()
{
    if (!LittleFS.exists("/index.html")) {

        server.send(
            503,
            "text/plain",
            "Filesystem niet gevonden. "
            "Voer uit: pio run --target uploadfs");

        return;
    }

    File f =
        LittleFS.open(
            "/index.html",
            "r");

    if (!f) {

        server.send(
            500,
            "text/plain",
            "Kan index.html niet openen");

        return;
    }

    String html =
        f.readString();

    f.close();

    html.replace(
        "%%API_TOKEN%%",
        API_TOKEN);

    server.send(
        200,
        "text/html",
        html);
}

// =============================================================================
//  WEB SERVER — MOTOR TOGGLE
// =============================================================================

void handleMovementToggle()
{
    if (!checkToken()) {
        return;
    }

    bool running =
        getMotorRunning();

    if (running) {

        // ---------------------------------------------------------------------
        // Turn motors OFF
        // ---------------------------------------------------------------------

        setMotorRunning(false);

        setMotorCmd(2);

        reverseEscapeCounter = 0;

        DBGLN("[WEB] Motors OFF");

    } else {

        // ---------------------------------------------------------------------
        // Turn motors ON
        // ---------------------------------------------------------------------

        setMotorRunning(true);

        // Start with a fresh navigation decision.

        reverseEscapeCounter = 0;

        DBGLN("[WEB] Motors ON");
    }

    server.send(
        200,
        "text/plain",
        "ok");
}

// =============================================================================
//  WEB SERVER — MOTOR ON COMPATIBILITY ROUTE
// =============================================================================

void handleMovementOn()
{
    if (!checkToken()) {
        return;
    }

    setMotorRunning(true);

    reverseEscapeCounter = 0;

    DBGLN("[WEB] Motors ON");

    server.send(
        200,
        "text/plain",
        "ok");
}

// =============================================================================
//  WEB SERVER — MOTOR OFF
// =============================================================================

void handleMovementOff()
{
    if (!checkToken()) {
        return;
    }

    setMotorRunning(false);

    setMotorCmd(2);

    reverseEscapeCounter = 0;

    DBGLN("[WEB] Motors OFF");

    server.send(
        200,
        "text/plain",
        "ok");
}

// =============================================================================
//  WEB SERVER ROUTES
// =============================================================================

void setupWebRoutes()
{
    // -------------------------------------------------------------------------
    // Root
    // -------------------------------------------------------------------------

    server.on(
        "/",
        handleRoot);

    // -------------------------------------------------------------------------
    // Read-only status
    // -------------------------------------------------------------------------

    server.on(
        "/status",
        handleStatus);

    // -------------------------------------------------------------------------
    // Display toggle
    // -------------------------------------------------------------------------

    server.on(
        "/display/toggle",
        []() {

            if (!checkToken()) {
                return;
            }

            bool current =
                getDisplayMode();

            if (current) {

                // Display OFF.

                setDisplayMode(false);

                // Turning the display off also stops motors and sound,
                // preserving the behaviour of the existing UI.

                setMotorRunning(false);
                setMotorCmd(2);

                setSoundEnabled(false);

                setDomeCmd(19);

                DBGLN("[WEB] Display OFF");

            } else {

                // Display ON.

                setDisplayMode(true);
                setSoundEnabled(true);

                setDomeCmd(18);

                DBGLN("[WEB] Display ON");
            }

            server.send(
                200,
                "text/plain",
                "ok");
        });

    // -------------------------------------------------------------------------
    // Motor toggle
    // -------------------------------------------------------------------------

    server.on(
        "/movement/toggle",
        handleMovementToggle);

    // -------------------------------------------------------------------------
    // Motor ON compatibility route
    // -------------------------------------------------------------------------

    server.on(
        "/movement/on",
        handleMovementOn);

    // -------------------------------------------------------------------------
    // Motor OFF
    // -------------------------------------------------------------------------

    server.on(
        "/movement/off",
        handleMovementOff);

    // -------------------------------------------------------------------------
    // Sound toggle
    // -------------------------------------------------------------------------

    server.on(
        "/sound/toggle",
        []() {

            if (!checkToken()) {
                return;
            }

            bool current =
                getSoundEnabled();

            bool newState =
                !current;

            setSoundEnabled(newState);

            setDomeCmd(
                newState ? 15 : 14);

            server.send(
                200,
                "text/plain",
                "ok");
        });

    // -------------------------------------------------------------------------
    // Volume
    // -------------------------------------------------------------------------

    server.on(
        "/volume/up",
        []() {

            if (!checkToken()) {
                return;
            }

            setDomeCmd(16);

            server.send(
                200,
                "text/plain",
                "ok");
        });

    server.on(
        "/volume/down",
        []() {

            if (!checkToken()) {
                return;
            }

            setDomeCmd(17);

            server.send(
                200,
                "text/plain",
                "ok");
        });

    // -------------------------------------------------------------------------
    // 404
    // -------------------------------------------------------------------------

    server.onNotFound(
        []() {

            server.send(
                404,
                "text/plain",
                "Niet gevonden");
        });
}

// =============================================================================
//  OTA
// =============================================================================

static bool otaStarted = false;

void setupOTA()
{
    if (otaStarted) {
        return;
    }

    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    ArduinoOTA.setHostname(
        "nsd-dalek");

    ArduinoOTA.setPasswordHash(
        OTA_PASSWORD_HASH);

    ArduinoOTA.onStart(
        []() {

            DBGLN("[OTA] Start");

            // Stop motors immediately during OTA.

            setMotorRunning(false);
            setMotorCmd(2);
        });

    ArduinoOTA.onEnd(
        []() {

            DBGLN("[OTA] End");
        });

    ArduinoOTA.onProgress(
        [](unsigned int progress,
           unsigned int total) {

            if (total == 0) {
                return;
            }

            unsigned int percent =
                (progress * 100U) / total;

            DBG("[OTA] ");
            DBG(percent);
            DBGLN("%");
        });

    ArduinoOTA.onError(
        [](ota_error_t error) {

            DBG("[OTA] Error: ");
            DBGLN(error);
        });

    ArduinoOTA.begin();

    otaStarted = true;

    DBGLN("[OTA] Ready");
}

// =============================================================================
//  WIFI
// =============================================================================

static unsigned long wifiReconnectStarted = 0;

static bool wifiReconnectPending = false;

void startWiFiReconnect()
{
    if (wifiReconnectPending) {
        return;
    }

    DBGLN("[WIFI] Reconnect gestart");

    WiFi.disconnect();

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD);

    wifiReconnectStarted = millis();

    wifiReconnectPending = true;
}

void processWiFi()
{
    if (WiFi.status() == WL_CONNECTED) {

        if (wifiReconnectPending) {

            wifiReconnectPending = false;

            DBGLN("[WIFI] Verbonden");

            DBG("[WIFI] IP: ");
            DBGLN(WiFi.localIP());

            setupOTA();
        }

        return;
    }

    if (!wifiReconnectPending) {
        return;
    }

    // Give the connection attempt up to 10 seconds.

    if (millis() - wifiReconnectStarted >=
        10000UL) {

        wifiReconnectPending = false;

        DBGLN(
            "[WIFI] Verbinding mislukt");
    }
}

// =============================================================================
//  SETUP
// =============================================================================

void setup()
{
#ifdef DEBUG
    Serial.begin(115200);

    delay(50);
#endif

    DBGLN("");
    DBGLN("");
    DBGLN("========================================");
    DBGLN("   NSD DALEK ESP32");
    DBG("   Firmware: ");
    DBGLN(FIRMWARE_VERSION);
    DBGLN("========================================");

    // -------------------------------------------------------------------------
    // Reset reason
    // -------------------------------------------------------------------------

    DBG("[BOOT] Reset reason: ");

    switch (esp_reset_reason()) {

        case ESP_RST_POWERON:
            DBGLN("Power on");
            break;

        case ESP_RST_SW:
            DBGLN("Software reset");
            break;

        case ESP_RST_PANIC:
            DBGLN("PANIC / CRASH");
            break;

        case ESP_RST_INT_WDT:
            DBGLN("Interrupt watchdog");
            break;

        case ESP_RST_TASK_WDT:
            DBGLN("Task watchdog");
            break;

        case ESP_RST_WDT:
            DBGLN("Watchdog");
            break;

        case ESP_RST_BROWNOUT:
            DBGLN("BROWNOUT");
            break;

        case ESP_RST_SDIO:
            DBGLN("SDIO reset");
            break;

        default:
            DBGLN("Unknown");
            break;
    }

    // -------------------------------------------------------------------------
    // LittleFS
    // -------------------------------------------------------------------------

    if (LittleFS.begin(true)) {

        DBGLN("[FS] LittleFS OK");

    } else {

        DBGLN("[FS] LittleFS ERROR");
    }

    // -------------------------------------------------------------------------
    // FastLED
    // -------------------------------------------------------------------------

    FastLED
        .addLeds<
            LED_CHIPSET,
            PIN_LED_DATA,
            LED_COLOR_ORDER
        >(leds, NUM_LEDS)
        .setCorrection(
            Typical8mmPixel);

    FastLED.clear(true);

    // -------------------------------------------------------------------------
    // Sensor pins
    // -------------------------------------------------------------------------

    pinMode(
        PIN_SONIC_TRIGGER,
        OUTPUT);

    digitalWrite(
        PIN_SONIC_TRIGGER,
        LOW);

    pinMode(
        PIN_SONIC_RIGHT,
        INPUT);

    pinMode(
        PIN_SONIC_CENTER,
        INPUT);

    pinMode(
        PIN_SONIC_LEFT,
        INPUT);

    // -------------------------------------------------------------------------
    // WiFi
    // -------------------------------------------------------------------------

    DBGLN("----------------------------------------");
    DBG("[WIFI] SSID: ");
    DBGLN(WIFI_SSID);

    WiFi.mode(WIFI_STA);

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD);

    unsigned long wifiStart =
        millis();

    while (WiFi.status() != WL_CONNECTED &&
           millis() - wifiStart < 10000UL) {

        delay(250);

        DBG(".");
    }

    DBGLN("");

    if (WiFi.status() == WL_CONNECTED) {

        DBGLN("[WIFI] CONNECTED");

        DBG("[WIFI] IP: ");
        DBGLN(WiFi.localIP());

        DBG("[WIFI] Gateway: ");
        DBGLN(WiFi.gatewayIP());

        DBG("[WIFI] RSSI: ");
        DBG(WiFi.RSSI());
        DBGLN(" dBm");

        DBG("[WIFI] Channel: ");
        DBGLN(WiFi.channel());

        DBG("[WIFI] MAC: ");
        DBGLN(WiFi.macAddress());

        setupOTA();

    } else {

        DBGLN(
            "[WIFI] Offline mode");
    }

    DBGLN("----------------------------------------");

    // -------------------------------------------------------------------------
    // DFPlayer
    // -------------------------------------------------------------------------

    Serial2.begin(
        9600,
        SERIAL_8N1,
        PIN_DFPLAYER_RX,
        PIN_DFPLAYER_TX);

    bool dfOK = false;

    unsigned long dfStart =
        millis();

    while (!dfOK &&
           millis() - dfStart < 2500UL) {

        if (mp3.begin(Serial2)) {

            dfOK = true;

        } else {

            DBGLN(
                "[DFPLAYER] Retry");

            delay(200);
        }
    }

    if (dfOK) {

        setDFPlayerAvailable(true);

        mp3.volume(
            DEFAULT_VOLUME);

        DBGLN(
            "[DFPLAYER] OK");

    } else {

        setDFPlayerAvailable(false);
        setSoundEnabled(false);

        DBGLN(
            "[DFPLAYER] NOT FOUND");
    }

    // -------------------------------------------------------------------------
    // Dome boot animation
    // -------------------------------------------------------------------------

    domeStateStart = millis();

    domeState = DOME_BOOT;

    DBGLN(
        "[DOME] Boot animation");

    while (updateDomeFSM()) {

        yield();
    }

    DBGLN(
        "[DOME] Boot complete");

    // -------------------------------------------------------------------------
    // Startup sound
    // -------------------------------------------------------------------------

    playSound(SND_MOAN);

    // -------------------------------------------------------------------------
    // Webserver
    // -------------------------------------------------------------------------

    setupWebRoutes();

    server.begin();

    DBGLN(
        "[WEB] Server started");

    // -------------------------------------------------------------------------
    // Motor task
    // -------------------------------------------------------------------------

    BaseType_t taskResult =
        xTaskCreatePinnedToCore(
            motorTask,
            "motorTask",
            8192,
            NULL,
            1,
            NULL,
            0);

    if (taskResult == pdPASS) {

        DBGLN(
            "[MOTOR] Task started on Core 0");

    } else {

        DBGLN(
            "[MOTOR] ERROR: task creation failed");
    }

    // -------------------------------------------------------------------------
    // Initial eye
    // -------------------------------------------------------------------------

    setDisplayMode(true);

    startFadeEvent(
        CRGB::Blue);

    DBGLN(
        "[BOOT] Startup complete");
}

// =============================================================================
//  LOOP — CORE 1
// =============================================================================

void loop()
{
    // -------------------------------------------------------------------------
    // Webserver
    // -------------------------------------------------------------------------

    server.handleClient();

    // -------------------------------------------------------------------------
    // OTA
    // -------------------------------------------------------------------------

    if (otaStarted) {
        ArduinoOTA.handle();
    }

    // -------------------------------------------------------------------------
    // WiFi
    // -------------------------------------------------------------------------

    processWiFi();

    // -------------------------------------------------------------------------
    // Watchdog
    // -------------------------------------------------------------------------

    esp_task_wdt_reset();

    // -------------------------------------------------------------------------
    // Persistent state
    // -------------------------------------------------------------------------

    static int previousDomeCmd = -1;

    static int boredCount = 0;

    static unsigned long lastBored =
        millis();

    static unsigned long lastPulse =
        millis();

    static unsigned long lastHeartbeat =
        millis();

    static unsigned long lastWiFiRetry =
        millis();

    unsigned long now =
        millis();

    // -------------------------------------------------------------------------
    // Heartbeat
    // -------------------------------------------------------------------------

    if (now - lastHeartbeat >=
        30000UL) {

        lastHeartbeat = now;

        DBG("[HB] uptime=");
        DBG(now / 1000UL);

        DBG("s heap=");
        DBG(ESP.getFreeHeap());

        DBG(" wifi=");

        if (WiFi.status() == WL_CONNECTED) {
            DBGLN("OK");
        } else {
            DBGLN("DOWN");
        }
    }

    // -------------------------------------------------------------------------
    // WiFi reconnect request
    // -------------------------------------------------------------------------

    if (WiFi.status() != WL_CONNECTED &&
        !wifiReconnectPending &&
        now - lastWiFiRetry >= 30000UL) {

        lastWiFiRetry = now;

        startWiFiReconnect();
    }

    // -------------------------------------------------------------------------
    // Dome command processing
    // -------------------------------------------------------------------------

    processDomeCmd(
        previousDomeCmd,
        boredCount,
        lastBored,
        lastPulse);

    // -------------------------------------------------------------------------
    // Current display state
    // -------------------------------------------------------------------------

    bool display =
        getDisplayMode();

    // -------------------------------------------------------------------------
    // Eye pulse
    // -------------------------------------------------------------------------

    if (display &&
        domeState == DOME_IDLE &&
        now - lastPulse >=
            PULSE_INTERVAL_MS) {

        startPulse();

        lastPulse = now;
    }

    // -------------------------------------------------------------------------
    // Bored timer
    // -------------------------------------------------------------------------

    if (display &&
        domeState == DOME_IDLE &&
        now - lastBored >=
            BORED_INTERVAL_MS) {

        DBGLN(
            "[DOME] Bored");

        int boredVolume =
            getVolume();

        if (getDFPlayerAvailable()) {

            mp3.volume(
                boredVolume);
        }

        if (boredCount <
            BORED_COUNT_MAX) {

            playSound(
                SND_MOAN);

            boredCount++;

        } else {

            DBGLN(
                "[DOME] Really bored");

            playSound(
                SND_REALLY_BORED);

            paletteIdx = 0;

            paletteRound = 0;

            domeState =
                DOME_BORED_PALETTE;

            boredCount = 0;
        }

        lastBored = now;

        lastPulse = now;
    }

    // -------------------------------------------------------------------------
    // Yield
    // -------------------------------------------------------------------------

    delay(1);
}