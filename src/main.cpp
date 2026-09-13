// =============================================================
//  main.cpp  -  Dalek ESP32 unified firmware - V0.52
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
//    3. Sensor PWM capture via interrupts - all three Maxbotix PW
//       outputs are measured concurrently after one shared trigger,
//       so one sensor cannot make another pulse get missed.
//    4. WiFi credentials in secrets.ini  - never in source code.
//    5. Motor direction invert flags  - configurable in config.h
//       without rewiring.
// =============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FastLED.h>
#include <FastAccelStepper.h>
#include <DFRobotDFPlayerMini.h>
#include <ArduinoOTA.h>
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "config.h"

static constexpr const char* FIRMWARE_VERSION = "V0.53";

// V0.52 WiFi/OTA state
static bool otaStarted = false;
static unsigned long lastWifiAttempt = 0;
static wl_status_t lastWifiStatus = WL_IDLE_STATUS;

// =============================================================
//  SHARED STATE
//  Written by Core 1 web handler or Core 1 dome logic,
//  read by Core 0 motor task (and vice-versa for sensor values).
//  Protected by a FreeRTOS spinlock (portMUX).
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
volatile bool dfplayerAvailable = false;

// Current DFPlayer playback state for the web interface.
static volatile bool mp3Playing = false;
static volatile int mp3CurrentTrack = 0;
static volatile int mp3LastEventType = -1;
static volatile int mp3LastEventValue = 0;

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
//  CORE 0 TASK  -  sensors + steppers
//
//  FastAccelStepper generates step pulses via the RMT peripheral
//  in the background, so this task only needs to call move/stop
//  when the command changes. Sensor reads use the MaxBotix
//  sequential RX->TX chain: one trigger starts the right sensor,
//  then its TX output triggers center, and center TX triggers left.
//  The three PW pulse widths are captured by interrupts while the
//  sequence completes.
// =============================================================

struct SensorFilter {
    long samples[3] = {SONIC_MAX_CM, SONIC_MAX_CM, SONIC_MAX_CM};
    uint8_t count = 0;
    uint8_t index = 0;
    long lastValid = SONIC_MAX_CM;
};

static SensorFilter filterRight, filterCenter, filterLeft;

// Maxbotix LV-MaxSonar-EZ1 PWM output: 147 microseconds per inch.
static constexpr float SONIC_US_PER_INCH = 147.0f;
static constexpr float SONIC_CM_PER_US   = 2.54f / SONIC_US_PER_INCH;

// Interrupt-captured pulse widths for the three Maxbotix PWM outputs.
static volatile uint32_t sensorStartRight = 0;
static volatile uint32_t sensorStartCenter = 0;
static volatile uint32_t sensorStartLeft = 0;
static volatile uint32_t sensorPulseRight = 0;
static volatile uint32_t sensorPulseCenter = 0;
static volatile uint32_t sensorPulseLeft = 0;
static volatile bool sensorPulseReadyRight = false;
static volatile bool sensorPulseReadyCenter = false;
static volatile bool sensorPulseReadyLeft = false;

static void IRAM_ATTR sensorRightISR() {
    uint32_t now = micros();
    if (digitalRead(PIN_SONIC_RIGHT)) {
        sensorStartRight = now;
    } else if (sensorStartRight != 0) {
        sensorPulseRight = now - sensorStartRight;
        sensorPulseReadyRight = true;
    }
}

static void IRAM_ATTR sensorCenterISR() {
    uint32_t now = micros();
    if (digitalRead(PIN_SONIC_CENTER)) {
        sensorStartCenter = now;
    } else if (sensorStartCenter != 0) {
        sensorPulseCenter = now - sensorStartCenter;
        sensorPulseReadyCenter = true;
    }
}

static void IRAM_ATTR sensorLeftISR() {
    uint32_t now = micros();
    if (digitalRead(PIN_SONIC_LEFT)) {
        sensorStartLeft = now;
    } else if (sensorStartLeft != 0) {
        sensorPulseLeft = now - sensorStartLeft;
        sensorPulseReadyLeft = true;
    }
}

static long median3(long a, long b, long c) {
    if (a > b) { long t=a; a=b; b=t; }
    if (b > c) { long t=b; b=c; c=t; }
    if (a > b) { long t=a; a=b; b=t; }
    return b;
}

static long filterSensor(SensorFilter& f, uint32_t pulseUs) {
    // No valid pulse: retain the last valid distance.
    if (pulseUs == 0) return f.lastValid;

    long cm = lroundf((float)pulseUs * SONIC_CM_PER_US);
    if (cm < 1) cm = 1;
    if (cm > SONIC_MAX_CM) cm = SONIC_MAX_CM;

    f.lastValid = cm;
    f.samples[f.index] = cm;
    f.index = (f.index + 1) % 3;
    if (f.count < 3) f.count++;

    if (f.count == 1) return f.samples[0];
    if (f.count == 2) return (f.samples[0] + f.samples[1]) / 2;
    return median3(f.samples[0], f.samples[1], f.samples[2]);
}

// Non-blocking MaxBotix sequential-chain sensor state machine.
// One external trigger starts the right sensor; TX->RX then cascades
// to center and left. ISRs capture the PW pulse from each sensor.
struct SensorSequence {
    bool active = false;
    unsigned long startedAt = 0;
};

static SensorSequence sensorSequence;
static unsigned long lastSensorSequence = 0;

// Forward declaration for the sensor sequence completion handler.
void sensorAction();
static constexpr unsigned long SENSOR_SEQUENCE_PERIOD_MS = 250UL;
static constexpr unsigned long SENSOR_SEQUENCE_TIMEOUT_MS = 220UL;

static void resetSensorCapture() {
    portENTER_CRITICAL(&cmdMux);
    sensorStartRight = 0;
    sensorStartCenter = 0;
    sensorStartLeft = 0;
    sensorPulseRight = 0;
    sensorPulseCenter = 0;
    sensorPulseLeft = 0;
    sensorPulseReadyRight = false;
    sensorPulseReadyCenter = false;
    sensorPulseReadyLeft = false;
    portEXIT_CRITICAL(&cmdMux);
}

static void startSensorSequence() {
    resetSensorCapture();

    digitalWrite(PIN_SONIC_TRIGGER, HIGH);
    delayMicroseconds(25);
    digitalWrite(PIN_SONIC_TRIGGER, LOW);

    sensorSequence.active = true;
    sensorSequence.startedAt = millis();
}

static void finishSensorSequence() {
    uint32_t rp = 0, cp = 0, lp = 0;
    bool rr = false, cr = false, lr = false;

    portENTER_CRITICAL(&cmdMux);
    rp = sensorPulseRight;
    cp = sensorPulseCenter;
    lp = sensorPulseLeft;
    rr = sensorPulseReadyRight;
    cr = sensorPulseReadyCenter;
    lr = sensorPulseReadyLeft;
    portEXIT_CRITICAL(&cmdMux);

    long r = filterSensor(filterRight, rr ? rp : 0);
    long c = filterSensor(filterCenter, cr ? cp : 0);
    long l = filterSensor(filterLeft, lr ? lp : 0);

    portENTER_CRITICAL(&cmdMux);
    rightCM = r;
    centerCM = c;
    leftCM = l;
    portEXIT_CRITICAL(&cmdMux);

    static unsigned long lastSensorLog = 0;
    if (millis() - lastSensorLog >= 5000UL) {
        lastSensorLog = millis();
        DBG("[US] R="); DBG(r); DBG(" C="); DBG(c); DBG(" L=");
        DBG(l);
        if (!rr || !cr || !lr) {
            DBG(" | miss:");
            if (!rr) DBG(" R");
            if (!cr) DBG(" C");
            if (!lr) DBG(" L");
        }
        DBGLN("");
    }

    sensorAction();
    sensorSequence.active = false;
    lastSensorSequence = millis();
}

static void updateSensorSequence() {
    const unsigned long now = millis();

    if (!sensorSequence.active) {
        if (now - lastSensorSequence >= SENSOR_SEQUENCE_PERIOD_MS) {
            startSensorSequence();
        }
        return;
    }

    bool rr, cr, lr;
    portENTER_CRITICAL(&cmdMux);
    rr = sensorPulseReadyRight;
    cr = sensorPulseReadyCenter;
    lr = sensorPulseReadyLeft;
    portEXIT_CRITICAL(&cmdMux);

    // Finish as soon as all three have arrived, or after the timeout so
    // one failed sensor can never stall the navigation loop.
    if ((rr && cr && lr) || (now - sensorSequence.startedAt >= SENSOR_SEQUENCE_TIMEOUT_MS)) {
        finishSensorSequence();
    }
}

// Sticky blocked/clear state with hysteresis: a side becomes "blocked" at
// SONIC_MIN_CM but doesn't clear again until it's SONIC_HYSTERESIS_CM
// further out, so a reading sitting right on the threshold doesn't flip
// the movement decision every cycle.
static inline void updateBlocked(bool &blocked, long distanceCM) {
    if (!blocked && distanceCM <= SONIC_MIN_CM) {
        blocked = true;
    } else if (blocked && distanceCM > SONIC_MIN_CM + SONIC_HYSTERESIS_CM) {
        blocked = false;
    }
}

void sensorAction() {
    long r, c, l;
    bool running;
    portENTER_CRITICAL(&cmdMux);
    r = rightCM; c = centerCM; l = leftCM; running = motorRunning;
    portEXIT_CRITICAL(&cmdMux);

    static bool midTriggered = false;
    static bool minTriggered = false;
    static bool rBlocked = false, cBlocked = false, lBlocked = false;
    static int reverseStreak = 0;

    updateBlocked(rBlocked, r);
    updateBlocked(cBlocked, c);
    updateBlocked(lBlocked, l);

    if (!running) return;

    if (!rBlocked && !cBlocked && !lBlocked) {
        setMotorCmd(1);
        setDomeCmd(10);
        midTriggered = false;
        minTriggered = false;
        reverseStreak = 0;
    } else if (rBlocked && cBlocked && lBlocked) {
        setMotorCmd(2);
        reverseStreak = 0;
    } else if (rBlocked && !cBlocked && !lBlocked) {
        setMotorCmd(3);
        reverseStreak = 0;
    } else if (!rBlocked && !cBlocked && lBlocked) {
        setMotorCmd(4);
        reverseStreak = 0;
    } else if (rBlocked && cBlocked && !lBlocked) {
        setMotorCmd(3);
        reverseStreak = 0;
    } else if (!rBlocked && cBlocked && lBlocked) {
        setMotorCmd(4);
        reverseStreak = 0;
    } else {
        if (reverseStreak >= MOTOR_REVERSE_ESCAPE_LIMIT) {
            setMotorCmd(r >= l ? 4 : 3);
            reverseStreak = 0;
        } else {
            setMotorCmd(5);
            reverseStreak++;
        }
    }

    if ((r <= SONIC_MID_CM || c <= SONIC_MID_CM || l <= SONIC_MID_CM) && !midTriggered) {
        setDomeCmd(11);
        midTriggered = true;
    }
    if ((rBlocked || cBlocked || lBlocked) && !minTriggered) {
        setDomeCmd(12);
        minTriggered = true;
    }
}

void applyMotorCmd(bool force = false) {
    static int prevAppliedCmd = -1;
    int cmd;
    portENTER_CRITICAL(&cmdMux);
    cmd = motorCmd;
    portEXIT_CRITICAL(&cmdMux);

    if (!leftStepper || !rightStepper) return;
    if (!force && cmd == prevAppliedCmd) return;
    prevAppliedCmd = cmd;

    switch (cmd) {
        case 1:
            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED);
            leftStepper->runForward();
            rightStepper->runForward();
            break;
        case 2:
            leftStepper->stopMove();
            rightStepper->stopMove();
            break;
        case 3:
            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED / MOTOR_TURN_SLOW_DIV);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED);
            leftStepper->runForward();
            rightStepper->runForward();
            break;
        case 4:
            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED / MOTOR_TURN_SLOW_DIV);
            leftStepper->runForward();
            rightStepper->runForward();
            break;
        case 5:
            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED / 2);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED / 2);
            leftStepper->runBackward();
            rightStepper->runBackward();
            break;
        default:
            break;
    }
}

void motorTask(void* pvParameters) {
    esp_task_wdt_add(NULL);
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

    bool wasRunning = false;

    for (;;) {
        updateSensorSequence();

        bool mr;
        portENTER_CRITICAL(&cmdMux);
        mr = motorRunning;
        portEXIT_CRITICAL(&cmdMux);
        if (mr) {
            applyMotorCmd(wasRunning == false);
        } else if (wasRunning) {
            // Only issue the stop command on the running -> stopped transition.
            setMotorCmd(2);
            applyMotorCmd(true);
        }
        wasRunning = mr;

        esp_task_wdt_reset();

        // FastAccelStepper handles pulses via RMT interrupt - no run() needed.
        // Yield to keep watchdog happy without a fixed 1 ms penalty.
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

    switch (domeState) {

        case DOME_IDLE:
            return false;   // ready for next event

        // ---- fade up (used for stay-away / exterminate / stalkBlue) ----
        case DOME_FADE_UP:
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
            fadeBrightness -= 3;
            if (fadeBrightness <= 20) { fadeBrightness = 20; domeState = DOME_PULSE_UP; }
            FastLED.setBrightness(fadeBrightness);
            FastLED.show();
            return true;

        case DOME_PULSE_UP:
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
    bool se, available;
    portENTER_CRITICAL(&cmdMux);
    se = soundEnabled;
    available = dfplayerAvailable;
    portEXIT_CRITICAL(&cmdMux);

    if (!se || !available) return;
    if (track < 1 || track > 255) return;

    mp3.playFolder(SND_FOLDER, track);

    portENTER_CRITICAL(&cmdMux);
    mp3Playing = true;
    mp3CurrentTrack = track;
    mp3LastEventType = -1;
    mp3LastEventValue = 0;
    portEXIT_CRITICAL(&cmdMux);

    DBG("[MP3] play folder "); DBG(SND_FOLDER);
    DBG(" track "); DBGLN(track);
}

void stopSound() {
    bool available;
    portENTER_CRITICAL(&cmdMux);
    available = dfplayerAvailable;
    portEXIT_CRITICAL(&cmdMux);

    if (!available) return;

    mp3.stop();

    portENTER_CRITICAL(&cmdMux);
    mp3Playing = false;
    mp3CurrentTrack = 0;
    portEXIT_CRITICAL(&cmdMux);

    DBGLN("[MP3] stop");
}

void processDFPlayerEvents() {
    bool available;
    portENTER_CRITICAL(&cmdMux);
    available = dfplayerAvailable;
    portEXIT_CRITICAL(&cmdMux);

    if (!available) return;

    while (mp3.available()) {
        uint8_t type = mp3.readType();
        int value = mp3.read();

        portENTER_CRITICAL(&cmdMux);
        mp3LastEventType = type;
        mp3LastEventValue = value;
        portEXIT_CRITICAL(&cmdMux);

        if (type == DFPlayerPlayFinished) {
            portENTER_CRITICAL(&cmdMux);
            mp3Playing = false;
            mp3CurrentTrack = 0;
            portEXIT_CRITICAL(&cmdMux);

            DBG("[MP3] finished track "); DBGLN(value);
        } else if (type == DFPlayerError) {
            portENTER_CRITICAL(&cmdMux);
            mp3Playing = false;
            mp3CurrentTrack = 0;
            portEXIT_CRITICAL(&cmdMux);

            DBG("[MP3] error code "); DBGLN(value);
        }
    }
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
        bool available;
        portENTER_CRITICAL(&cmdMux); available = dfplayerAvailable; portEXIT_CRITICAL(&cmdMux);
        if (volume < 30) { volume++; if (available) mp3.volumeUp(); }
        DBGLN("Volume UP");
        setDomeCmd(prevCmd);
        return;
    }
    if (cmd == 17) {
        bool available;
        portENTER_CRITICAL(&cmdMux); available = dfplayerAvailable; portEXIT_CRITICAL(&cmdMux);
        if (volume > 0) { volume--; if (available) mp3.volumeDown(); }
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
                bool available;
                portENTER_CRITICAL(&cmdMux); available = dfplayerAvailable; portEXIT_CRITICAL(&cmdMux);
                if (available) mp3.volume(volume);
                startFadeEvent(CRGB::White);
                playSound(SND_STAY_AWAY);
                lastBored = millis();
                setDomeCmd(10);
                prevCmd = 10;
            }
            break;

        case 12:  // exterminate
            DBGLN("Exterminate!!");
            bool available;
            portENTER_CRITICAL(&cmdMux); available = dfplayerAvailable; portEXIT_CRITICAL(&cmdMux);
            if (available) mp3.volume(30);
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
            domeState = DOME_IDLE;
            fadeBrightness = 0;
            FastLED.clear();
            FastLED.setBrightness(0);
            FastLED.show();
            DBGLN("Display mode OFF");
            prevCmd = cmd;
            break;
    }
}

// =============================================================
//  WEB SERVER  (Core 1)
// =============================================================

bool checkToken() {
    if (server.hasHeader("X-Token") && server.header("X-Token") == String(API_TOKEN)) {
        return true;
    }
    server.send(403, "text/plain", "Forbidden");
    return false;
}

// Main web interface stored in flash, not on the loopTask stack.
static const char WEB_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="nl">
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DALEK COMMAND</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@500;600;700;800&family=Rajdhani:wght@500;600;700&display=swap');
:root{
  --bg:#050a07;
  --panel:#0a1410;
  --panel2:#0e1c16;
  --text:#c8ffe8;
  --muted:#72a890;
  --line:#214332;
  --on:#00ff88;
  --off:#ff3344;
  --warn:#ffc857;
  --blue:#4da6ff;
}
*{box-sizing:border-box}
body{
  background:radial-gradient(circle at top,#102018 0,#050a07 55%);
  color:var(--text);
  font-family:monospace;
  margin:0;
  padding:18px;
}
.wrap{max-width:900px;margin:0 auto}
h1{margin:6px 0 2px;font-family:'Orbitron','Rajdhani','Arial Narrow',sans-serif;font-size:34px;font-weight:800;letter-spacing:5px;text-align:center;text-transform:uppercase;text-shadow:0 0 10px rgba(0,255,136,.22)}
.subtitle{color:var(--muted);margin-bottom:18px;text-align:center;font-family:'Rajdhani','Arial Narrow',sans-serif;font-size:16px;letter-spacing:2px}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin-bottom:14px}
@media(max-width:700px){.grid{grid-template-columns:1fr}.sensor-grid{grid-template-columns:1fr!important}}
.card{
  background:rgba(10,20,16,.94);
  border:1px solid var(--line);
  border-radius:12px;
  padding:14px;
  box-shadow:0 0 20px rgba(0,0,0,.25);
}
.card h2{font-size:15px;margin:0 0 10px;color:var(--muted);font-weight:normal}
.state{
  font-size:24px;
  font-weight:bold;
  margin-bottom:10px;
  text-shadow:0 0 8px currentColor;
}
.state.on{color:var(--on)}
.state.off{color:var(--off)}
button{
  width:100%;
  border:1px solid var(--line);
  border-radius:9px;
  padding:12px 10px;
  margin-top:8px;
  background:#0b1712;
  color:var(--text);
  font-family:'Rajdhani','Arial Narrow',sans-serif;
  font-weight:600;
  font-size:18px;
  letter-spacing:1px;
  cursor:pointer;
  transition:.15s ease;
}
button:hover{filter:brightness(1.25);transform:translateY(-1px)}
button.active{border-color:var(--on);background:#06351f;color:#fff;box-shadow:0 0 12px rgba(0,255,136,.18)}
button.inactive{border-color:var(--off);background:#351016;color:#fff;box-shadow:0 0 12px rgba(255,51,68,.12)}
button:disabled{opacity:.45;cursor:not-allowed;transform:none}
.sensor-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
.sensor{text-align:center;background:var(--panel2);border:1px solid var(--line);border-radius:10px;padding:12px}
.sensor .name{color:var(--muted);font-size:14px}
.sensor .dist{font-size:29px;font-weight:bold;margin:5px 0}
.sensor .dist.clear{color:var(--on)}
.sensor .dist.warn{color:var(--warn)}
.sensor .dist.blocked{color:var(--off)}
.sensor .state{font-size:13px;margin:0;text-shadow:none}
.sensor .state.clear{color:var(--on)}
.sensor .state.warn{color:var(--warn)}
.sensor .state.blocked{color:var(--off)}
.controls{display:grid;grid-template-columns:1fr 1fr;gap:12px}
@media(max-width:700px){.controls{grid-template-columns:1fr}}
.volume{display:flex;align-items:center;justify-content:center;gap:10px}
.volume button{width:58px;min-width:58px;height:48px;margin:0;display:flex;align-items:center;justify-content:center;font-family:Arial,sans-serif;font-size:30px;font-weight:400;line-height:1;padding:0}
.volume-symbol{position:relative;display:block;width:20px;height:20px}
.volume-minus::before{content:'';position:absolute;left:0;right:0;top:8px;height:3px;background:currentColor;border-radius:2px}
.volume-plus::before,.volume-plus::after{content:'';position:absolute;background:currentColor;border-radius:2px}
.volume-plus::before{left:0;right:0;top:8px;height:3px}
.volume-plus::after{top:0;bottom:0;left:8px;width:3px}
.volume-value{min-width:56px;text-align:center;font-family:'Orbitron',monospace;font-size:24px;font-weight:600}
.info{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;font-size:14px}
@media(max-width:700px){.info{grid-template-columns:1fr}}
.info-row{display:flex;justify-content:space-between;border-bottom:1px solid #163124;padding:6px 0}
.label{color:var(--muted)}
#connection{font-weight:bold}
#connection.ok{color:var(--on)}
#connection.down{color:var(--off)}
#notice{min-height:22px;margin:12px 0;color:var(--muted);font-size:14px}
.mp3-now{font-family:'Orbitron',monospace;font-size:21px;color:var(--on);margin-bottom:5px}
.mp3-meta{color:var(--muted);font-size:14px;margin-bottom:12px}
.mp3-controls{display:grid;grid-template-columns:110px 1fr 1fr;gap:8px}
.mp3-controls input{width:100%;border:1px solid var(--line);border-radius:9px;background:#08120e;color:var(--text);font-family:'Orbitron',monospace;font-size:20px;text-align:center;padding:8px}
.mp3-controls button{margin:0}
.play-btn{border-color:var(--on)!important}
.stop-btn{border-color:var(--off)!important}
.media-icon{display:inline-block;vertical-align:-2px;margin-right:9px;position:relative;width:0;height:0}
.media-play{border-top:9px solid transparent;border-bottom:9px solid transparent;border-left:14px solid currentColor}
.media-stop{width:15px;height:15px;border-radius:2px;background:currentColor;vertical-align:-2px}
.quick-title{margin-top:14px;color:var(--muted);font-size:12px;letter-spacing:2px}
.quick-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
.quick-grid button{font-size:12px;margin-top:8px;padding:10px 6px}
.quick-grid button span{color:var(--muted);font-size:11px}
@media(max-width:700px){.mp3-controls{grid-template-columns:90px 1fr 1fr}.quick-grid{grid-template-columns:1fr 1fr}}
.footer{text-align:center;color:#537562;font-size:12px;margin:15px 0 5px}
</style>
</head>
<body>
<div class="wrap">
  <h1>!! EXTERMINATE !!</h1>
  <div class="subtitle">DALEK COMMAND &nbsp;|&nbsp; <span id="version">---</span></div>

  <div class="grid">
    <div class="card">
      <h2>DISPLAY</h2>
      <div id="displayState" class="state off">UIT</div>
      <button id="displayBtn" onclick="cmd('/display/toggle')">DISPLAY</button>
    </div>
    <div class="card">
      <h2>MOTOREN</h2>
      <div id="motorState" class="state off">GESTOPT</div>
      <button id="motorBtn" onclick="cmd('/movement/toggle')">MOTOREN</button>
    </div>
    <div class="card">
      <h2>GELUID</h2>
      <div id="soundState" class="state off">UIT</div>
      <button id="soundBtn" onclick="cmd('/sound/toggle')">GELUID</button>
    </div>
  </div>

  <div class="card" style="margin-bottom:14px">
    <h2>AFSTANDSSENSOREN</h2>
    <div class="sensor-grid">
      <div class="sensor">
        <div class="name">RECHTS</div>
        <div id="r" class="dist">---</div>
        <div id="rs" class="state">---</div>
      </div>
      <div class="sensor">
        <div class="name">MIDDEN</div>
        <div id="c" class="dist">---</div>
        <div id="cs" class="state">---</div>
      </div>
      <div class="sensor">
        <div class="name">LINKS</div>
        <div id="l" class="dist">---</div>
        <div id="ls" class="state">---</div>
      </div>
    </div>
  </div>

  <div class="controls">
    <div class="card">
      <h2>VOLUME</h2>
      <div class="volume">
        <button class="volume-btn" onclick="cmd('/volume/down')" aria-label="Volume lager"><span class="volume-symbol volume-minus"></span></button>
        <div id="v" class="volume-value">--</div>
        <button class="volume-btn" onclick="cmd('/volume/up')" aria-label="Volume hoger"><span class="volume-symbol volume-plus"></span></button>
      </div>
    </div>
    <div class="card">
      <h2>VERBINDING</h2>
      <div id="connection" class="down">OFFLINE</div>
      <div id="notice">Status wordt geladen...</div>
    </div>
  </div>

  <div class="card" style="margin-top:14px">
    <h2>GELUIDSPLAYER</h2>
    <div class="mp3-now" id="mp3Now">GEEN GELUID</div>
    <div class="mp3-meta" id="mp3Meta">DFPlayer: ---</div>
    <div class="mp3-controls">
      <input id="trackInput" type="number" min="1" max="255" value="1" aria-label="MP3 track">
      <button class="play-btn" onclick="playTrack()"><span class="media-icon media-play" aria-hidden="true"></span>AFSPELEN</button>
      <button class="stop-btn" onclick="cmd('/sound/stop')"><span class="media-icon media-stop" aria-hidden="true"></span>STOP</button>
    </div>
    <div class="quick-title">SNELKEUZE</div>
    <div class="quick-grid">
      <button onclick="playTrack(1)">EXTERMINATE<br><span>001</span></button>
      <button onclick="playTrack(3)">MOAN<br><span>003</span></button>
      <button onclick="playTrack(4)">STAY AWAY<br><span>004</span></button>
      <button onclick="playTrack(10)">REALLY BORED<br><span>010</span></button>
    </div>
  </div>

  <div class="card" style="margin-top:14px">
    <h2>SYSTEEM</h2>
    <div class="info">
      <div class="info-row"><span class="label">IP</span><span id="ip">---</span></div>
      <div class="info-row"><span class="label">WiFi RSSI</span><span id="rssi">---</span></div>
      <div class="info-row"><span class="label">Kanaal</span><span id="channel">---</span></div>
      <div class="info-row"><span class="label">DFPlayer</span><span id="dfplayer">---</span></div>
      <div class="info-row"><span class="label">Uptime</span><span id="uptime">---</span></div>
    </div>
  </div>

  <div class="footer">Automatische update elke 2 seconden</div>
</div>

<script>
const token='%%API_TOKEN%%';
const MIN_CM=30;
const MID_CM=50;
const MAX_CM=300;

function setState(id, on, onText, offText){
  const el=document.getElementById(id);
  el.textContent=on?onText:offText;
  el.className='state '+(on?'on':'off');
}

function setButton(id, on){
  const el=document.getElementById(id);
  el.className=on?'active':'inactive';
  el.textContent=on?'AAN':'UIT';
}

function sensor(id, stateId, value){
  const el=document.getElementById(id);
  const st=document.getElementById(stateId);
  if(value>=MAX_CM){
    el.textContent='---';
    el.className='dist clear';
    st.textContent='VRIJ';
    st.className='state clear';
  }else{
    el.textContent=value+' cm';
    if(value<=MIN_CM){
      el.className='dist blocked';
      st.textContent='GEBLOKKEERD';
      st.className='state blocked';
    }else if(value<=MID_CM){
      el.className='dist warn';
      st.textContent='DICHTBIJ';
      st.className='state warn';
    }else{
      el.className='dist clear';
      st.textContent='VRIJ';
      st.className='state clear';
    }
  }
}

async function cmd(u){
  try{
    const r=await fetch(u,{headers:{'X-Token':token},cache:'no-store'});
    if(!r.ok) throw new Error((await r.text())||('HTTP '+r.status));
    document.getElementById('notice').textContent='Commando uitgevoerd';
    await poll();
  }catch(e){
    document.getElementById('notice').textContent='Commando mislukt: '+e.message;
  }
}

async function playTrack(track){
  const input=document.getElementById('trackInput');
  if(track===undefined) track=Number(input.value);
  if(!Number.isInteger(track) || track<1 || track>255){
    document.getElementById('notice').textContent='Track moet 1-255 zijn';
    return;
  }
  input.value=track;
  await cmd('/sound/play?track='+encodeURIComponent(track));
}

async function poll(){
  try{
    const r=await fetch('/status',{cache:'no-store'});
    if(!r.ok) throw new Error('HTTP '+r.status);
    const d=await r.json();

    setState('displayState',d.display,'AAN','UIT');
    setState('motorState',d.motors,'RIJDEND','GESTOPT');
    setState('soundState',d.sound,'AAN','UIT');
    setButton('displayBtn',d.display);
    setButton('motorBtn',d.motors);
    setButton('soundBtn',d.sound);

    sensor('r','rs',Number(d.right));
    sensor('c','cs',Number(d.center));
    sensor('l','ls',Number(d.left));

    document.getElementById('v').textContent=d.volume;
    document.getElementById('version').textContent=d.version;
    document.getElementById('ip').textContent=d.ip||'---';
    document.getElementById('rssi').textContent=d.wifi?d.rssi+' dBm':'---';
    document.getElementById('channel').textContent=d.wifi?d.channel:'---';
    document.getElementById('dfplayer').textContent=d.dfplayer?'OK':'NIET GEVONDEN';
    const mp3Now=document.getElementById('mp3Now');
    const mp3Meta=document.getElementById('mp3Meta');
    if(d.playing){
      mp3Now.textContent='SPEELT: MAP 10 / TRACK '+String(d.track).padStart(3,'0');
      mp3Meta.textContent=d.dfplayer?'DFPlayer: AFSPELEN':'DFPlayer: NIET GEVONDEN';
    }else{
      mp3Now.textContent='GEEN GELUID';
      mp3Meta.textContent=d.dfplayer?'DFPlayer: KLAAR':'DFPlayer: NIET GEVONDEN';
    }
    document.getElementById('uptime').textContent=Math.floor(d.uptime/60)+' min '+(d.uptime%60)+' s';

    const conn=document.getElementById('connection');
    conn.textContent=d.wifi?'ONLINE':'OFFLINE';
    conn.className=d.wifi?'ok':'down';
    document.getElementById('notice').textContent='LIVE';
  }catch(e){
    document.getElementById('connection').textContent='OFFLINE';
    document.getElementById('connection').className='down';
    document.getElementById('notice').textContent='Geen verbinding met de Dalek';
  }
}

poll();
setInterval(poll,2000);
</script>
</body>
</html>)rawliteral";


void handleRoot() {
    String page = FPSTR(WEB_HTML);
    page.replace("%%API_TOKEN%%", API_TOKEN);
    server.send(200,"text/html",page);
}

void handleStatus() {
    bool dm,mr,se,df,playing;
    int vol,track;
    long r,c,l;
    portENTER_CRITICAL(&cmdMux);
    dm=displayMode; mr=motorRunning; se=soundEnabled; vol=volume;
    r=rightCM; c=centerCM; l=leftCM; df=dfplayerAvailable;
    playing=mp3Playing; track=mp3CurrentTrack;
    portEXIT_CRITICAL(&cmdMux);
    char json[512];
    bool wifi = (WiFi.status() == WL_CONNECTED);
    String ip = wifi ? WiFi.localIP().toString() : String();
    snprintf(json,sizeof(json),
        "{\"version\":\"%s\",\"display\":%s,\"motors\":%s,\"sound\":%s,\"dfplayer\":%s,\"playing\":%s,\"track\":%d,\"volume\":%d,\"right\":%ld,\"center\":%ld,\"left\":%ld,\"wifi\":%s,\"ip\":\"%s\",\"rssi\":%d,\"channel\":%d,\"uptime\":%lu}",
        FIRMWARE_VERSION,dm?"true":"false",mr?"true":"false",se?"true":"false",df?"true":"false",
        playing?"true":"false",track,vol,r,c,l,wifi?"true":"false",ip.c_str(),
        wifi?(int)WiFi.RSSI():0,wifi?(int)WiFi.channel():0,millis()/1000UL);
    server.send(200,"application/json",json);
}

void setupWebRoutes() {
    const char* headerKeys[] = {"X-Token"};
    server.collectHeaders(headerKeys, 1);
    server.on("/", handleRoot);
    server.on("/status", handleStatus);

    server.on("/display/toggle", [](){
        if(!checkToken()) return;
        bool dm; portENTER_CRITICAL(&cmdMux); dm=displayMode; portEXIT_CRITICAL(&cmdMux);
        if(dm){
            portENTER_CRITICAL(&cmdMux); displayMode=false; motorRunning=false; soundEnabled=false; portEXIT_CRITICAL(&cmdMux);
            setMotorCmd(2); setDomeCmd(14);
        }else{
            portENTER_CRITICAL(&cmdMux); displayMode=true; soundEnabled=true; portEXIT_CRITICAL(&cmdMux);
            setDomeCmd(15);
        }
        server.send(200,"text/plain","ok");
    });
    server.on("/movement/toggle", [](){
        if(!checkToken()) return;
        bool mr; portENTER_CRITICAL(&cmdMux); mr=motorRunning; portEXIT_CRITICAL(&cmdMux);
        portENTER_CRITICAL(&cmdMux); motorRunning=!mr; portEXIT_CRITICAL(&cmdMux);
        if(mr) setMotorCmd(2);
        server.send(200,"text/plain","ok");
    });
    server.on("/movement/on", [](){
        if(!checkToken()) return;
        portENTER_CRITICAL(&cmdMux); motorRunning=true; portEXIT_CRITICAL(&cmdMux);
        server.send(200,"text/plain","ok");
    });
    server.on("/movement/off", [](){
        if(!checkToken()) return;
        portENTER_CRITICAL(&cmdMux); motorRunning=false; portEXIT_CRITICAL(&cmdMux);
        setMotorCmd(2); server.send(200,"text/plain","ok");
    });
    server.on("/sound/toggle", [](){
        if(!checkToken()) return;
        bool se; portENTER_CRITICAL(&cmdMux); se=soundEnabled; portEXIT_CRITICAL(&cmdMux);
        portENTER_CRITICAL(&cmdMux); soundEnabled=!se; portEXIT_CRITICAL(&cmdMux);
        setDomeCmd(se?14:15); server.send(200,"text/plain","ok");
    });
    server.on("/sound/play", [](){
        if(!checkToken()) return;

        if (!server.hasArg("track")) {
            server.send(400,"text/plain","Track ontbreekt");
            return;
        }

        int track = server.arg("track").toInt();
        if (track < 1 || track > 255) {
            server.send(400,"text/plain","Track moet 1-255 zijn");
            return;
        }

        bool se, available;
        portENTER_CRITICAL(&cmdMux);
        se = soundEnabled;
        available = dfplayerAvailable;
        portEXIT_CRITICAL(&cmdMux);

        if (!available) {
            server.send(503,"text/plain","DFPlayer niet beschikbaar");
            return;
        }
        if (!se) {
            server.send(409,"text/plain","Geluid staat UIT");
            return;
        }

        playSound(track);
        server.send(200,"text/plain","ok");
    });

    server.on("/sound/stop", [](){
        if(!checkToken()) return;
        stopSound();
        server.send(200,"text/plain","ok");
    });
    server.on("/volume/up", [](){if(!checkToken())return;setDomeCmd(16);server.send(200,"text/plain","ok");});
    server.on("/volume/down", [](){if(!checkToken())return;setDomeCmd(17);server.send(200,"text/plain","ok");});
    server.onNotFound([](){server.send(404,"text/plain","Niet gevonden");});
}

void setupOTA() {
    ArduinoOTA.setHostname("nsd-dalek");
    ArduinoOTA.setPasswordHash(OTA_PASSWORD_HASH);
    ArduinoOTA.onStart([](){DBGLN("OTA: Start");});
    ArduinoOTA.onEnd([](){DBGLN("OTA: Einde");});
    ArduinoOTA.onProgress([](unsigned int progress,unsigned int total){
        DBG("OTA: "); DBG(progress/(total/100)); DBGLN("%");
    });
    ArduinoOTA.onError([](ota_error_t error){DBG("OTA fout: ");DBGLN(error);});
    ArduinoOTA.begin();
    DBGLN("OTA klaar      : OK");
}

// =============================================================
//  SETUP  (Core 1)
// =============================================================
void setup() {
    Serial.begin(115200);
    DBGLN("\n\n========================================");
    DBG("Dalek firmware : "); DBGLN(FIRMWARE_VERSION);
    DBG("Reset-reden   : "); DBGLN(esp_reset_reason());

    FastLED.addLeds<LED_CHIPSET, PIN_LED_DATA, LED_COLOR_ORDER>(leds, NUM_LEDS)
           .setCorrection(Typical8mmPixel);
    FastLED.clear(true);

    Serial2.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
    bool dfOK=false;
    int retries=10;
    while(!dfOK && retries-- > 0){
        if(mp3.begin(Serial2)) dfOK=true;
        else {DBGLN("DFPlayer not ready, retrying...");delay(250);}
    }
    portENTER_CRITICAL(&cmdMux); dfplayerAvailable=dfOK; if(!dfOK) soundEnabled=false; portEXIT_CRITICAL(&cmdMux);
    if(dfOK){mp3.volume(DEFAULT_VOLUME);DBGLN("DFPlayer      : OK");}
    else DBGLN("DFPlayer      : NIET GEVONDEN — geluid uitgeschakeld");

    // Ultrasonic pin mapping: GPIO32 trigger, GPIO34 right PWM,
    // GPIO35 center PWM, GPIO33 left PWM.
    pinMode(PIN_SONIC_TRIGGER,OUTPUT);
    pinMode(PIN_SONIC_RIGHT,INPUT);
    pinMode(PIN_SONIC_CENTER,INPUT);
    pinMode(PIN_SONIC_LEFT,INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_SONIC_RIGHT), sensorRightISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SONIC_CENTER), sensorCenterISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SONIC_LEFT),  sensorLeftISR,  CHANGE);
    DBGLN("Ultrasonic sequential-chain interrupts attached");

    domeStateStart=millis(); domeState=DOME_BOOT;
    DBGLN("Boot animation started");
    while(updateDomeFSM()) { yield(); }
    DBGLN("Boot complete");
    playSound(SND_MOAN);

    DBGLN("----------------------------------------");
    DBG("WiFi SSID     : "); DBGLN(WIFI_SSID);
    DBGLN("Verbinden...");
    WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
    int wifiRetries=40;
    while(WiFi.status()!=WL_CONNECTED && wifiRetries-- > 0){delay(500);DBG(".");}
    DBGLN("");
    if(WiFi.status()==WL_CONNECTED){
        DBGLN("WiFi-status   : VERBONDEN");
        DBG("IP-adres      : "); DBGLN(WiFi.localIP());
        DBG("Gateway       : "); DBGLN(WiFi.gatewayIP());
        DBG("Signaal (RSSI): "); DBG(WiFi.RSSI()); DBGLN(" dBm");
        DBG("Kanaal        : "); DBGLN(WiFi.channel());
        DBG("MAC-adres     : "); DBGLN(WiFi.macAddress());
        setupOTA();
        otaStarted = true;
        lastWifiStatus = WL_CONNECTED;
    }else{
        DBGLN("WiFi-status   : FOUT — offline modus");
        lastWifiStatus = WiFi.status();
    }
    DBGLN("----------------------------------------");

    setupWebRoutes();
    server.begin();
    DBGLN("Web server started");

    xTaskCreatePinnedToCore(motorTask,"motorTask",8192,NULL,1,NULL,0);
    DBGLN("Motor task started on Core 0");
    startFadeEvent(CRGB::Blue);
}

// =============================================================
//  LOOP  (Core 1)  -  web server + dome FSM
// =============================================================
void loop() {
    server.handleClient();
    processDFPlayerEvents();
    if (otaStarted) ArduinoOTA.handle();
    esp_task_wdt_reset();

    static int prevDomeCmd=-1;
    static int boredCount=0;
    static unsigned long lastBored=millis();
    static unsigned long lastPulse=millis();
    static unsigned long lastHB=millis();

    // WiFi state machine: reconnect attempts never block the main loop.
    wl_status_t wifiStatus = WiFi.status();
    if (wifiStatus != lastWifiStatus) {
        lastWifiStatus = wifiStatus;
        DBG("[WiFi] status = "); DBGLN((int)wifiStatus);
        if (wifiStatus == WL_CONNECTED) {
            DBGLN("[WiFi] VERBONDEN");
            DBG("[WiFi] SSID    : "); DBGLN(WiFi.SSID());
            DBG("[WiFi] IP      : "); DBGLN(WiFi.localIP());
            DBG("[WiFi] Gateway : "); DBGLN(WiFi.gatewayIP());
            DBG("[WiFi] RSSI    : "); DBG(WiFi.RSSI()); DBGLN(" dBm");
            DBG("[WiFi] Channel : "); DBGLN(WiFi.channel());
            DBG("[WiFi] MAC     : "); DBGLN(WiFi.macAddress());
            if (!otaStarted) {
                setupOTA();
                otaStarted = true;
            }
        }
    }

    if (wifiStatus != WL_CONNECTED && millis() - lastWifiAttempt >= 10000UL) {
        lastWifiAttempt = millis();
        DBGLN("[WiFi] reconnect poging...");
        WiFi.reconnect();
    }

    if(millis()-lastHB>=30000UL){
        lastHB=millis();
        DBG("[HB] uptime=");DBG(millis()/1000);DBG("s heap=");DBG(ESP.getFreeHeap());
        DBGLN(wifiStatus==WL_CONNECTED?" wifi=OK":" wifi=DOWN");
    }

    processDomeCmd(prevDomeCmd,boredCount,lastBored,lastPulse);
    unsigned long now=millis();
    bool dm; portENTER_CRITICAL(&cmdMux); dm=displayMode; portEXIT_CRITICAL(&cmdMux);

    if(dm && domeState==DOME_IDLE && now-lastPulse>=PULSE_INTERVAL_MS){startPulse();lastPulse=now;}
    if(dm && domeState==DOME_IDLE && now-lastBored>=BORED_INTERVAL_MS){
        int boredVol; bool available;
        portENTER_CRITICAL(&cmdMux); boredVol=volume; available=dfplayerAvailable; portEXIT_CRITICAL(&cmdMux);
        if(available) mp3.volume(boredVol);
        if(boredCount<BORED_COUNT_MAX){playSound(SND_MOAN);boredCount++;}
        else{playSound(SND_REALLY_BORED);paletteIdx=0;paletteRound=0;domeState=DOME_BORED_PALETTE;boredCount=0;}
        lastBored=now;lastPulse=now;
    }
    delay(1);
}

