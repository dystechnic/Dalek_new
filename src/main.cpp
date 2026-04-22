#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FastLED.h>
#include <FastAccelStepper.h>
#include <DFRobotDFPlayerMini.h>
#include "config.h"

// WiFi credentials injected via secrets.ini build flags
#ifndef WIFI_SSID
  #define WIFI_SSID     "no_ssid_set"
  #define WIFI_PASSWORD "no_password_set"
#endif

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
        // Yield to keep watchdog happy without a fixed 1 ms penalty.
        taskYIELD();
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
            mp3.volume(30);
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

static String sDisplayMode   = "aan";
static String sMovementState = "uit";
static String sSoundState    = "aan";

const char HTML_HEADER[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html>
<head>
  <title>Dalek control station</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="10">
  <link rel="icon" href="data:,">
  <style>
    html { background-color:#B0C4DE; font-family:Helvetica;
           display:inline-block; margin:0 auto; text-align:center; }
    .button { background-color:#ff2d00; border:none; border-radius:12px;
              color:white; width:150px; padding:10px;
              font-size:30px; margin:2px; cursor:pointer; }
    .button2 { background-color:#08b203; }
    .round  { height:40px; width:55px; background-color:#eae0c2;
              font-size:30px; margin:2px; cursor:pointer;
              border-radius:12%; border:none; }
    .p1 { font-size:x-large; color:black; }
  </style>
</head>
<body>
<h1>!! Exterminate !!</h1>
)rawliteral";

void handleRoot() {
    String html = FPSTR(HTML_HEADER);

    html += "<p><b>DisplayMode = " + sDisplayMode + "</b></p>";
    if (sDisplayMode == "uit")
        html += "<p><a href='/display/on'><button class='button'>display</button></a></p>";
    else
        html += "<p><a href='/display/off'><button class='button button2'>display</button></a></p>";

    html += "<p><b>Motoren = " + sMovementState + "</b></p>";
    if (sMovementState == "uit")
        html += "<p><a href='/movement/on'><button class='button'>motoren</button></a></p>";
    else
        html += "<p><a href='/movement/off'><button class='button button2'>motoren</button></a></p>";

    html += "<p><b>Geluid = " + sSoundState + "</b></p>";
    if (sSoundState == "uit")
        html += "<p><a href='/sound/on'><button class='button'>geluid</button></a></p>";
    else
        html += "<p><a href='/sound/off'><button class='button button2'>geluid</button></a></p>";

    html += "<p><b>Volume</b></p>";
    html += "<p class='p1'><a href='/volume/up'><button class='round'>+</button></a>";
    html += "&nbsp;" + String(volume) + "&nbsp;";
    html += "<a href='/volume/down'><button class='round'>-</button></a></p>";

    html += "<hr><p><small>R:" + String(rightCM) + "cm &nbsp; C:" + String(centerCM)
          + "cm &nbsp; L:" + String(leftCM) + "cm</small></p>";

    html += "</body></html>";
    server.send(200, "text/html", html);
}

void setupWebRoutes() {
    server.on("/", handleRoot);

    server.on("/display/on", [](){
        sDisplayMode = "aan"; sMovementState = "uit"; sSoundState = "aan";
        soundEnabled = true;  motorRunning = false;
        setMotorCmd(2); setDomeCmd(15);
        server.sendHeader("Location", "/"); server.send(303);
    });
    server.on("/display/off", [](){
        sDisplayMode = "uit"; sMovementState = "uit"; sSoundState = "uit";
        soundEnabled = false; motorRunning = false;
        setMotorCmd(2); setDomeCmd(14);
        server.sendHeader("Location", "/"); server.send(303);
    });
    server.on("/movement/on", [](){
        if (sDisplayMode == "uit") {
            sMovementState = "aan";
            motorRunning = true;
            setDomeCmd(19);
        }
        server.sendHeader("Location", "/"); server.send(303);
    });
    server.on("/movement/off", [](){
        sMovementState = "uit"; motorRunning = false;
        setMotorCmd(2); setDomeCmd(18);
        server.sendHeader("Location", "/"); server.send(303);
    });
    server.on("/sound/on",  [](){ sSoundState = "aan"; setDomeCmd(15); server.sendHeader("Location", "/"); server.send(303); });
    server.on("/sound/off", [](){ sSoundState = "uit"; setDomeCmd(14); server.sendHeader("Location", "/"); server.send(303); });
    server.on("/volume/up",   [](){ setDomeCmd(16); server.sendHeader("Location", "/"); server.send(303); });
    server.on("/volume/down", [](){ setDomeCmd(17); server.sendHeader("Location", "/"); server.send(303); });
}

// =============================================================
//  SETUP  (Core 1)
// =============================================================
void setup() {
    Serial.begin(115200);
    DBGLN("\n\n--- Dalek ESP32 booting ---");

    // FastLED
    FastLED.addLeds<LED_CHIPSET, PIN_LED_DATA, LED_COLOR_ORDER>(leds, NUM_LEDS)
           .setCorrection(Typical8mmPixel);
    FastLED.clear(true);

    // DFPlayer on hardware Serial2
    Serial2.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
    int retries = 10;
    while (!mp3.begin(Serial2) && retries-- > 0) {
        DBGLN("DFPlayer not ready, retrying...");
        delay(500);
    }
    mp3.volume(DEFAULT_VOLUME);

    // Sonic sensor pins
    pinMode(PIN_SONIC_TRIGGER, OUTPUT);
    pinMode(PIN_SONIC_RIGHT,   INPUT);
    pinMode(PIN_SONIC_CENTER,  INPUT);
    pinMode(PIN_SONIC_LEFT,    INPUT);

    // Boot animation (non-blocking FSM, starts immediately)
    domeStateStart = millis();
    domeState = DOME_BOOT;
    DBGLN("Boot animation started");

    // Spin in the boot animation until it finishes
    while (updateDomeFSM()) { /* yields via loop in FSM */ }
    DBGLN("Boot complete");
    mp3.volume(DEFAULT_VOLUME);
    playSound(SND_MOAN);

    // WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    DBG("Connecting to WiFi");
    int wifiRetries = 30;
    while (WiFi.status() != WL_CONNECTED && wifiRetries-- > 0) {
        delay(500); DBG(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        DBGLN("\nWiFi connected. IP:");
        DBGLN(WiFi.localIP());
    } else {
        DBGLN("\nWiFi FAILED - running offline");
    }

    setupWebRoutes();
    server.begin();
    DBGLN("Web server started");

    // Start motor/sensor task on Core 0
    xTaskCreatePinnedToCore(
        motorTask,   // function
        "motorTask", // name
        4096,        // stack bytes
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

    static int          prevDomeCmd = -1;
    static int          boredCount  = 0;
    static unsigned long lastBored  = millis();
    static unsigned long lastPulse  = millis();

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
        if (boredCount <= 2) {
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