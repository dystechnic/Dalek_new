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
//    3. Sensor reads moved to a short sub-task window  - pulseIn
//       calls are still sequential (hardware constraint of the
//       daisy-chain) but happen in a timed 500 ms slot so the
//       stepper task loop is free the rest of the time.
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

static constexpr const char* FIRMWARE_VERSION = "V0.52";

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

struct SensorFilter {
    long samples[3] = {SONIC_MAX_CM, SONIC_MAX_CM, SONIC_MAX_CM};
    uint8_t count = 0;
    uint8_t index = 0;
    long lastValid = SONIC_MAX_CM;
};

static SensorFilter filterRight, filterCenter, filterLeft;

static long median3(long a, long b, long c) {
    if (a > b) { long t=a; a=b; b=t; }
    if (b > c) { long t=b; b=c; c=t; }
    if (a > b) { long t=a; a=b; b=t; }
    return b;
}

static long filterSensor(SensorFilter& f, long pulse) {
    if (pulse <= 0) return f.lastValid;
    long cm = pulse / 58;
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

void readSensors() {
    digitalWrite(PIN_SONIC_TRIGGER, HIGH);
    delayMicroseconds(25);
    digitalWrite(PIN_SONIC_TRIGGER, LOW);

    long rp = pulseIn(PIN_SONIC_RIGHT,  HIGH, SONIC_PULSE_TIMEOUT_US);
    long cp = pulseIn(PIN_SONIC_CENTER, HIGH, SONIC_PULSE_TIMEOUT_US);
    long lp = pulseIn(PIN_SONIC_LEFT,   HIGH, SONIC_PULSE_TIMEOUT_US);

    long r = filterSensor(filterRight, rp);
    long c = filterSensor(filterCenter, cp);
    long l = filterSensor(filterLeft, lp);

    portENTER_CRITICAL(&cmdMux);
    rightCM = r;
    centerCM = c;
    leftCM = l;
    portEXIT_CRITICAL(&cmdMux);
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
    if (cmd == prevAppliedCmd) return;
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

    static unsigned long lastSensorRead = 0;

    for (;;) {
        unsigned long now = millis();

        if (now - lastSensorRead >= 500) {
            readSensors();
            sensorAction();
            lastSensorRead = now;
        }

        bool mr;
        portENTER_CRITICAL(&cmdMux);
        mr = motorRunning;
        portEXIT_CRITICAL(&cmdMux);
        if (mr) {
            applyMotorCmd();
        } else {
            // Ensure stopped when movement is disabled
            if (leftStepper  && leftStepper->isRunning())  leftStepper->stopMove();
            if (rightStepper && rightStepper->isRunning()) rightStepper->stopMove();
            // Force the stop command into the driver after movement was disabled.
            setMotorCmd(2);
            applyMotorCmd(true);
        }

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
    if (se && available) mp3.playFolder(SND_FOLDER, track);
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

void handleRoot() {
    const char html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>DALEK COMMAND</title><style>
body{background:#050a07;color:#c8ffe8;font-family:monospace;text-align:center;margin:20px}
button{font-size:22px;padding:14px;margin:5px;min-width:150px;background:#0a1410;color:#c8ffe8;border:1px solid #1a3a28}
.on{border-color:#00ff88}.off{border-color:#ff2200}.sensor{font-size:24px;margin:10px}
</style></head><body><h1>!! EXTERMINATE !!</h1>
<div class="sensor">R: <span id="r">---</span> cm | C: <span id="c">---</span> cm | L: <span id="l">---</span> cm</div>
<div><button onclick="cmd('/display/toggle')">DISPLAY</button><button onclick="cmd('/movement/toggle')">MOTOREN</button></div>
<div><button onclick="cmd('/sound/toggle')">GELUID</button></div>
<div><button onclick="cmd('/volume/down')">− VOLUME</button><span id="v">--</span><button onclick="cmd('/volume/up')">+ VOLUME</button></div>
<p id="status">LIVE</p><script>
const token='%%API_TOKEN%%';
async function cmd(u){try{await fetch(u,{headers:{'X-Token':token}});poll()}catch(e){document.getElementById('status').textContent='OFFLINE'}}
async function poll(){try{let r=await fetch('/status');let d=await r.json();
for(let x of [['r',d.right],['c',d.center],['l',d.left]])document.getElementById(x[0]).textContent=x[1]>=300?'---':x[1];
document.getElementById('v').textContent=d.volume;document.getElementById('status').textContent='LIVE | '+d.version+' | '+(d.motors?'RIJDEND':'GESTOPT');
}catch(e){document.getElementById('status').textContent='OFFLINE'}}poll();setInterval(poll,3000);
</script></body></html>)rawliteral";
    String page = FPSTR(html);
    page.replace("%%API_TOKEN%%", API_TOKEN);
    server.send(200,"text/html",page);
}

void handleStatus() {
    bool dm,mr,se,df;
    int vol;
    long r,c,l;
    portENTER_CRITICAL(&cmdMux);
    dm=displayMode; mr=motorRunning; se=soundEnabled; vol=volume;
    r=rightCM; c=centerCM; l=leftCM; df=dfplayerAvailable;
    portEXIT_CRITICAL(&cmdMux);
    char json[512];
    bool wifi = (WiFi.status() == WL_CONNECTED);
    String ip = wifi ? WiFi.localIP().toString() : String();
    snprintf(json,sizeof(json),
        "{\"version\":\"%s\",\"display\":%s,\"motors\":%s,\"sound\":%s,\"dfplayer\":%s,\"volume\":%d,\"right\":%ld,\"center\":%ld,\"left\":%ld,\"wifi\":%s,\"ip\":\"%s\",\"rssi\":%d,\"channel\":%d,\"uptime\":%lu}",
        FIRMWARE_VERSION,dm?"true":"false",mr?"true":"false",se?"true":"false",df?"true":"false",
        vol,r,c,l,wifi?"true":"false",ip.c_str(),wifi?(int)WiFi.RSSI():0,wifi?(int)WiFi.channel():0,millis()/1000UL);
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
        if(mr) setMotorCmd(2); else setDomeCmd(19);
        server.send(200,"text/plain","ok");
    });
    server.on("/movement/on", [](){
        if(!checkToken()) return;
        portENTER_CRITICAL(&cmdMux); motorRunning=true; portEXIT_CRITICAL(&cmdMux);
        setDomeCmd(19); server.send(200,"text/plain","ok");
    });
    server.on("/movement/off", [](){
        if(!checkToken()) return;
        portENTER_CRITICAL(&cmdMux); motorRunning=false; portEXIT_CRITICAL(&cmdMux);
        setMotorCmd(2); setDomeCmd(18); server.send(200,"text/plain","ok");
    });
    server.on("/sound/toggle", [](){
        if(!checkToken()) return;
        bool se; portENTER_CRITICAL(&cmdMux); se=soundEnabled; portEXIT_CRITICAL(&cmdMux);
        portENTER_CRITICAL(&cmdMux); soundEnabled=!se; portEXIT_CRITICAL(&cmdMux);
        setDomeCmd(se?14:15); server.send(200,"text/plain","ok");
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

    pinMode(PIN_SONIC_TRIGGER,OUTPUT);
    pinMode(PIN_SONIC_RIGHT,INPUT);
    pinMode(PIN_SONIC_CENTER,INPUT);
    pinMode(PIN_SONIC_LEFT,INPUT);

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
    ArduinoOTA.handle();
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

