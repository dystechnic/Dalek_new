// =============================================================================
//  main.cpp  —  Dalek ESP32 unified firmware
//
//  Overzicht
//  ─────────
//  Dit is de enige firmware voor de NSD-Dalek. Hij draait op één ESP32-
//  WROOM-32U (DevKitC V4) en vervangt het oude ontwerp met vier losse
//  Arduino's (ESP-01, Mega, Pro Mini, Nano).
//
//  Architectuur
//  ────────────
//    Core 0  (motorTask)   —  stappenmotors aansturen + sensoren uitlezen
//    Core 1  (loop/setup)  —  WiFi, webserver, LED, DFPlayer
//
//  Wat er nieuw/anders is t.o.v. de eerste versie:
//    1. FastAccelStepper gebruikt de ESP32 RMT-hardware voor step-pulsen;
//       de motortiming is interrupt-gestuurd en mist nooit een stap.
//    2. Non-blocking dome-events — doStayAway / doExterminate / doBored
//       gebruiken millis()-toestandsmachines i.p.v. delay(), waardoor de
//       webserver altijd responsief blijft.
//    3. Sensoren worden uitgelezen in een vast 500-ms tijdslot, zodat de
//       motorTask de rest van de tijd niks hoeft te doen.
//    4. WiFi en API-token staan in secrets.ini — nooit in de broncode.
//    5. Motor-omkeer-vlaggen in config.h — aanpassen zonder te solderen.
//    6. snprintf JSON i.p.v. String-concatenatie (geen heap-fragmentatie).
//    7. sensorAction() behandelt nu alle 8 combinaties van 3 sensoren
//       (was: miste 2-van-3-geblokkeerde gevallen).
//    8. Achteruit rijden is continu (was: 200 steps one-shot).
//    9. Task Watchdog correct geregistreerd voor motorTask.
//   10. API_TOKEN heeft geen fallback meer — compileert niet zonder.
//
//  Beveiliging
//  ───────────
//    - Elke HTTP-route (behalve / en /status) checkt de X-Token header.
//    - CORS-header verwijderd; de UI is same-origin (wordt door dezelfde
//      ESP32 geserveerd).
//    - 404-handler voor onbekende routes.
//    - secrets.ini staat in .gitignore — wordt nooit gecommit.
//
//  Belangrijk: dit is een fanproject voor een Dalek-robot. Als er iets
//  vlam vat, een Dalek op hol slaat of je kat geëxtermineerd wordt, ben
//  jij daar zelf verantwoordelijk voor. Wij zijn inmiddels door naar het
//  volgende project en aanvaarden geen enkele aansprakelijkheid.
//
//  Geen.
//
//  Goededag.
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

// WiFi-gegevens worden geïnjecteerd via secrets.ini build flags.
// Zonder deze defines compileert de firmware niet (zie #error in config.h).

// =============================================================================
//  GEDEELDE TOESTAND
//
//  Geschreven door Core 1 (web-handler of dome-logica), gelezen door Core 0
//  (motortask) en omgekeerd voor sensorwaarden. Beschermd door een FreeRTOS
//  spinlock (portMUX).
//
//  Er is geen shadow-state meer; de volatile variabelen hieronder zijn de
//  enige bron van waarheid. Web-handlers lezen/schrijven ze direct onder de
//  mutex.
// =============================================================================

portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

// Motor-commando's:
//   1 = vooruit      2 = stop       3 = linksaf
//   4 = rechtsaf     5 = achteruit  6 = beweging uit  7 = beweging aan
volatile int  motorCmd     = 2;
volatile bool motorRunning = false;

// Dome/display-commando's:
//   10 = normaal (blauw oog)  11 = stay away    12 = exterminate
//   14 = geluid uit           15 = geluid aan
//   16 = volume omhoog        17 = volume omlaag
//   18 = display aan          19 = display uit
volatile int  domeCmd = 10;

// Sensorwaarden (geschreven door Core 0, gelezen door Core 1 voor web)
volatile long rightCM = 999, centerCM = 999, leftCM = 999;

// Modus-vlaggen (geschreven door Core 1, gelezen door beide cores)
volatile bool displayMode  = true;
volatile bool soundEnabled = true;
volatile int  volume       = DEFAULT_VOLUME;

// =============================================================================
//  HARDWARE-OBJECTEN
// =============================================================================

// FastAccelStepper — gebruikt RMT-peripheral; geen handmatige run() nodig.
FastAccelStepperEngine stepperEngine = FastAccelStepperEngine();
FastAccelStepper* leftStepper  = nullptr;
FastAccelStepper* rightStepper = nullptr;

// DFPlayer op hardware Serial2
DFRobotDFPlayerMini mp3;

// FastLED
CRGB leds[NUM_LEDS];

// Webserver op poort 80
WebServer server(80);

// Kleurenpalet voor de "echt verveeld"-animatie
static const CRGB palette[] = {
    CRGB::Khaki, CRGB::Aqua, CRGB::DarkMagenta, CRGB::DarkSeaGreen,
    CRGB::Amethyst, CRGB::RosyBrown, CRGB::OrangeRed, CRGB::Yellow,
    CRGB::LightCoral, CRGB::OldLace
};
static const int PALETTE_SIZE = sizeof(palette) / sizeof(palette[0]);

// =============================================================================
//  VEIlige commando-setters  (aanroepbaar vanuit elke core)
// =============================================================================

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

// =============================================================================
//  BEVEILIGING  —  tokencontrole
// =============================================================================

// Geeft true terug als het verzoek een geldige API-token in de X-Token header
// heeft. Bij ongeldige token stuurt hij 403 en returnt false.
bool checkToken() {
    if (server.hasHeader("X-Token") &&
        server.header("X-Token") == String(API_TOKEN)) {
        return true;
    }
    server.send(403, "text/plain", "Forbidden");
    return false;
}

// =============================================================================
//  CORE 0  —  sensoren + motoren  (motorTask)
//
//  FastAccelStepper genereert de step-pulsen via de RMT-hardware op de
//  achtergrond, dus deze taak hoeft alleen move/stop aan te roepen als het
//  commando verandert. Sensor-uitlezing gebeurt elke 500 ms; de rest van de
//  tijd staat de taak onmiddellijk weer zijn tijd af.
// =============================================================================

// ── Sensor-uitlezing ─────────────────────────────────────────────────────────
//  Stuurt een trigger-puls naar alle drie Maxbotix EZ1-sensoren (daisy-chain)
//  en leest de PWM-uitgangen uit met pulseIn().
void readSensors() {
    digitalWrite(PIN_SONIC_TRIGGER, HIGH);
    delayMicroseconds(25);
    digitalWrite(PIN_SONIC_TRIGGER, LOW);

    long rp = pulseIn(PIN_SONIC_RIGHT,  HIGH, SONIC_PULSE_TIMEOUT_US);
    long cp = pulseIn(PIN_SONIC_CENTER, HIGH, SONIC_PULSE_TIMEOUT_US);
    long lp = pulseIn(PIN_SONIC_LEFT,   HIGH, SONIC_PULSE_TIMEOUT_US);

    // Geluidssnelheid ≈ 29 µs/cm enkele reis; pulseIn meet heen+terug, dus /2.
    rightCM  = (rp > 0) ? rp / 29 / 2 : SONIC_MAX_CM;
    centerCM = (cp > 0) ? cp / 29 / 2 : SONIC_MAX_CM;
    leftCM   = (lp > 0) ? lp / 29 / 2 : SONIC_MAX_CM;
}

// ── Obstakel-navigatie ───────────────────────────────────────────────────────
//  Beslist op basis van de 3 sensorwaarden wat de Dalek moet doen.
//  Behandelt nu expliciet alle 8 mogelijke combinaties van
//  (rechts, midden, links) × (geblokkeerd/vrij).
//
//  Logica:
//    0/3 geblokkeerd  →  vooruit (alles vrij)
//    3/3 geblokkeerd  →  stop (volledig ingesloten)
//    1 sensor         →  draai weg van de geblokkeerde kant
//    2 sensoren       →  achteruit (uit de benauwing)
void sensorAction() {
    long r = rightCM, c = centerCM, l = leftCM;
    static bool midTriggered = false;
    static bool minTriggered = false;
    bool mr;

    portENTER_CRITICAL(&cmdMux);
    mr = motorRunning;
    portEXIT_CRITICAL(&cmdMux);
    if (!mr) return;

    bool rB = (r <= SONIC_MIN_CM);
    bool cB = (c <= SONIC_MIN_CM);
    bool lB = (l <= SONIC_MIN_CM);

    // --- Bewegingsbeslissingen: alle 8 combinaties ---------------------------
    if (!rB && !cB && !lB) {
        // 000 — alles vrij → vooruit
        setMotorCmd(1);
        setDomeCmd(10);
        midTriggered = false;
        minTriggered = false;
    } else if (rB && cB && lB) {
        // 111 — volledig ingesloten → stop
        setMotorCmd(2);
    } else if (rB && !cB && !lB) {
        // 100 — alleen rechts geblokkeerd → linksaf
        setMotorCmd(3);
    } else if (!rB && !cB && lB) {
        // 001 — alleen links geblokkeerd → rechtsaf
        setMotorCmd(4);
    } else if (!rB && cB && !lB) {
        // 010 — alleen midden geblokkeerd → achteruit
        setMotorCmd(5);
    } else if (rB && cB && !lB) {
        // 110 — rechts + midden → achteruit
        setMotorCmd(5);
    } else if (!rB && cB && lB) {
        // 011 — midden + links → achteruit
        setMotorCmd(5);
    } else if (rB && !cB && lB) {
        // 101 — links + rechts (maar midden vrij) → achteruit
        setMotorCmd(5);
    }

    // --- Alarm-niveaus (eenmalig per nadering) --------------------------------
    if ((r <= SONIC_MID_CM || c <= SONIC_MID_CM || l <= SONIC_MID_CM) && !midTriggered) {
        setDomeCmd(11);   // "Stay Away!"
        midTriggered = true;
    }
    if ((r <= SONIC_MIN_CM || c <= SONIC_MIN_CM || l <= SONIC_MIN_CM) && !minTriggered) {
        setDomeCmd(12);   // "Exterminate!"
        minTriggered = true;
    }
}

// ── Motorcommando uitvoeren ──────────────────────────────────────────────────
//  Vertaalt motorCmd naar FastAccelStepper-aanroepen.
//  Slaat over als het commando niet veranderd is (voorkomt geratel).
//
//  Opgelet: case 5 (achteruit) is nu CONTINU, niet langer een one-shot
//  van 200 steps. De Dalek blijft achteruit rijden tot de sensoren weer
//  vrij zijn of een ander commando wordt gegeven.
void applyMotorCmd() {
    static int prevAppliedCmd = -1;
    int cmd;
    portENTER_CRITICAL(&cmdMux);
    cmd = motorCmd;
    portEXIT_CRITICAL(&cmdMux);

    if (cmd == prevAppliedCmd) return;
    prevAppliedCmd = cmd;

    switch (cmd) {
        case 1:  // vooruit
            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED);
            leftStepper->runForward();
            rightStepper->runForward();
            break;

        case 2:  // volledige stop
            leftStepper->stopMove();
            rightStepper->stopMove();
            break;

        case 3:  // linksaf (links langzaam, rechts snel)
            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED / MOTOR_TURN_SLOW_DIV);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED - 500);
            leftStepper->runForward();
            rightStepper->runForward();
            break;

        case 4:  // rechtsaf (links snel, rechts langzaam)
            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED - 500);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED / MOTOR_TURN_SLOW_DIV);
            leftStepper->runForward();
            rightStepper->runForward();
            break;

        case 5:  // achteruit (continu — blijft rijden tot stop-commando)
            leftStepper->setSpeedInHz(MOTOR_MAX_SPEED / 2);
            rightStepper->setSpeedInHz(MOTOR_MAX_SPEED / 2);
            leftStepper->runBackward();
            rightStepper->runBackward();
            break;

        default:
            break;
    }
}

// ── motorTask (Core 0) ──────────────────────────────────────────────────────
//  Oneindige lus die om de 500 ms de sensoren uitleest en de motoren
//  aanstuurt. De FastAccelStepper regelt de RMT-pulsen op de achtergrond.
void motorTask(void* pvParameters) {
    // Task Watchdog inschakelen voor deze taak. Zonder registratie doet
    // esp_task_wdt_reset() niets en kan de WDT onverwacht afgaan.
    esp_task_wdt_add(NULL);

    stepperEngine.init();

    leftStepper  = stepperEngine.stepperConnectToPin(PIN_LEFT_STEP);
    rightStepper = stepperEngine.stepperConnectToPin(PIN_RIGHT_STEP);

    if (leftStepper) {
        leftStepper->setDirectionPin(PIN_LEFT_DIR,  INVERT_LEFT_MOTOR);
        leftStepper->setAcceleration(MOTOR_ACCEL);
        leftStepper->setSpeedInHz(MOTOR_MAX_SPEED);
    } else {
        DBGLN("Linker motor   : FOUT — controleer PIN_LEFT_STEP");
    }
    if (rightStepper) {
        rightStepper->setDirectionPin(PIN_RIGHT_DIR, INVERT_RIGHT_MOTOR);
        rightStepper->setAcceleration(MOTOR_ACCEL);
        rightStepper->setSpeedInHz(MOTOR_MAX_SPEED);
    } else {
        DBGLN("Rechter motor  : FOUT — controleer PIN_RIGHT_STEP");
    }

    static unsigned long lastSensorRead = 0;

    for (;;) {
        unsigned long now = millis();

        // Elke 500 ms sensoren uitlezen + navigatie
        if (now - lastSensorRead >= 500) {
            readSensors();
            sensorAction();
            lastSensorRead = now;
        }

        // Check of beweging ingeschakeld is
        bool mr;
        portENTER_CRITICAL(&cmdMux);
        mr = motorRunning;
        portEXIT_CRITICAL(&cmdMux);

        if (mr) {
            applyMotorCmd();
        } else {
            // Geforceerd stoppen als beweging uit staat
            if (leftStepper  && leftStepper->isRunning())  leftStepper->stopMove();
            if (rightStepper && rightStepper->isRunning()) rightStepper->stopMove();
        }

        // FastAccelStepper regelt de pulsen via RMT-interrupt — geen run()
        // nodig. vTaskDelay(1) geeft de IDLE-taak op Core 0 een gegarandeerde
        // beurt, wat de WDT-reset voor de idle-taak mogelijk maakt.
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =============================================================================
//  NON-BLOCKING DOME-STATUSMACHINE  (Core 1)
//
//  Elk "event" (stay away, exterminate, bored, boot, pulse) is gemodelleerd
//  als een kleine toestandsmachine, zodat loop() nooit blokkeert en de
//  webserver altijd responsief blijft.
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

static DomeState       domeState       = DOME_BOOT;
static int             fadeBrightness  = 0;
static unsigned long   domeStateStart  = 0;
static int             paletteIdx      = 0;
static int             paletteRound    = 0;

// ── Statusmachine-stap ───────────────────────────────────────────────────────
//  Roept één iteratie van de actieve animatie aan. Returnt true zolang de
//  animatie bezig is (blokkeert nieuwe events).
bool updateDomeFSM() {
    unsigned long now = millis();

    // Maximaal ~60 stappen/seconde zodat loop() tijd overhoudt voor de server
    static unsigned long lastFadeStep = 0;
    bool fadeReady = (now - lastFadeStep >= 16);

    switch (domeState) {

        case DOME_IDLE:
            return false;

        // ── Fade-up (stay away / exterminate / blauw oog) ───────────────────
        case DOME_FADE_UP:
            if (!fadeReady) return true;
            lastFadeStep = now;
            fadeBrightness += 3;
            if (fadeBrightness >= 255) {
                fadeBrightness = 255;
                domeState = DOME_HOLD;
                domeStateStart = now;
            }
            FastLED.setBrightness(fadeBrightness);
            FastLED.show();
            return true;

        // ── Vast houden (2 seconden) ─────────────────────────────────────────
        case DOME_HOLD:
            if (now - domeStateStart >= 2000) {
                domeState = DOME_FADE_DOWN;
            }
            return true;

        // ── Fade-down → IDLE ─────────────────────────────────────────────────
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

        // ── Oogpuls (dim → fel) ──────────────────────────────────────────────
        case DOME_PULSE_DOWN:
            if (!fadeReady) return true;
            lastFadeStep = now;
            fadeBrightness -= 3;
            if (fadeBrightness <= 20) {
                fadeBrightness = 20;
                domeState = DOME_PULSE_UP;
            }
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

        // ── Opstartanimatie (rood-wit-geel knipperen) ────────────────────────
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

        // ── "Verveeld"-kleurencirkel ─────────────────────────────────────────
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

// ── Helpers: animatie starten ───────────────────────────────────────────────
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
    bool se;
    portENTER_CRITICAL(&cmdMux);
    se = soundEnabled;
    portEXIT_CRITICAL(&cmdMux);
    if (se) mp3.playFolder(SND_FOLDER, track);
}

// =============================================================================
//  DOME-COMMANDOVERWERKER  (Core 1, aangeroepen vanuit loop)
// =============================================================================

void processDomeCmd(int& prevCmd, int& boredCount,
                    unsigned long& lastBored, unsigned long& lastPulse)
{
    // Niet onderbreken zolang er een animatie loopt (behalve volume)
    bool busy = updateDomeFSM();

    int cmd;
    portENTER_CRITICAL(&cmdMux);
    cmd = domeCmd;
    portEXIT_CRITICAL(&cmdMux);

    // Volume wordt altijd direct afgehandeld, ongeacht de animatie-status
    if (cmd == 16) {
        portENTER_CRITICAL(&cmdMux);
        if (volume < 30) { volume++; mp3.volumeUp(); }
        portEXIT_CRITICAL(&cmdMux);
        DBGLN("Volume OMHOOG");
        setDomeCmd(prevCmd);
        return;
    }
    if (cmd == 17) {
        portENTER_CRITICAL(&cmdMux);
        if (volume > 0) { volume--; mp3.volumeDown(); }
        portEXIT_CRITICAL(&cmdMux);
        DBGLN("Volume OMLAAG");
        setDomeCmd(prevCmd);
        return;
    }

    if (busy || cmd == prevCmd) return;

    switch (cmd) {
        case 10:
            DBGLN("Normaal — blauw oog");
            startFadeEvent(CRGB::Blue);
            prevCmd = cmd;
            break;

        case 11: {  // stay away
            portENTER_CRITICAL(&cmdMux);
            bool dm = displayMode;
            int  vol = volume;
            portEXIT_CRITICAL(&cmdMux);
            if (dm) {
                DBGLN("Stay Away!!");
                mp3.volume(vol);
                startFadeEvent(CRGB::White);
                playSound(SND_STAY_AWAY);
                lastBored = millis();
                setDomeCmd(10);
                prevCmd = 10;
            }
            break;
        }

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
            portENTER_CRITICAL(&cmdMux);
            soundEnabled = false;
            portEXIT_CRITICAL(&cmdMux);
            DBGLN("Geluid UIT");
            prevCmd = cmd;
            break;

        case 15:
            portENTER_CRITICAL(&cmdMux);
            soundEnabled = true;
            portEXIT_CRITICAL(&cmdMux);
            DBGLN("Geluid AAN");
            prevCmd = cmd;
            break;

        case 18:
            portENTER_CRITICAL(&cmdMux);
            displayMode = true;
            portEXIT_CRITICAL(&cmdMux);
            DBGLN("Display AAN");
            prevCmd = cmd;
            break;

        case 19:
            portENTER_CRITICAL(&cmdMux);
            displayMode = false;
            portEXIT_CRITICAL(&cmdMux);
            DBGLN("Display UIT");
            prevCmd = cmd;
            break;

        default:
            break;
    }
}

// =============================================================================
//  WEBSERVER  (Core 1)
// =============================================================================

// ── Status-endpoint (JSON) ───────────────────────────────────────────────────
//  Wordt elke 3 seconden door de webpagina gepolled via fetch().
//  Gebruikt snprintf i.p.v. String-concatenatie om heap-fragmentatie te
//  voorkomen.
void handleStatus() {
    char json[256];
    bool dm, mr, se;
    int  vol;
    long rc, cc, lc;

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

    server.send(200, "application/json", json);
}

// ── HTML root (uit LittleFS) ──────────────────────────────────────────────────
//  Upload het bestandssysteem met: pio run --target uploadfs
void handleRoot() {
    if (LittleFS.exists("/index.html")) {
        File f = LittleFS.open("/index.html", "r");
        server.streamFile(f, "text/html");
        f.close();
    } else {
        server.send(503, "text/plain",
            "Filesystem niet gevonden. Voer uit: pio run --target uploadfs");
    }
}

// ── Routes registreren ───────────────────────────────────────────────────────
void setupWebRoutes() {
    // Root — geen token nodig (browsernavigatie)
    server.on("/", handleRoot);

    // Status is read-only telemetrie — geen token nodig
    server.on("/status", handleStatus);

    // Alle schrijvende routes hebben X-Token nodig
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

    // Catch-all 404
    server.onNotFound([](){
        server.send(404, "text/plain", "Niet gevonden");
    });
}

// =============================================================================
//  OTA  (Over-The-Air updates)
// =============================================================================

void setupOTA() {
    ArduinoOTA.setHostname("nsd-dalek");
    ArduinoOTA.setPasswordHash(OTA_PASSWORD_HASH);

    ArduinoOTA.onStart([]() {
        DBGLN("OTA: Start");
    });
    ArduinoOTA.onEnd([]() {
        DBGLN("OTA: Einde");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        DBG("OTA: Voortgang "); DBG(progress / (total / 100)); DBGLN("%");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        DBG("OTA: Fout "); DBGLN(error);
    });

    ArduinoOTA.begin();
    DBGLN("OTA klaar      : OK");
}

// =============================================================================
//  SETUP  (Core 1)
// =============================================================================

void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    #endif
    DBGLN("\n\n--- Dalek ESP32 opstarten ---");

    // ── Reset-reden (helpt bij debuggen van onverwachte reboots) ─────────
    DBG("Reset-reden   : ");
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   DBGLN("Power on");               break;
        case ESP_RST_SW:        DBGLN("Software-reset");         break;
        case ESP_RST_PANIC:     DBGLN("*** PANIEK/CRASH ***");   break;
        case ESP_RST_INT_WDT:   DBGLN("*** INT WATCHDOG ***");   break;
        case ESP_RST_TASK_WDT:  DBGLN("*** TAAK WATCHDOG ***");  break;
        case ESP_RST_WDT:       DBGLN("*** WATCHDOG ***");       break;
        case ESP_RST_BROWNOUT:  DBGLN("*** BROWNOUT ***");       break;
        case ESP_RST_SDIO:      DBGLN("SDIO-reset");             break;
        default:                DBGLN("Onbekend");               break;
    }

    // ── LittleFS ──────────────────────────────────────────────────────────
    if (LittleFS.begin(true)) {
        DBGLN("LittleFS      : OK");
    } else {
        DBGLN("LittleFS      : FOUT");
    }

    // ── FastLED ───────────────────────────────────────────────────────────
    FastLED.addLeds<LED_CHIPSET, PIN_LED_DATA, LED_COLOR_ORDER>(leds, NUM_LEDS)
           .setCorrection(Typical8mmPixel);
    FastLED.clear(true);

    // ── WiFi (vóór trage initialisaties zodat output snel zichtbaar is) ───
    DBGLN("----------------------------------------");
    DBG("WiFi SSID     : "); DBGLN(WIFI_SSID);
    DBGLN("Verbinden...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int wifiRetries = 40;
    while (WiFi.status() != WL_CONNECTED && wifiRetries-- > 0) {
        delay(500); DBG(".");
    }
    DBGLN("");
    if (WiFi.status() == WL_CONNECTED) {
        DBGLN("WiFi-status   : VERBONDEN");
        DBG("IP-adres      : "); DBGLN(WiFi.localIP());
        DBG("Gateway       : "); DBGLN(WiFi.gatewayIP());
        DBG("Signaal (RSSI): "); DBG(WiFi.RSSI()); DBGLN(" dBm");
        DBG("Kanaal        : "); DBGLN(WiFi.channel());
        DBG("MAC-adres     : "); DBGLN(WiFi.macAddress());
        setupOTA();
    } else {
        DBGLN("WiFi-status   : FOUT — offline modus");
        DBG("Statuscode    : ");
        switch (WiFi.status()) {
            case WL_NO_SSID_AVAIL:  DBGLN("WL_NO_SSID_AVAIL (SSID niet gevonden)");  break;
            case WL_CONNECT_FAILED: DBGLN("WL_CONNECT_FAILED (verkeerd wachtwoord?)"); break;
            case WL_DISCONNECTED:   DBGLN("WL_DISCONNECTED (timeout)");                break;
            default: DBGLN(WiFi.status()); break;
        }
    }
    DBGLN("----------------------------------------");

    // ── DFPlayer (niet-fataal als deze ontbreekt) ─────────────────────────
    bool dfplayerOK = false;
    Serial2.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
    unsigned long dfTimeout = millis() + 2500UL;
    while (!dfplayerOK && millis() < dfTimeout) {
        if (mp3.begin(Serial2)) {
            dfplayerOK = true;
        } else {
            DBGLN("DFPlayer niet gereed, opnieuw proberen...");
            delay(200);
        }
    }
    if (dfplayerOK) {
        mp3.volume(DEFAULT_VOLUME);
        DBGLN("DFPlayer      : OK");
    } else {
        DBGLN("DFPlayer      : NIET GEVONDEN — geluid uitgeschakeld");
        portENTER_CRITICAL(&cmdMux);
        soundEnabled = false;
        portEXIT_CRITICAL(&cmdMux);
    }

    // ── Sensor-pinnen ─────────────────────────────────────────────────────
    pinMode(PIN_SONIC_TRIGGER, OUTPUT);
    pinMode(PIN_SONIC_RIGHT,   INPUT);
    pinMode(PIN_SONIC_CENTER,  INPUT);
    pinMode(PIN_SONIC_LEFT,    INPUT);

    // ── Opstartanimatie (non-blocking FSM) ────────────────────────────────
    domeStateStart = millis();
    domeState = DOME_BOOT;
    DBGLN("Opstartanimatie gestart");

    while (updateDomeFSM()) { yield(); }
    DBGLN("Opstarten voltooid");
    playSound(SND_MOAN);

    // ── Webserver ──────────────────────────────────────────────────────────
    setupWebRoutes();
    server.begin();
    DBGLN("Webserver gestart");

    // ── Motor/sensor-taak op Core 0 starten ────────────────────────────────
    // Stack op 8192 bytes: sensor-uitlezing + FastAccelStepper + debug
    // kunnen de standaard 4096-byte stack in het nauw drijven.
    xTaskCreatePinnedToCore(
        motorTask,
        "motorTask",
        8192,
        NULL,
        1,
        NULL,
        0
    );
    DBGLN("Motortaak gestart op Core 0");

    // Oog op blauw (Dalek is klaar)
    startFadeEvent(CRGB::Blue);
}

// =============================================================================
//  LOOP  (Core 1)  —  webserver + dome-statusmachine
// =============================================================================

void loop() {
    server.handleClient();
    ArduinoOTA.handle();
    esp_task_wdt_reset();

    static int           prevDomeCmd = -1;
    static int           boredCount  = 0;
    static unsigned long lastBored   = millis();
    static unsigned long lastPulse   = millis();
    static unsigned long lastHB      = millis();

    // ── Heartbeat elke 30 seconden ───────────────────────────────────────────
    if (millis() - lastHB >= 30000UL) {
        lastHB = millis();
        DBG("[HB] uptime="); DBG(millis()/1000);
        DBG("s  heap=");     DBG(ESP.getFreeHeap());
        DBG("  wifi=");      DBGLN(WiFi.status() == WL_CONNECTED ? "OK" : "DOWN");

        // Robuuste herverbinding: eerst disconnecten, dan opnieuw beginnen.
        // WiFi.reconnect() kan stilletjes falen na een langere onderbreking.
        if (WiFi.status() != WL_CONNECTED) {
            DBGLN("[HB] WiFi verloren — opnieuw verbinden...");
            WiFi.disconnect();
            delay(100);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
    }

    // ── Dome-commando verwerken ──────────────────────────────────────────────
    processDomeCmd(prevDomeCmd, boredCount, lastBored, lastPulse);

    unsigned long now = millis();

    bool dm;
    portENTER_CRITICAL(&cmdMux);
    dm = displayMode;
    portEXIT_CRITICAL(&cmdMux);

    // ── Oogpuls elke 10 seconden (alleen in display-modus, alleen als idle) ──
    if (dm && domeState == DOME_IDLE && (now - lastPulse >= PULSE_INTERVAL_MS)) {
        startPulse();
        lastPulse = now;
    }

    // ── Verveeld-timer ───────────────────────────────────────────────────────
    if (dm && domeState == DOME_IDLE && (now - lastBored >= BORED_INTERVAL_MS)) {
        DBGLN("Verveeld!");
        portENTER_CRITICAL(&cmdMux);
        int boredVol = volume;
        portEXIT_CRITICAL(&cmdMux);
        mp3.volume(boredVol);
        if (boredCount < BORED_COUNT_MAX) {
            playSound(SND_MOAN);
            boredCount++;
        } else {
            DBGLN("Echt verveeld!");
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