# Dalek-V2.0

 =============================================================
  main.cpp  -  Dalek ESP32 unified firmware

  Consolidates previous version with four separate Arduinos into one ESP32:
    - dalek_WiFi.ino   (ESP-01)    -> WiFiServer + web UI
    - dalek_main.ino   (Mega)      -> Ultrasonic sensors
    - dalek_dome.ino   (Pro Mini)  -> FastLED + DFPlayer
    - dalek_motors.ino (Nano)      -> stepper motors

  Architecture:
    Core 0 (motorTask)  - stepper run loop + sensor polling
    Core 1 (Arduino)    - WiFi server + LED + sound (setup/loop)

  Optimisations vs first version:
    1. FastAccelStepper  - uses ESP32 RMT hardware peripheral for
       step pulses; stepper timing is interrupt-driven and never
       misses a step regardless of what else is running.
    2. Non-blocking dome events  - doStayAway / doExterminate /
       doBored use millis() state machines instead of delay(),
       so the web server stays responsive during sound/light events.
    3. Sensor reads moved to a short sub-task window  - pulseIn
       calls are still sequential (hardware constraint of the
       daisy-chain) but happen in a timed 500 ms slot so the
       stepper task loop is free the rest of the time.
    4. WiFi credentials in secrets.ini  - never in source code.
    5. Motor direction invert flags  - configurable in config.h
       without rewiring.
 =============================================================