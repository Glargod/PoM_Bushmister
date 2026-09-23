# PoM (Piss-o-Matic) — Maritimes Bushmister

ESP32 + DHT11 + LDR + button-toggle mister.

## Boot OFF fix

`setup()` drives the toggle pin HIGH immediately, then reads `STATUS_PIN` and pulses until the mister reports OFF (up to 6 tries). Same check runs again after the auto sequence and right before deep sleep.

GPIO5 is a strapping pin and can glitch LOW at reset. If it still wakes ON after this firmware, move the mister wire from D5 to **GPIO18** and change:

```c
#define MISTER_PIN 18
```

## BLE dashboard

Board advertises as `PoM-Bushmister` for 25 s after each wake. If a phone connects it stays awake up to 3 minutes, then sleeps.

1. Flash `PoM_Bushmister.ino` (ESP32 + DHT sensor library + ESP32 BLE Arduino).
2. Open `dashboard.html` in **Chrome or Edge** on the phone/laptop.
   - Web Bluetooth needs HTTPS or `http://localhost`.
   - Opening the file as `file://` usually fails. Host it on GitHub Pages or run `python3 -m http.server` next to the file.
3. Tap Connect → pick PoM-Bushmister.

Commands: Force OFF/ON, 3-burst mist, fart, auto on/off, reset calibration + log, dump last 48 samples, set sleep minutes, sleep now.

Status JSON streams every 2 s while connected.

## Power

BLE radio is only on during the advertise / connected window. Deep sleep still uses the 8-minute timer (or whatever you set).
