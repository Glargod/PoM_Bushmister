# PoM (Piss-o-Matic) — field firmware

Set-and-forget scent mister for ESP32 + DHT11 + LDR.

No Wi-Fi, no BLE. Power on, it forces the spray **OFF**, then every 8 minutes: read sensors, update LDR calibration, mist only on pre-dusk / post-dawn creep inside the temp/RH gates, force OFF, sleep.

## Hardware

| Function | GPIO | Notes |
|---|---|---|
| Mister button pulse | 5 | Active LOW 100 ms. **GPIO5 can glitch at reset.** If it boots spraying, move this wire to **GPIO18** and change `MISTER_PIN`. |
| Mister LED / ON sense | 4 | HIGH ≈ 2.7 V = ON |
| DHT11 data | 15 | |
| LDR divider | 32 | Higher raw = brighter (as wired) |

## Calibration (LDR)

- Min/max only move after **two consecutive** wakes agree (spike reject).
- ADC rails 0 and 4095 are ignored.
- After ~24 h (`180` wakes) if max\u2212min is still `< 800`, min/max reset (covered / iced LDR).
- Trusted min/max get a mild daily decay toward the recent mean so July sun does not own November.

## Power

Deep sleep between wakes. A 10 000 mAh pack should last months on this firmware.

`dashboard.html` in this repo is a leftover bench experiment. Field unit does not serve it.
