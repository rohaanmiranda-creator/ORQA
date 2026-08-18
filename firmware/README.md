# ORQA V1 — firmware

## hydrationpod_xiao/ — WORKING app-connectable firmware (XIAO nRF52840 Sense)

`hydrationpod_xiao/hydrationpod_xiao.ino` implements the **real BLE protocol the ORQA app
uses** (service 0x180D, device name `HydrationPod`, text-frame characteristics 0x2A57/58/59/5A
— reverse-engineered from `index-v2.html` on the app repo's `test` branch). Flash it to the
XIAO nRF52840 Sense and the app connects with full control: live telemetry at 1 Hz, sip/refill/
dispose events, ACK/NAK command console (TARE, SETGOAL, GLOW, LIGHT, SHOW, VIBE, BUZZ,
RESETDAY, RESTART, UPDATE→DFU).

- Real onboard IMU drives AX/AY/AZ, tilt, stability, and a sip gesture (upright → tilt >35°
  for 0.5–8 s → upright = one sip; hold inverted 2 s = dispose). Gestures arm only after the
  pod has sat upright ~1 s, so a board lying on the desk fires nothing.
- Water volume is **simulated** (no load cell on the bare XIAO). Set `WEIGHT_SIM 0` and wire
  an HX711 to D2/D3 (P0.28/P0.29 — the same nets as the ORQA V1 PCB) for real weight.
- USB serial console at 115200 for manual testing: `sip / add / dispose / tare / calw / cal /
  goal / day / in / out / unsettled / batt / status / debug / help`.

### Load-cell calibration

Two numbers live on the pod and persist in `/pod.cfg` on internal flash: `hxTare` (raw counts
at zero) and `hxScale` (counts per gram). Everything downstream is `(raw − tare) / scale`.

| Command | Serial | BLE | What it does |
|---|---|---|---|
| Zero the cell | `tare` | `TARE` | waits for the reading to settle, then stores the zero |
| Set the factor | `calw <grams>` | `CALW:<grams>` | settles, then `scale = (raw − tare) / grams` |
| Read the pair | `cal` | `CAL` | prints/notifies the live tare + scale |
| Pin a pair | `cal <tare> <scale>` | `CAL:<tare>:<scale>` | writes a known-good calibration directly |

`tare` and `calw` **wait for the cell to settle** before storing anything — three consecutive
medians of 10 agreeing within 0.5 g. They refuse (NAK) rather than write a number captured
while the reading was still creeping, which is what silently corrupted the scale factor before.
Expect them to take ~3 s, up to 15 s before giving up. `calw` also refuses to run with less
than 20 g actually on the cell, and rejects any resulting scale outside 20–20000 counts/g.

**`cal <tare> <scale>` is the restore path.** A sketch upload preserves `/pod.cfg`, but a full
chip erase or bootloader reflash wipes it and the pod falls back to `CAL_DEFAULT_SCALE`
(420 counts/g) — a plausible-looking number that is *not* a calibration. Record the pair from
`cal` after every good calibration and write it straight back afterwards; no reference mass
needed. The app's Dev tab exposes the same thing under **Pin calibration**, and flags a pod
still running 420 as uncalibrated rather than showing it as a normal value.

> The `CAL_DEFAULT_*` constants at the top of the sketch are the fallback only. Don't edit them
> to hold a real calibration — use `cal`/`CAL:` so the value persists and travels with the pod.

Build: arduino-cli, board `Seeeduino:nrf52:xiaonRF52840Sense` (Seeed nRF52 Boards package),
library "Seeed Arduino LSM6DS3". Compile from a path **without spaces** (the Seeed platform
recipe breaks on them). Flash: 1200-baud touch → bootloader COM port → 
`adafruit-nrfutil dfu serial --package <build>.zip -p COMx -b 115200`.

> **Note:** the scaffold below (`orqa_main.ino`) still carries a placeholder custom-UUID BLE
> service that does NOT match the app — when porting to the real ORQA PCB, take the BLE layer
> and protocol from `hydrationpod_xiao` and the pin map / power switches from the scaffold.

## Original V1 PCB scaffold

Starter firmware for the ORQA smart-bottle PCB (Nordic nRF52840). **Not production code** —
the structure, pin map, and power logic are ready; sensor/radio driver bodies are `TODO`.
The board has not been built, so nothing here has been run on hardware.

| File | What it is |
|---|---|
| `orqa_board.h` | Pin map — taken directly from the V1 netlist, verified against the schematic |
| `orqa_power.h` / `orqa_power.cpp` | Power management: the two load switches + IMU gate, weigh cycle, sleep. This is what delivers the battery target. |
| `orqa_main.ino` | Application skeleton: BLE service, wake-on-motion, weigh, feedback, sleep |

**Toolchain:** Arduino IDE + Adafruit nRF52 core (matches the board's pin naming) for bring-up;
port to nRF Connect SDK (Zephyr) for production. See `../ORQA_V1_Firmware_and_App_Spec.pdf`
for the full pin map, BLE service, runtime behaviour, and the companion-app feature set.

## Two things to confirm before writing drivers
1. **Button S1 is wired to RESET, not a GPIO** — a press resets the MCU unless P0.18 is
   reconfigured as GPIO in UICR (disables hardware reset). Decide this deliberately.
2. **Haptic (DA7280) I²C routing** — its SDA/SCL also appear on P0.04/P0.05; confirm on the
   schematic whether it shares the P0.27/P0.07 bus before instantiating the driver.
