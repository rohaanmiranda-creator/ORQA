/*
 * hydrationpod_xiao.ino — ORQA pod firmware for Seeed XIAO nRF52840 Sense
 * ========================================================================  v3
 * Component-validation firmware for the ORQA pod system. Speaks the EXACT BLE
 * protocol the ORQA app expects and drives the real peripherals.
 *
 * v3 additions:
 *   - LED state PERSISTS across reboot (last GLOW colour / LIGHT palette / OFF restored)
 *   - Load-cell-ABSENT mode: HX711 alive but bridge unplugged is auto-detected
 *     (floating-input drift signature) -> simulated volume until it returns
 *   - POWER-SAVE mode (AUTO on battery / ON / OFF): HX711 sleeps between weighs,
 *     gyro off, idle LED dark. App command POWERSAVE:AUTO|ON|OFF, serial 'ps'.
 *   - Haptic PATTERN engine: VIBE:amp[:ms[:type]] with type 0..5
 *     (single/double/triple/long/heartbeat/ramp), BUZZ:n = n crisp ticks,
 *     persisted global strength via HAPCFG:<10-200 %>
 *   - Real CLOCK: app pushes SETTIME:<epoch-seconds> on connect; T: shows true
 *     time and daily counters AUTO-RESET at midnight
 *   - SETNAME:<name> over BLE (no USB needed to rename a pod)
 *   - Daily counters, goal, tare, calibration, name, LED state, power mode and
 *     haptic strength all persist in internal flash (LittleFS)
 *
 * Protocol (service 0x180D):
 *   0x2A57 notify live 1 Hz  W|VOL|VOLD|SIPS|TOT|REF|RTOT|DISP|DTOT|TILT|STBL|AX|AY|AZ|ST|GOAL|PCT|BATT|VBAT|T
 *   0x2A58 notify sip        N|ML|DUR|VOL
 *   0x2A5A notify events     EVT|READY/ADD/DISPOSE/BOTTLE/UNSETTLED/DAYRESET/ACK/NAK/...
 *   0x2A59 write commands    TARE SETGOAL RESETDAY RESTART UPDATE SHOW BUZZ VIBE GLOW LIGHT
 *                            SETTIME SETNAME POWERSAVE HAPCFG CALW CAL
 *
 * v12 — load-cell calibration hardening:
 *   - TARE and CALW wait for the reading to SETTLE (3 consecutive medians of 10
 *     agreeing within 0.5 g) before storing, and NAK rather than write a bad
 *     number. Creep during placement can no longer be baked into the scale.
 *     Both block for ~3 s typical, 15 s worst case, then NAK. There is no
 *     watchdog on this build, and the softdevice keeps the BLE link up, so the
 *     only visible effect is that live frames pause for the duration.
 *   - CALW rejects a result outside a plausibility band and refuses to run with
 *     less than 20 g actually on the cell (the old guard was 50 counts ≈ 0.12 g).
 *   - CAL — new. `CAL` reports the live pair as EVT|CAL|TARE:<t>|SC:<s>|DEF:<0|1>;
 *     `CAL:<tare>:<scale>` pins a known-good calibration directly, no reference
 *     mass needed. This is the restore path after a chip erase wipes /pod.cfg.
 *   - The absent-detector is muted while a calibration is sampling, so placing
 *     and removing a reference mass can't latch the cell ABSENT mid-measurement.
 *   - VOLD: — volume at 0.1 g on the live frame, alongside the integer VOL:,
 *     so calibration verification isn't limited by 1 g quantisation.
 *
 * Serial console 115200: help for the full list.
 *
 * v5 additions (Development tab support):
 *   - DBG:1/0 — stream two diagnostic frames per second on the live char
 *     (D1| raw cell + hardware presence + power state, D2| working LED config
 *     + per-component power model). Auto-off on disconnect.
 *   - EVT|DECIDE:<SIP|DISPOSE|REFILL|NONE> after every gesture evaluation with
 *     the inputs (PRE/POST/D/PEAK/DUR/WHY) so the app can show the reasoning.
 *   - BATTCAP:<mAh> — persisted battery capacity for runtime estimates.
 *
 * v6 additions (component-testing):
 *   - SIPCFG:<tilt>:<minMs>:<maxMs>:<minMl>:<settleMs> — tune how a sip
 *     registers, persisted; SIPCFG:RESET restores defaults. Live values ride D2.
 *   - SIPFX:<hex>:<ms>:<pat>:<amp> | SIPFX:OFF — the on-pod flash + haptic
 *     that plays when a sip registers, persisted. SIMSIP[:ml] fakes a sip.
 *   - D3| battery frame: last-charge epoch, boot epoch, reset reason
 *     (RESETREAS), voltage-stability window, charge ETA, learned drain %/h.
 */

#include <bluefruit.h>
#include <Wire.h>
#include <limits.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

/* ------------------------------------------------------------------ config -- */
#define HAS_ONBOARD_IMU   1
#define BOTTLE_CAPACITY_ML  600
#define BOTTLE_TARE_G       180
#define LIVE_PERIOD_MS     1000
#define IMU_PERIOD_MS        50
#define RING_COUNT           14
#define RING_PIN_DEFAULT      8

#define TILT_UPRIGHT_DEG    20.0f
#define DISPOSE_DEG        150.0f
#define DISPOSE_HOLD_MS    2000

/* sip registration — v6: runtime-tunable via SIPCFG, persisted. These are the defaults. */
#define DEF_SIP_TILT       35
#define DEF_SIP_MIN_MS     500
#define DEF_SIP_MAX_MS     8000
#define DEF_SIP_MIN_ML     5
#define DEF_SETTLE_MS      8000   /* v13: the STILL window a weigh must survive  */

/* ------------------------------------------------- v13 drink measurement ----
 * Measurement is driven by STILLNESS, not by the tilt gesture. Picking the
 * bottle up and putting it down swings the cell hard; the old 900 ms settle
 * measured while it was still moving, and the leftover swing read as a sip or
 * a toss-out. Now: motion means the weight is untrustworthy, and a reading only
 * counts once the IMU has been still and upright for DEF_SETTLE_MS.            */
#define STILL_ACC_TOL      0.05f  /* |accel| may deviate this much from 1 g     */
#define SIP_WIN_SAMPLE_MS   500   /* weigh cadence inside the still window      */
#define SIP_MAX_ML          400   /* a single sip larger than this is not a sip */
#define REFILL_MIN_ML        20   /* weight GAIN this large is a refill         */
static uint16_t sipTilt   = DEF_SIP_TILT;     /* arm the gesture above this tilt      */
static uint16_t sipMinMs  = DEF_SIP_MIN_MS;   /* shortest tilt that can be a sip      */
static uint16_t sipMaxMs  = DEF_SIP_MAX_MS;   /* longest                              */
static uint8_t  sipMinMl  = DEF_SIP_MIN_ML;   /* weight delta below this = noise      */
static uint16_t settleMs  = DEF_SETTLE_MS;    /* wait after upright before weighing   */

/* what the pod does when a sip registers — v6: SIPFX, persisted                */
static uint8_t  sipFxOn   = 1;
static uint32_t sipFxCol  = 0x00C83C;         /* LED flash colour                     */
static uint16_t sipFxMs   = 400;              /* flash length                         */
static uint8_t  sipFxPat  = 1;                /* haptic pattern 0-9 (1 = double)      */
static uint8_t  sipFxAmp  = 60;               /* haptic strength 1-100                */

/* battery history — v6                                                          */
static uint32_t resetReas = 0;                /* RESETREAS captured at boot           */
static uint32_t lastChargeEpoch = 0;          /* charger unplug / charge-done moment  */
static float    drainPerHr = 0;               /* learned discharge %/h (0 = unlearned)*/
static float    lastVbat = 0;                 /* newest pack voltage sample           */
static float    vWinLo = 99, vWinHi = 0;      /* rolling ~10 min voltage window       */
static bool     chgNow = false;
static uint8_t  chg100 = 0;                   /* 1 = HICHG active → 100 mA charge (CHGCUR:) */

#define PS_WEIGH_PERIOD_MS 60000   /* powersave: periodic weigh cadence          */
#define HX_SETTLE_MS         450   /* HX711 wake-from-powerdown settle           */

/* ------------------------------------------------- load-cell calibration ---
 * CAL_DEFAULT_* are the FALLBACK pair, used only when /pod.cfg is missing —
 * a fresh chip, or a full erase / bootloader reflash. A pod that has ever been
 * calibrated carries its own tare+scale in flash and ignores these entirely.
 *
 * To pin a known-good calibration after an erase, don't edit these — send
 *   CAL:<tare>:<scale>        (BLE)   or   cal <tare> <scale>   (serial)
 * and it persists. 'cal' with no arguments prints the live pair.
 *
 * TARE and CALW no longer trust a single burst: they wait for the reading to
 * settle (CAL_SETTLE_HITS consecutive medians agreeing inside CAL_SETTLE_TOL_G)
 * before storing anything, so creep during placement can't be baked into the
 * scale factor. They NAK instead of writing a bad number.                      */
#define CAL_DEFAULT_TARE      0L        /* raw counts at zero load               */
#define CAL_DEFAULT_SCALE   420.0f      /* counts per gram                       */
#define CAL_SCALE_MIN        20.0f      /* plausibility band — reject outside    */
#define CAL_SCALE_MAX     20000.0f
#define CAL_MIN_DELTA_G      20.0f      /* CALW needs at least this much load on */
/* Sizing note: the HX711 runs at 10 SPS unless its RATE pin is strapped high,
 * so one median of 10 costs ~1 s and the 3-in-a-row settle needs ~3 s minimum.
 * The timeout must leave room for several failed attempts on top of that — at
 * 24 samples a median alone was 2.4 s and the settle could never finish inside
 * a short budget, so every calibration would have NAKed.                       */
#define CAL_SAMPLES            10       /* median width for a calibration read   */
#define CAL_SETTLE_TOL_G      0.5f      /* consecutive medians must agree within */
#define CAL_SETTLE_HITS         3
#define CAL_SETTLE_TIMEOUT  15000       /* ms before giving up and NAKing        */

#if HAS_ONBOARD_IMU
  #include "LSM6DS3.h"
  LSM6DS3 imu(I2C_MODE, 0x6A);
#endif

/* forward declarations */
struct PowerEst { float board, ble, cell, imu, led, hap, total; };   /* defined before the
                                    * auto-generated prototypes reference it     */
static void sendEvt(const char* s);
static void ledPulse(uint8_t r, uint8_t g, uint8_t b, uint16_t ms);
static float readVbat();
static void doDayReset();
static void cfgSave();
static void cfgMarkDirty();
static float netMl();
static void hapticQueue(uint8_t ampPct, uint16_t ms, uint16_t gapMs);
static void setBottleIn(bool in);
static bool gestureReset = false;     /* clears the gesture machine after a pause */

/* --------------------------------------------------------------- BLE objects -- */
BLEService        podSvc(0x180D);
BLECharacteristic chrLive(0x2A57);
BLECharacteristic chrSip (0x2A58);
BLECharacteristic chrCmd (0x2A59);
BLECharacteristic chrEvt (0x2A5A);

/* ------------------------------------------------------------------- state -- */
static float    simVolMl   = BOTTLE_CAPACITY_ML;
static uint32_t sips = 0, totMl = 0, refills = 0, refillMl = 0, disposes = 0, disposeMl = 0;
static uint32_t goalMl = 2500, sipSeq = 0;
static bool     bottleIn = true;

static float    tiltDeg = 0, ax = 0, ay = 0, az = 1;
static bool     stableFlag = true, imuOk = false;
static const char* podState = "IDLE";

static bool     connected = false, introSent = false, dbgFrames = false;
static bool     bleDbg = false;       /* app dev panel open: stream D1/D2 frames  */
static uint16_t battCap = 500;        /* battery capacity mAh (BATTCAP:, persisted) */
static char     devName[28] = "ORQA APP TESTER V1";

/* clock (set by the app via SETTIME) */
static uint32_t epochBase = 0;        /* epoch seconds at millis()==0; 0 = not set */
static uint32_t lastDayNum = 0;

/* power save: 0 = AUTO (on when running from battery), 1 = ON, 2 = OFF */
static uint8_t  psMode = 0;
static bool     psActive = false;

/* ---- HX711 ---- */
static bool   hxFound = false;        /* amplifier chip detected                  */
static bool   cellAbsent = false;     /* chip alive but bridge unplugged          */
static uint8_t hxDout = 2, hxSck = 3;
static long   hxTare  = CAL_DEFAULT_TARE;
static float  hxScale = CAL_DEFAULT_SCALE;
static bool   calBusy = false;        /* a calibration is sampling — mute the
                                       * absent-detector so placing/removing a
                                       * reference mass can't latch ABSENT      */
static uint8_t cellOff = 0;           /* CELLOFF:1 — load cell disabled from the app:
                                       * no probe, no reads, chip held in power-down */
static long   hxLastRaw = LONG_MIN;   /* newest good raw average, for diagnostics */
static float  lastNetMl = 0;
static float  preTiltMl = 0;
static bool   hxSleeping = false;     /* powersave: PD_SCK held high              */
static uint8_t cellBadBursts = 0, cellGoodBursts = 0;

/* ---- DA7280 haptic ---- */
static bool     hapFound = false;
static uint8_t  hapAddr  = 0x4A;
static TwoWire* hapWire  = &Wire;
static uint8_t  hapStrength = 100;    /* global scale 10-200 %, persisted         */
static bool     hapEnabled = true;    /* HAPTIC:0/1 master switch, persisted      */
/* pattern queue: pairs of (on amp,ms) + gap */
struct HapStep { uint8_t amp; uint16_t ms, gap; };
static HapStep  hapQ[8]; static uint8_t hapQn = 0, hapQi = 0;
static uint32_t hapPhaseEnd = 0; static bool hapDriving = false;

/* ---- LED ring + onboard LED — two lights + find-my-pod ----
 *  1. ALWAYS-ON light: Off/Solid/Gradient/Rainbow/Spin/Pulse/Breathe, up to 6
 *     colours, brightness 5-100 %, power-saver (light only while app connected;
 *     otherwise glows until battery < 5 %).
 *  2. REMINDER light: solid (strobe or breathe) or rainbow — plays as a timed
 *     overlay, then the always-on light resumes untouched.
 *  3. RING mode: red/white alternating + vibration for the first 3 s; cancelled
 *     by RING:OFF or by picking the pod up.                                     */
Adafruit_NeoPixel ring(RING_COUNT, RING_PIN_DEFAULT, NEO_GRB + NEO_KHZ800);
static uint8_t ringPin = RING_PIN_DEFAULT;
static uint8_t ringN = RING_COUNT;    /* actual pixel count on the wires — RINGN:<n>, persisted */

/* ring POWER gate — RINGPWR:<pin>[:AH]. The WS2812s pull ~10 mA from the rail
 * even showing black; a MOSFET on the rail (V1 board / bench bodge) driven by
 * this GPIO kills that. 0xFF = no gate wired, rail assumed hardwired on.       */
static uint8_t  ringPwrPin = 0xFF;
static uint8_t  ringPwrAH  = 0;       /* 1 = enable is active-HIGH (N-FET / EN pin) */
static bool     ringPowered = true;
static uint32_t ringLitMs = 0;        /* newest moment any pixel was non-black  */
static uint8_t  ringFit = 1;          /* RINGFIT:0 = ring physically absent — the
                                       * data line is write-only so we can't sense
                                       * it; the bench has to tell us            */

enum PMode { PM_OFF = 0, PM_SOLID, PM_GRADIENT, PM_RAINBOW, PM_SPIN, PM_PULSE, PM_BREATHE, PM_ECOSPIN };
static uint8_t  pMode = PM_BREATHE;
static uint32_t animCols[6] = {0x46e0d2, 0, 0, 0, 0, 0}; static uint8_t animN = 1;
static uint8_t  brightPct = 55;       /* 5-100, persisted                        */
static bool     lightSaver = false;   /* ON: always-on light only while app connected */
static uint8_t  lastBattPct = 100;

enum Ovl { OV_NONE = 0, OV_FX, OV_REMIND, OV_RING };
static uint8_t  ovl = OV_NONE;
static uint32_t ovlUntil = 0, ovlStart = 0;
static uint32_t fxCols[6]; static uint8_t fxN = 0; static bool fxIsShow = false;
static float    ringStartTilt = -1;
static uint8_t  remLMode = 0;         /* 0 solid-breathe · 1 solid-strobe · 2 rainbow */

static uint32_t pulseUntil = 0; static uint8_t pulseR, pulseG, pulseB;
static bool     ringLitNow = false;   /* did the last ledTick render any light?  */

/* the 5 ms fast pace is ONLY for active output (LED frames, haptic timing) —
 * a bare BLE link needs nothing from the loop that 20 ms can't provide        */
static bool podBusy()
{
  return hapDriving || hapQn || ringLitNow;
}

/* deferred config save (LED commands can arrive in bursts — don't wear flash) */
static uint32_t cfgDirtyAt = 0;

/* (timed effects use the Ovl overlay system above — the always-on light state
 * is never touched by reminders, ring mode, or app effects)                   */

/* ---- ON-POD reminder engine (runs with or without the app) ----
 * remMins==0 disables. Window is minutes-of-day; smart skips the nudge when a
 * sip already happened inside the current interval.                           */
static uint16_t remMins = 0, remFrom = 8 * 60, remTo = 22 * 60;
static bool     remSmart = true;
static uint8_t  remStyle = 1;         /* 0 gentle · 1 standard · 2 urgent        */
static uint32_t remLightC = 0xffb45a; /* 0 = no reminder light                   */
static uint32_t lastSipMs = 0, lastNudgeMs = 0, bootGraceEnd = 0;

/* ---- offline sip journal in flash: synced to the app on connect ---- */
struct HistEntry { uint32_t at; uint16_t ml; uint16_t dur; };
static uint32_t histCount = 0;        /* entries in /siplog.bin                  */
static uint32_t histSynced = 0;       /* how many the app has already received   */
static uint32_t nextHistSendMs = 0;

/* ================================================================== clock == */
static uint32_t epochNow() { return epochBase ? epochBase + millis() / 1000 : 0; }

static void clockTick()
{
  if (!epochBase) return;
  static uint32_t nextClockSave = 0;
  if (millis() > nextClockSave) {       /* checkpoint the clock so it survives a
                                         * reboot (stale by the off-time, healed
                                         * by SETTIME on the next app connect)   */
    nextClockSave = millis() + 600000UL;
    cfgMarkDirty();
  }
  uint32_t day = epochNow() / 86400UL;
  if (lastDayNum == 0) { lastDayNum = day; return; }
  if (day != lastDayNum) {              /* midnight rollover -> fresh day        */
    lastDayNum = day;
    doDayReset();
    Serial.println("Midnight — daily counters auto-reset.");
  }
}

/* =================================================== offline sip journal   == */
static void histAppend(uint16_t ml, uint16_t dur)
{
  HistEntry e = { epochNow(), ml, dur };
  File f(InternalFS);
  if (f.open("/siplog.bin", FILE_O_WRITE)) {
    f.seek(f.size());
    f.write((uint8_t*)&e, sizeof e);
    f.close();
    histCount++;
    if (histCount > 600) {              /* rotate: keep the newest 300           */
      HistEntry keep[300];
      if (f.open("/siplog.bin", FILE_O_READ)) {
        f.seek((histCount - 300) * sizeof(HistEntry));
        f.read((uint8_t*)keep, sizeof keep);
        f.close();
        InternalFS.remove("/siplog.bin");
        if (f.open("/siplog.bin", FILE_O_WRITE)) { f.write((uint8_t*)keep, sizeof keep); f.close(); }
        uint32_t dropped = histCount - 300;
        histCount = 300;
        histSynced = (histSynced > dropped) ? histSynced - dropped : 0;
        cfgMarkDirty();
      }
    }
  }
}

static void histInit()
{
  File f(InternalFS);
  if (f.open("/siplog.bin", FILE_O_READ)) { histCount = f.size() / sizeof(HistEntry); f.close(); }
  if (histSynced > histCount) histSynced = histCount;
}

/* Trickle unsynced entries to the app after it connects (2 per ~100 ms).       */
static void histSyncTick()
{
  if (!connected || !introSent || histSynced >= histCount) return;
  if (!chrEvt.notifyEnabled() || millis() < nextHistSendMs) return;
  nextHistSendMs = millis() + 100;

  File f(InternalFS);
  if (!f.open("/siplog.bin", FILE_O_READ)) return;
  HistEntry e;
  f.seek(histSynced * sizeof(HistEntry));
  if (f.read((uint8_t*)&e, sizeof e) == sizeof e) {
    char b[80];
    snprintf(b, sizeof b, "EVT|HIST:%u|DUR:%u|AT:%lu", e.ml, e.dur, (unsigned long)e.at);
    bleNotify(chrEvt, b);
    histSynced++;
    if (histSynced >= histCount) { cfgMarkDirty(); Serial.println("Offline sip log synced to app."); }
  }
  f.close();
}

/* =================================================== on-pod reminder tick  == */
static void reminderNudge()
{
  Serial.println("REMINDER — nudging");
  sendEvt("EVT|REMIND");                              /* banner if the app is on  */
  if (remLightC) {                                    /* overlay — the always-on
                                                       * light resumes after      */
    ovl = OV_REMIND; ovlStart = millis(); ovlUntil = millis() + 7000;
  }
  uint8_t pat = remStyle == 0 ? 9 : remStyle == 2 ? 8 : 1;
  uint8_t amp = remStyle == 0 ? 40 : remStyle == 2 ? 90 : 70;
  hapticPlayPattern(amp, 160, pat);
  lastNudgeMs = millis();
}

static void reminderTick()
{
  static uint32_t nextChk = 0;
  if (millis() < nextChk) return;
  nextChk = millis() + 20000;

  if (!remMins || !bottleIn) return;
  if (millis() < bootGraceEnd) return;

  if (epochBase) {                                    /* honour the active window */
    uint32_t mod = (epochNow() % 86400UL) / 60;       /* minute of day            */
    bool inWin = (remFrom <= remTo) ? (mod >= remFrom && mod < remTo)
                                    : (mod >= remFrom || mod < remTo);
    if (!inWin) return;
  }
  uint32_t iv = (uint32_t)remMins * 60000UL;
  uint32_t now = millis();
  if (remSmart && lastSipMs && now - lastSipMs < iv) return;   /* they drank — skip round */
  uint32_t anchor = max(lastSipMs, lastNudgeMs);
  if (!anchor) anchor = bootGraceEnd;
  if (now - anchor >= iv) reminderNudge();
}

/* ================================================================== HX711 == */
static void hxWake()
{
  if (!hxFound || !hxSleeping) return;
  digitalWrite(hxSck, LOW);             /* leave power-down                      */
  hxSleeping = false;
  delay(HX_SETTLE_MS);                  /* first conversion after wake           */
}

static void hxSleep()
{
  if (!hxFound || hxSleeping) return;
  digitalWrite(hxSck, HIGH);            /* PD_SCK high >60 us = power-down       */
  hxSleeping = true;
}

static long hxReadRaw(uint8_t dout, uint8_t sck, uint16_t timeoutMs)
{
  uint32_t t0 = millis();
  while (digitalRead(dout) == HIGH) {
    if (millis() - t0 > timeoutMs) return LONG_MIN;
    delay(1);
  }
  long v = 0;
  noInterrupts();
  for (int i = 0; i < 24; i++) {
    digitalWrite(sck, HIGH); delayMicroseconds(1);
    v = (v << 1) | digitalRead(dout);
    digitalWrite(sck, LOW);  delayMicroseconds(1);
  }
  digitalWrite(sck, HIGH); delayMicroseconds(1);
  digitalWrite(sck, LOW);
  interrupts();
  if (v & 0x800000L) v |= 0xFF000000L;
  return v;
}

static long hxAverage(uint8_t samples)
{
  if (!hxFound) return LONG_MIN;
  bool wasSleeping = hxSleeping;
  hxWake();
  int64_t acc = 0; uint8_t n = 0;
  for (uint8_t i = 0; i < samples; i++) {
    long r = hxReadRaw(hxDout, hxSck, 250);
    if (r != LONG_MIN) { acc += r; n++; }
  }
  if (wasSleeping && psActive) hxSleep();
  if (n) hxLastRaw = (long)(acc / n);
  return n ? (long)(acc / n) : LONG_MIN;
}

/* Median of n raw samples — used for calibration, where one spike landing in a
 * mean is the difference between a good scale factor and a silently wrong one. */
static long hxMedian(uint8_t n)
{
  if (!hxFound) return LONG_MIN;
  if (n > CAL_SAMPLES) n = CAL_SAMPLES;
  long buf[CAL_SAMPLES]; uint8_t k = 0;
  bool wasSleeping = hxSleeping;
  hxWake();
  for (uint8_t i = 0; i < n; i++) {
    long r = hxReadRaw(hxDout, hxSck, 250);
    if (r != LONG_MIN) buf[k++] = r;
  }
  if (wasSleeping && psActive) hxSleep();
  if (!k) return LONG_MIN;
  for (uint8_t i = 1; i < k; i++) {     /* insertion sort — k <= 24             */
    long v = buf[i]; int8_t j = i - 1;
    while (j >= 0 && buf[j] > v) { buf[j + 1] = buf[j]; j--; }
    buf[j + 1] = v;
  }
  hxLastRaw = buf[k / 2];
  return buf[k / 2];
}

/* Wait for the cell to stop moving, then return the settled median.
 * Takes medians back to back until CAL_SETTLE_HITS of them agree inside
 * CAL_SETTLE_TOL_G, or CAL_SETTLE_TIMEOUT elapses. Returns LONG_MIN on
 * timeout so the caller can NAK rather than store a drifting reading.          */
static long hxSettled()
{
  if (!hxFound || cellAbsent) return LONG_MIN;
  long tol = (long)(CAL_SETTLE_TOL_G * (hxScale > 1 ? hxScale : CAL_DEFAULT_SCALE));
  if (tol < 1) tol = 1;
  uint32_t t0 = millis();
  long prev = LONG_MIN; uint8_t hits = 0;
  while (millis() - t0 < CAL_SETTLE_TIMEOUT) {
    long m = hxMedian(CAL_SAMPLES);
    if (m == LONG_MIN) continue;
    if (prev != LONG_MIN && labs(m - prev) <= tol) {
      if (++hits >= CAL_SETTLE_HITS) return m;
    } else hits = 0;
    prev = m;
  }
  return LONG_MIN;
}

/* Apply and persist a calibration pair, rejecting implausible scales. One path
 * for BLE CAL:, serial 'cal', and the CALW result, so they can't diverge.      */
static bool calApply(long tare, float scale)
{
  if (!(scale >= CAL_SCALE_MIN && scale <= CAL_SCALE_MAX)) return false;
  hxTare = tare; hxScale = scale;
  cfgSave();
  Serial.print("CAL: tare="); Serial.print(hxTare);
  Serial.print(" scale=");    Serial.print(hxScale, 2);
  Serial.println(" counts/g");
  return true;
}

/* Bridge-absent detector: with no load cell the HX711 inputs float and the raw
 * value swings wildly (100k+ counts within a second) — nothing a real bridge
 * does, even when pressed. Two bad bursts latch ABSENT; two clean ones clear. */
static void cellHealthCheck()
{
  if (!hxFound || !bottleIn) return;
  if (bleDbg || calBusy) return;        /* bench-pressing the cell during a dev
                                         * session — or placing a reference mass
                                         * mid-calibration — swings raw hard; don't
                                         * let the absent-detector latch on either */
  static uint32_t nextCheck = 0;
  if (millis() < nextCheck) return;
  nextCheck = millis() + (cellAbsent ? 5000 : 15000);

  bool wasSleeping = hxSleeping;
  hxWake();
  long mn = LONG_MAX, mx = LONG_MIN;
  for (uint8_t i = 0; i < 4; i++) {
    long r = hxReadRaw(hxDout, hxSck, 250);
    if (r == LONG_MIN) return;          /* chip busy — try next round            */
    if (r < mn) mn = r;
    if (r > mx) mx = r;
  }
  if (wasSleeping && psActive) hxSleep();

  long range = mx - mn;
  bool railed = labs(mn) > 0x7C0000L && labs(mx) > 0x7C0000L;   /* pinned at full-scale = open input */
  if (range > 80000L || railed) {
    cellGoodBursts = 0;
    if (cellBadBursts < 2 && ++cellBadBursts == 2 && !cellAbsent) {
      cellAbsent = true;
      sendEvt("EVT|UNSETTLED");
      Serial.println("Load cell ABSENT (floating input) — simulated volume until it returns");
    }
  } else if (range < 5000L) {
    cellBadBursts = 0;
    if (cellAbsent && ++cellGoodBursts >= 2) {
      cellAbsent = false; cellGoodBursts = 0;
      sendEvt("EVT|READY");
      Serial.println("Load cell back — live weight resumed");
    }
  }
}

/* "Pod out of the bottle" pause mode: freeze measurements + sleep the cell     */
static void setBottleIn(bool in)
{
  if (bottleIn == in) return;
  bottleIn = in;
  if (!in) {
    hxSleep();
    podState = "OUT";
    sendEvt("EVT|BOTTLE:OUT");
    Serial.println("Bottle mode: OUT — measurements paused, load cell asleep");
  } else {
    if (!psActive) hxWake();
    gestureReset = true;
    if (hxFound && !cellAbsent) {       /* re-baseline: the out-period is not a sip */
      long r = hxAverage(3);
      if (r != LONG_MIN) { lastNetMl = (r - hxTare) / hxScale; if (lastNetMl < 0) lastNetMl = 0; }
    }
    preTiltMl = lastNetMl;
    podState = "IDLE";
    sendEvt("EVT|BOTTLE:IN");
    Serial.println("Bottle mode: IN — measurements resumed");
  }
  cfgSave();
}

static float netMl()
{
  if (!hxFound || cellAbsent) return simVolMl;
  if (!bottleIn) return lastNetMl;      /* paused: hold the last known reading   */
  if (hxSleeping) return lastNetMl;     /* powersave: no fresh weigh this frame  */
  long r = hxAverage(2);
  if (r == LONG_MIN) return lastNetMl;
  lastNetMl = (r - hxTare) / hxScale;
  if (lastNetMl < 0) lastNetMl = 0;
  return lastNetMl;
}

static bool hxTryPair(uint8_t dout, uint8_t sck)
{
  pinMode(sck, OUTPUT); digitalWrite(sck, LOW);
  pinMode(dout, INPUT_PULLUP);
  long v = hxReadRaw(dout, sck, 900);
  delayMicroseconds(5);
  bool wentHigh = (digitalRead(dout) == HIGH);
  bool nextConv = false;
  if (wentHigh) {
    uint32_t t0 = millis();
    while (millis() - t0 < 300) { if (digitalRead(dout) == LOW) { nextConv = true; break; } delay(2); }
  }
  if (v != LONG_MIN && v != 0 && v != -1 && wentHigh && nextConv) {
    /* dangling wires can crosstalk-fake a conversation — believe a chip only
     * after THREE consistent, non-railed reads in a row                        */
    long v2 = hxReadRaw(dout, sck, 300);
    long v3 = (v2 != LONG_MIN) ? hxReadRaw(dout, sck, 300) : LONG_MIN;
    if (v2 != LONG_MIN && v3 != LONG_MIN &&
        labs(v2 - v) < 0x200000L && labs(v3 - v2) < 0x200000L &&
        labs(v2) < 0x7C0000L && labs(v3) < 0x7C0000L) {
      hxFound = true; hxDout = dout; hxSck = sck;
      return true;
    }
  }
  pinMode(sck, INPUT); pinMode(dout, INPUT);
  return false;
}

static bool pinLowAgainstPullup(uint8_t pin)
{
  pinMode(pin, INPUT_PULLUP); delayMicroseconds(300);
  bool low = (digitalRead(pin) == LOW);
  pinMode(pin, INPUT_PULLDOWN);
  return low;
}

static void hxProbe()
{
  hxSleeping = false;
  if (hxTryPair(2, 3)) {                 /* known wiring: DT=D2 SCK=D3           */
    Serial.println("HX711: FOUND on DT=D2 SCK=D3");
    long r = hxAverage(4);
    Serial.print("  raw="); Serial.println(r);
    if (hxTare == 0 && r != LONG_MIN) hxTare = r;
    return;
  }
  const uint8_t cand[] = {0, 1, 2, 3, 6, 7, 8, 9, 10};
  const uint8_t nc = sizeof cand;
  /* NEVER touch the ring's data line (or its rail gate) — clocking it latches
   * garbage colours into the WS2812 chain that can glow until next driven      */
  const auto skip = [&](uint8_t p){ return p == ringPin || p == ringPwrPin; };
  for (uint8_t i = 0; i < nc; i++) if (!skip(cand[i])) pinMode(cand[i], INPUT_PULLDOWN);
  delay(700);
  for (uint8_t di = 0; di < nc && !hxFound; di++) {
    if (skip(cand[di]) || !pinLowAgainstPullup(cand[di])) continue;
    for (uint8_t si = 0; si < nc && !hxFound; si++) {
      if (si == di || skip(cand[si])) continue;
      if (hxTryPair(cand[di], cand[si])) break;
      delay(30);
    }
  }
  for (uint8_t i = 0; i < nc; i++)
    if (!skip(cand[i]) && !(hxFound && (cand[i] == hxDout || cand[i] == hxSck))) pinMode(cand[i], INPUT);
  ring.clear(); ring.show();            /* wipe anything the scan may have latched */
  if (hxFound) {
    Serial.print("HX711: FOUND  DOUT=D"); Serial.print(hxDout);
    Serial.print(" SCK=D"); Serial.println(hxSck);
    long r = hxAverage(4);
    if (hxTare == 0 && r != LONG_MIN) hxTare = r;
  } else {
    Serial.println("HX711: not found — simulated volume ('hxprobe' to retry)");
  }
}

/* ================================================================= DA7280 == */
static bool i2cWrite8(TwoWire& w, uint8_t addr, uint8_t reg, uint8_t val)
{ w.beginTransmission(addr); w.write(reg); w.write(val); return w.endTransmission() == 0; }

static int i2cRead8(TwoWire& w, uint8_t addr, uint8_t reg)
{ w.beginTransmission(addr); w.write(reg);
  if (w.endTransmission(false) != 0) return -1;
  if (w.requestFrom(addr, (uint8_t)1) != 1) return -1;
  return w.read(); }

static void i2cScan(TwoWire& w, const char* name)
{
  Serial.print("I2C scan ("); Serial.print(name); Serial.print("):");
  uint8_t found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    w.beginTransmission(a);
    if (w.endTransmission() == 0) {
      Serial.print(" 0x"); Serial.print(a, HEX); found++;
      if (!hapFound && a >= 0x48 && a <= 0x4B) {
        int rev = i2cRead8(w, a, 0x00);
        if (rev > 0) { hapFound = true; hapAddr = a; hapWire = &w;
          Serial.print("(DA7280 rev=0x"); Serial.print(rev, HEX); Serial.print(")"); }
      }
    }
  }
  if (!found) Serial.print(" none");
  Serial.println();
}

/* ---- haptic pattern engine (non-blocking) ----
 * DRO mode: TOP_CTL1 (0x22) [2:0]=1, TOP_CTL2 (0x23) = drive level.            */
static void hapDrive(uint8_t ampPct)
{
  uint16_t scaled = (uint16_t)ampPct * hapStrength / 100;   /* global strength   */
  if (scaled > 100) scaled = 100;
  uint8_t drive = (uint8_t)(scaled * 0x7F / 100);           /* full drive range  */
  if (hapFound) {
    i2cWrite8(*hapWire, hapAddr, 0x22, 0x01);
    i2cWrite8(*hapWire, hapAddr, 0x23, drive);
  }
}

static void hapStop()
{
  if (hapFound) { i2cWrite8(*hapWire, hapAddr, 0x23, 0x00); i2cWrite8(*hapWire, hapAddr, 0x22, 0x00); }
}

static void hapticQueue(uint8_t ampPct, uint16_t ms, uint16_t gapMs)
{
  if (hapQn >= 8) return;
  hapQ[hapQn].amp = ampPct; hapQ[hapQn].ms = ms; hapQ[hapQn].gap = gapMs;
  hapQn++;
}

static void hapticPlayPattern(uint8_t amp, uint16_t ms, uint8_t type)
{
  if (!hapEnabled) return;
  hapQn = hapQi = 0; hapDriving = false; hapPhaseEnd = 0; hapStop();
  if (!hapFound) ledPulse(180, 180, 255, 200);   /* visible stand-in on bare board */
  switch (type) {
    default:
    case 0: hapticQueue(amp, ms, 0); break;                                     /* single    */
    case 1: hapticQueue(amp, ms, 90); hapticQueue(amp, ms, 0); break;           /* double    */
    case 2: for (int i = 0; i < 3; i++) hapticQueue(amp, ms, 90); break;        /* triple    */
    case 3: hapticQueue(amp, ms * 3 > 1800 ? 1800 : ms * 3, 0); break;          /* long      */
    case 4: hapticQueue(amp, 120, 100); hapticQueue(amp / 2, 220, 0); break;    /* heartbeat */
    case 5: hapticQueue(amp / 3, ms, 40); hapticQueue((uint8_t)(amp * 2 / 3), ms, 40);
            hapticQueue(amp, ms, 0); break;                                     /* ramp      */
    case 6: hapticQueue(100, 45, 70); hapticQueue(100, 45, 0); break;           /* knock     */
    case 7: for (int i = 0; i < 5; i++) hapticQueue(amp, 50, 60); break;        /* train     */
    case 8: hapticQueue(amp / 3, 150, 80); hapticQueue(amp / 2, 150, 80);       /* escalate  */
            hapticQueue((uint8_t)(amp * 3 / 4), 150, 80); hapticQueue(100, 250, 0); break;
    case 9: for (int i = 0; i < 6; i++) hapticQueue(amp, 90, 40); break;        /* purr      */
  }
}

static void hapticTick()
{
  static uint32_t kickUntil = 0; static uint8_t curAmp = 0;
  uint32_t now = millis();
  if (kickUntil && now >= kickUntil) { kickUntil = 0; hapDrive(curAmp); }  /* settle after kick */
  if (hapPhaseEnd && now < hapPhaseEnd) return;
  if (hapDriving) {                     /* a pulse just finished -> stop + gap   */
    hapStop(); hapDriving = false; kickUntil = 0;
    uint16_t gap = (hapQi > 0 && hapQi <= hapQn) ? hapQ[hapQi - 1].gap : 0;
    hapPhaseEnd = gap ? now + gap : 0;
    return;
  }
  if (hapQi < hapQn) {                  /* start the next pulse                  */
    HapStep& s = hapQ[hapQi++];
    uint16_t ms = constrain(s.ms, (uint16_t)30, (uint16_t)2000);
    curAmp = s.amp;
    if (s.amp >= 50 && ms > 45) {       /* overdrive kick: full power first 25 ms
                                         * so the motor SNAPS instead of winding up */
      hapDrive(100); kickUntil = now + 25;
    } else hapDrive(s.amp);
    hapDriving = true;
    hapPhaseEnd = now + ms;
  } else if (hapQn) { hapQn = hapQi = 0; hapPhaseEnd = 0; }
}

/* ============================================================== LED engine == */
static uint32_t hsvWheel(uint8_t pos)   /* 0-255 -> rainbow colour               */
{
  pos = 255 - pos;
  if (pos < 85)  return ((uint32_t)(255 - pos * 3) << 16) | pos * 3;
  if (pos < 170) { pos -= 85; return ((uint32_t)(pos * 3) << 8) | (255 - pos * 3); }
  pos -= 170;    return ((uint32_t)(pos * 3) << 16) | ((uint32_t)(255 - pos * 3) << 8);
}

static uint32_t colBlend(uint32_t a, uint32_t b, float f)
{
  return ((uint32_t)(((a>>16)&0xFF)*(1-f) + ((b>>16)&0xFF)*f) << 16)
       | ((uint32_t)(((a>>8 )&0xFF)*(1-f) + ((b>>8 )&0xFF)*f) << 8)
       |  (uint32_t)(( a     &0xFF)*(1-f) + ( b     &0xFF)*f);
}

static uint32_t colScale(uint32_t c, float f)
{
  if (f > 1) f = 1; if (f < 0) f = 0;
  return ((uint32_t)(((c>>16)&0xFF)*f) << 16) | ((uint32_t)(((c>>8)&0xFF)*f) << 8) | (uint32_t)((c&0xFF)*f);
}

static void ledOnboard(uint32_t c)
{
  analogWrite(LED_RED,   255 - ((c >> 16) & 0xFF));
  analogWrite(LED_GREEN, 255 - ((c >> 8)  & 0xFF));
  analogWrite(LED_BLUE,  255 -  (c        & 0xFF));
}

static void ringUniform(uint32_t c)
{
  for (int i = 0; i < ringN; i++) ring.setPixelColor(i, c);
}

static void ledPulse(uint8_t r, uint8_t g, uint8_t b, uint16_t ms)
{ pulseR = r; pulseG = g; pulseB = b; pulseUntil = millis() + ms; }

static void ringPower(bool on)
{
  if (ringPwrPin > 10) { ringPowered = true; return; }   /* no gate — rail is hardwired */
  if (on == ringPowered) return;
  ringPowered = on;
  pinMode(ringPwrPin, OUTPUT);
  digitalWrite(ringPwrPin, on ? (ringPwrAH ? HIGH : LOW) : (ringPwrAH ? LOW : HIGH));
  if (on) { delay(2); ring.clear(); ring.show(); }   /* rail up: start black, this frame paints */
  else digitalWrite(ringPin, LOW);   /* data LOW — no parasitic feed through the input diode */
  Serial.print("Ring rail: "); Serial.println(on ? "ON" : "OFF (quiescent draw gone)");
}

static void cancelRing(const char* why)
{
  if (ovl != OV_RING) return;
  ovl = OV_NONE; ovlUntil = 0; ringStartTilt = -1;
  hapQn = hapQi = 0; hapStop();
  Serial.print("Ring mode ended ("); Serial.print(why); Serial.println(")");
}

static void ledTick()
{
  static uint32_t lastShow = 0;
  uint32_t now = millis();
  uint32_t out = 0;                     /* colour for this frame (uniform modes) */
  bool spatial = false;                 /* true when pixels were set individually */

  /* overlay expiry */
  if (ovl != OV_NONE && ovlUntil && now > ovlUntil) {
    if (ovl == OV_FX && fxIsShow) sendEvt("EVT|SHOWDONE");
    ovl = OV_NONE; ovlUntil = 0; ringStartTilt = -1;
  }
  /* picking the pod up ends ring mode */
  if (ovl == OV_RING && ringStartTilt >= 0 && fabsf(tiltDeg - ringStartTilt) > 25) cancelRing("picked up");

  if (pulseUntil > now) {               /* short event flash beats everything    */
    out = ((uint32_t)pulseR << 16) | ((uint32_t)pulseG << 8) | pulseB;
  }
  else if (ovl == OV_RING) {            /* find-my-pod: red / white alternating  */
    out = ((now - ovlStart) / 250) & 1 ? 0xFFFFFF : 0xFF0000;
  }
  else if (ovl == OV_REMIND) {
    uint32_t t = now - ovlStart;
    if (remLMode == 2)      out = hsvWheel((t / 8) & 0xFF);                        /* rainbow */
    else if (remLMode == 1) out = ((t / 150) & 1) ? remLightC : 0;                 /* strobe  */
    else                    out = colScale(remLightC, 0.15f + 0.85f * (sinf(t / 900.0f * PI) + 1) * 0.5f); /* breathe */
  }
  else if (ovl == OV_FX) {              /* app effect / show: palette crossfade  */
    uint8_t n = fxN ? fxN : 1;
    uint32_t t = now % (n * 1200UL);
    uint8_t i = t / 1200, j = (i + 1) % n;
    out = colBlend(fxCols[i], fxCols[j], (t % 1200) / 1200.0f);
  }
  else {                                /* ---- the ALWAYS-ON light ---- */
    bool suppressed = (lastBattPct < 5) ||                    /* battery guard   */
                      (lightSaver && !connected) ||           /* power saver     */
                      (psActive && pMode == PM_BREATHE && animN == 1 && animCols[0] == 0x46e0d2);
    if (suppressed || pMode == PM_OFF) out = 0;
    else {
      uint8_t n = animN ? animN : 1;
      float bf = brightPct / 100.0f;
      switch (pMode) {
        case PM_SOLID: out = colScale(animCols[0], bf); break;
        case PM_GRADIENT: {
          uint32_t t = now % (n * 2400UL);
          uint8_t i = t / 2400, j = (i + 1) % n;
          out = colScale(colBlend(animCols[i], animCols[j], (t % 2400) / 2400.0f), bf);
          break;
        }
        case PM_RAINBOW: {                /* spatial: hue wheel around the ring  */
          uint8_t base = (now / 12) & 0xFF;
          for (int i = 0; i < ringN; i++)
            ring.setPixelColor(i, colScale(hsvWheel(base + i * 256 / ringN), bf));
          spatial = true;
          break;
        }
        case PM_SPIN: {                   /* a 3-pixel arc chasing round the ring */
          uint32_t step = now / 70;
          uint8_t head = step % ringN;
          uint32_t c = colScale(animCols[(step / ringN) % n], bf);
          ringUniform(0);
          for (int k = 0; k < 3; k++) {
            int px = (head - k + ringN) % ringN;
            ring.setPixelColor(px, colScale(c, 1.0f - k * 0.35f));
          }
          spatial = true;
          break;
        }
        case PM_ECOSPIN: {                /* power-saver: ONE sweep, then ~6 s fully
                                           * dark — the rail gate cuts power mid-gap */
          const uint32_t SPIN_MS = 1000;   /* one full revolution in 1 s, any count */
          uint32_t t = now % (SPIN_MS + 6000UL);
          if (t >= SPIN_MS) { out = 0; break; }
          uint8_t head = (uint8_t)((t * ringN / SPIN_MS) % ringN);
          uint32_t c = colScale(animCols[0], bf);
          ringUniform(0);
          for (int k = 0; k < 3; k++) {
            int px = (head - k + ringN) % ringN;
            ring.setPixelColor(px, colScale(c, 1.0f - k * 0.35f));
          }
          spatial = true;
          break;
        }
        case PM_PULSE: {                  /* crisp flash, colour advances each pulse */
          uint32_t cyc = now / 900;
          out = ((now % 900) < 200) ? colScale(animCols[cyc % n], bf) : 0;
          break;
        }
        case PM_BREATHE:
        default: {                        /* smooth swell, colour advances each cycle */
          uint32_t cyc = now / 2400;
          float br = 0.08f + 0.92f * (sinf((now % 2400) / 2400.0f * 2 * PI - PI / 2) + 1) * 0.5f;
          out = colScale(animCols[cyc % n], bf * br);
          break;
        }
      }
    }
  }

  /* rail gate: any light re-powers instantly; solid black cuts the rail. Eco
   * spin cuts ~0.1 s into its gap (the WHOLE 6 s must be dark power); other
   * modes keep the 2 s hysteresis to ride out pulse gaps + breathe minima      */
  bool lit = spatial || out != 0;
  ringLitNow = lit;                     /* feeds podBusy(): dark ring = slow pace */
  uint32_t offDelay = (pMode == PM_ECOSPIN && ovl == OV_NONE && pulseUntil <= now) ? 120 : 2000;
  if (lit) { ringLitMs = now; if (!ringPowered) ringPower(true); }
  else if (ringPowered && now - ringLitMs > offDelay) ringPower(false);

  if (ringPowered && ringFit) {
    if (!spatial) ringUniform(out);     /* effects drive the RING only          */
    if (now - lastShow > 30) { lastShow = now; ring.show(); }
  } else {
    /* marked not-fitted (or rail cut): sweep away any glitch-latched pixels — a
     * WS2812 holds its last colour for as long as it has power                 */
    static uint32_t nextWipe = 0;
    if (now >= nextWipe) { nextWipe = now + 5000; ring.clear(); ring.show(); }
  }
  ledOnboardStatus(now);                /* onboard LED = battery status, nothing else */
}

/* The XIAO's own RGB LED is a BATTERY indicator only (never mirrors effects):
 *   charging      → steady yellow blending to green as the charge climbs
 *   full on USB   → steady green (charger done)
 *   battery < 5 % → short red flashes
 *   otherwise     → dark                                                       */
static void ledOnboardStatus(uint32_t now)
{
  static bool usb = false; static uint32_t nextUsb = 0;
  if (now >= nextUsb) { nextUsb = now + 2000; usb = usbPresent(); }
  if (chgNow)                          ledOnboard(colBlend(0xFFB400, 0x00C832, lastBattPct / 100.0f));
  else if (usb && lastBattPct >= 97)   ledOnboard(0x00C832);
  else if (lastBattPct < 5)            ledOnboard(((now % 1600) < 140) ? 0xFF0000 : 0);
  else                                 ledOnboard(0);
}

/* ========================================================== config persist == */
struct PodCfgV1 { long tare; float scale; uint32_t goal; uint8_t rpin; };
struct PodCfgV2 { long tare; float scale; uint32_t goal; uint8_t rpin; char name[28]; };
struct PodCfgV3 { long tare; float scale; uint32_t goal; uint8_t rpin; char name[28];
                  uint32_t sips, totMl, refills, refillMl, disposes, disposeMl, sipSeq; };
struct PodCfgV4 { long tare; float scale; uint32_t goal; uint8_t rpin; char name[28];
                  uint32_t sips, totMl, refills, refillMl, disposes, disposeMl, sipSeq;
                  uint8_t ledMode; uint32_t glowRGB; uint32_t animCols[6]; uint8_t animN;
                  uint8_t psMode; uint8_t hapStrength; uint8_t bottleOut; };
struct PodCfgV5 { long tare; float scale; uint32_t goal; uint8_t rpin; char name[28];
                  uint32_t sips, totMl, refills, refillMl, disposes, disposeMl, sipSeq;
                  uint8_t ledMode; uint32_t glowRGB; uint32_t animCols[6]; uint8_t animN;
                  uint8_t psMode; uint8_t hapStrength; uint8_t bottleOut; uint8_t hapOff; };
struct PodCfgV6 { long tare; float scale; uint32_t goal; uint8_t rpin; char name[28];
                  uint32_t sips, totMl, refills, refillMl, disposes, disposeMl, sipSeq;
                  uint8_t ledMode; uint32_t glowRGB; uint32_t animCols[6]; uint8_t animN;
                  uint8_t psMode; uint8_t hapStrength; uint8_t bottleOut; uint8_t hapOff;
                  uint8_t ecoLight; uint8_t remSmart; uint8_t remStyle; uint8_t _pad;
                  uint16_t remMins; uint16_t remFrom; uint16_t remTo; uint16_t _pad2;
                  uint32_t remLight; uint32_t epochSaved; uint32_t histSynced; };
struct PodCfgV7 { long tare; float scale; uint32_t goal; uint8_t rpin; char name[28];
                  uint32_t sips, totMl, refills, refillMl, disposes, disposeMl, sipSeq;
                  uint8_t ledMode; uint32_t glowRGB; uint32_t animCols[6]; uint8_t animN;
                  uint8_t psMode; uint8_t hapStrength; uint8_t bottleOut; uint8_t hapOff;
                  uint8_t ecoLight; uint8_t remSmart; uint8_t remStyle; uint8_t _pad;
                  uint16_t remMins; uint16_t remFrom; uint16_t remTo; uint16_t _pad2;
                  uint32_t remLight; uint32_t epochSaved; uint32_t histSynced;
                  uint8_t bright; uint8_t lightSaver; uint8_t remLMode; uint8_t _pad3; };
struct PodCfgV8 { long tare; float scale; uint32_t goal; uint8_t rpin; char name[28];
                  uint32_t sips, totMl, refills, refillMl, disposes, disposeMl, sipSeq;
                  uint8_t ledMode; uint32_t glowRGB; uint32_t animCols[6]; uint8_t animN;
                  uint8_t psMode; uint8_t hapStrength; uint8_t bottleOut; uint8_t hapOff;
                  uint8_t ecoLight; uint8_t remSmart; uint8_t remStyle; uint8_t _pad;
                  uint16_t remMins; uint16_t remFrom; uint16_t remTo; uint16_t _pad2;
                  uint32_t remLight; uint32_t epochSaved; uint32_t histSynced;
                  uint8_t bright; uint8_t lightSaver; uint8_t remLMode; uint8_t _pad3;
                  uint16_t battCap; uint16_t _pad4; };
struct PodCfgV9 { long tare; float scale; uint32_t goal; uint8_t rpin; char name[28];
                  uint32_t sips, totMl, refills, refillMl, disposes, disposeMl, sipSeq;
                  uint8_t ledMode; uint32_t glowRGB; uint32_t animCols[6]; uint8_t animN;
                  uint8_t psMode; uint8_t hapStrength; uint8_t bottleOut; uint8_t hapOff;
                  uint8_t ecoLight; uint8_t remSmart; uint8_t remStyle; uint8_t _pad;
                  uint16_t remMins; uint16_t remFrom; uint16_t remTo; uint16_t _pad2;
                  uint32_t remLight; uint32_t epochSaved; uint32_t histSynced;
                  uint8_t bright; uint8_t lightSaver; uint8_t remLMode; uint8_t _pad3;
                  uint16_t battCap; uint16_t _pad4;
                  /* v9: sip rules + sip effects + battery history               */
                  uint16_t sipTilt, sipMinMs, sipMaxMs, settleMs;
                  uint8_t sipMinMl, sipFxOn, sipFxPat, sipFxAmp;
                  uint32_t sipFxCol; uint16_t sipFxMs; uint16_t _pad5;
                  uint32_t lastChargeEpoch; float drainPerHr; };
struct PodCfgV10 { long tare; float scale; uint32_t goal; uint8_t rpin; char name[28];
                  uint32_t sips, totMl, refills, refillMl, disposes, disposeMl, sipSeq;
                  uint8_t ledMode; uint32_t glowRGB; uint32_t animCols[6]; uint8_t animN;
                  uint8_t psMode; uint8_t hapStrength; uint8_t bottleOut; uint8_t hapOff;
                  uint8_t ecoLight; uint8_t remSmart; uint8_t remStyle; uint8_t _pad;
                  uint16_t remMins; uint16_t remFrom; uint16_t remTo; uint16_t _pad2;
                  uint32_t remLight; uint32_t epochSaved; uint32_t histSynced;
                  uint8_t bright; uint8_t lightSaver; uint8_t remLMode; uint8_t _pad3;
                  uint16_t battCap; uint16_t _pad4;
                  uint16_t sipTilt, sipMinMs, sipMaxMs, settleMs;
                  uint8_t sipMinMl, sipFxOn, sipFxPat, sipFxAmp;
                  uint32_t sipFxCol; uint16_t sipFxMs; uint16_t _pad5;
                  uint32_t lastChargeEpoch; float drainPerHr;
                  uint8_t ringPwrPin, ringPwrAH; uint8_t chg100; uint8_t ringUnfit; };
struct PodCfgV11 { long tare; float scale; uint32_t goal; uint8_t rpin; char name[28];
                  uint32_t sips, totMl, refills, refillMl, disposes, disposeMl, sipSeq;
                  uint8_t ledMode; uint32_t glowRGB; uint32_t animCols[6]; uint8_t animN;
                  uint8_t psMode; uint8_t hapStrength; uint8_t bottleOut; uint8_t hapOff;
                  uint8_t ecoLight; uint8_t remSmart; uint8_t remStyle; uint8_t _pad;
                  uint16_t remMins; uint16_t remFrom; uint16_t remTo; uint16_t _pad2;
                  uint32_t remLight; uint32_t epochSaved; uint32_t histSynced;
                  uint8_t bright; uint8_t lightSaver; uint8_t remLMode; uint8_t _pad3;
                  uint16_t battCap; uint16_t _pad4;
                  uint16_t sipTilt, sipMinMs, sipMaxMs, settleMs;
                  uint8_t sipMinMl, sipFxOn, sipFxPat, sipFxAmp;
                  uint32_t sipFxCol; uint16_t sipFxMs; uint16_t _pad5;
                  uint32_t lastChargeEpoch; float drainPerHr;
                  uint8_t ringPwrPin, ringPwrAH; uint8_t chg100; uint8_t ringUnfit;
                  uint8_t cellOff; uint8_t ringN; uint16_t _pad8; };

static void cfgLoad()
{
  InternalFS.begin();
  File f(InternalFS);
  if (!f.open("/pod.cfg", FILE_O_READ)) return;
  PodCfgV11 c = {};
  int got = f.read((uint8_t*)&c, sizeof c);
  f.close();
  if (got != (int)sizeof(PodCfgV1) && got != (int)sizeof(PodCfgV2) &&
      got != (int)sizeof(PodCfgV3) && got != (int)sizeof(PodCfgV4) &&
      got != (int)sizeof(PodCfgV5) && got != (int)sizeof(PodCfgV6) &&
      got != (int)sizeof(PodCfgV7) && got != (int)sizeof(PodCfgV8) &&
      got != (int)sizeof(PodCfgV9) && got != (int)sizeof(PodCfgV10) &&
      got != (int)sizeof(PodCfgV11)) return;

  hxTare = c.tare;
  hxScale = (c.scale >= CAL_SCALE_MIN && c.scale <= CAL_SCALE_MAX) ? c.scale : CAL_DEFAULT_SCALE;
  goalMl = (c.goal >= 100 ? c.goal : 2500);
  if (c.rpin <= 10) { ringPin = c.rpin; ring.setPin(ringPin); }
  if (got >= (int)sizeof(PodCfgV2) && c.name[0]) {
    c.name[sizeof c.name - 1] = 0;
    strncpy(devName, c.name, sizeof devName);
  }
  if (got >= (int)sizeof(PodCfgV3)) {
    sips = c.sips; totMl = c.totMl; refills = c.refills; refillMl = c.refillMl;
    disposes = c.disposes; disposeMl = c.disposeMl; sipSeq = c.sipSeq;
  }
  if (got >= (int)sizeof(PodCfgV5)) hapEnabled = !c.hapOff;
  if (got >= (int)sizeof(PodCfgV6)) {
    remSmart = c.remSmart; remStyle = (c.remStyle <= 2) ? c.remStyle : 1;
    remMins = (c.remMins <= 720) ? c.remMins : 0;
    remFrom = c.remFrom % 1440; remTo = c.remTo % 1440;
    remLightC = c.remLight;
    if (c.epochSaved > 1600000000UL && !epochBase) {  /* resume the clock (stale
                                         * by the powered-off time; SETTIME heals) */
      epochBase = c.epochSaved - millis() / 1000;
      lastDayNum = c.epochSaved / 86400UL;
    }
    histSynced = c.histSynced;
  }
  if (got >= (int)sizeof(PodCfgV8)) {
    if (c.battCap >= 100 && c.battCap <= 5000) battCap = c.battCap;
  }
  if (got >= (int)sizeof(PodCfgV10)) {
    if (c.ringPwrPin <= 10 || c.ringPwrPin == 0xFF) ringPwrPin = c.ringPwrPin;
    ringPwrAH = c.ringPwrAH ? 1 : 0;
    chg100 = c.chg100 ? 1 : 0;
    ringFit = c.ringUnfit ? 0 : 1;    /* stored inverted so old configs read "fitted" */
  }
  if (got == (int)sizeof(PodCfgV11)) {
    cellOff = c.cellOff ? 1 : 0;
    if (c.ringN >= 1 && c.ringN <= 60) ringN = c.ringN;   /* 0 = older save → default 14 */
  }
  if (got >= (int)sizeof(PodCfgV9)) {
    if (c.sipTilt >= 10 && c.sipTilt <= 80) sipTilt = c.sipTilt;
    if (c.sipMinMs >= 100 && c.sipMinMs <= 3000) sipMinMs = c.sipMinMs;
    if (c.sipMaxMs >= 2000 && c.sipMaxMs <= 20000) sipMaxMs = c.sipMaxMs;
    if (c.sipMinMl >= 1 && c.sipMinMl <= 50) sipMinMl = c.sipMinMl;
    /* v13: the window is now the 8 s stillness gate, not a 900 ms post-tilt
     * settle. Anything under 3 s is a stale pre-v13 value — take the default. */
    if (c.settleMs >= 3000 && c.settleMs <= 20000) settleMs = c.settleMs;
    sipFxOn = c.sipFxOn ? 1 : 0;
    if (c.sipFxPat <= 9) sipFxPat = c.sipFxPat;
    if (c.sipFxAmp >= 1 && c.sipFxAmp <= 100) sipFxAmp = c.sipFxAmp;
    if (c.sipFxMs >= 100 && c.sipFxMs <= 2000) sipFxMs = c.sipFxMs;
    sipFxCol = c.sipFxCol & 0xFFFFFF;
    lastChargeEpoch = c.lastChargeEpoch;
    if (c.drainPerHr > 0.2f && c.drainPerHr < 8) drainPerHr = c.drainPerHr;   /* discard glitch-taught rates */
  }
  if (got >= (int)sizeof(PodCfgV7)) {   /* current format: direct restore        */
    if (c.ledMode <= PM_ECOSPIN) pMode = c.ledMode;
    memcpy(animCols, c.animCols, sizeof animCols);
    animN = (c.animN >= 1 && c.animN <= 6) ? c.animN : 1;
    brightPct = (c.bright >= 5 && c.bright <= 100) ? c.bright : 55;
    lightSaver = c.lightSaver;
    remLMode = (c.remLMode <= 2) ? c.remLMode : 0;
  } else if (got >= (int)sizeof(PodCfgV4)) {  /* older light format: best-effort map */
    switch (c.ledMode) {                /* old enum: 0 AUTO 1 OFF 2 GLOW 3 ANIM  */
      case 1: pMode = PM_OFF; break;
      case 2: pMode = PM_SOLID; animCols[0] = c.glowRGB; animN = 1; break;
      case 3: pMode = PM_GRADIENT;
        memcpy(animCols, c.animCols, sizeof animCols);
        animN = (c.animN >= 1 && c.animN <= 6) ? c.animN : 1; break;
      default: pMode = PM_BREATHE; break;
    }
  }
  if (got >= (int)sizeof(PodCfgV4)) {
    psMode = (c.psMode <= 2) ? c.psMode : 0;
    hapStrength = constrain(c.hapStrength, (uint8_t)10, (uint8_t)200);
    if (hapStrength < 10) hapStrength = 100;
    bottleIn = !c.bottleOut;            /* a paused pod stays paused after reboot */
    if (!bottleIn) podState = "OUT";
  }
}

static void cfgSave()
{
  File f(InternalFS);
  InternalFS.remove("/pod.cfg");
  if (f.open("/pod.cfg", FILE_O_WRITE)) {
    /* overlays never touch animCols/pMode any more — persist directly           */
    PodCfgV11 c = {};
    c.ringPwrPin = ringPwrPin; c.ringPwrAH = ringPwrAH; c.chg100 = chg100; c.ringUnfit = (uint8_t)(!ringFit);
    c.cellOff = cellOff; c.ringN = ringN;
    c.tare = hxTare; c.scale = hxScale; c.goal = goalMl; c.rpin = ringPin;
    c.sips = sips; c.totMl = totMl; c.refills = refills; c.refillMl = refillMl;
    c.disposes = disposes; c.disposeMl = disposeMl; c.sipSeq = sipSeq;
    c.ledMode = pMode; c.glowRGB = animCols[0]; c.animN = animN;
    c.psMode = psMode; c.hapStrength = hapStrength;
    c.bottleOut = (uint8_t)(!bottleIn); c.hapOff = (uint8_t)(!hapEnabled);
    c.remSmart = (uint8_t)remSmart; c.remStyle = remStyle;
    c.remMins = remMins; c.remFrom = remFrom; c.remTo = remTo;
    c.remLight = remLightC; c.epochSaved = epochNow(); c.histSynced = histSynced;
    c.bright = brightPct; c.lightSaver = (uint8_t)lightSaver; c.remLMode = remLMode;
    c.battCap = battCap;
    c.sipTilt = sipTilt; c.sipMinMs = sipMinMs; c.sipMaxMs = sipMaxMs; c.settleMs = settleMs;
    c.sipMinMl = sipMinMl; c.sipFxOn = sipFxOn; c.sipFxPat = sipFxPat; c.sipFxAmp = sipFxAmp;
    c.sipFxCol = sipFxCol; c.sipFxMs = sipFxMs;
    c.lastChargeEpoch = lastChargeEpoch; c.drainPerHr = drainPerHr;
    memcpy(c.animCols, animCols, sizeof c.animCols);
    strncpy(c.name, devName, sizeof c.name - 1);
    f.write((uint8_t*)&c, sizeof c);
    f.close();
  }
  cfgDirtyAt = 0;
}

/* LED commands can arrive many times a second during app effects — batch them */
static void cfgMarkDirty() { cfgDirtyAt = millis(); }
static void cfgDirtyTick() { if (cfgDirtyAt && millis() - cfgDirtyAt > 5000) cfgSave(); }

/* ============================================================ power save  == */
static bool usbPresent()
{
  uint32_t st = 0;
  return (sd_power_usbregstatus_get(&st) == NRF_SUCCESS) && (st & 1);
}

static void psTick()
{
  static uint32_t nextEval = 0, nextPsWeigh = 0;
  if (millis() < nextEval) return;
  nextEval = millis() + 2000;

  bool want = (psMode == 1) || (psMode == 0 && !usbPresent());
  if (want != psActive) {
    psActive = want;
    Serial.print("Power save: "); Serial.println(psActive ? "ACTIVE" : "off");
    if (psActive) hxSleep(); else hxWake();
  }
  if (psActive && hxFound && !cellAbsent && millis() > nextPsWeigh) {
    nextPsWeigh = millis() + PS_WEIGH_PERIOD_MS;   /* periodic fresh reading      */
    long r = hxAverage(2);
    if (r != LONG_MIN) { lastNetMl = (r - hxTare) / hxScale; if (lastNetMl < 0) lastNetMl = 0; }
  }
}

/* ============================================================ BLE notifies == */
static void bleNotify(BLECharacteristic& ch, const char* s)
{ if (connected && ch.notifyEnabled()) ch.notify(s, strlen(s)); }

static void sendEvt(const char* s)
{
  bleNotify(chrEvt, s);
  if (dbgFrames) { Serial.print("[evt] "); Serial.println(s); }
}
static void sendAck(const char* cmd) { char b[40]; snprintf(b, sizeof b, "EVT|ACK:%s", cmd); sendEvt(b); }
static void sendNak(const char* cmd) { char b[48]; snprintf(b, sizeof b, "EVT|NAK:%s", cmd); sendEvt(b); }

/* ------------------------------------------------------------------ battery -- */
static float vbatEma = 0;   /* reset (=0) on charger plug/unplug so the reading re-seeds fast */
static float readVbat()
{
#if defined(PIN_VBAT) && defined(VBAT_ENABLE)
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);
  delay(3);
  analogReadResolution(12);
  analogRead(PIN_VBAT);
  uint32_t acc = 0;
  for (uint8_t i = 0; i < 8; i++) acc += analogRead(PIN_VBAT);
  pinMode(VBAT_ENABLE, INPUT);
  float vAdc = (acc / 8.0f) * 3.6f / 4095.0f;
  float v = vAdc * (1000.0f + 510.0f) / 510.0f;
  if (vbatEma < 0.5f) vbatEma = v;
  vbatEma += 0.25f * (v - vbatEma);
  return vbatEma;
#else
  return 0;
#endif
}

static uint8_t vbatToPct(float v)
{
  /* LiPo REST-voltage curve — the old linear 3.30-4.15 map under-reported the
   * top: a full pack relaxes to ~4.05-4.10 V once the charger terminates and
   * would sit at "88%" forever                                                 */
  static const float   vp[] = { 3.30f, 3.50f, 3.60f, 3.70f, 3.75f, 3.80f, 3.85f, 3.90f, 3.95f, 4.00f, 4.05f, 4.10f, 4.15f };
  static const uint8_t pp[] = {    0,     5,    10,    25,    40,    55,    65,    75,    82,    88,    92,    96,   100 };
  if (v <= 0.5f) return 100;            /* no battery sense                     */
  if (v <= vp[0]) return 0;
  for (uint8_t i = 12; i > 0; i--)
    if (v >= vp[i - 1] && v < vp[i])
      return (uint8_t)(pp[i - 1] + (pp[i] - pp[i - 1]) * (v - vp[i - 1]) / (vp[i] - vp[i - 1]));
  return 100;
}

static bool isCharging()
{
  nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(0, 17), NRF_GPIO_PIN_PULLUP);
  delayMicroseconds(50);
  return nrf_gpio_pin_read(NRF_GPIO_PIN_MAP(0, 17)) == 0;
}

/* ------------------------------------------------------------ battery tick  -- */
/* One place samples the pack: voltage window for stability, charger-unplug
 * timestamps, and a learned discharge rate (EMA of measured %-drop per hour
 * while genuinely on battery). Runs once a minute.                             */
static void battTick()
{
  static uint32_t nextSample = 0;
  static uint8_t  winN = 0;
  static uint32_t drainT0 = 0; static uint8_t drainP0 = 0;
  if (millis() < nextSample) return;
  nextSample = millis() + 60000UL;

  lastVbat = readVbat();
  lastBattPct = vbatToPct(lastVbat);

  bool chg = isCharging();
  bool usb = usbPresent();
  if (chgNow != chg) {                  /* plug/unplug/charge-done transition    */
    vbatEma = 0;                        /* voltage steps on transitions — re-seed */
    winN = 0; vWinLo = 99; vWinHi = 0;  /* fresh stability window                */
    if (chgNow && !chg) { lastChargeEpoch = epochNow(); cfgMarkDirty(); }
  }
  chgNow = chg;
  /* the charger's termination is the authoritative "full": on USB, not charging,
   * pack rested at ≥4.0 V → 100%. (A rested full LiPo reads 4.05-4.10, never 4.2.)
   * After UNPLUG the voltage curve would cliff to ~92 — instead GLIDE from the
   * 100% anchor at the real drain rate until the curve catches up.              */
  static float socAnchor = -1; static uint32_t socAnchorMs = 0;
  if (usb && !chg && lastVbat >= 4.0f) { lastBattPct = 100; socAnchor = 100; socAnchorMs = millis(); }
  else if (chg) socAnchor = -1;
  else if (socAnchor > 0 && !usb) {
    float hrs = (millis() - socAnchorMs) / 3600000.0f;
    float drainEst = (drainPerHr > 0.2f) ? drainPerHr
                   : powerEst(connected, psActive).total / battCap * 100.0f;
    float est = socAnchor - drainEst * hrs;
    if (est <= lastBattPct) socAnchor = -1;   /* voltage curve caught up — hand off */
    else lastBattPct = (uint8_t)(est + 0.5f);
  }

  if (lastVbat > 0.5f) {                /* rolling ~10-sample voltage window     */
    if (lastVbat < vWinLo) vWinLo = lastVbat;
    if (lastVbat > vWinHi) vWinHi = lastVbat;
    if (++winN >= 10) { winN = 0; vWinLo = vWinHi = lastVbat; }
  }

  /* drain learning: only while truly discharging (no charger, no USB)          */
  if (!chg && !usb && epochNow()) {
    if (!drainT0) { drainT0 = epochNow(); drainP0 = lastBattPct; }
    else if (drainP0 >= lastBattPct + 2) {          /* ≥2% real drop measured    */
      float hrs = (epochNow() - drainT0) / 3600.0f;
      float rate = hrs > 0.01f ? (drainP0 - lastBattPct) / hrs : 99;
      /* sanity: ≥15 min window and a physically plausible rate — power glitches
       * once taught us 13%/h in a single bad minute                            */
      if (hrs >= 0.25f && rate > 0.2f && rate < 8.0f) {
        drainPerHr = (drainPerHr > 0) ? drainPerHr * 0.7f + rate * 0.3f : rate;
        cfgMarkDirty();
      }
      drainT0 = epochNow(); drainP0 = lastBattPct;  /* re-anchor                 */
    }
  } else drainT0 = 0;                               /* charging/USB: no learning */
}

/* voltage judged steady enough for the sip pipeline + BLE radio               */
static bool vbatStable()
{
  if (lastVbat < 0.5f) return true;     /* no battery sense — don't cry wolf    */
  return (vWinHi - vWinLo) <= 0.15f && lastVbat >= 3.45f;
}

/* minutes to full at the configured charge rate (CV-taper fudged), 0 = n/a    */
static uint16_t chargeEtaMin()
{
  if (!chgNow || lastBattPct >= 100) return 0;
  float mAh = battCap * (100 - lastBattPct) / 100.0f;
  return (uint16_t)(mAh / (chg100 ? 100.0f : 50.0f) * 60.0f * 1.2f);
}

/* XIAO HICHG (P0.13): driven LOW = 100 mA charge, floating = 50 mA default    */
static void applyChgCur()
{
  if (chg100) {
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 13));
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 13));
  } else nrf_gpio_cfg_default(NRF_GPIO_PIN_MAP(0, 13));
}

/* ------------------------------------------------------------- power model  -- */
/* Estimated draws (mA) — datasheet/typical figures, NOT measurements. They give
 * the app's Development tab a component breakdown + runtime estimate.           */
#define MA_BOARD        3.5f   /* nRF52840, 5 ms loop pace (connected / animating) */
#define MA_BOARD_IDLE   2.2f   /* 20 ms pace + 5 Hz IMU when alone and dark        */
#define MA_BLE_CONN     0.7f   /* connected @ 30-50 ms interval                  */
#define MA_BLE_ADV      0.15f  /* advertising @ 100-152 ms                       */
#define MA_CELL_ACTIVE  2.9f   /* HX711 (1.5) + 350R-ish bridge excitation (1.4) */
#define MA_CELL_NOCELL  1.5f   /* chip powered, bridge unplugged                 */
#define MA_CELL_SLEEP   0.05f  /* PD_SCK high power-down                         */
#define MA_IMU          0.2f   /* LSM6DS3 accel @ 52 Hz, gyro off                */
#define MA_HAP_IDLE     0.05f  /* DA7280 standby                                 */
#define MA_PIXEL_IDLE   0.7f   /* WS2812 quiescent, per pixel — even when dark   */
#define MA_PIXEL_FULL   45.0f  /* per pixel at full white                        */

static float ledMaEst(bool asConnected)
{
  if (!ringFit) return 0;              /* no ring on the wires — nothing draws   */
  float quiescent = ringN * MA_PIXEL_IDLE;
  bool suppressed = (lastBattPct < 5) || (lightSaver && !asConnected) || pMode == PM_OFF;
  if (suppressed) return (ringPwrPin <= 10) ? 0.05f : quiescent;   /* gated rail: dark ring = rail cut */
  uint8_t n = animN ? animN : 1;
  float ci = 0;                          /* mean colour intensity 0-1             */
  for (uint8_t i = 0; i < n; i++)
    ci += (((animCols[i] >> 16) & 0xFF) + ((animCols[i] >> 8) & 0xFF) + (animCols[i] & 0xFF)) / 765.0f;
  ci /= n;
  float duty;                            /* mean on-fraction of each mode         */
  switch (pMode) {
    case PM_SOLID:    duty = 1.0f;  break;
    case PM_GRADIENT: duty = 0.9f;  break;
    case PM_RAINBOW:  duty = 1.0f; ci = 0.33f; break;   /* wheel sums to 255      */
    case PM_SPIN:     duty = 3.0f / ringN; break;
    case PM_PULSE:    duty = 0.22f; break;
    case PM_ECOSPIN:  duty = (3.0f / ringN) * (1.0f / 7.0f);   /* 1 s sweep per 7 s  */
      if (ringPwrPin <= 10) quiescent *= 0.16f;   /* rail dead for the whole 6 s gap */
      break;
    default:          duty = 0.5f;  break;              /* breathe                */
  }
  return quiescent + ringN * MA_PIXEL_FULL * (brightPct / 100.0f) * ci * duty;
}

/* asConnected/asPs let the same model answer "right now" AND "standalone"       */
static PowerEst powerEst(bool asConnected, bool asPs)
{
  PowerEst p;
  p.board = asConnected ? MA_BOARD : MA_BOARD_IDLE;   /* alone: slower loop + IMU */
  p.ble   = asConnected ? MA_BLE_CONN : MA_BLE_ADV;
  if (!hxFound)                     p.cell = 0;
  else if (asPs || !bottleIn)       p.cell = MA_CELL_SLEEP;
  else if (cellAbsent)              p.cell = MA_CELL_NOCELL;
  else                              p.cell = MA_CELL_ACTIVE;
  p.imu = imuOk ? MA_IMU : 0;
  p.led = ledMaEst(asConnected);
  p.hap = hapFound ? MA_HAP_IDLE : 0;
  p.total = p.board + p.ble + p.cell + p.imu + p.led + p.hap;
  return p;
}

/* instantaneous LED draw — from the pixels ACTUALLY lit this frame, not a mode
 * average: the app's live bars breathe with the animation                      */
static float ledMaNow()
{
  if (!ringFit) return 0;
  if (!ringPowered) return 0.05f;       /* rail cut by the gate                  */
  float sum = 0;
  for (int i = 0; i < ringN; i++) {
    uint32_t c = ring.getPixelColor(i);
    sum += (((c >> 16) & 0xFF) + ((c >> 8) & 0xFF) + (c & 0xFF)) / 765.0f;
  }
  return ringN * MA_PIXEL_IDLE + sum * MA_PIXEL_FULL;
}

/* ------------------------------------------------------- diagnostic frames  -- */
/* Two frames per live tick while DBG:1 (the app's Development tab is open):
 *   D1| load cell raw/cal + hardware presence + power state
 *   D2| working LED config + power model breakdown                              */
static void sendDiagFrames()
{
  if (!connected || !bleDbg || !chrLive.notifyEnabled()) return;
  char f[240];

  const char* cellSt = cellOff ? "OFF" : !hxFound ? "NONE" : cellAbsent ? "ABSENT" : "OK";
  snprintf(f, sizeof f,
    "D1|RAW:%ld|TARE:%ld|SC:%.1f|CELL:%s|HX:%u/%u|SLP:%u|IMU:%u|HAP:%u"
    "|PS:%u/%u|USB:%u|CHG:%u|BOT:%u|UP:%lu|HS:%lu/%lu|VER:6",
    (hxLastRaw == LONG_MIN) ? 0 : hxLastRaw, hxTare, hxScale, cellSt, hxDout, hxSck,
    hxSleeping ? 1 : 0, imuOk ? 1 : 0, hapFound ? 1 : 0,
    psMode, psActive ? 1 : 0, usbPresent() ? 1 : 0, isCharging() ? 1 : 0,
    bottleIn ? 1 : 0, millis() / 1000,
    (unsigned long)histSynced, (unsigned long)histCount);
  chrLive.notify(f, strlen(f));

  PowerEst now;                         /* LIVE draw — what the hardware is doing THIS second */
  now.board = podBusy() ? MA_BOARD : MA_BOARD_IDLE;   /* reflects the real loop pace */
  now.ble   = connected ? MA_BLE_CONN : MA_BLE_ADV;
  now.cell  = (cellOff || !hxFound) ? 0 : hxSleeping ? MA_CELL_SLEEP
            : cellAbsent ? MA_CELL_NOCELL : bottleIn ? MA_CELL_ACTIVE : MA_CELL_SLEEP;
  now.imu   = imuOk ? MA_IMU : 0;
  now.led   = ledMaNow();
  now.hap   = hapFound ? (hapDriving ? 60.0f : MA_HAP_IDLE) : 0;
  now.total = now.board + now.ble + now.cell + now.imu + now.led + now.hap;
  PowerEst solo = powerEst(false, psMode != 2);   /* disconnected, powersave unless forced off */
  char cols[48]; int cn = 0;
  uint8_t n = animN ? animN : 1;
  for (uint8_t i = 0; i < n && cn < (int)sizeof cols - 8; i++)
    cn += snprintf(cols + cn, sizeof cols - cn, "%s%06lx", i ? "-" : "", (unsigned long)(animCols[i] & 0xFFFFFF));
  snprintf(f, sizeof f,
    "D2|LED:%u|LN:%u|LC:%s|LB:%u|LS:%u|OVL:%u|RL:%06lx|RM:%u"
    "|MA:%.1f|PW:%.1f/%.1f/%.1f/%.1f/%.1f/%.1f|MAS:%.1f|CAP:%u"
    "|SCT:%u|SMN:%u|SMX:%u|SML:%u|SST:%u|SFX:%06lx/%u/%u/%u/%u|RP:%u/%u/%u|RD:%u|RF:%u|RN:%u",
    pMode, n, cols, brightPct, lightSaver ? 1 : 0, ovl,
    (unsigned long)(remLightC & 0xFFFFFF), remLMode,
    now.total, now.board, now.ble, now.cell, now.imu, now.led, now.hap,
    solo.total, battCap,
    sipTilt, sipMinMs, sipMaxMs, sipMinMl, settleMs,
    (unsigned long)(sipFxCol & 0xFFFFFF), sipFxMs, sipFxPat, sipFxAmp, sipFxOn,
    ringPwrPin, ringPwrAH, ringPowered ? 1 : 0, ringPin, ringFit, ringN);
  chrLive.notify(f, strlen(f));

  const char* rr = (resetReas & 1) ? "PIN" : (resetReas & 2) ? "WATCHDOG"
                 : (resetReas & 4) ? "SOFTWARE" : (resetReas & 8) ? "LOCKUP"
                 : (resetReas & 0x10000) ? "WAKEOFF" : "POWERON";
  snprintf(f, sizeof f,
    "D3|LCH:%lu|BOOT:%lu|RR:%s|RRX:%lx|VST:%u|VLO:%.2f|VHI:%.2f|CHT:%u|DPH:%.2f|CHG:%u|CC:%u",
    (unsigned long)lastChargeEpoch,
    (unsigned long)(epochNow() ? epochNow() - millis() / 1000 : 0),
    rr, (unsigned long)resetReas,
    vbatStable() ? 1 : 0, (vWinLo > 90 ? lastVbat : vWinLo), (vWinHi < 1 ? lastVbat : vWinHi),
    chargeEtaMin(), drainPerHr, chgNow ? 1 : 0, chg100 ? 100 : 50);
  chrLive.notify(f, strlen(f));
  if (dbgFrames) { Serial.print("[diag] "); Serial.println(f); }
}

/* fast raw stream — the app's live load-cell trace (~6-7 Hz while DBG:1).
 * Single unaveraged reading per frame so a finger-press shows instantly.       */
static void sendRawStream()
{
  static uint32_t nextDr = 0;
  static long drPrev = LONG_MIN; static uint8_t drBig = 0, drCalm = 0, drRail = 0, drN = 0, drDead = 0;
  if (!connected || !bleDbg || !hxFound || !chrLive.notifyEnabled()) return;
  if (millis() < nextDr) return;
  nextDr = millis() + 150;
  if (hxSleeping) hxWake();             /* keep the cell alive while streaming   */
  long r = hxReadRaw(hxDout, hxSck, 120);
  if (r == LONG_MIN) {                  /* chip stopped answering mid-session?   */
    if (++drDead >= 10) {               /* ~1.5 s of silence = amplifier gone    */
      drDead = 0; hxFound = false; cellAbsent = false; drPrev = LONG_MIN; drN = drBig = drCalm = 0;
      sendEvt("EVT|UNSETTLED");
      Serial.println("HX711 vanished mid-session — devReprobeTick will re-find it when replugged");
    }
    return;
  }
  drDead = 0;
  /* dev-session absent detector (cellHealthCheck is muted while DBG streams so
   * bench presses can't false-latch): a PRESS moves raw in 1-2 big steps; a
   * FLOATING input jumps wildly on nearly every sample. Judge every 8 samples. */
  if (drPrev != LONG_MIN) {
    long d = labs(r - drPrev);
    drN++;
    if (d > 70000L) drBig++;
    if (d < 5000L)  drCalm++;
    if (labs(r) > 0x7C0000L) drRail++;  /* pinned near ±full-scale = open input   */
    if (drN >= 8) {
      if ((drBig >= 6 || drRail >= 6) && !cellAbsent) {
        cellAbsent = true; sendEvt("EVT|UNSETTLED");
        Serial.println(drRail >= 6 ? "Load cell ABSENT (input railed at full-scale)"
                                   : "Load cell ABSENT (floating-input signature on the dev stream)");
      } else if (drCalm >= 8 && drRail == 0 && cellAbsent) {
        cellAbsent = false; sendEvt("EVT|READY");
        Serial.println("Load cell back — steady readings on the dev stream");
      }
      drN = drBig = drCalm = drRail = 0;
    }
  }
  drPrev = r;
  hxLastRaw = r;
  char f[80];
  snprintf(f, sizeof f, "DR|RAW:%ld|G:%.1f|CELL:%s",
           r, (r - hxTare) / hxScale, cellAbsent ? "ABSENT" : "OK");
  chrLive.notify(f, strlen(f));
}

/* while the Dev tab is open and no HX711 was found at boot, quietly retry the
 * known wiring every 5 s — plugging the amplifier in mid-session just works    */
static void devReprobeTick()
{
  static uint32_t next = 0;
  if (cellOff || !bleDbg || hxFound || millis() < next) return;
  next = millis() + 5000;
  if (hxTryPair(2, 3)) {
    Serial.println("HX711: appeared on DT=D2 SCK=D3 (dev re-probe)");
    long r = hxAverage(4);
    if (hxTare == 0 && r != LONG_MIN) hxTare = r;
    sendEvt("EVT|READY");
  }
}

/* ---------------------------------------------------------------- live frame -- */
static void sendLiveFrame()
{
  float vbat = lastVbat;                /* battTick samples the pack once a minute */
  float vol = netMl();
  uint32_t pct = goalMl ? (uint32_t)(totMl * 100UL / goalMl) : 0;

  uint32_t hh, mm, ss;
  if (epochBase) { uint32_t t = epochNow() % 86400UL; hh = t / 3600; mm = t / 60 % 60; ss = t % 60; }
  else           { uint32_t t = millis() / 1000;      hh = t / 3600 % 24; mm = t / 60 % 60; ss = t % 60; }

  char f[260];
  snprintf(f, sizeof f,
    "W:%d|VOL:%d|VOLD:%.1f|SIPS:%lu|TOT:%lu|REF:%lu|RTOT:%lu|DISP:%lu|DTOT:%lu"
    "|TILT:%d|STBL:%d|AX:%.2f|AY:%.2f|AZ:%.2f|ST:%s|GOAL:%lu|PCT:%lu"
    "|BATT:%u|VBAT:%.2f|T:%02lu:%02lu:%02lu",
    (int)(BOTTLE_TARE_G + vol), (int)vol, vol,
    (unsigned long)sips, (unsigned long)totMl,
    (unsigned long)refills, (unsigned long)refillMl,
    (unsigned long)disposes, (unsigned long)disposeMl,
    (int)tiltDeg, stableFlag ? 1 : 0, ax, ay, az, podState,
    (unsigned long)goalMl, (unsigned long)pct,
    lastBattPct, vbat,                  /* battTick's value — includes the charger-done 100% clamp */
    (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);

  if (connected && chrLive.notifyEnabled()) {
    bool ok = chrLive.notify(f, strlen(f));
    static uint32_t lastFail = 0;
    if (!ok && millis() - lastFail > 5000) { lastFail = millis(); Serial.println("[live] NOTIFY FAILED"); }
  }
  if (dbgFrames) { Serial.print("[live] "); Serial.println(f); }

  /* motion-response check — the permanent automatic tell: a REAL bridge must
   * wiggle when the bottle visibly moves. 8+ s of clear tilting with a frozen
   * reading (<~0.25 g of change) means no bridge is connected.                 */
  static long mrLo = LONG_MAX, mrHi = LONG_MIN; static float mtLo = 999, mtHi = -999; static uint8_t mrN = 0;
  if (hxFound && !cellAbsent && bottleIn && !hxSleeping && !stableFlag && hxLastRaw != LONG_MIN) {
    if (hxLastRaw < mrLo) mrLo = hxLastRaw;
    if (hxLastRaw > mrHi) mrHi = hxLastRaw;
    if (tiltDeg < mtLo) mtLo = tiltDeg;
    if (tiltDeg > mtHi) mtHi = tiltDeg;
    if (++mrN >= 8) {
      if (mtHi - mtLo >= 12 && (mrHi - mrLo) < 100) {
        cellAbsent = true; sendEvt("EVT|UNSETTLED");
        Serial.println("Load cell ABSENT (no response to motion)");
      }
      mrN = 0; mrLo = LONG_MAX; mrHi = LONG_MIN; mtLo = 999; mtHi = -999;
    }
  } else { mrN = 0; mrLo = LONG_MAX; mrHi = LONG_MIN; mtLo = 999; mtHi = -999; }
}

/* --------------------------------------------------------------- water events -- */
static void doSip(uint32_t ml, uint32_t durMs)
{
  if (!ml) return;
  if (!hxFound || cellAbsent) { if (ml > (uint32_t)simVolMl) ml = (uint32_t)simVolMl; simVolMl -= ml; }
  if (!ml) return;
  sips++; totMl += ml; sipSeq++;
  lastSipMs = millis();                              /* smart reminders anchor    */
  char b[64];
  snprintf(b, sizeof b, "N:%lu|ML:%lu|DUR:%lu|VOL:%d",
           (unsigned long)sipSeq, (unsigned long)ml, (unsigned long)durMs, (int)netMl());
  bool delivered = connected && chrSip.notifyEnabled();
  histAppend((uint16_t)min(ml, (uint32_t)65535), (uint16_t)min(durMs, (uint32_t)65535));
  if (delivered) histSynced = histCount;             /* app got it live — no replay */
  bleNotify(chrSip, b);
  if (sipFxOn) {                                   /* SIPFX-configured feedback */
    hapticPlayPattern(sipFxAmp, 60, sipFxPat);
    ledPulse((sipFxCol >> 16) & 0xFF, (sipFxCol >> 8) & 0xFF, sipFxCol & 0xFF, sipFxMs);
  }
  cfgSave();
  Serial.print("SIP  +"); Serial.print(ml); Serial.print(" ml  bottle="); Serial.print((int)netMl()); Serial.println(" ml");
}

static void doAdd(uint32_t ml)
{
  if (!hxFound || cellAbsent) {
    if ((uint32_t)simVolMl + ml > BOTTLE_CAPACITY_ML) ml = BOTTLE_CAPACITY_ML - (uint32_t)simVolMl;
    if (!ml) return;
    simVolMl += ml;
  }
  refills++; refillMl += ml;
  char b[64];
  snprintf(b, sizeof b, "EVT|ADD:%lu|VOL:%d|RTOT:%lu", (unsigned long)ml, (int)netMl(), (unsigned long)refillMl);
  sendEvt(b);
  ledPulse(40, 120, 255, 400);
  cfgSave();
  Serial.print("ADD  +"); Serial.print(ml); Serial.print(" ml  bottle="); Serial.print((int)netMl()); Serial.println(" ml");
}

static void doDispose(uint32_t ml, int peakDeg, uint32_t durMs)
{
  if (!hxFound || cellAbsent) { if (ml > (uint32_t)simVolMl) ml = (uint32_t)simVolMl; if (!ml) return; simVolMl -= ml; }
  disposes++; disposeMl += ml;
  char b[96];
  snprintf(b, sizeof b, "EVT|DISPOSE:%lu|VOL:%d|DTOT:%lu|PEAK:%d|DUR:%lu",
           (unsigned long)ml, (int)netMl(), (unsigned long)disposeMl, peakDeg, (unsigned long)durMs);
  sendEvt(b);
  hapticPlayPattern(85, 90, 6);                    /* firm knock                */
  ledPulse(255, 40, 40, 400);
  cfgSave();
  Serial.print("DISPOSE -"); Serial.print(ml); Serial.print(" ml  bottle="); Serial.print((int)netMl()); Serial.println(" ml");
}

static void doDayReset()
{
  sips = totMl = refills = refillMl = disposes = disposeMl = 0;
  cfgSave();
  sendEvt("EVT|DAYRESET");
  Serial.println("Daily counters zeroed.");
}

/* --------------------------------------------------------- IMU + drink logic -- */
static void imuTick()
{
#if HAS_ONBOARD_IMU
  if (imuOk) { ax = imu.readFloatAccelX(); ay = imu.readFloatAccelY(); az = imu.readFloatAccelZ(); }
#endif
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  if (mag < 0.05f) mag = 1;
  float c = az / mag; c = constrain(c, -1.0f, 1.0f);
  float newTilt = acosf(c) * 180.0f / PI;

  static float lastTilt = 0;
  stableFlag = fabsf(newTilt - lastTilt) < 3.0f;
  lastTilt = tiltDeg = newTilt;

  /* --------------------------------------------- v13: stillness-driven weigh --
   * motion            -> the bottle is being handled, weight means nothing
   * still + upright   -> open a window, average every weigh taken inside it
   * window survives settleMs (8 s) -> that average IS the new stable weight
   * sip = previous stable weight - new stable weight
   * A measurement only ARMS after motion has been seen, so a pod sitting
   * untouched cannot grind out phantom sips from slow drift.                   */
  static bool     motionSeen  = false;   /* handled since the last measurement  */
  static bool     baseValid   = false;   /* preTiltMl holds a real baseline     */
  static uint32_t stillSince  = 0;       /* when the current still spell began  */
  static uint32_t lastWinWeigh = 0;
  static float    winSum      = 0;
  static uint16_t winN        = 0;
  uint32_t now = millis();

  /* stillness: tilt not changing AND the accelerometer sitting at ~1 g. The
   * tilt test alone misses a straight lift, which does not rotate the bottle. */
  static float lastMag = 1.0f;
  bool accStill = fabsf(mag - 1.0f) < STILL_ACC_TOL && fabsf(mag - lastMag) < STILL_ACC_TOL;
  lastMag = mag;

  if (gestureReset) { gestureReset = false; motionSeen = false; stillSince = 0; winSum = 0; winN = 0; }
  if (!bottleIn) { podState = "OUT"; return; }     /* paused: no drink detection */

  bool upright = (tiltDeg < TILT_UPRIGHT_DEG);
  bool still   = stableFlag && accStill;

  if (!still || !upright) {                        /* being handled            */
    stillSince = 0; winSum = 0; winN = 0;
    motionSeen = true;
    podState = upright ? "MOVE" : "TILT";
    return;
  }

  if (!stillSince) { stillSince = now; winSum = 0; winN = 0; lastWinWeigh = 0; }

  /* accumulate weighs across the window rather than taking one reading at the
   * end — the average is what the 8 s is FOR.                                 */
  if (hxFound && !cellAbsent && (now - lastWinWeigh >= SIP_WIN_SAMPLE_MS)) {
    lastWinWeigh = now;
    hxWake();
    long r = hxAverage(2);
    if (r != LONG_MIN) {
      float g = (float)(r - hxTare) / hxScale; if (g < 0) g = 0;
      lastNetMl = g; winSum += g; winN++;
    }
    if (psActive) hxSleep();
  }

  if (now - stillSince < settleMs) {               /* window still running     */
    podState = motionSeen ? "SETTLE" : "IDLE";
    return;
  }

  /* ---- the window completed: this reading is trustworthy ---- */
  float avg = winN ? (winSum / winN) : lastNetMl;
  stillSince = now; winSum = 0; winN = 0;          /* re-arm the next window   */

  if (!baseValid) {                                /* first ever reading       */
    preTiltMl = avg; baseValid = true; motionSeen = false; podState = "IDLE"; return;
  }
  if (!motionSeen) {                               /* untouched — track drift
                                                    * quietly so it can never
                                                    * accumulate into a "sip"  */
    preTiltMl = avg; podState = "IDLE"; return;
  }

  motionSeen = false;
  float delta = preTiltMl - avg;                   /* + = went down = drunk    */
  const char* verdict = (delta >= sipMinMl && delta <= SIP_MAX_ML) ? "SIP"
                      : (delta <= -REFILL_MIN_ML)                  ? "REFILL" : "NONE";
  char why[110];
  snprintf(why, sizeof why, "EVT|DECIDE:%s|PRE:%d|POST:%d|D:%d|N:%u|WIN:%lu%s",
           verdict, (int)preTiltMl, (int)avg, (int)delta, winN,
           (unsigned long)settleMs,
           strcmp(verdict, "NONE") ? "" : (delta > SIP_MAX_ML ? "|WHY:HUGE" : "|WHY:TINY"));
  sendEvt(why);
  if      (!strcmp(verdict, "SIP"))    doSip((uint32_t)delta, settleMs);
  else if (!strcmp(verdict, "REFILL")) doAdd((uint32_t)(-delta));
  preTiltMl = avg;
  podState = "IDLE";
}

/* the old tilt-gesture machine lived here. It armed on a >35 deg tilt and
 * measured 900 ms after the bottle returned upright, which is far too soon for
 * a cell that was just set down — and its DISPOSE branch (hold past 150 deg)
 * fired on ordinary handling. Both are gone; see the stillness window above.
 * doDispose() itself is kept only as the manual serial 'dispose' test hook.   */
#if 0
static void legacyGestureMachine(uint32_t now, float mag)
{
  static uint32_t tiltStart = 0, uprightSince = 0; static float peak = 0;
  static bool armed = false, wasUpright = false;
  static uint32_t settleAt = 0; static uint32_t gestureDur = 0; static float gesturePeak = 0;

  if (tiltDeg < TILT_UPRIGHT_DEG) {
    if (!uprightSince) uprightSince = now;
    if (now - uprightSince > 800) {
      if (!wasUpright) preTiltMl = netMl();
      wasUpright = true;
    }
  } else uprightSince = 0;

  if (settleAt && now > settleAt && tiltDeg < TILT_UPRIGHT_DEG) {
    settleAt = 0;
    char why[110];                                 /* the decision + its inputs, for the app's dev panel */
    if (hxFound && !cellAbsent) {
      hxWake();                                    /* powersave: fresh weigh    */
      long r = hxAverage(3);
      float post = (r != LONG_MIN) ? (r - hxTare) / hxScale : preTiltMl;
      if (post < 0) post = 0;
      lastNetMl = post;
      if (psActive) hxSleep();
      float delta = preTiltMl - post;
      const char* verdict = (gesturePeak >= DISPOSE_DEG && delta > sipMinMl)   ? "DISPOSE"
                          : (delta >= sipMinMl && delta <= 400)                ? "SIP"
                          : (delta <= -20)                                     ? "REFILL" : "NONE";
      snprintf(why, sizeof why, "EVT|DECIDE:%s|PRE:%d|POST:%d|D:%d|PEAK:%d|DUR:%lu%s",
               verdict, (int)preTiltMl, (int)post, (int)delta, (int)gesturePeak, (unsigned long)gestureDur,
               strcmp(verdict, "NONE") ? "" : (delta > 400 ? "|WHY:HUGE" : "|WHY:TINY"));
      sendEvt(why);
      if      (!strcmp(verdict, "DISPOSE")) doDispose((uint32_t)delta, (int)gesturePeak, gestureDur);
      else if (!strcmp(verdict, "SIP"))     doSip((uint32_t)delta, gestureDur);
      else if (!strcmp(verdict, "REFILL"))  doAdd((uint32_t)(-delta));
      preTiltMl = post;
    } else {
      bool disp = gesturePeak >= DISPOSE_DEG;
      snprintf(why, sizeof why, "EVT|DECIDE:%s|PEAK:%d|DUR:%lu|WHY:SIM",
               disp ? "DISPOSE" : "SIP", (int)gesturePeak, (unsigned long)gestureDur);
      sendEvt(why);
      if (disp) doDispose(min((uint32_t)simVolMl, (uint32_t)150), (int)gesturePeak, gestureDur);
      else doSip(constrain((uint32_t)(gestureDur / 40), (uint32_t)10, (uint32_t)90), gestureDur);
    }
    podState = "IDLE";
  }

  if (!armed) {
    if (wasUpright && tiltDeg > sipTilt) { armed = true; tiltStart = now; peak = tiltDeg; podState = "TILT"; }
    else if (!settleAt) podState = stableFlag ? "IDLE" : "MOVE";
  } else {
    if (tiltDeg > peak) peak = tiltDeg;
    uint32_t held = now - tiltStart;

    if (peak >= DISPOSE_DEG && held >= DISPOSE_HOLD_MS && tiltDeg >= DISPOSE_DEG && (!hxFound || cellAbsent)) {
      doDispose(min((uint32_t)simVolMl, (uint32_t)150), (int)peak, held);
      armed = false; wasUpright = false; podState = "SETTLE";
    }
    else if (tiltDeg < TILT_UPRIGHT_DEG) {
      armed = false; wasUpright = false;
      if (held >= sipMinMs && held <= sipMaxMs) {
        gestureDur = held; gesturePeak = peak;
        settleAt = now + settleMs;
        podState = "SETTLE";
      } else {
        char why[96];
        snprintf(why, sizeof why, "EVT|DECIDE:NONE|PEAK:%d|DUR:%lu|WHY:%s",
                 (int)peak, (unsigned long)held, held < sipMinMs ? "SHORT" : "LONG");
        sendEvt(why);
        podState = "IDLE";
      }
    }
    else if (held > (uint32_t)sipMaxMs + 4000) {
      armed = false; wasUpright = false; podState = "IDLE";
      char why[96];
      snprintf(why, sizeof why, "EVT|DECIDE:NONE|PEAK:%d|DUR:%lu|WHY:TIMEOUT", (int)peak, (unsigned long)held);
      sendEvt(why);
    }
  }
}
#endif  /* legacy tilt-gesture machine */

/* ------------------------------------------------------------ command parser -- */
static uint32_t parseHexColor(const char* t)
{
  if (t[0] == '0' && (t[1] == 'x' || t[1] == 'X')) t += 2;
  return (uint32_t)strtoul(t, nullptr, 16);
}

static void handleCommand(char* cmd)
{
  for (char* p = cmd; *p && *p != ':'; p++) *p = toupper(*p);
  char* arg = strchr(cmd, ':'); if (arg) *arg++ = 0;

  Serial.print("[cmd] "); Serial.print(cmd);
  if (arg) { Serial.print(":"); Serial.print(arg); }
  Serial.println();

  if (!strcmp(cmd, "TARE")) {
    if (hxFound && !cellAbsent) {
      calBusy = true;
      long r = hxSettled();             /* wait for it to stop moving           */
      calBusy = false;
      if (r == LONG_MIN) {              /* never settled — keep the old zero    */
        sendNak("TARE");
        Serial.println("TARE: reading never settled — zero unchanged");
        return;
      }
      hxTare = r; cfgSave();
      char b[48];
      snprintf(b, sizeof b, "EVT|TARE:%ld", hxTare);
      sendEvt(b);
    }
    else simVolMl = 0;
    sendAck("TARE"); sendEvt("EVT|READY");
  }
  else if (!strcmp(cmd, "SETGOAL")) {
    uint32_t g = arg ? strtoul(arg, nullptr, 10) : 0;
    if (g >= 100 && g <= 20000) { goalMl = g; cfgSave(); sendAck("SETGOAL"); }
    else sendNak("SETGOAL");
  }
  else if (!strcmp(cmd, "SETTIME")) {
    /* SETTIME:<epoch seconds> — real clock + automatic midnight day-reset      */
    uint32_t e = arg ? strtoul(arg, nullptr, 10) : 0;
    if (e > 1600000000UL) {
      epochBase = e - millis() / 1000;
      lastDayNum = e / 86400UL;
      sendAck("SETTIME");
    } else sendNak("SETTIME");
  }
  else if (!strcmp(cmd, "SETNAME")) {
    if (arg && arg[0]) {
      strncpy(devName, arg, sizeof devName - 1); devName[sizeof devName - 1] = 0;
      cfgSave();
      sendAck("SETNAME"); sendEvt("EVT|REBOOTING");
      delay(400); NVIC_SystemReset();
    } else sendNak("SETNAME");
  }
  else if (!strcmp(cmd, "POWERSAVE")) {
    if      (arg && !strncasecmp(arg, "AUTO", 4)) psMode = 0;
    else if (arg && !strncasecmp(arg, "ON", 2))   psMode = 1;
    else if (arg && !strncasecmp(arg, "OFF", 3))  psMode = 2;
    else { sendNak("POWERSAVE"); return; }
    cfgSave(); sendAck("POWERSAVE");
  }
  else if (!strcmp(cmd, "HAPCFG")) {
    uint32_t s = arg ? strtoul(arg, nullptr, 10) : 0;
    if (s >= 10 && s <= 200) { hapStrength = s; cfgSave(); sendAck("HAPCFG"); hapticPlayPattern(60, 90, 1); }
    else sendNak("HAPCFG");
  }
  else if (!strcmp(cmd, "HAPTIC")) {
    /* HAPTIC:0/1 — master haptics switch (the app's feedback-channel toggle)    */
    if (arg) { hapEnabled = (arg[0] != '0'); if (!hapEnabled) { hapQn = hapQi = 0; hapStop(); }
      cfgSave(); sendAck("HAPTIC"); if (hapEnabled) hapticPlayPattern(60, 80, 1); }
    else sendNak("HAPTIC");
  }
  else if (!strcmp(cmd, "SOUND")) {
    /* no speaker/buzzer on this hardware — accept quietly so the app's sound
     * toggle doesn't surface a "Pod rejected" error                             */
    sendAck("SOUND");
  }
  else if (!strcmp(cmd, "BOTTLE")) {
    /* BOTTLE:OUT pauses all measurement (pod removed); BOTTLE:IN resumes        */
    if      (arg && !strncasecmp(arg, "OUT", 3)) { setBottleIn(false); sendAck("BOTTLE"); }
    else if (arg && !strncasecmp(arg, "IN", 2))  { setBottleIn(true);  sendAck("BOTTLE"); }
    else sendNak("BOTTLE");
  }
  else if (!strcmp(cmd, "DBG")) {
    /* DBG:1 streams D1/D2 diagnostic frames for the app's Development tab       */
    if (arg) { bleDbg = (arg[0] != '0');
      if (!bleDbg && (psActive || !bottleIn)) hxSleep();   /* the raw stream kept the cell awake */
      sendAck("DBG"); }
    else sendNak("DBG");
  }
  else if (!strcmp(cmd, "CALW")) {
    /* CALW:<grams> — scale calibration against a known weight on the cell
     * (BLE twin of the serial 'calw'). Waits for the reading to settle first,
     * so creep during placement can't be baked into the factor, and rejects a
     * result outside the plausibility band. Replies EVT|CALW on success.       */
    uint32_t g = arg ? strtoul(arg, nullptr, 10) : 0;
    if (!(g >= 20 && g <= 5000) || !hxFound || cellAbsent) { sendNak("CALW"); return; }
    calBusy = true;
    long r = hxSettled();
    calBusy = false;
    if (r == LONG_MIN) {
      sendNak("CALW");
      Serial.println("CALW: reading never settled — scale unchanged");
      return;
    }
    /* the load actually on the cell must be a real one, not noise around zero  */
    float onCell = fabsf((float)(r - hxTare)) / (hxScale > 1 ? hxScale : CAL_DEFAULT_SCALE);
    if (onCell < CAL_MIN_DELTA_G) {
      sendNak("CALW");
      Serial.println("CALW: nothing on the cell — put the reference weight on first");
      return;
    }
    float sc = (float)(r - hxTare) / (float)g;
    if (!calApply(hxTare, sc)) {
      sendNak("CALW");
      Serial.print("CALW: rejected implausible scale "); Serial.println(sc, 2);
      return;
    }
    sendAck("CALW");
    char b[72];
    snprintf(b, sizeof b, "EVT|CALW:%lu|SC:%.2f|RAW:%ld|TARE:%ld",
             (unsigned long)g, hxScale, r, hxTare);
    sendEvt(b);
  }
  else if (!strcmp(cmd, "CAL")) {
    /* CAL              — report the live pair as EVT|CAL:TARE:<t>:SC:<s>
     * CAL:<tare>:<scale> — pin a known-good calibration directly, persisted.
     * This is the post-erase restore path: no reference mass needed.           */
    if (!arg) {
      char b[72];
      snprintf(b, sizeof b, "EVT|CAL|TARE:%ld|SC:%.2f|DEF:%u",
               hxTare, hxScale, (hxScale == CAL_DEFAULT_SCALE) ? 1 : 0);
      sendEvt(b); sendAck("CAL");
      return;
    }
    char* colon = strchr(arg, ':');
    if (!colon) { sendNak("CAL"); return; }
    *colon = 0;
    long  t = strtol(arg, nullptr, 10);
    float s = strtod(colon + 1, nullptr);
    if (!calApply(t, s)) { sendNak("CAL"); return; }
    sendAck("CAL");
    char b[72];
    snprintf(b, sizeof b, "EVT|CAL|TARE:%ld|SC:%.2f|DEF:0", hxTare, hxScale);
    sendEvt(b);
  }
  else if (!strcmp(cmd, "SIPCFG")) {
    /* SIPCFG:<tilt 10-80>:<minMs 100-3000>:<maxMs 2000-20000>:<minMl 1-50>:<settleMs 3000-20000>
     * v13: only minMl and settleMs still affect detection — settleMs IS the
     * stillness window. tilt/minMs/maxMs are retained for protocol compatibility.
     * or SIPCFG:RESET — how a sip gets registered, persisted                    */
    if (arg && !strncasecmp(arg, "RESET", 5)) {
      sipTilt = DEF_SIP_TILT; sipMinMs = DEF_SIP_MIN_MS; sipMaxMs = DEF_SIP_MAX_MS;
      sipMinMl = DEF_SIP_MIN_ML; settleMs = DEF_SETTLE_MS;
      cfgSave(); sendAck("SIPCFG");
    } else if (arg) {
      uint32_t v[5]; uint8_t n = 0; char* t = arg;
      while (t && n < 5) { v[n++] = strtoul(t, nullptr, 10); t = strchr(t, ':'); if (t) t++; }
      if (n == 5 &&
          v[0] >= 10 && v[0] <= 80 && v[1] >= 100 && v[1] <= 3000 &&
          v[2] >= 2000 && v[2] <= 20000 && v[2] > v[1] &&
          v[3] >= 1 && v[3] <= 50 && v[4] >= 3000 && v[4] <= 20000) {   /* v13: 8 s window */
        sipTilt = v[0]; sipMinMs = v[1]; sipMaxMs = v[2]; sipMinMl = v[3]; settleMs = v[4];
        gestureReset = true;            /* re-arm cleanly under the new rules    */
        cfgSave(); sendAck("SIPCFG");
        Serial.print("Sip rules: tilt>"); Serial.print(sipTilt);
        Serial.print(" "); Serial.print(sipMinMs); Serial.print("-"); Serial.print(sipMaxMs);
        Serial.print("ms min"); Serial.print(sipMinMl); Serial.print("ml settle"); Serial.println(settleMs);
      } else sendNak("SIPCFG");
    } else sendNak("SIPCFG");
  }
  else if (!strcmp(cmd, "SIPFX")) {
    /* SIPFX:OFF | SIPFX:<hex>:<ms 100-2000>:<pat 0-9>:<amp 1-100> — feedback on sip */
    if (arg && !strncasecmp(arg, "OFF", 3)) { sipFxOn = 0; cfgSave(); sendAck("SIPFX"); }
    else if (arg) {
      char* p1 = strchr(arg, ':');
      uint32_t col = parseHexColor(arg);
      uint16_t ms = sipFxMs; uint8_t pat = sipFxPat, amp = sipFxAmp;
      if (p1) { ms = (uint16_t)constrain(strtoul(p1 + 1, nullptr, 10), 100UL, 2000UL);
        char* p2 = strchr(p1 + 1, ':');
        if (p2) { pat = (uint8_t)constrain(strtoul(p2 + 1, nullptr, 10), 0UL, 9UL);
          char* p3 = strchr(p2 + 1, ':');
          if (p3) amp = (uint8_t)constrain(strtoul(p3 + 1, nullptr, 10), 1UL, 100UL); } }
      sipFxOn = 1; sipFxCol = col; sipFxMs = ms; sipFxPat = pat; sipFxAmp = amp;
      cfgSave(); sendAck("SIPFX");
      hapticPlayPattern(sipFxAmp, 60, sipFxPat);   /* instant preview            */
      ledPulse((col >> 16) & 0xFF, (col >> 8) & 0xFF, col & 0xFF, ms);
    } else sendNak("SIPFX");
  }
  else if (!strcmp(cmd, "SIMSIP")) {
    /* SIMSIP[:ml] — fake a registered sip end-to-end (events, journal, effects) */
    uint32_t ml = arg ? constrain(strtoul(arg, nullptr, 10), 5UL, 400UL) : 30;
    sendAck("SIMSIP");
    char db[80];
    snprintf(db, sizeof db, "EVT|DECIDE:SIP|D:%lu|PEAK:0|DUR:1200|WHY:SIM", (unsigned long)ml);
    sendEvt(db);
    doSip(ml, 1200);
  }
  else if (!strcmp(cmd, "CHGCUR")) {
    /* CHGCUR:50|100 — battery charge current via the XIAO's HICHG pin, persisted */
    uint32_t c = arg ? strtoul(arg, nullptr, 10) : 0;
    if (c == 50 || c == 100) { chg100 = (c == 100); applyChgCur(); cfgSave(); sendAck("CHGCUR"); }
    else sendNak("CHGCUR");
  }
  else if (!strcmp(cmd, "RINGN")) {
    /* RINGN:<1-60> — how many pixels are actually on the wires; drives every
     * animation revolution, the power model, and the app's simulator           */
    uint32_t n2 = arg ? strtoul(arg, nullptr, 10) : 0;
    if (n2 >= 1 && n2 <= 60) {
      ringN = (uint8_t)n2;
      ring.updateLength(ringN); ring.clear(); ring.show();
      cfgSave(); sendAck("RINGN");
    } else sendNak("RINGN");
  }
  else if (!strcmp(cmd, "RINGFIT")) {
    /* RINGFIT:0|1 — is a ring physically on the wires? (write-only line, we
     * can't sense it). 0 zeroes its power-model share + stops driving data.    */
    if (arg) { ringFit = (arg[0] != '0');
      if (!ringFit) { ring.clear(); ring.show(); digitalWrite(ringPin, LOW); }
      cfgSave(); sendAck("RINGFIT"); }
    else sendNak("RINGFIT");
  }
  else if (!strcmp(cmd, "RINGPWR")) {
    /* RINGPWR:<pin 0-10>[:AH] — GPIO gating the ring's power rail (V1 board /
     * bench MOSFET; AH = enable is active-high). RINGPWR:OFF removes the gate. */
    if (arg && !strncasecmp(arg, "OFF", 3)) { ringPower(true); ringPwrPin = 0xFF; cfgSave(); sendAck("RINGPWR"); }
    else if (arg) {
      uint32_t p = strtoul(arg, nullptr, 10);
      char* pol = strchr(arg, ':');
      bool clash = p > 10 || p == ringPin || p == 4 || p == 5 ||
                   (hxFound && (p == hxDout || p == hxSck));
      if (!clash) {
        ringPwrAH = (pol && !strncasecmp(pol + 1, "AH", 2)) ? 1 : 0;
        ringPwrPin = (uint8_t)p;
        ringPowered = false; ringPower(true);   /* claim the pin, drive rail ON */
        cfgSave(); sendAck("RINGPWR");
      } else sendNak("RINGPWR");
    } else sendNak("RINGPWR");
  }
  else if (!strcmp(cmd, "CELLOFF")) {
    /* CELLOFF:1 — completely disable the load cell (persisted): chip held in
     * power-down, no probing, simulated volume. CELLOFF:0 re-probes.           */
    if (arg && arg[0] == '1') {
      if (hxFound) hxSleep();           /* PD_SCK high = chip power-down          */
      else { pinMode(3, OUTPUT); digitalWrite(3, HIGH); }   /* known wiring: park it anyway */
      hxFound = false; cellAbsent = false; cellOff = 1;
      cfgSave(); sendAck("CELLOFF"); sendEvt("EVT|UNSETTLED");
      Serial.println("Load cell switched OFF from the app");
    } else if (arg && arg[0] == '0') {
      cellOff = 0; hxFound = false; cellAbsent = false; hxProbe();
      cfgSave(); sendAck("CELLOFF");
      sendEvt(hxFound ? "EVT|READY" : "EVT|UNSETTLED");
    } else sendNak("CELLOFF");
  }
  else if (!strcmp(cmd, "HXPROBE")) {
    /* full re-scan for the HX711 — the Dev tab's re-detect button (also clears
     * a CELLOFF, since re-detecting is an explicit request to use the cell)     */
    cellOff = 0; hxFound = false; cellAbsent = false; cellBadBursts = cellGoodBursts = 0;
    hxProbe();
    sendAck("HXPROBE");
    sendEvt(hxFound ? "EVT|READY" : "EVT|UNSETTLED");
  }
  else if (!strcmp(cmd, "BATTCAP")) {
    /* BATTCAP:<100-5000> — battery capacity mAh for runtime estimates, persisted */
    uint32_t c = arg ? strtoul(arg, nullptr, 10) : 0;
    if (c >= 100 && c <= 5000) { battCap = (uint16_t)c; cfgSave(); sendAck("BATTCAP"); }
    else sendNak("BATTCAP");
  }
  else if (!strcmp(cmd, "RESETDAY")) { doDayReset(); sendAck("RESETDAY"); }
  else if (!strcmp(cmd, "SHOW")) {
    sendAck("SHOW");
    static const uint32_t showPal[] = {0x46e0d2, 0xffd166, 0xff6ad5, 0x9a6cff, 0x22c55e};
    memcpy(fxCols, showPal, sizeof showPal); fxN = 5; fxIsShow = true;
    ovl = OV_FX; ovlStart = millis(); ovlUntil = millis() + 6000;
    hapticPlayPattern(50, 100, 2);
  }
  else if (!strcmp(cmd, "BUZZ")) {
    uint8_t n = arg ? (uint8_t)constrain(strtoul(arg, nullptr, 10), 1UL, 5UL) : 1;
    sendAck("BUZZ");
    hapQn = hapQi = 0;
    for (uint8_t i = 0; i < n; i++) hapticQueue(80, 60, 120);
    ledPulse(255, 255, 255, 120);
  }
  else if (!strcmp(cmd, "VIBE")) {
    /* VIBE:amp[:ms[:type]]  type: 0 single 1 double 2 triple 3 long 4 heartbeat 5 ramp */
    uint8_t amp = 60; uint16_t ms = 200; uint8_t type = 0;
    if (arg) {
      amp = (uint8_t)constrain(strtoul(arg, nullptr, 10), 1UL, 100UL);
      char* m = strchr(arg, ':');
      if (m) { ms = (uint16_t)constrain(strtoul(m + 1, nullptr, 10), 30UL, 2000UL);
        char* t = strchr(m + 1, ':');
        if (t) type = (uint8_t)constrain(strtoul(t + 1, nullptr, 10), 0UL, 5UL); }
    }
    sendAck("VIBE"); hapticPlayPattern(amp, ms, type);
  }
  else if (!strcmp(cmd, "GLOW")) {
    /* legacy command: untimed = set the always-on light SOLID; timed = overlay  */
    if (!arg) { sendNak("GLOW"); return; }
    if (!strncasecmp(arg, "AUTO", 4)) { ovl = OV_NONE; ovlUntil = 0; sendAck("GLOW"); }  /* end overlay */
    else if (!strncasecmp(arg, "OFF", 3)) { ovl = OV_NONE; pMode = PM_OFF; cfgMarkDirty(); sendAck("GLOW"); }
    else {
      char* ms = strchr(arg, ':'); if (ms) *ms++ = 0;
      uint32_t c = parseHexColor(arg);
      uint32_t dur = ms ? strtoul(ms, nullptr, 10) : 0;
      if (dur) { fxCols[0] = c; fxN = 1; fxIsShow = false;
        ovl = OV_FX; ovlStart = millis(); ovlUntil = millis() + dur; }
      else { pMode = PM_SOLID; animCols[0] = c; animN = 1; ovl = OV_NONE; cfgMarkDirty(); }
      sendAck("GLOW");
    }
  }
  else if (!strcmp(cmd, "LIGHT")) {
    /* legacy app effect — always a timed overlay; always-on light untouched     */
    if (!arg) { sendNak("LIGHT"); return; }
    char* mode = strchr(arg, ':'); if (mode) *mode++ = 0;
    char* ms = mode ? strchr(mode, ':') : nullptr; if (ms) *ms++ = 0;
    fxN = 0; fxIsShow = false;
    if (strchr(arg, '-')) {
      char* tok = strtok(arg, "-");
      while (tok && fxN < 6) { fxCols[fxN++] = parseHexColor(tok); tok = strtok(nullptr, "-"); }
    }
    if (fxN == 0) {
      static const uint32_t rain[] = {0xff0000, 0xff8800, 0xffff00, 0x00cc44, 0x2266ff, 0x9a6cff};
      memcpy(fxCols, rain, sizeof rain); fxN = 6;
    }
    uint32_t dur = ms ? strtoul(ms, nullptr, 10) : 4000;
    ovl = OV_FX; ovlStart = millis(); ovlUntil = millis() + (dur ? dur : 4000);
    sendAck("LIGHT");
  }
  else if (!strcmp(cmd, "LEDMODE")) {
    /* LEDMODE:<off|solid|gradient|rainbow|spin|pulse|breathe>[:<hex[-hex…]>]    */
    if (!arg) { sendNak("LEDMODE"); return; }
    char* cols = strchr(arg, ':'); if (cols) *cols++ = 0;
    uint8_t m;
    if      (!strncasecmp(arg, "OFF", 3))      m = PM_OFF;
    else if (!strncasecmp(arg, "SOLID", 5))    m = PM_SOLID;
    else if (!strncasecmp(arg, "GRADIENT", 8)) m = PM_GRADIENT;
    else if (!strncasecmp(arg, "RAINBOW", 7))  m = PM_RAINBOW;
    else if (!strncasecmp(arg, "SPIN", 4))     m = PM_SPIN;
    else if (!strncasecmp(arg, "PULSE", 5))    m = PM_PULSE;
    else if (!strncasecmp(arg, "BREATHE", 7))  m = PM_BREATHE;
    else if (!strncasecmp(arg, "ECOSPIN", 7))  m = PM_ECOSPIN;
    else { sendNak("LEDMODE"); return; }
    if (cols && cols[0] && cols[0] != '-') {
      uint8_t n = 0; uint32_t nc[6];
      char* tok = strtok(cols, "-");
      while (tok && n < 6) { nc[n++] = parseHexColor(tok); tok = strtok(nullptr, "-"); }
      if (n) { memcpy(animCols, nc, sizeof animCols); animN = n; }
    }
    pMode = m; ovl = OV_NONE;
    cfgSave(); sendAck("LEDMODE");
  }
  else if (!strcmp(cmd, "BRIGHT")) {
    uint32_t b = arg ? strtoul(arg, nullptr, 10) : 0;
    if (b >= 5 && b <= 100) { brightPct = b; cfgSave(); sendAck("BRIGHT"); }
    else sendNak("BRIGHT");
  }
  else if (!strcmp(cmd, "LIGHTPS")) {
    /* LIGHTPS:ON — always-on light shows only while the app is connected;
     * LIGHTPS:OFF — glows persistently until the battery drops below 5 %        */
    if      (arg && !strncasecmp(arg, "ON", 2))  lightSaver = true;
    else if (arg && !strncasecmp(arg, "OFF", 3)) lightSaver = false;
    else { sendNak("LIGHTPS"); return; }
    cfgSave(); sendAck("LIGHTPS");
  }
  else if (!strcmp(cmd, "REMLIGHT")) {
    /* REMLIGHT:<solid|rainbow|off>[:<hex>[:<breathe|strobe>]]                   */
    if (!arg) { sendNak("REMLIGHT"); return; }
    char* hex = strchr(arg, ':'); if (hex) *hex++ = 0;
    char* anim = hex ? strchr(hex, ':') : nullptr; if (anim) *anim++ = 0;
    if      (!strncasecmp(arg, "OFF", 3))     remLightC = 0;
    else if (!strncasecmp(arg, "RAINBOW", 7)) { remLMode = 2; if (!remLightC) remLightC = 0xffb45a; }
    else if (!strncasecmp(arg, "SOLID", 5)) {
      if (hex && hex[0]) remLightC = parseHexColor(hex);
      if (!remLightC) remLightC = 0xffb45a;
      remLMode = (anim && !strncasecmp(anim, "STROBE", 6)) ? 1 : 0;
    } else { sendNak("REMLIGHT"); return; }
    cfgSave(); sendAck("REMLIGHT");
  }
  else if (!strcmp(cmd, "RING")) {
    /* find-my-pod: red/white flashing; vibration for the first 3 s; ends after
     * 15 s, on RING:OFF, or the moment the pod is picked up                     */
    if (arg && !strncasecmp(arg, "OFF", 3)) { cancelRing("app"); sendAck("RING"); return; }
    ovl = OV_RING; ovlStart = millis(); ovlUntil = millis() + 15000;
    ringStartTilt = tiltDeg;
    hapQn = hapQi = 0; hapStop();
    for (uint8_t i = 0; i < 5; i++) hapticQueue(90, 300, 300);   /* ~3 s of buzzing */
    sendAck("RING");
  }
  else if (!strcmp(cmd, "REMCFG")) {
    /* REMCFG:<mins>:<fromMin>:<toMin>:<smart>:<hex|OFF>:<style 0-2>
     * mins=0 disables. Stored on the pod — reminders run WITHOUT the app.       */
    uint16_t m = 0, fr = remFrom, to = remTo; uint8_t sm = remSmart, st = remStyle; uint32_t lc = remLightC;
    bool ok = false;
    if (arg) {
      char* t1 = strchr(arg, ':');
      m = (uint16_t)strtoul(arg, nullptr, 10);
      if (t1) { fr = (uint16_t)strtoul(t1 + 1, nullptr, 10) % 1440;
        char* t2 = strchr(t1 + 1, ':');
        if (t2) { to = (uint16_t)strtoul(t2 + 1, nullptr, 10) % 1440;
          char* t3 = strchr(t2 + 1, ':');
          if (t3) { sm = (t3[1] == '1');
            char* t4 = strchr(t3 + 1, ':');
            if (t4) { lc = strncasecmp(t4 + 1, "OFF", 3) ? parseHexColor(t4 + 1) : 0;
              char* t5 = strchr(t4 + 1, ':');
              if (t5) { st = (uint8_t)constrain(strtoul(t5 + 1, nullptr, 10), 0UL, 2UL); ok = true; } } } } }
    }
    if (ok && m <= 720) {
      remMins = m; remFrom = fr; remTo = to; remSmart = sm; remStyle = st; remLightC = lc;
      lastNudgeMs = millis();           /* restart the round from now             */
      cfgSave(); sendAck("REMCFG");
      Serial.print("Reminders: "); Serial.print(remMins); Serial.println(remMins ? " min interval (on-pod)" : " — off");
    } else sendNak("REMCFG");
  }
  else if (!strcmp(cmd, "FACTORY")) {
    sendAck("FACTORY"); sendEvt("EVT|REBOOTING");
    InternalFS.remove("/pod.cfg"); InternalFS.remove("/siplog.bin");
    delay(300); NVIC_SystemReset();
  }
  else if (!strcmp(cmd, "RESTART")) {
    sendAck("RESTART"); sendEvt("EVT|REBOOTING");
    delay(300); NVIC_SystemReset();
  }
  else if (!strcmp(cmd, "UPDATE")) {
    sendAck("UPDATE"); sendEvt("EVT|DFU");
    delay(300);
    NRF_POWER->GPREGRET = 0x4E;
    NVIC_SystemReset();
  }
  else sendNak(cmd);
}

static void cmdWriteCb(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len)
{
  static char buf[72];
  len = min(len, (uint16_t)(sizeof buf - 1));
  memcpy(buf, data, len); buf[len] = 0;
  handleCommand(buf);
}

/* ------------------------------------------------------- USB serial console -- */
static void serialTick()
{
  static char line[64]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (n < sizeof line - 1) line[n++] = c; continue; }
    line[n] = 0; n = 0;
    if (!line[0]) continue;

    char* sp = strchr(line, ' ');
    long v = sp ? strtol(sp + 1, nullptr, 10) : -1;

    if      (!strncasecmp(line, "sip", 3))      doSip(v > 0 ? v : (uint32_t)random(20, 60), 1200);
    else if (!strncasecmp(line, "add", 3))      doAdd(v > 0 ? v : 250);
    else if (!strncasecmp(line, "dispose", 7))  doDispose(v > 0 ? v : 100, 165, 2000);
    else if (!strncasecmp(line, "tare", 4)) {
      if (hxFound && !cellAbsent) {
        Serial.println("taring — hold still...");
        calBusy = true; long r = hxSettled(); calBusy = false;
        if (r == LONG_MIN) Serial.println("reading never settled — zero unchanged");
        else { hxTare = r; cfgSave(); Serial.print("tared at raw "); Serial.println(hxTare); }
      }
      else { simVolMl = 0; Serial.println("tared (sim)"); }
    }
    else if (!strncasecmp(line, "calw", 4)) {
      if (!hxFound || cellAbsent) Serial.println("no live load cell");
      else if (v > 0) {
        Serial.println("calibrating — hold still...");
        calBusy = true; long r = hxSettled(); calBusy = false;
        if (r == LONG_MIN) { Serial.println("reading never settled — scale unchanged"); }
        else {
          float onCell = fabsf((float)(r - hxTare)) / (hxScale > 1 ? hxScale : CAL_DEFAULT_SCALE);
          if (onCell < CAL_MIN_DELTA_G) Serial.println("nothing on the cell — put the weight on first");
          else {
            float sc = (float)(r - hxTare) / (float)v;
            if (!calApply(hxTare, sc)) { Serial.print("rejected implausible scale "); Serial.println(sc, 2); }
          }
        }
      } else Serial.println("usage: calw <grams-on-cell>");
    }
    else if (!strncasecmp(line, "cal", 3) && (line[3] == 0 || line[3] == ' ')) {
      /* cal                    — print the live pair
       * cal <tare> <scale>     — pin a known calibration (post-erase restore)  */
      if (!sp) {
        Serial.print("cal tare="); Serial.print(hxTare);
        Serial.print(" scale=");   Serial.print(hxScale, 2);
        Serial.print(" counts/g");
        if (hxScale == CAL_DEFAULT_SCALE) Serial.print("  (FACTORY DEFAULT — not calibrated)");
        Serial.println();
      } else {
        char* sp2 = strchr(sp + 1, ' ');
        if (!sp2) Serial.println("usage: cal <tare> <scale>   (or 'cal' to read)");
        else {
          long  t = strtol(sp + 1, nullptr, 10);
          float s = strtod(sp2 + 1, nullptr);
          if (!calApply(t, s)) { Serial.print("rejected implausible scale "); Serial.println(s, 2); }
        }
      }
    }
    else if (!strncasecmp(line, "weigh", 5)) {
      if (hxFound) { long r = hxAverage(4);
        Serial.print("raw="); Serial.print(r);
        Serial.print(" tare="); Serial.print(hxTare);
        Serial.print(" scale="); Serial.print(hxScale);
        Serial.print(" cell="); Serial.print(cellAbsent ? "ABSENT" : "ok");
        Serial.print(" -> "); Serial.print(netMl()); Serial.println(" ml"); }
      else Serial.println("no HX711 — sim vol only");
    }
    else if (!strncasecmp(line, "happat", 6)) {
      uint8_t t = (v >= 0 && v <= 9) ? v : 0;
      Serial.print("pattern "); Serial.println(t);
      hapticPlayPattern(70, 150, t);
    }
    else if (!strncasecmp(line, "hapcfg", 6)) {
      if (v >= 10 && v <= 200) { hapStrength = v; cfgSave(); Serial.print("haptic strength "); Serial.print(v); Serial.println("%"); }
      else Serial.println("usage: hapcfg <10-200>");
    }
    else if (!strncasecmp(line, "hap", 3)) {
      uint8_t amp = (v > 0 && v <= 100) ? v : 60;
      char* sp2 = sp ? strchr(sp + 1, ' ') : nullptr;
      uint16_t ms = sp2 ? (uint16_t)strtol(sp2 + 1, nullptr, 10) : 300;
      hapticPlayPattern(amp, ms, 0);
    }
    else if (!strncasecmp(line, "ps", 2) && (line[2] == 0 || line[2] == ' ')) {
      if (sp) {
        if      (!strncasecmp(sp + 1, "auto", 4)) psMode = 0;
        else if (!strncasecmp(sp + 1, "on", 2))   psMode = 1;
        else if (!strncasecmp(sp + 1, "off", 3))  psMode = 2;
        cfgSave();
      }
      Serial.print("powersave mode="); Serial.print(psMode == 0 ? "AUTO" : psMode == 1 ? "ON" : "OFF");
      Serial.print(" active="); Serial.print(psActive);
      Serial.print(" usb="); Serial.println(usbPresent());
    }
    else if (!strncasecmp(line, "time", 4)) {
      if (epochBase) { uint32_t t = epochNow() % 86400UL;
        Serial.print("clock "); Serial.print(t / 3600); Serial.print(":"); Serial.print(t / 60 % 60); Serial.println(" (app-synced)"); }
      else Serial.println("clock not set (app sends SETTIME on connect)");
    }
    else if (!strncasecmp(line, "scan", 4)) { i2cScan(Wire, "external D4/D5"); i2cScan(Wire1, "internal"); }
    else if (!strncasecmp(line, "hxprobe", 7)) { hxFound = false; cellAbsent = false; hxProbe(); }
    else if (!strncasecmp(line, "ringpin", 7)) {
      if (v >= 0 && v <= 10 && v != 4 && v != 5) {
        ringPin = (uint8_t)v; ring.setPin(ringPin); cfgSave();
        Serial.print("ring pin -> D"); Serial.println(ringPin);
      } else Serial.println("usage: ringpin <0-10, not 4/5>");
    }
    else if (!strncasecmp(line, "ring", 4) || !strncasecmp(line, "led", 3)) {
      int r = 0, g = 0, b = 0;
      if (sp) sscanf(sp + 1, "%d %d %d", &r, &g, &b);
      ledPulse(r, g, b, 3000);
    }
    else if (!strncasecmp(line, "goal", 4))     { if (v > 0) { goalMl = v; cfgSave(); Serial.println("goal set"); } }
    else if (!strncasecmp(line, "day", 3))      doDayReset();
    else if (!strncasecmp(line, "out", 3))      setBottleIn(false);
    else if (!strncasecmp(line, "in", 2))       setBottleIn(true);
    else if (!strncasecmp(line, "unsettled", 9)) sendEvt("EVT|UNSETTLED");
    else if (!strncasecmp(line, "restart", 7))  { Serial.println("rebooting…"); delay(200); NVIC_SystemReset(); }
    else if (!strncasecmp(line, "bledrop", 7)) {
      /* force-drop a stuck central so advertising resumes                        */
      if (connected) { Bluefruit.disconnect(Bluefruit.connHandle()); Serial.println("dropped BLE link"); }
      else Serial.println("no central connected");
    }
    else if (!strncasecmp(line, "batt", 4)) {
      Serial.print("VBAT "); Serial.print(readVbat()); Serial.print(" V  ");
      Serial.println(isCharging() ? "CHARGING" : "not charging (full, no USB, or no battery)");
    }
    else if (!strncasecmp(line, "setname", 7)) {
      if (sp && sp[1]) {
        strncpy(devName, sp + 1, sizeof devName - 1); devName[sizeof devName - 1] = 0;
        cfgSave();
        Serial.print("name -> '"); Serial.print(devName); Serial.println("' — rebooting");
        delay(300); NVIC_SystemReset();
      } else Serial.println("usage: setname <new BLE name>");
    }
    else if (!strncasecmp(line, "blestat", 7)) {
      Serial.print("connected="); Serial.print(connected);
      if (connected) {
        BLEConnection* conn = Bluefruit.Connection(Bluefruit.connHandle());
        if (conn) { Serial.print(" mtu="); Serial.print(conn->getMtu()); }
      }
      Serial.println();
      Serial.print("notify live="); Serial.print(chrLive.notifyEnabled());
      Serial.print(" sip=");        Serial.print(chrSip.notifyEnabled());
      Serial.print(" evt=");        Serial.println(chrEvt.notifyEnabled());
    }
    else if (!strncasecmp(line, "pinv", 4)) {
      const uint8_t dp[] = {0, 1, 2, 3};
      const uint8_t an[] = {A0, A1, A2, A3};
      analogReadResolution(12);
      for (uint8_t i = 0; i < 4; i++) {
        if (hxFound && (dp[i] == hxDout || dp[i] == hxSck)) {
          Serial.print("D"); Serial.print(dp[i]); Serial.println(" = (in use by HX711)");
          continue;
        }
        pinMode(dp[i], INPUT_PULLUP); delay(2);
        uint32_t mv = (uint32_t)(analogRead(an[i]) * 3600UL / 4095UL);
        Serial.print("D"); Serial.print(dp[i]); Serial.print(" = ");
        Serial.print(mv); Serial.print(" mV");
        if (mv < 120) Serial.println("  (hard low — actively driven)");
        else if (mv < 900) Serial.println("  (diode clamp — chip on this line is UNPOWERED)");
        else if (mv > 2800) Serial.println("  (floating — follows pull-up)");
        else Serial.println();
        pinMode(dp[i], INPUT);
      }
    }
    else if (!strncasecmp(line, "debug", 5))    { dbgFrames = !dbgFrames; Serial.println(dbgFrames ? "debug ON" : "debug OFF"); }
    else if (!strncasecmp(line, "status", 6)) {
      Serial.print("vol=");   Serial.print((int)netMl());
      Serial.print(" sips="); Serial.print(sips);
      Serial.print(" tot=");  Serial.print(totMl);
      Serial.print(" goal="); Serial.print(goalMl);
      Serial.print(" tilt="); Serial.print((int)tiltDeg);
      Serial.print(" imu=");  Serial.print(imuOk ? "ok" : "OFF");
      Serial.print(" hx711=");Serial.print(!hxFound ? "SIM" : cellAbsent ? "NO-CELL" : "ok");
      if (hxFound) { Serial.print("(D"); Serial.print(hxDout); Serial.print("/D"); Serial.print(hxSck); Serial.print(")"); }
      Serial.print(" da7280=");Serial.print(hapFound ? "ok" : "NONE");
      Serial.print("@"); Serial.print(hapStrength); Serial.print("%");
      Serial.print(hapEnabled ? "" : "(disabled)");
      Serial.print(" ps=");   Serial.print(psMode == 0 ? "auto" : psMode == 1 ? "on" : "off");
      Serial.print(psActive ? "(ACTIVE)" : "");
      Serial.print(" bottle=");Serial.print(bottleIn ? "in" : "OUT/paused");
      Serial.print(" rem=");  if (remMins) { Serial.print(remMins); Serial.print("m ");
        Serial.print(remFrom / 60); Serial.print(":"); Serial.print(remFrom % 60);
        Serial.print("-"); Serial.print(remTo / 60); Serial.print(":"); Serial.print(remTo % 60);
        Serial.print(remSmart ? " smart" : ""); } else Serial.print("off");
      Serial.print(" lightSaver=");Serial.print(lightSaver);
      Serial.print(" hist=");  Serial.print(histSynced); Serial.print("/"); Serial.print(histCount);
      Serial.print(" led=");  Serial.print((int)pMode);
      Serial.print("@"); Serial.print(brightPct); Serial.print("%");
      Serial.print(" ble=");  Serial.print(connected ? "connected" : "advertising");
      Serial.print(" name='");Serial.print(devName);
      Serial.print("' up="); Serial.print(millis() / 1000); Serial.println("s");
    }
    else Serial.println("cmds: status blestat scan weigh tare calw cal hxprobe pinv hap happat hapcfg ps time ring ringpin setname sip add dispose goal day in out unsettled batt debug\n"
                        "  tare              zero the cell (waits for it to settle)\n"
                        "  calw <grams>      set scale against a reference mass on the cell\n"
                        "  cal               print the live tare/scale pair\n"
                        "  cal <tare> <sc>   pin a known calibration (restore after a chip erase)");
  }
}

/* --------------------------------------------------------------- BLE setup -- */
static void onConnect(uint16_t)    { connected = true;  introSent = false; Serial.println("BLE: app connected"); }
static void onDisconnect(uint16_t, uint8_t r)
{
  connected = false; introSent = false;
  if (bleDbg && (psActive || !bottleIn)) hxSleep();   /* the raw stream kept the cell awake */
  bleDbg = false;
  Serial.print("BLE: disconnected (0x"); Serial.print(r, HEX); Serial.print(") ");
  switch (r) {
    case 0x08: Serial.println("= supervision timeout: radio lost (range/interference/power dip)"); break;
    case 0x13: Serial.println("= the APP/central closed the link"); break;
    case 0x16: Serial.println("= we closed it (local host)"); break;
    case 0x22: Serial.println("= LMP/LL response timeout"); break;
    case 0x3D: Serial.println("= MIC failure"); break;
    case 0x3E: Serial.println("= connection failed to establish"); break;
    default:   Serial.println();
  }
}

static void bleSetup()
{
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.autoConnLed(false);
  Bluefruit.begin();
  Bluefruit.setTxPower(8);              /* full power — BLE runs full-time, never
                                         * battery-gated; link reliability beats the
                                         * ~0.3 mA a lower TX level would save     */
  Bluefruit.setName(devName);
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  ble_gap_conn_params_t cp;
  cp.min_conn_interval = 24;
  cp.max_conn_interval = 40;
  cp.slave_latency     = 0;
  cp.conn_sup_timeout  = 600;
  sd_ble_gap_ppcp_set(&cp);

  podSvc.begin();

  chrLive.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  chrLive.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrLive.setMaxLen(260); chrLive.begin();   /* matches the live-frame buffer, which grew for VOLD */

  chrSip.setProperties(CHR_PROPS_NOTIFY);
  chrSip.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrSip.setMaxLen(80); chrSip.begin();

  chrCmd.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  chrCmd.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  chrCmd.setMaxLen(72); chrCmd.begin();
  chrCmd.setWriteCallback(cmdWriteCb);

  chrEvt.setProperties(CHR_PROPS_NOTIFY);
  chrEvt.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrEvt.setMaxLen(120); chrEvt.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(podSvc);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

/* -------------------------------------------------------------------- main -- */
void setup()
{
  resetReas = NRF_POWER->RESETREAS;     /* why did we boot? capture + clear so   */
  NRF_POWER->RESETREAS = 0xFFFFFFFF;    /* the NEXT boot reads its own cause     */

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2500) {}

  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT); pinMode(LED_BLUE, OUTPUT);

  cfgLoad();
  applyChgCur();                             /* HICHG pin per the persisted charge current */
  ring.begin();
  if (ringN != RING_COUNT) ring.updateLength(ringN);   /* persisted pixel count */
  ring.clear(); ring.show();                 /* per-frame colours already carry brightPct */

#if HAS_ONBOARD_IMU
  #ifdef PIN_LSM6DS3TR_C_POWER
    pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
    digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
    delay(60);
  #endif
  imu.settings.gyroEnabled = 0;          /* gyro unused — saves ~0.5 mA          */
  imu.settings.accelSampleRate = 26;     /* we poll at 20 Hz — 52 Hz internal was waste */
  imuOk = (imu.begin() == 0);
#endif

  Wire.begin();
  Wire1.begin();
  i2cScan(Wire,  "external D4/D5");
  i2cScan(Wire1, "internal");
  if (!cellOff) hxProbe();
  else { pinMode(3, OUTPUT); digitalWrite(3, HIGH);   /* park a present chip in power-down */
         Serial.println("HX711: disabled by CELLOFF — simulated volume"); }
  histInit();
  bootGraceEnd = millis() + 120000UL;   /* no reminder in the first 2 min        */

  bleSetup();
  battTick();                           /* real battery sample NOW — the onboard LED
                                         * must not spend its first minute claiming
                                         * "full/green" off the 100% boot default   */

  Serial.println();
  Serial.print("ORQA pod firmware v6 — advertising as '"); Serial.print(devName); Serial.println("'");
  Serial.print("IMU:");     Serial.print(imuOk ? "ok" : "NO");
  Serial.print("  HX711:"); Serial.print(hxFound ? "ok" : "SIM");
  Serial.print("  DA7280:");Serial.print(hapFound ? "ok" : "NO");
  Serial.print("  ring D"); Serial.print(ringPin);
  Serial.print("  ps=");    Serial.println(psMode == 0 ? "auto" : psMode == 1 ? "on" : "off");
  Serial.println("Type 'help' for the console.");
}

void loop()
{
  static uint32_t nextLive = 0, nextImu = 0;
  uint32_t now = millis();

  /* paused AND alone: nothing consumes motion — poll the IMU at 5 Hz, not 20   */
  uint16_t imuPeriod = (!bottleIn && !connected) ? 200 : IMU_PERIOD_MS;
  if (now >= nextImu)  { nextImu = now + imuPeriod;  imuTick(); }

  if (connected && !introSent && chrEvt.notifyEnabled()) {
    introSent = true;
    sendEvt("EVT|CONNECTED");
    delay(30);
    sendEvt("EVT|READY");
  }

  if (now >= nextLive) { nextLive = now + LIVE_PERIOD_MS; sendLiveFrame(); sendDiagFrames(); }

  sendRawStream();
  devReprobeTick();
  serialTick();
  ledTick();
  hapticTick();
  cellHealthCheck();
  battTick();
  psTick();
  clockTick();
  reminderTick();
  histSyncTick();
  cfgDirtyTick();
  /* adaptive pacing: FreeRTOS sleeps the CPU inside delay() — 5 ms only while
   * light or haptics are actually running; everything else lives fine at 20 ms */
  delay(podBusy() ? 5 : 20);
}
