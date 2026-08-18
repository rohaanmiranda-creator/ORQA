/*
 * orqa_main.ino  —  ORQA V1 firmware skeleton (Adafruit nRF52 core)
 * ----------------------------------------------------------------
 * Boots low-power, advertises BLE, sleeps, wakes on motion, weighs, updates
 * the app, and drives light/haptic feedback. Peripheral driver bodies are
 * stubs — fill them against the device datasheets.
 *
 * Board: Tools -> Board -> "Nordic nRF52840 (PCA10056)" or an Adafruit nRF52840
 * variant. Verify the LFCLK source is the 32.768 kHz crystal (LFXO).
 */
#include <bluefruit.h>
#include <Adafruit_NeoPixel.h>
#include "orqa_board.h"
#include "orqa_power.h"

/* ---- BLE: custom ORQA Hydration Service (128-bit UUIDs — replace with yours) ---- */
static const uint8_t ORQA_SVC_UUID[16] = {
  0x00,0x51,0xAA,0xBB, 0xCC,0xDD, 0xEE,0xFF, 0x10,0x20, 0x30,0x40,0x50,0x60,0x70,0x80
};
BLEService        orqaSvc(ORQA_SVC_UUID);
BLECharacteristic chrWaterLevel;   /* notify: current volume (ml)          */
BLECharacteristic chrBattery;      /* read/notify: battery %               */
BLECharacteristic chrGoalDaily;    /* read/write: daily goal (ml)          */
BLECharacteristic chrLedControl;   /* write: ring pattern / colour         */
BLECharacteristic chrHapticTrig;   /* write: play a haptic effect          */

Adafruit_NeoPixel ring(RGB_LED_COUNT, PIN_RGB_DIN, NEO_GRB + NEO_KHZ800);

/* ---- app state ---- */
static uint32_t g_dailyGoal_ml = 2500;
static uint32_t g_intakeToday_ml = 0;
static int32_t  g_lastWeightRaw = 0;

/* ================= setup ================= */
void setup()
{
    power_init();               /* FIRST: force LED rail, load cell, IMU all OFF */

    // flash_init();            /* W25Q128 over QSPI: init, mount log store       */
    // imu_init();              /* power IMU, configure wake-on-motion on INT1    */
    // haptic_init();           /* DA7280 over I2C                                */

    bleSetup();
    ring.begin();               /* data pin configured; ring stays dark (rail off) */

    /* Calibrate tare once on a known-empty bottle, store in flash. */
    // loadcell_tare();
}

/* ================= main loop ================= */
void loop()
{
    /* 1) Wake reason handled by the IMU interrupt or the periodic RTC tick. */

    /* 2) Take a weight sample only when it makes sense (bottle still, set down). */
    int32_t raw = weigh_once(/*settle_ms=*/400, /*samples=*/8);
    uint32_t volume_ml = weightRawToMl(raw);

    /* 3) Detect a drink: a sustained drop in volume = intake logged. */
    if (detectDrink(raw)) {
        uint32_t sip = lastVolume_ml() - volume_ml;
        g_intakeToday_ml += sip;
        // flash_logDrink(sip, rtcNow());
        notifyWaterLevel(volume_ml);
        confirmSip();           /* short green pulse + soft haptic */
    }

    /* 4) Reminder if behind goal and past the quiet-hours window. */
    if (behindSchedule() && !inQuietHours()) {
        remindToDrink();        /* amber ring breathe + haptic buzz */
    }

    /* 5) Battery housekeeping. */
    static uint32_t lastBatt = 0;
    if (millis() - lastBatt > 60000UL) {
        notifyBattery(battery_millivolts(), is_charging());
        lastBatt = millis();
    }

    /* 6) Nothing to do -> everything off, sleep until motion or next tick. */
    enter_deep_sleep();
}

/* ================= BLE ================= */
void bleSetup()
{
    Bluefruit.begin();
    Bluefruit.setName("ORQA Bottle");
    Bluefruit.setTxPower(4);          /* 5 m target -> low TX power is fine */

    orqaSvc.begin();

    chrWaterLevel.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    chrWaterLevel.setFixedLen(4);  chrWaterLevel.begin();

    chrBattery.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    chrBattery.setFixedLen(1);     chrBattery.begin();

    chrGoalDaily.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    chrGoalDaily.setFixedLen(4);   chrGoalDaily.begin();
    chrGoalDaily.setWriteCallback(onGoalWrite);

    chrLedControl.setProperties(CHR_PROPS_WRITE);
    chrLedControl.setMaxLen(8);    chrLedControl.begin();
    chrLedControl.setWriteCallback(onLedWrite);

    chrHapticTrig.setProperties(CHR_PROPS_WRITE);
    chrHapticTrig.setFixedLen(1);  chrHapticTrig.begin();
    chrHapticTrig.setWriteCallback(onHapticWrite);

    Bluefruit.Advertising.addService(orqaSvc);
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(244, 1600); /* fast then slow, saves power */
    Bluefruit.Advertising.start(0);
}

void notifyWaterLevel(uint32_t ml) { chrWaterLevel.notify(&ml, 4); }
void notifyBattery(uint16_t mv, bool charging) {
    uint8_t pct = battPercent(mv); chrBattery.notify(&pct, 1);
}
void onGoalWrite(uint16_t, BLECharacteristic*, uint8_t* d, uint16_t) { memcpy(&g_dailyGoal_ml, d, 4); }
void onLedWrite (uint16_t, BLECharacteristic*, uint8_t* d, uint16_t len) { playLedPattern(d, len); }
void onHapticWrite(uint16_t, BLECharacteristic*, uint8_t* d, uint16_t) { hapticPlay(d[0]); }

/* ================= feedback helpers ================= */
void confirmSip() {
    led_rail_on();
    for (int i=0;i<RGB_LED_COUNT;i++) ring.setPixelColor(i, ring.Color(0,60,0));
    ring.show(); delay(150);
    ring.clear(); ring.show();
    led_rail_off();                 /* critical: kill the rail again */
    // hapticPlay(EFFECT_SOFT_TICK);
}
void remindToDrink() {
    led_rail_on();
    /* amber breathe */
    ring.clear(); ring.show();
    led_rail_off();
    // hapticPlay(EFFECT_BUZZ);
}
void playLedPattern(uint8_t*, uint16_t) { /* TODO map app payload -> ring effect */ }

/* ================= stubs to implement ================= */
uint32_t weightRawToMl(int32_t raw) { return 0; }   /* apply tare + calibration slope */
bool     detectDrink(int32_t raw)   { return false; }
uint32_t lastVolume_ml()            { return 0; }
bool     behindSchedule()           { return false; }
bool     inQuietHours()             { return false; }
uint8_t  battPercent(uint16_t mv)   { return 0; }
uint8_t  battFromMv(uint16_t mv)    { return 0; }
void     hapticPlay(uint8_t effect) { /* DA7280 register write */ }
