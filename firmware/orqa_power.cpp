/*
 * orqa_power.cpp  —  power management implementation (scaffold)
 * ------------------------------------------------------------
 * GPIO uses the nrf_gpio HAL (absolute pin numbers) so it is correct on the
 * Adafruit nRF52 Arduino core AND on bare nRF SDK / Zephyr with minor edits.
 * The HX711 read and SAADC read are marked TODO — wire them to your driver.
 */
#include "orqa_board.h"
#include "orqa_power.h"
#include <nrf_gpio.h>

#if defined(ARDUINO)
  #include <Arduino.h>
  #define DELAY_MS(x) delay(x)
#else
  #include "nrf_delay.h"
  #define DELAY_MS(x) nrf_delay_ms(x)
#endif

static bool s_led_on = false;

void power_init(void)
{
    /* Boost/LED rail OFF (drive low). */
    nrf_gpio_cfg_output(PIN_BOOST_EN);
    nrf_gpio_pin_clear(PIN_BOOST_EN);
    s_led_on = false;

    /* Load cell OFF. Inverted logic -> drive HIGH to keep the P-MOSFET off. */
    nrf_gpio_cfg_output(PIN_LOADCELL_EN);
    nrf_gpio_pin_set(PIN_LOADCELL_EN);

    /* IMU OFF (drive low). */
    nrf_gpio_cfg_output(PIN_IMU_PWR);
    nrf_gpio_pin_clear(PIN_IMU_PWR);

    /* HX711 held in power-down (PD_SCK high) until a reading is wanted. */
    nrf_gpio_cfg_output(PIN_HX711_SCK);
    nrf_gpio_pin_set(PIN_HX711_SCK);
    nrf_gpio_cfg_input(PIN_HX711_DOUT, NRF_GPIO_PIN_NOPULL);

    /* Battery divider disabled until needed. */
    nrf_gpio_cfg_output(PIN_VBAT_SENSE_EN);
    nrf_gpio_pin_clear(PIN_VBAT_SENSE_EN);

    /* Charger status is an input. */
    nrf_gpio_cfg_input(PIN_CHG_STATUS, NRF_GPIO_PIN_PULLUP);
}

/* ---- LED ring / 5V boost ---- */
void led_rail_on(void)
{
    nrf_gpio_pin_set(PIN_BOOST_EN);   /* enable boost + connect its input */
    DELAY_MS(2);                      /* let 5V rise before driving SK6812 data */
    s_led_on = true;
}
void led_rail_off(void)
{
    nrf_gpio_pin_clear(PIN_BOOST_EN); /* 5V collapses to 0V -> all 14 LEDs dead */
    s_led_on = false;
}
bool led_rail_is_on(void) { return s_led_on; }

/* ---- Load cell ---- */
void loadcell_on(void)  { nrf_gpio_pin_clear(PIN_LOADCELL_EN); }  /* LOW = ON  */
void loadcell_off(void) { nrf_gpio_pin_set(PIN_LOADCELL_EN);   }  /* HIGH = OFF */

/* ---- IMU power gate ---- */
void imu_power_on(void)  { nrf_gpio_pin_set(PIN_IMU_PWR);   DELAY_MS(5); } /* boot time */
void imu_power_off(void) { nrf_gpio_pin_clear(PIN_IMU_PWR); }

/* ---- One weigh cycle: power only for as long as the reading takes ---- */
int32_t weigh_once(uint16_t settle_ms, uint8_t samples)
{
    loadcell_on();
    DELAY_MS(settle_ms);              /* bridge + HX711 settle (~50ms @80SPS, ~400ms @10SPS) */

    int64_t acc = 0;
    for (uint8_t i = 0; i < samples; i++) {
        /* TODO: acc += hx711_read_raw();  // bit-bang PD_SCK/DOUT, 25 pulses, gain 128 */
    }

    loadcell_off();
    return (samples > 0) ? (int32_t)(acc / samples) : 0;
}

/* ---- Battery voltage ---- */
uint16_t battery_millivolts(void)
{
    nrf_gpio_pin_set(PIN_VBAT_SENSE_EN);   /* enable divider */
    DELAY_MS(1);
    uint16_t mv = 0;
    /* TODO: read SAADC on AIN7 (VBAT_ADC_AIN), then mv = raw * divider_ratio. */
    nrf_gpio_pin_clear(PIN_VBAT_SENSE_EN); /* disable divider to stop its leakage */
    return mv;
}

bool is_charging(void)
{
    /* CHG is active-low from the BQ25101 while charging. */
    return nrf_gpio_pin_read(PIN_CHG_STATUS) == 0;
}

/* ---- Deep sleep ---- */
void enter_deep_sleep(void)
{
    led_rail_off();
    loadcell_off();
    /* Leave the IMU powered ONLY if you need wake-on-motion; otherwise: */
    /* imu_power_off(); */

    /* Configure PIN_IMU_INT1 as a sense/wake source, then enter System-ON idle.
     * On the Adafruit core: attachInterrupt(...) + __WFE()/waitForEvent().
     * On nRF SDK/Zephyr: sd_app_evt_wait() / k_sleep with the GPIOTE PORT event. */
#if defined(ARDUINO)
    waitForEvent();
#else
    /* sd_app_evt_wait(); */
#endif
}
