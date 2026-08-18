/*
 * orqa_power.h  —  power management for ORQA V1
 * ---------------------------------------------
 * This module is the heart of hitting the 5-7 day battery target.
 * It owns the two load switches (boost/LED rail and load cell) and the
 * IMU power gate, and enforces "everything off unless in use".
 *
 * Rails and their control (from the netlist):
 *   - 5V boost + LED ring : PIN_BOOST_EN   HIGH = ON   (default OFF at boot)
 *   - Load-cell excitation: PIN_LOADCELL_EN LOW = ON   (INVERTED; default OFF)
 *   - IMU (LSM6DS3)       : PIN_IMU_PWR    HIGH = ON   (default OFF)
 *   - 3V3 VDD (MCU rail)  : always on (cannot be switched)
 */
#ifndef ORQA_POWER_H
#define ORQA_POWER_H

#include <stdint.h>
#include <stdbool.h>

/* Call once at boot, before anything else, to force all switchable rails OFF. */
void power_init(void);

/* LED ring / 5V boost */
void led_rail_on(void);
void led_rail_off(void);
bool led_rail_is_on(void);

/* Load cell: power the bridge, allow it to settle, then the caller reads HX711. */
void loadcell_on(void);
void loadcell_off(void);

/* IMU power gate */
void imu_power_on(void);
void imu_power_off(void);

/* One complete, self-contained weigh cycle:
 *   powers the bridge -> waits settle_ms -> (caller-supplied read) -> powers off.
 * Returns the averaged raw reading. Keeps the bridge energised for the minimum time. */
int32_t weigh_once(uint16_t settle_ms, uint8_t samples);

/* Read battery millivolts. Enables the divider only for the measurement. */
uint16_t battery_millivolts(void);

/* True while USB is supplying charge (BQ25101 CHG line, active low). */
bool is_charging(void);

/* Enter System-ON deep sleep. Wakes on the IMU motion interrupt (PIN_IMU_INT1)
 * or the RTC. Everything switchable is left OFF. */
void enter_deep_sleep(void);

#endif /* ORQA_POWER_H */
