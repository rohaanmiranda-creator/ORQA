/*
 * orqa_board.h  —  ORQA Prototype V1 board definition
 * -----------------------------------------------------
 * Pin map generated from the V1 KiCad netlist (ORQA Prototype v1 PCB.kicad_pcb)
 * and verified against the schematic symbol pin names.
 *
 * MCU: Nordic nRF52840 (aQFN73).  Pins are given as ABSOLUTE nRF GPIO numbers
 * via NRF_GPIO_PIN_MAP(port, pin) so they are unambiguous regardless of the
 * Arduino core's pin map.  Use the nrf_gpio_* HAL calls with these, or convert
 * for libraries that expect an Arduino pin index.
 *
 * STATUS: starter scaffold. Assignments are correct; items marked CONFIRM
 * depend on datasheet pinouts and should be checked before writing drivers.
 */
#ifndef ORQA_BOARD_H
#define ORQA_BOARD_H

#include <nrf_gpio.h>

/* ---- Power control (the two switches added in the V1 fix; see handover §4) ---- */
#define PIN_BOOST_EN      NRF_GPIO_PIN_MAP(1,12)  /* P1.12  HIGH = 5V boost + LED ring ON.  Default LOW. */
#define PIN_LOADCELL_EN   NRF_GPIO_PIN_MAP(1,10)  /* P1.10  INVERTED: LOW = load cell ON, HIGH = OFF.   */
#define PIN_IMU_PWR       NRF_GPIO_PIN_MAP(1,8)   /* P1.08  6D_PWR: HIGH powers the IMU directly.       */

/* ---- RGB ring: 14 x SK6812 (side view) on the 5V rail ---- */
#define PIN_RGB_DIN       NRF_GPIO_PIN_MAP(1,13)  /* P1.13 -> D3 data-in, chained D3..D14 */
#define RGB_LED_COUNT     14

/* ---- Weight: HX711 (bit-banged) ---- */
#define PIN_HX711_SCK     NRF_GPIO_PIN_MAP(0,29)  /* P0.29  PD_SCK (also powers down HX711 when held high) */
#define PIN_HX711_DOUT    NRF_GPIO_PIN_MAP(0,28)  /* P0.28  DOUT / data ready (active low) */

/* ---- Shared I2C bus: IMU (LSM6DS3) + Haptic (DA7280) ---- */
#define PIN_I2C_SCL       NRF_GPIO_PIN_MAP(0,27)  /* P0.27  INTERNAL_I2C_SCL */
#define PIN_I2C_SDA       NRF_GPIO_PIN_MAP(0,7)   /* P0.07  INTERNAL_I2C_SDA */
#define I2C_ADDR_IMU      0x6A                    /* LSM6DS3TR-C, SDO/SA0 low (CONFIRM strap) */
#define I2C_ADDR_HAPTIC   0x4A                    /* DA7280 default (CONFIRM) */

/* ---- IMU extra lines ---- */
#define PIN_IMU_INT1      NRF_GPIO_PIN_MAP(0,11)  /* P0.11  6D_INT1 (motion / wake interrupt) */

/* ---- Haptic (DA7280) ---- */
#define PIN_HAPTIC_IRQ    NRF_GPIO_PIN_MAP(1,11)  /* P1.11  NIRQ (active low) */
/* NOTE: DA7280 SDA/SCL appear on nets P0.04/P0.05 in addition to the shared bus.
 * CONFIRM on the schematic whether the haptic sits on the P0.27/P0.07 bus or on
 * a second bus at P0.04(SDA)/P0.05(SCL) before instantiating its driver.        */
#define PIN_HAPTIC_SDA_ALT NRF_GPIO_PIN_MAP(0,4)  /* CONFIRM */
#define PIN_HAPTIC_SCL_ALT NRF_GPIO_PIN_MAP(0,5)  /* CONFIRM */

/* ---- 16 MB QSPI flash: W25Q128 ---- */
#define PIN_QSPI_SCK      NRF_GPIO_PIN_MAP(0,21)
#define PIN_QSPI_CSN      NRF_GPIO_PIN_MAP(0,25)
#define PIN_QSPI_IO0      NRF_GPIO_PIN_MAP(0,20)
#define PIN_QSPI_IO1      NRF_GPIO_PIN_MAP(0,24)
#define PIN_QSPI_IO2      NRF_GPIO_PIN_MAP(0,22)
#define PIN_QSPI_IO3      NRF_GPIO_PIN_MAP(0,23)

/* ---- Battery / charger (BQ25101) ---- */
#define PIN_VBAT_ADC      NRF_GPIO_PIN_MAP(0,31)  /* P0.31 / AIN7 — battery voltage through divider */
#define PIN_VBAT_SENSE_EN NRF_GPIO_PIN_MAP(0,14)  /* P0.14  READ_BAT — enables the divider only when measuring */
#define PIN_CHG_STATUS    NRF_GPIO_PIN_MAP(0,17)  /* P0.17  CHG (charge status from BQ25101, active low) */
#define PIN_CHG_HICHG     NRF_GPIO_PIN_MAP(0,13)  /* P0.13  HICHG (selects high charge current) */
#define VBAT_ADC_AIN      7                        /* AIN7 for the SAADC */

/* ---- Low-frequency clock: 32.768 kHz crystal Y2 on P0.00/P0.01 (use LFXO) ---- */

/* ---- Items to confirm before use ---- */
/* Buzzer (MLT-8530): driven by NPN Q3; base net not yet mapped to a GPIO. CONFIRM. */
/* Reed switch: brought out on test points TP1/TP2; assign/confirm a GPIO.          */
/* Button S1: wired to the nRF RESET line (P0.18), NOT a normal GPIO. As drawn a    */
/*   press RESETS the MCU. To read it in firmware, reconfigure P0.18 as GPIO in     */
/*   UICR (disables hardware reset) — decide this deliberately.                     */
/* Spare GPIOs available: P0.06, P0.26, P0.30 (USER RGB, to test points),           */
/*   P0.08, P0.15, P0.19, P1.00-P1.07.                                              */

#endif /* ORQA_BOARD_H */
