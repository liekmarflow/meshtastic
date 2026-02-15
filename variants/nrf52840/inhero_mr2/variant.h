/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_INHERO_MR2_
#define _VARIANT_INHERO_MR2_

#ifndef INHERO_MR2
#define INHERO_MR2
#endif

/** Master clock frequency */
#define VARIANT_MCK (64000000ul)

#define USE_LFXO // Board uses 32khz crystal for LF
// define USE_LFRC    // Board uses RC for LF

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT (48)
#define NUM_DIGITAL_PINS (48)
#define NUM_ANALOG_INPUTS (6)
#define NUM_ANALOG_OUTPUTS (0)

// LEDs
#define PIN_LED1 (35)
#define PIN_LED2 (36)

#define LED_BLUE PIN_LED1  // P1.03
#define LED_RED PIN_LED2   // P1.04

#define LED_STATE_ON 1 // State when LED is litted

/*
 * Buttons
 */
// No user buttons on Inhero MR-2

/*
 * Analog pins
 */
#define PIN_A0 (5)
#define PIN_A1 (31)
#define PIN_A2 (28)
#define PIN_A3 (29)
#define PIN_A4 (30)
#define PIN_A5 (31)
#define PIN_A6 (0xff)
#define PIN_A7 (0xff)

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;
#define ADC_RESOLUTION 14

// Other pins
#define PIN_AREF (2)
#define PIN_NFC1 (9)
#define PIN_NFC2 (10)

static const uint8_t AREF = PIN_AREF;

/*
 * Serial interfaces
 */
// TXD1 RXD1 on Base Board
#define PIN_SERIAL1_RX (15)
#define PIN_SERIAL1_TX (16)

// TXD0 RXD0 on Base Board
#define PIN_SERIAL2_RX (19)
#define PIN_SERIAL2_TX (20)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO (45)
#define PIN_SPI_MOSI (44)
#define PIN_SPI_SCK (43)

static const uint8_t SS = 42;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA (13)
#define PIN_WIRE_SCL (14)

#define PIN_WIRE1_SDA (24)
#define PIN_WIRE1_SCL (25)

// QSPI Pins
#define PIN_QSPI_SCK 3
#define PIN_QSPI_CS 26
#define PIN_QSPI_IO0 30
#define PIN_QSPI_IO1 29
#define PIN_QSPI_IO2 28
#define PIN_QSPI_IO3 2

// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES IS25LP080D
#define EXTERNAL_FLASH_USE_QSPI

/* Inhero MR-2 LoRa radio (SX1262 on RAK4630 module)
 *
 * P1.10   NSS     SPI NSS (Arduino GPIO number 42)
 * P1.11   SCK     SPI CLK (Arduino GPIO number 43)
 * P1.12   MOSI    SPI MOSI (Arduino GPIO number 44)
 * P1.13   MISO    SPI MISO (Arduino GPIO number 45)
 * P1.14   BUSY    BUSY signal (Arduino GPIO number 46)
 * P1.15   DIO1    DIO1 event interrupt (Arduino GPIO number 47)
 * P1.06   NRESET  NRESET manual reset of the SX1262 (Arduino GPIO number 38)
 * P1.05   POWER_EN  Power enable for SX1262 (Arduino GPIO number 37)
 *
 * DIO2 controls the antenna switch
 * DIO3 controls the TCXO power supply (1.8V)
 */

#define USE_SX1262
#define SX126X_CS (42)
#define SX126X_DIO1 (47)
#define SX126X_BUSY (46)
#define SX126X_RESET (38)
#define SX126X_POWER_EN (37)
// DIO2 controls an antenna switch and the TCXO voltage is controlled by DIO3
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8

// Testing USB detection
#define NRF_APM

// enables 3.3V periphery like GPS or IO Module
#define PIN_3V3_EN (34)
#define WB_IO2 PIN_3V3_EN

// I2C addresses for onboard peripherals
#define RV3028_RTC (uint8_t)0b1010010 // RV-3028-C7 RTC at 0x52

// GPS module support (active on Serial1)
#define GPS_RX_PIN PIN_SERIAL1_RX
#define GPS_TX_PIN PIN_SERIAL1_TX

// Battery
// The battery sense is hooked to pin A0 (5)
#define BATTERY_PIN PIN_A0
// and has 12 bit resolution
#define BATTERY_SENSE_RESOLUTION_BITS 12
#define BATTERY_SENSE_RESOLUTION 4096.0
#undef AREF_VOLTAGE
#define AREF_VOLTAGE 3.0
#define VBAT_AR_INTERNAL AR_INTERNAL_3_0
#define ADC_MULTIPLIER (1.73)

// RAK4630-based: AIN0 = nrf52840 AIN3 = Pin 5
#define BATTERY_LPCOMP_INPUT NRF_LPCOMP_INPUT_3
#define BATTERY_LPCOMP_THRESHOLD NRF_LPCOMP_REF_SUPPLY_11_16

// SSD1306 OLED display support
#define HAS_SCREEN 1
#define USE_SSD1306

// Buzzer on IO3 (Slot C)
#define PIN_BUZZER 21

// Onboard sensor configuration
#define HAS_TELEMETRY 1
#define HAS_SENSOR 1

// INA228 power monitor on I2C (0x40) - handled by InheroMr2Module
// BQ25798 solar charger on I2C (0x6B) - handled by InheroMr2Module
// RV-3028 RTC on I2C (0x52) - handled by Meshtastic RTC infrastructure

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
