/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)
  Copyright (c) 2026, Inhero GmbH — GPREGRET2 danger zone logic

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

  Inhero MR-2 Variant — Early Boot & GPREGRET2 Danger Zone

  initVariant() runs before Meshtastic's setup(). It performs:
    1. GPIO initialization (LEDs, 3V3 rail)
    2. INA228 voltage check via One-Shot ADC (no driver init needed)
    3. GPREGRET2 danger-zone flag check:
       - Normal boot: threshold = 2800 mV
       - After danger-zone shutdown: threshold = 3200 mV (hysteresis)
    4. If voltage below threshold: SYSTEMOFF with GPREGRET2 saved
    5. If voltage OK: clear danger-zone flag, continue to Meshtastic
*/

#include "variant.h"
#include "nrf.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include <Wire.h>
#include <nrf_soc.h>

// Include INA228 driver for early boot voltage check
#include "Ina228Driver.h"

// GPREGRET2 shutdown reason codes (must match InheroMr2Module.h)
#define SHUTDOWN_REASON_LOW_VOLTAGE   0x01
#define GPREGRET2_IN_DANGER_ZONE      0x04

const uint32_t g_ADigitalPinMap[] = {
    // P0
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,

    // P1
    32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47};

void initVariant()
{
    // LED1 & LED2
    pinMode(PIN_LED1, OUTPUT);
    ledOff(PIN_LED1);

    pinMode(PIN_LED2, OUTPUT);
    ledOff(PIN_LED2);

    // 3V3 Power Rail
    pinMode(PIN_3V3_EN, OUTPUT);
    digitalWrite(PIN_3V3_EN, HIGH);

    // BQ25798 CE pin: Drive HIGH (charging disabled) on every boot.
    // External pull-up holds CE HIGH when RAK is unpowered, but we make the
    // safe state explicit. applyChemistryConfig() in InheroMr2Module will
    // pull CE LOW only after successful BQ I2C config with a known battery type.
#ifdef BQ_CE_PIN
    pinMode(BQ_CE_PIN, OUTPUT);
    digitalWrite(BQ_CE_PIN, HIGH);
#endif

    // Early boot voltage check via INA228
    // Prevents motorboating (boot-crash-reboot loop) when battery is critically low
    Wire.begin();
    delay(5); // Give I2C bus time to stabilize

    // Read GPREGRET2 to check if we woke from a danger-zone shutdown
    // GPREGRET2 survives System OFF wakes and warm resets
    uint32_t gpregret2 = NRF_POWER->GPREGRET2;

    uint16_t vbat_mv = Ina228Driver::readVBATDirect(&Wire, INA228_I2C_ADDR_DEFAULT);

    // If waking from a low-voltage shutdown (danger zone flag set),
    // require a higher voltage before allowing full boot
    uint16_t threshold_mv = 2800;
    if (gpregret2 & GPREGRET2_IN_DANGER_ZONE) {
        threshold_mv = 3200; // Require recovery before allowing boot
    }

    if (vbat_mv > 0 && vbat_mv < threshold_mv) {
        // Battery dangerously low - go directly to deep sleep
        // Turn off 3V3 rail and LEDs
        digitalWrite(PIN_3V3_EN, LOW);
        ledOff(PIN_LED1);
        ledOff(PIN_LED2);

        // Store low-voltage reason in GPREGRET2
        sd_power_gpregret_clr(1, 0xFF);
        sd_power_gpregret_set(1, SHUTDOWN_REASON_LOW_VOLTAGE | GPREGRET2_IN_DANGER_ZONE);

        // Enter System OFF mode (lowest power, ~1µA)
        // Will wake on USB connection (VBUS detect) or RTC alarm
        NRF_POWER->SYSTEMOFF = 1;
        __DSB();
        while (1)
            ; // Should never reach here
    }

    // Clear danger zone flag on successful boot
    if (gpregret2 & GPREGRET2_IN_DANGER_ZONE) {
        sd_power_gpregret_clr(1, GPREGRET2_IN_DANGER_ZONE);
    }
}
