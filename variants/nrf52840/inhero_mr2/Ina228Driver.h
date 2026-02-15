/*
 * Copyright (c) 2026 Inhero GmbH
 *
 * SPDX-License-Identifier: MIT
 *
 * INA228 Power Monitor Driver for Inhero MR-2
 * Ported from MeshCore to Meshtastic
 *
 * Features:
 * - Voltage, current, power monitoring (24-bit ADC)
 * - Coulomb counter (accumulated charge)
 * - Alert pin for hardware UVLO (under-voltage lockout)
 * - Early boot static voltage read (readVBATDirect)
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

// INA228 I2C Address
#define INA228_I2C_ADDR_DEFAULT 0x40

// INA228 Register Map
#define INA228_REG_CONFIG 0x00
#define INA228_REG_ADC_CONFIG 0x01
#define INA228_REG_SHUNT_CAL 0x02
#define INA228_REG_SHUNT_TEMP 0x03
#define INA228_REG_VSHUNT 0x04
#define INA228_REG_VBUS 0x05
#define INA228_REG_DIETEMP 0x06
#define INA228_REG_CURRENT 0x07
#define INA228_REG_POWER 0x08
#define INA228_REG_ENERGY 0x09
#define INA228_REG_CHARGE 0x0A
#define INA228_REG_DIAG_ALRT 0x0B
#define INA228_REG_SOVL 0x0C
#define INA228_REG_SUVL 0x0D
#define INA228_REG_BOVL 0x0E
#define INA228_REG_BUVL 0x0F
#define INA228_REG_TEMP_LIMIT 0x10
#define INA228_REG_PWR_LIMIT 0x11
#define INA228_REG_MANUFACTURER 0x3E
#define INA228_REG_DEVICE_ID 0x3F

// INA228 Configuration bits
#define INA228_CONFIG_RST (1 << 15)
#define INA228_CONFIG_ADCRANGE (1 << 4) // 0=±163.84mV, 1=±40.96mV

// ADC Configuration - Mode
#define INA228_ADC_MODE_CONT_ALL 0x0F

// ADC Configuration - Averaging
#define INA228_ADC_AVG_1 0x00
#define INA228_ADC_AVG_4 0x01
#define INA228_ADC_AVG_16 0x02
#define INA228_ADC_AVG_64 0x03
#define INA228_ADC_AVG_128 0x04
#define INA228_ADC_AVG_256 0x05
#define INA228_ADC_AVG_512 0x06
#define INA228_ADC_AVG_1024 0x07

// ADC Configuration - Conversion Time
#define INA228_ADC_CT_50us 0x00
#define INA228_ADC_CT_84us 0x01
#define INA228_ADC_CT_150us 0x02
#define INA228_ADC_CT_280us 0x03
#define INA228_ADC_CT_540us 0x04
#define INA228_ADC_CT_1052us 0x05
#define INA228_ADC_CT_2074us 0x06
#define INA228_ADC_CT_4120us 0x07

// Alert Configuration
#define INA228_DIAG_ALRT_ALATCH (1 << 15)
#define INA228_DIAG_ALRT_CNVR (1 << 14)
#define INA228_DIAG_ALRT_SLOWALERT (1 << 13)
#define INA228_DIAG_ALRT_APOL (1 << 12)
#define INA228_DIAG_ALRT_ENERGYOF (1 << 11)
#define INA228_DIAG_ALRT_CHARGEOF (1 << 10)
#define INA228_DIAG_ALRT_MATHOF (1 << 9)
#define INA228_DIAG_ALRT_TMPOL (1 << 7)
#define INA228_DIAG_ALRT_SHNTOL (1 << 6)
#define INA228_DIAG_ALRT_SHNTUL (1 << 5)
#define INA228_DIAG_ALRT_BUSOL (1 << 4)
#define INA228_DIAG_ALRT_BUSUL (1 << 3)
#define INA228_DIAG_ALRT_POL (1 << 2)
#define INA228_DIAG_ALRT_CNVRF (1 << 1)
#define INA228_DIAG_ALRT_MEMSTAT (1 << 0)

/// Battery telemetry from INA228
typedef struct {
    uint16_t voltage_mv;  ///< Battery voltage in mV
    int16_t current_ma;   ///< Battery current in mA (+ = charging, - = discharging)
    int32_t power_mw;     ///< Battery power in mW
    int32_t energy_mwh;   ///< Accumulated energy in mWh (since last reset)
    float charge_mah;     ///< Accumulated charge in mAh (since last reset)
    float die_temp_c;     ///< Die temperature in °C
} Ina228BatteryData;

class Ina228Driver
{
  public:
    Ina228Driver(uint8_t i2c_addr = INA228_I2C_ADDR_DEFAULT);

    bool begin(float shunt_resistor_mohm = 10.0f);
    bool isConnected();
    void reset();

    // Readings
    uint16_t readVoltage_mV();
    int16_t readCurrent_mA();
    float readCurrent_mA_precise();
    int32_t readPower_mW();
    int32_t readEnergy_mWh();
    float readCharge_mAh();
    float readDieTemperature_C();
    bool readAll(Ina228BatteryData *data);

    // Coulomb Counter
    void resetCoulombCounter();

    // Alerts
    bool setUnderVoltageAlert(uint16_t voltage_mv);
    bool setOverVoltageAlert(uint16_t voltage_mv);
    void enableAlert(bool enable_uvlo = true, bool active_high = false, bool latch_alert = false);
    bool isAlertActive();
    void clearAlert();
    uint16_t getDiagnosticFlags();

    // Power management
    void shutdown();
    void wakeup();

    // Calibration
    float calibrateCurrent(float actual_current_ma);
    void setCalibrationFactor(float factor);
    float getCalibrationFactor() const;

    /// Read battery voltage directly without driver init (for early boot)
    static uint16_t readVBATDirect(TwoWire *wire = &Wire, uint8_t i2c_addr = INA228_I2C_ADDR_DEFAULT);

  private:
    uint8_t _i2c_addr;
    float _shunt_mohm;
    float _current_lsb;
    uint16_t _base_shunt_cal;
    float _calibration_factor;

    bool writeRegister16(uint8_t reg, uint16_t value);
    uint16_t readRegister16(uint8_t reg);
    int32_t readRegister24(uint8_t reg);
    int64_t readRegister40(uint8_t reg);
};
