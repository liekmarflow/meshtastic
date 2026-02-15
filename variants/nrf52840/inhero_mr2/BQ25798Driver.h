/*
 * Copyright (c) 2026 Inhero GmbH
 *
 * SPDX-License-Identifier: MIT
 *
 * BQ25798 Solar Charger Driver for Inhero MR-2
 * Ported from MeshCore to Meshtastic
 *
 * Extends Adafruit_BQ25798 with:
 * - NTC thermistor temperature calculation (Steinhart-Hart)
 * - JEITA temperature control
 * - Complete ADC telemetry (solar, battery, system)
 * - One-shot ADC conversion management
 */

#pragma once

#include <Adafruit_BQ25798.h>
#include <Wire.h>
#include <math.h>

// NTC thermistor network resistor values
#define BQ_R_PULLUP 5600.0f
#define BQ_R_PARALLEL 27000.0f

// Steinhart-Hart coefficients for NCP15XH103F03RC NTC (10kΩ, B=3380)
#define BQ_SH_A 8.7248136876e-04f
#define BQ_SH_B 2.5405556775e-04f
#define BQ_SH_C 1.8122847672e-07f
#define BQ_IBUS_ADC_OFFSET_MA 27

/// Solar input telemetry
typedef struct {
    uint16_t voltage; ///< Solar voltage in mV
    int16_t current;  ///< Solar current in mA
    int32_t power;    ///< Solar power in mW
    bool mppt;        ///< MPPT enabled
} BqSolarData;

/// Battery telemetry from BQ25798 ADC
typedef struct {
    uint16_t voltage;  ///< Battery voltage in mV
    float current;     ///< Battery current in mA
    int32_t power;     ///< Battery power in mW
    float temperature; ///< Battery temperature in °C (from NTC)
} BqBattData;

/// System voltage
typedef struct {
    uint16_t voltage; ///< System voltage in mV
} BqSysData;

/// Main telemetry container
typedef struct {
    BqSysData system;
    BqSolarData solar;
    BqBattData battery;
} BqTelemetry;

/// JEITA voltage setting for warm/cool regions
typedef enum {
    BQ_JEITA_VSET_SUSPEND = 0x00,
    BQ_JEITA_VSET_MINUS_800MV = 0x01,
    BQ_JEITA_VSET_MINUS_600MV = 0x02,
    BQ_JEITA_VSET_MINUS_400MV = 0x03,
    BQ_JEITA_VSET_MINUS_300MV = 0x04,
    BQ_JEITA_VSET_MINUS_200MV = 0x05,
    BQ_JEITA_VSET_MINUS_100MV = 0x06,
    BQ_JEITA_VSET_UNCHANGED = 0x07
} bq_jeita_vset_t;

/// JEITA current setting for hot region
typedef enum {
    BQ_JEITA_ISETH_SUSPEND = 0x00,
    BQ_JEITA_ISETH_20_PERCENT = 0x01,
    BQ_JEITA_ISETH_40_PERCENT = 0x02,
    BQ_JEITA_ISETH_UNCHANGED = 0x03
} bq_jeita_iseth_t;

/// JEITA current setting for cold region
typedef enum {
    BQ_JEITA_ISETC_SUSPEND = 0x00,
    BQ_JEITA_ISETC_20_PERCENT = 0x01,
    BQ_JEITA_ISETC_40_PERCENT = 0x02,
    BQ_JEITA_ISETC_UNCHANGED = 0x03
} bq_jeita_isetc_t;

/// Charging status
typedef enum {
    BQ_CHARGE_NOT_CHARGING = 0x00,
    BQ_CHARGE_TRICKLE = 0x01,
    BQ_CHARGE_PRE_CHARGING = 0x02,
    BQ_CHARGE_CC = 0x03,
    BQ_CHARGE_CV = 0x04,
    BQ_CHARGE_TOP_OFF = 0x06,
    BQ_CHARGE_DONE = 0x07
} bq_charging_status_t;

/// TS Cool threshold
typedef enum {
    BQ_TS_COOL_5C = 0x00,
    BQ_TS_COOL_10C = 0x01,
    BQ_TS_COOL_15C = 0x02,
    BQ_TS_COOL_20C = 0x03
} bq_ts_cool_t;

/// TS Warm threshold
typedef enum {
    BQ_TS_WARM_40C = 0x00,
    BQ_TS_WARM_45C = 0x01,
    BQ_TS_WARM_50C = 0x02,
    BQ_TS_WARM_55C = 0x03
} bq_ts_warm_t;

class BQ25798Driver : public Adafruit_BQ25798
{
  public:
    BQ25798Driver();
    ~BQ25798Driver();

    bool begin(uint8_t i2c_addr = BQ25798_DEFAULT_ADDR, TwoWire *wire = &Wire);

    // JEITA control
    bq_jeita_vset_t getJeitaVSet();
    bool setJeitaVSet(bq_jeita_vset_t setting);
    bq_jeita_iseth_t getJeitaISetH();
    bool setJeitaISetH(bq_jeita_iseth_t setting);
    bq_jeita_isetc_t getJeitaISetC();
    bool setJeitaISetC(bq_jeita_isetc_t setting);

    // Temperature thresholds
    bq_ts_cool_t getTsCool();
    bool setTsCool(bq_ts_cool_t threshold);
    bq_ts_warm_t getTsWarm();
    bool setTsWarm(bq_ts_warm_t threshold);
    bool getTsIgnore();
    bool setTsIgnore(bool ignore);

    // Telemetry
    const BqTelemetry *getTelemetryData();

    // Charger status
    bool getChargerStatusPowerGood();
    bq_charging_status_t getChargingStatus();

    // Interrupt management
    bool configureSolarOnlyInterrupts();
    bool checkAndClearPgFlag();

    // IBAT ADC control
    bool stopIbatADC();
    bool startIbatADC();

    // Direct register access
    bool writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);

    /// Read VBAT directly via I2C (no initialization required)
    static uint16_t readVBATDirect(TwoWire *wire = &Wire);

  protected:
    Adafruit_I2CDevice *ih_i2c_dev = nullptr;

  private:
    bool startADCOneShot();
    bool setADCEnabled(bool enabled);

    int16_t getIBUS();
    int16_t getIBAT();
    uint16_t getVBUS();
    uint16_t getVBAT();
    uint16_t getVSYS();
    float getTS();
    float getTDIE();

    float calculateBatteryTemp(float ts_pct);
    BqTelemetry telemetryData = {0};
};
