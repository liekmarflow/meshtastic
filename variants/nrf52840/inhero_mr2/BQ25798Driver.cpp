/*
 * Copyright (c) 2026 Inhero GmbH
 *
 * SPDX-License-Identifier: MIT
 *
 * BQ25798 Solar Charger Driver - Ported from MeshCore to Meshtastic
 */

#include "BQ25798Driver.h"
#include "meshUtils.h" // For LOG_DEBUG, etc.

BQ25798Driver::BQ25798Driver() {}

BQ25798Driver::~BQ25798Driver()
{
    if (ih_i2c_dev) {
        delete ih_i2c_dev;
        ih_i2c_dev = nullptr;
    }
}

bool BQ25798Driver::begin(uint8_t i2c_addr, TwoWire *wire)
{
    if (!Adafruit_BQ25798::begin(i2c_addr, wire)) {
        if (ih_i2c_dev) {
            delete ih_i2c_dev;
            ih_i2c_dev = nullptr;
        }
        return false;
    }
    if (ih_i2c_dev) {
        delete ih_i2c_dev;
    }
    ih_i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);
    if (!ih_i2c_dev->begin()) {
        delete ih_i2c_dev;
        ih_i2c_dev = nullptr;
        return false;
    }
    LOG_INFO("BQ25798: Initialized at 0x%02X", i2c_addr);
    return true;
}

bool BQ25798Driver::getChargerStatusPowerGood()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_STATUS_0);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 1, 3);
    return (bool)bits.read();
}

bq_charging_status_t BQ25798Driver::getChargingStatus()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_STATUS_1);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 3, 5);
    return (bq_charging_status_t)bits.read();
}

bool BQ25798Driver::configureSolarOnlyInterrupts()
{
    Adafruit_BusIO_Register mask0 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_MASK_0);
    if (!mask0.write(0xF7))
        return false;
    Adafruit_BusIO_Register mask1 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_MASK_1);
    if (!mask1.write(0xFF))
        return false;
    Adafruit_BusIO_Register mask2 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_MASK_2);
    if (!mask2.write(0xFF))
        return false;
    Adafruit_BusIO_Register mask3 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_MASK_3);
    if (!mask3.write(0xFF))
        return false;
    Adafruit_BusIO_Register fault0 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_FAULT_MASK_0);
    if (!fault0.write(0xFF))
        return false;
    Adafruit_BusIO_Register fault1 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_FAULT_MASK_1);
    if (!fault1.write(0xFF))
        return false;
    // Clear existing interrupts
    Adafruit_BusIO_Register flag0 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_FLAG_0);
    uint8_t dummy;
    flag0.read(&dummy);
    return true;
}

bool BQ25798Driver::checkAndClearPgFlag()
{
    Adafruit_BusIO_Register flag0 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_FLAG_0);
    uint8_t val;
    if (!flag0.read(&val))
        return false;
    return (val & 0x08);
}

const BqTelemetry *BQ25798Driver::getTelemetryData()
{
    telemetryData = {0};
    this->startIbatADC();
    bool success = this->startADCOneShot();

    if (!success) {
        return &telemetryData;
    }

    // Wait for ADC conversion (typical 150ms, extended for TS)
    delay(250);

    telemetryData.solar.voltage = getVBUS();
    telemetryData.solar.current = getIBUS() - BQ_IBUS_ADC_OFFSET_MA;
    if (telemetryData.solar.current < 0) {
        telemetryData.solar.current = 0;
    }
    telemetryData.solar.power = ((int32_t)telemetryData.solar.voltage * telemetryData.solar.current) / 1000;
    telemetryData.solar.mppt = getMPPTenable();

    telemetryData.battery.voltage = getVBAT();
    telemetryData.battery.current = (float)getIBAT();
    telemetryData.battery.power = (int32_t)((telemetryData.battery.voltage * telemetryData.battery.current) / 1000.0f);
    telemetryData.battery.temperature = this->calculateBatteryTemp(getTS());

    telemetryData.system.voltage = getVSYS();

    this->stopIbatADC();
    this->setADCEnabled(false); // Save ~1.5mA

    return &telemetryData;
}

float BQ25798Driver::calculateBatteryTemp(float ts_pct)
{
    if (ts_pct == -1.0f)
        return -999.0f;
    if (ts_pct == -2.0f)
        return -888.0f;

    float k = ts_pct / 100.0f;
    if (k > 0.99f)
        return -99.0f; // NTC open
    if (k < 0.01f)
        return 99.0f; // NTC short

    float r_bottom_total = BQ_R_PULLUP * (k / (1.0f - k));
    float g_total = 1.0f / r_bottom_total;
    float g_rt2 = 1.0f / BQ_R_PARALLEL;

    if (g_total <= g_rt2) {
        return -99.0f;
    }

    float r_ntc = 1.0f / (g_total - g_rt2);
    float ln_r = logf(r_ntc);
    float inv_T = BQ_SH_A + BQ_SH_B * ln_r + BQ_SH_C * ln_r * ln_r * ln_r;

    return (1.0f / inv_T) - 273.15f;
}

// JEITA Control
bq_jeita_vset_t BQ25798Driver::getJeitaVSet()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 3, 5);
    return (bq_jeita_vset_t)bits.read();
}

bool BQ25798Driver::setJeitaVSet(bq_jeita_vset_t setting)
{
    if (setting > BQ_JEITA_VSET_UNCHANGED)
        return false;
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 3, 5);
    bits.write((uint8_t)setting);
    return true;
}

bq_jeita_iseth_t BQ25798Driver::getJeitaISetH()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 2, 3);
    return (bq_jeita_iseth_t)bits.read();
}

bool BQ25798Driver::setJeitaISetH(bq_jeita_iseth_t setting)
{
    if (setting > BQ_JEITA_ISETH_UNCHANGED)
        return false;
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 2, 3);
    bits.write((uint8_t)setting);
    return true;
}

bq_jeita_isetc_t BQ25798Driver::getJeitaISetC()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 2, 1);
    return (bq_jeita_isetc_t)bits.read();
}

bool BQ25798Driver::setJeitaISetC(bq_jeita_isetc_t setting)
{
    if (setting > BQ_JEITA_ISETC_UNCHANGED)
        return false;
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 2, 1);
    bits.write((uint8_t)setting);
    return true;
}

bq_ts_cool_t BQ25798Driver::getTsCool()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 2, 6);
    return (bq_ts_cool_t)bits.read();
}

bool BQ25798Driver::setTsCool(bq_ts_cool_t threshold)
{
    if (threshold > BQ_TS_COOL_20C)
        return false;
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 2, 6);
    bits.write((uint8_t)threshold);
    return true;
}

bq_ts_warm_t BQ25798Driver::getTsWarm()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 2, 4);
    return (bq_ts_warm_t)bits.read();
}

bool BQ25798Driver::setTsWarm(bq_ts_warm_t threshold)
{
    if (threshold > BQ_TS_WARM_55C)
        return false;
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 2, 4);
    bits.write((uint8_t)threshold);
    return true;
}

bool BQ25798Driver::getTsIgnore()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 1, 0);
    return (bool)bits.read();
}

bool BQ25798Driver::setTsIgnore(bool ignore)
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
    Adafruit_BusIO_RegisterBits bits = Adafruit_BusIO_RegisterBits(&reg, 1, 0);
    bits.write((uint8_t)ignore);
    return true;
}

// ADC Control
bool BQ25798Driver::startADCOneShot()
{
    Adafruit_BusIO_Register disable_reg_0 = Adafruit_BusIO_Register(ih_i2c_dev, 0x2F);
    Adafruit_BusIO_Register disable_reg_1 = Adafruit_BusIO_Register(ih_i2c_dev, 0x30);

    if (!disable_reg_0.write(0x00))
        return false;
    if (!disable_reg_1.write(0x00))
        return false;

    Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
    return adc_ctrl_reg.write(0xC0);
}

bool BQ25798Driver::stopIbatADC()
{
    Adafruit_BusIO_Register ctrl5_reg = Adafruit_BusIO_Register(ih_i2c_dev, 0x14);
    Adafruit_BusIO_RegisterBits en_ibat_bit = Adafruit_BusIO_RegisterBits(&ctrl5_reg, 1, 5);
    return en_ibat_bit.write(0);
}

bool BQ25798Driver::startIbatADC()
{
    Adafruit_BusIO_Register ctrl5_reg = Adafruit_BusIO_Register(ih_i2c_dev, 0x14);
    Adafruit_BusIO_RegisterBits en_ibat_bit = Adafruit_BusIO_RegisterBits(&ctrl5_reg, 1, 5);
    return en_ibat_bit.write(1);
}

bool BQ25798Driver::setADCEnabled(bool enabled)
{
    Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
    Adafruit_BusIO_RegisterBits adc_en_bits = Adafruit_BusIO_RegisterBits(&adc_ctrl_reg, 1, 7);
    return adc_en_bits.write((uint8_t)enabled);
}

// ADC Readings
int16_t BQ25798Driver::getIBUS()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_IBUS_ADC, 2, MSBFIRST);
    uint16_t raw;
    if (!reg.read(&raw))
        return 0;
    return (int16_t)raw; // mA
}

int16_t BQ25798Driver::getIBAT()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_IBAT_ADC, 2, MSBFIRST);
    uint16_t raw;
    if (!reg.read(&raw))
        return 0;
    return (int16_t)raw; // mA
}

uint16_t BQ25798Driver::getVBUS()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_VBUS_ADC, 2, MSBFIRST);
    uint16_t val;
    if (!reg.read(&val))
        return 0;
    return val; // mV
}

uint16_t BQ25798Driver::getVBAT()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_VBAT_ADC, 2, MSBFIRST);
    uint16_t val;
    if (!reg.read(&val))
        return 0;
    return val; // mV
}

uint16_t BQ25798Driver::readVBATDirect(TwoWire *wire)
{
    const uint8_t BQ_I2C_ADDR = 0x6B;
    wire->beginTransmission(BQ_I2C_ADDR);
    wire->write(BQ25798_REG_VBAT_ADC);
    if (wire->endTransmission(false) != 0) {
        return 0;
    }
    wire->requestFrom(BQ_I2C_ADDR, (uint8_t)2);
    if (wire->available() < 2) {
        return 0;
    }
    uint8_t msb = wire->read();
    uint8_t lsb = wire->read();
    return (msb << 8) | lsb; // mV
}

uint16_t BQ25798Driver::getVSYS()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_VSYS_ADC, 2, MSBFIRST);
    uint16_t val;
    if (!reg.read(&val))
        return 0;
    return val; // mV
}

float BQ25798Driver::getTS()
{
    Adafruit_BusIO_Register ts_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_TS_ADC, 2, MSBFIRST);
    uint16_t val;

    for (int retry = 0; retry < 3; retry++) {
        if (!ts_reg.read(&val)) {
            delay(20);
            continue;
        }
        if (val == 0 || val == 0xFFFF) {
            if (retry < 2) {
                delay(50);
                continue;
            }
            return -2.0f;
        }
        return val * 0.09765625f; // 1/1024 %/LSB
    }
    return -1.0f;
}

float BQ25798Driver::getTDIE()
{
    Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_TDIE_ADC, 2, MSBFIRST);
    uint16_t raw;
    if (!reg.read(&raw))
        return 0.0f;
    return (int16_t)raw * 0.5f; // 0.5 °C/LSB
}

bool BQ25798Driver::writeReg(uint8_t reg, uint8_t val)
{
    if (!ih_i2c_dev)
        return false;
    uint8_t buffer[2] = {reg, val};
    return ih_i2c_dev->write(buffer, 2);
}

uint8_t BQ25798Driver::readReg(uint8_t reg)
{
    if (!ih_i2c_dev)
        return 0;
    uint8_t buffer[1] = {reg};
    if (!ih_i2c_dev->write_then_read(buffer, 1, buffer, 1)) {
        return 0;
    }
    return buffer[0];
}
