/*
 * Copyright (c) 2026 Inhero GmbH
 *
 * SPDX-License-Identifier: MIT
 *
 * INA228 Power Monitor Driver - Ported from MeshCore to Meshtastic
 */

#include "Ina228Driver.h"
#include "meshUtils.h" // For LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR

Ina228Driver::Ina228Driver(uint8_t i2c_addr)
    : _i2c_addr(i2c_addr), _shunt_mohm(10.0f), _current_lsb(0.0f), _base_shunt_cal(0), _calibration_factor(1.0f)
{
}

bool Ina228Driver::begin(float shunt_resistor_mohm)
{
    _shunt_mohm = shunt_resistor_mohm;

    if (!isConnected()) {
        LOG_WARN("INA228: Device not found at 0x%02X", _i2c_addr);
        return false;
    }
    LOG_INFO("INA228: Device connected at 0x%02X", _i2c_addr);

    // Configure ADC: Continuous mode, all channels, long conversion times, 256 samples averaging
    // AVG_256 filters TX voltage peaks (prevents false UVLO triggers during transmit)
    uint16_t adc_config = (INA228_ADC_MODE_CONT_ALL << 12) | // MODE: Continuous all
                          (INA228_ADC_CT_2074us << 9) |       // VBUSCT: 2074µs
                          (INA228_ADC_CT_4120us << 6) |       // VSHCT: 4120µs (current accuracy)
                          (INA228_ADC_CT_540us << 3) |        // VTCT: 540µs (temp less critical)
                          (INA228_ADC_AVG_256 << 0);          // AVG: 256 samples

    // Write ADC_CONFIG with retry and verify
    bool adc_config_ok = false;
    for (int retry = 0; retry < 5; retry++) {
        writeRegister16(INA228_REG_ADC_CONFIG, adc_config);
        delay(10);
        uint16_t readback = readRegister16(INA228_REG_ADC_CONFIG);
        if (readback == adc_config) {
            adc_config_ok = true;
            break;
        }
        delay(20);
    }

    if (!adc_config_ok) {
        LOG_ERROR("INA228: Failed to set ADC_CONFIG after 5 retries");
        return false;
    }

    // Calculate current LSB: Max expected current / 2^19
    // With 20mΩ shunt and ±40.96mV ADC range: Max = 40.96mV / 0.02Ω = 2.048A
    _current_lsb = 2.0f / 524288.0f; // ~3.81 µA per LSB

    // Calculate shunt calibration value
    // SHUNT_CAL = 13107.2 × 10^6 × CURRENT_LSB × R_SHUNT
    float shunt_ohm = _shunt_mohm / 1000.0f;
    _base_shunt_cal = (uint16_t)(13107.2e6 * _current_lsb * shunt_ohm);

    // Per INA228 datasheet section 7.3.1.1:
    // "the value of SHUNT_CAL must be multiplied by 4 for ADCRANGE = 1"
    _base_shunt_cal *= 4;

    // Apply calibration factor to SHUNT_CAL
    uint16_t calibrated_shunt_cal = (uint16_t)(_base_shunt_cal * _calibration_factor);
    writeRegister16(INA228_REG_SHUNT_CAL, calibrated_shunt_cal);
    delay(5);

    // Configure INA228: ADC range ±40.96mV for better resolution with 20mΩ shunt
    uint16_t config = INA228_CONFIG_ADCRANGE;
    writeRegister16(INA228_REG_CONFIG, config);
    delay(5);

    LOG_INFO("INA228: Initialized (shunt=%.1fmΩ, LSB=%.2fµA)", _shunt_mohm, _current_lsb * 1e6);
    return true;
}

bool Ina228Driver::isConnected()
{
    Wire.beginTransmission(_i2c_addr);
    uint8_t i2c_result = Wire.endTransmission();

    if (i2c_result != 0) {
        return false;
    }

    // Read Manufacturer ID (should be 0x5449 = "TI")
    uint16_t mfg_id = readRegister16(INA228_REG_MANUFACTURER);
    if (mfg_id == 0x0000 || mfg_id == 0xFFFF) {
        return false;
    }

    // Read Device ID
    uint16_t dev_id = readRegister16(INA228_REG_DEVICE_ID);
    if (dev_id == 0x0000 || dev_id == 0xFFFF) {
        return false;
    }

    return true;
}

void Ina228Driver::reset()
{
    writeRegister16(INA228_REG_CONFIG, INA228_CONFIG_RST);
}

uint16_t Ina228Driver::readVoltage_mV()
{
    int32_t vbus_raw = readRegister24(INA228_REG_VBUS);
    // 20-bit ADC left-aligned in 24-bit register → right-shift by 4
    vbus_raw >>= 4;
    // VBUS LSB = 195.3125 µV
    float vbus_v = vbus_raw * 195.3125e-6;
    return (uint16_t)(vbus_v * 1000.0f);
}

int16_t Ina228Driver::readCurrent_mA()
{
    int32_t current_raw = readRegister24(INA228_REG_CURRENT);
    current_raw >>= 4;
    // Sign convention: INVERT (shunt oriented for battery perspective)
    float current_a = current_raw * _current_lsb;
    return (int16_t)(-current_a * 1000.0f);
}

float Ina228Driver::readCurrent_mA_precise()
{
    int32_t current_raw = readRegister24(INA228_REG_CURRENT);
    current_raw >>= 4;
    float current_a = current_raw * _current_lsb;
    return -current_a * 1000.0f;
}

int32_t Ina228Driver::readPower_mW()
{
    int32_t power_raw = readRegister24(INA228_REG_POWER);
    // Power LSB = 3.2 × CURRENT_LSB
    float power_w = power_raw * (3.2f * _current_lsb);
    return (int32_t)(-power_w * 1000.0f);
}

int32_t Ina228Driver::readEnergy_mWh()
{
    int64_t energy_raw = readRegister40(INA228_REG_ENERGY);
    // Energy LSB = 16 × 3.2 × CURRENT_LSB (in J), convert to Wh: /3600
    float energy_j = energy_raw * (16.0f * 3.2f * _current_lsb);
    float energy_wh = energy_j / 3600.0f;
    return (int32_t)(energy_wh * 1000.0f);
}

float Ina228Driver::readCharge_mAh()
{
    int64_t charge_raw = readRegister40(INA228_REG_CHARGE);
    // Charge LSB = CURRENT_LSB (in C = A·s), convert to Ah: /3600
    float charge_c = charge_raw * _current_lsb;
    float charge_ah = charge_c / 3600.0f;
    return -charge_ah * 1000.0f; // mAh, inverted for battery perspective
}

float Ina228Driver::readDieTemperature_C()
{
    int16_t temp_raw = (int16_t)readRegister16(INA228_REG_DIETEMP);
    return temp_raw * 7.8125e-3;
}

bool Ina228Driver::readAll(Ina228BatteryData *data)
{
    if (!isConnected()) {
        return false;
    }

    data->voltage_mv = readVoltage_mV();
    data->current_ma = readCurrent_mA();
    data->power_mw = readPower_mW();
    data->energy_mwh = readEnergy_mWh();
    data->charge_mah = readCharge_mAh();
    data->die_temp_c = readDieTemperature_C();

    return true;
}

void Ina228Driver::resetCoulombCounter()
{
    uint16_t config = readRegister16(INA228_REG_CONFIG);
    config |= (1 << 14); // RSTACC
    writeRegister16(INA228_REG_CONFIG, config);
}

bool Ina228Driver::setUnderVoltageAlert(uint16_t voltage_mv)
{
    // BUVL register: 3.125 mV/LSB
    uint16_t buvl_value = (uint16_t)(voltage_mv / 3.125f);
    return writeRegister16(INA228_REG_BUVL, buvl_value);
}

bool Ina228Driver::setOverVoltageAlert(uint16_t voltage_mv)
{
    uint16_t bovl_value = (uint16_t)(voltage_mv / 3.125f);
    return writeRegister16(INA228_REG_BOVL, bovl_value);
}

void Ina228Driver::enableAlert(bool enable_uvlo, bool active_high, bool latch_alert)
{
    uint16_t diag_alrt = 0;
    if (enable_uvlo)
        diag_alrt |= INA228_DIAG_ALRT_BUSUL;
    if (active_high)
        diag_alrt |= INA228_DIAG_ALRT_APOL;
    if (latch_alert)
        diag_alrt |= INA228_DIAG_ALRT_ALATCH;
    writeRegister16(INA228_REG_DIAG_ALRT, diag_alrt);
}

bool Ina228Driver::isAlertActive()
{
    uint16_t diag_flags = getDiagnosticFlags();
    return (diag_flags & (INA228_DIAG_ALRT_BUSUL | INA228_DIAG_ALRT_BUSOL)) != 0;
}

void Ina228Driver::clearAlert()
{
    getDiagnosticFlags();
}

uint16_t Ina228Driver::getDiagnosticFlags()
{
    return readRegister16(INA228_REG_DIAG_ALRT);
}

void Ina228Driver::shutdown()
{
    uint16_t adc_config = 0x0000; // MODE = 0x0 (Shutdown)
    writeRegister16(INA228_REG_ADC_CONFIG, adc_config);
}

void Ina228Driver::wakeup()
{
    uint16_t adc_config = (INA228_ADC_MODE_CONT_ALL << 12) | (INA228_ADC_CT_2074us << 9) | (INA228_ADC_CT_4120us << 6) |
                          (INA228_ADC_CT_540us << 3) | (INA228_ADC_AVG_256 << 0);
    writeRegister16(INA228_REG_ADC_CONFIG, adc_config);
}

uint16_t Ina228Driver::readVBATDirect(TwoWire *wire, uint8_t i2c_addr)
{
    // Static method for early boot - before driver initialization
    wire->beginTransmission(i2c_addr);
    if (wire->endTransmission() != 0) {
        return 0;
    }

    // One-Shot ADC: MODE = 0x1 (single-shot bus voltage only)
    uint16_t adc_config = (0x1 << 12);
    wire->beginTransmission(i2c_addr);
    wire->write(INA228_REG_ADC_CONFIG);
    wire->write((adc_config >> 8) & 0xFF);
    wire->write(adc_config & 0xFF);
    if (wire->endTransmission() != 0) {
        return 0;
    }

    delay(2); // Wait for conversion (~200µs typical)

    // Read VBUS register (24-bit)
    wire->beginTransmission(i2c_addr);
    wire->write(INA228_REG_VBUS);
    if (wire->endTransmission(false) != 0) {
        return 0;
    }

    wire->requestFrom(i2c_addr, (uint8_t)3);
    if (wire->available() < 3) {
        return 0;
    }

    int32_t vbus_raw = wire->read() << 16;
    vbus_raw |= wire->read() << 8;
    vbus_raw |= wire->read();

    // Sign-extend 24-bit to 32-bit
    if (vbus_raw & 0x800000) {
        vbus_raw |= 0xFF000000;
    }

    vbus_raw >>= 4; // 20-bit left-aligned in 24-bit
    float vbus_v = vbus_raw * 195.3125e-6;
    return (uint16_t)(vbus_v * 1000.0f);
}

// === Calibration ===

float Ina228Driver::calibrateCurrent(float actual_current_ma)
{
    setCalibrationFactor(1.0f);
    delay(10);

    int16_t measured_current_ma = readCurrent_mA();
    if (measured_current_ma == 0) {
        return 1.0f;
    }

    float new_factor = (float)measured_current_ma / actual_current_ma;
    setCalibrationFactor(new_factor);
    return new_factor;
}

void Ina228Driver::setCalibrationFactor(float factor)
{
    if (factor < 0.5f)
        factor = 0.5f;
    if (factor > 2.0f)
        factor = 2.0f;

    _calibration_factor = factor;

    if (_base_shunt_cal > 0) {
        uint16_t calibrated_shunt_cal = (uint16_t)(_base_shunt_cal * factor);
        writeRegister16(INA228_REG_SHUNT_CAL, calibrated_shunt_cal);
    }
}

float Ina228Driver::getCalibrationFactor() const
{
    return _calibration_factor;
}

// === Private I2C Methods ===

bool Ina228Driver::writeRegister16(uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);
    Wire.write(value & 0xFF);
    return (Wire.endTransmission() == 0);
}

uint16_t Ina228Driver::readRegister16(uint8_t reg)
{
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_i2c_addr, (uint8_t)2);
    if (Wire.available() < 2) {
        return 0;
    }

    uint16_t value = Wire.read() << 8;
    value |= Wire.read();
    return value;
}

int32_t Ina228Driver::readRegister24(uint8_t reg)
{
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_i2c_addr, (uint8_t)3);
    if (Wire.available() < 3) {
        return 0;
    }

    int32_t value = Wire.read() << 16;
    value |= Wire.read() << 8;
    value |= Wire.read();

    // Sign-extend 24-bit to 32-bit
    if (value & 0x800000) {
        value |= 0xFF000000;
    }

    return value;
}

int64_t Ina228Driver::readRegister40(uint8_t reg)
{
    Wire.beginTransmission(_i2c_addr);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_i2c_addr, (uint8_t)5);
    if (Wire.available() < 5) {
        return 0;
    }

    int64_t value = (int64_t)Wire.read() << 32;
    value |= (int64_t)Wire.read() << 24;
    value |= (int64_t)Wire.read() << 16;
    value |= (int64_t)Wire.read() << 8;
    value |= (int64_t)Wire.read();

    // Sign-extend 40-bit to 64-bit
    if (value & 0x8000000000LL) {
        value |= 0xFFFFFF0000000000LL;
    }

    return value;
}
