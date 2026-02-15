/*
 * Copyright (c) 2026 Inhero GmbH
 *
 * SPDX-License-Identifier: MIT
 *
 * Inhero MR-2 Board Module - Implementation
 * Ported from MeshCore BoardConfigContainer + InheroMr2Board to Meshtastic
 */

#include "InheroMr2Module.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "main.h"
#include "mesh/generated/meshtastic/telemetry.pb.h"
#include <pb_encode.h>

InheroMr2Module *InheroMr2Module::instance = nullptr;

// Chemistry voltage parameters (from MeshCore BoardConfigContainer)
static const ChemistryParams chemistryTable[] = {
    // LTO_2S:     charge=5400, danger=4200, nominal=4600, full=5200, empty=4400
    {5400, 4200, 4600, 5200, 4400},
    // LiFePO4_1S: charge=3500, danger=2900, nominal=3200, full=3400, empty=2950
    {3500, 2900, 3200, 3400, 2950},
    // Li_Ion_1S:  charge=4100, danger=3400, nominal=3700, full=4050, empty=3500
    {4100, 3400, 3700, 4050, 3500},
    // BAT_UNKNOWN: safe defaults (= LiFePO4 thresholds)
    {3500, 2900, 3200, 3400, 2950},
};

InheroMr2Module::InheroMr2Module()
    : concurrency::OSThread("InheroMr2"),
      SinglePortModule("InheroMr2", meshtastic_PortNum_PRIVATE_APP),
      ina228(INA228_I2C_ADDR_DEFAULT)
{
    instance = this;
    // Allow receiving from localhost (so config commands from serial/BLE work)
    loopbackOk = true;
}

const ChemistryParams &InheroMr2Module::getChemistryParams(BatteryChemistry chem)
{
    uint8_t idx = (uint8_t)chem;
    if (idx >= sizeof(chemistryTable) / sizeof(chemistryTable[0])) {
        idx = (uint8_t)BatteryChemistry::BAT_UNKNOWN;
    }
    return chemistryTable[idx];
}

bool InheroMr2Module::setupDrivers()
{
    LOG_INFO("InheroMr2: Initializing drivers...");

    // Load persistent configuration
    loadConfig();

    // Initialize INA228 power monitor (address 0x40 on Wire)
    ina228Ok = ina228.begin(20.0f); // 20mΩ shunt resistor
    if (ina228Ok) {
        LOG_INFO("InheroMr2: INA228 OK");
        // Apply stored calibration factor
        if (boardConfig.inaCalibration != 1.0f) {
            ina228.setCalibrationFactor(boardConfig.inaCalibration);
        }
    } else {
        LOG_WARN("InheroMr2: INA228 FAILED");
    }

    // Initialize BQ25798 solar charger (address 0x6B on Wire)
    bq25798Ok = bq25798.begin(BQ25798_DEFAULT_ADDR, &Wire);
    if (bq25798Ok) {
        LOG_INFO("InheroMr2: BQ25798 OK");
        // Apply battery chemistry config to charger
        applyChemistryConfig();
        // Configure interrupts for solar events only
        bq25798.configureSolarOnlyInterrupts();
    } else {
        LOG_WARN("InheroMr2: BQ25798 FAILED");
    }

    driversInitialized = true;
    return ina228Ok || bq25798Ok;
}

int32_t InheroMr2Module::runOnce()
{
    if (!driversInitialized) {
        // First run - initialize drivers
        // Give I2C bus time to settle after boot
        setupDrivers();
        return 5000; // Next check in 5s
    }

    uint32_t now = millis();

    // Read INA228 battery data
    if (ina228Ok) {
        ina228.readAll(&batteryData);

        // Voltage danger check
        const ChemistryParams &params = getChemistryParams(boardConfig.chemistry);
        if (batteryData.voltage_mv > 0 && batteryData.voltage_mv < params.dangerVoltage_mV) {
            LOG_WARN("InheroMr2: Battery DANGER! %umV < %umV danger threshold", batteryData.voltage_mv,
                     params.dangerVoltage_mV);
            // In danger zone - reduce activity (Meshtastic's power FSM will handle deep sleep)
        }
    }

    // Read BQ25798 solar data periodically (has 250ms ADC delay, don't read every cycle)
    if (bq25798Ok && (now - lastSensorRead > 30000 || lastSensorRead == 0)) {
        solarData = bq25798.getTelemetryData();
        lastSensorRead = now;
    }

    // Send telemetry periodically
    if (now - lastTelemetrySend > telemetryIntervalMs || lastTelemetrySend == 0) {
        sendPowerTelemetry();
        lastTelemetrySend = now;
    }

    // Update LED indicators
    updateLEDs();

    return 10000; // Run every 10 seconds
}

void InheroMr2Module::sendPowerTelemetry()
{
    meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
    m.time = getTime();
    m.which_variant = meshtastic_Telemetry_power_metrics_tag;
    m.variant.power_metrics = meshtastic_PowerMetrics_init_zero;

    // Channel 1: Battery (from INA228)
    if (ina228Ok) {
        m.variant.power_metrics.has_ch1_voltage = true;
        m.variant.power_metrics.has_ch1_current = true;
        m.variant.power_metrics.ch1_voltage = batteryData.voltage_mv / 1000.0f;
        m.variant.power_metrics.ch1_current = batteryData.current_ma;
    }

    // Channel 2: Solar (from BQ25798)
    if (bq25798Ok && solarData) {
        m.variant.power_metrics.has_ch2_voltage = true;
        m.variant.power_metrics.has_ch2_current = true;
        m.variant.power_metrics.ch2_voltage = solarData->solar.voltage / 1000.0f;
        m.variant.power_metrics.ch2_current = solarData->solar.current;
    }

    // Channel 3: System voltage
    if (bq25798Ok && solarData) {
        m.variant.power_metrics.has_ch3_voltage = true;
        m.variant.power_metrics.ch3_voltage = solarData->system.voltage / 1000.0f;
    }

    // Allocate and send packet on TELEMETRY_APP port
    meshtastic_MeshPacket *p = router->allocForSending();
    p->decoded.portnum = meshtastic_PortNum_TELEMETRY_APP;
    p->decoded.payload.size =
        pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_Telemetry_msg, &m);
    p->to = NODENUM_BROADCAST;
    p->decoded.want_response = false;
    p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;

    LOG_INFO("InheroMr2: Send power telem ch1=%.2fV/%.0fmA ch2=%.2fV/%.0fmA", m.variant.power_metrics.ch1_voltage,
             m.variant.power_metrics.ch1_current, m.variant.power_metrics.ch2_voltage, m.variant.power_metrics.ch2_current);

    service->sendToMesh(p, RX_SRC_LOCAL, true);
}

// === Config Command Handling ===

ProcessMessage InheroMr2Module::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (mp.decoded.portnum != meshtastic_PortNum_PRIVATE_APP) {
        return ProcessMessage::CONTINUE;
    }

    // Extract text payload
    const char *payload = (const char *)mp.decoded.payload.bytes;
    size_t len = mp.decoded.payload.size;

    if (len == 0 || len > 200) {
        return ProcessMessage::CONTINUE;
    }

    handleConfigCommand(mp, payload, len);
    return ProcessMessage::STOP;
}

void InheroMr2Module::handleConfigCommand(const meshtastic_MeshPacket &mp, const char *payload, size_t len)
{
    // Make a null-terminated copy
    char cmd[201];
    size_t copyLen = (len < 200) ? len : 200;
    memcpy(cmd, payload, copyLen);
    cmd[copyLen] = '\0';

    char response[256] = {0};

    // Parse command
    if (strncmp(cmd, "get status", 10) == 0) {
        // Return all sensor data
        snprintf(response, sizeof(response), "v=%u i=%d p=%d t=%.1f sv=%u si=%d sp=%d bt=%.1f sys=%u soc=%d", batteryData.voltage_mv,
                 batteryData.current_ma, batteryData.power_mw, batteryData.die_temp_c,
                 solarData ? solarData->solar.voltage : 0, solarData ? solarData->solar.current : 0,
                 solarData ? solarData->solar.power : 0, solarData ? solarData->battery.temperature : -999.0f,
                 solarData ? solarData->system.voltage : 0, estimateSOC());

    } else if (strncmp(cmd, "get config", 10) == 0) {
        const char *chemStr = "unknown";
        switch (boardConfig.chemistry) {
        case BatteryChemistry::LTO_2S:
            chemStr = "lto2s";
            break;
        case BatteryChemistry::LiFePO4_1S:
            chemStr = "lifepo4";
            break;
        case BatteryChemistry::Li_Ion_1S:
            chemStr = "liion";
            break;
        default:
            break;
        }
        snprintf(response, sizeof(response), "bat=%s imax=%u mppt=%u leds=%u frost=%u cal=%.4f", chemStr,
                 boardConfig.chargeCurrentMax_mA, boardConfig.mpptEnabled ? 1 : 0, boardConfig.ledsEnabled ? 1 : 0,
                 boardConfig.frostProtect ? 1 : 0, boardConfig.inaCalibration);

    } else if (strncmp(cmd, "set bat ", 8) == 0) {
        const char *val = cmd + 8;
        if (strcmp(val, "lto2s") == 0)
            boardConfig.chemistry = BatteryChemistry::LTO_2S;
        else if (strcmp(val, "lifepo4") == 0)
            boardConfig.chemistry = BatteryChemistry::LiFePO4_1S;
        else if (strcmp(val, "liion") == 0)
            boardConfig.chemistry = BatteryChemistry::Li_Ion_1S;
        else {
            snprintf(response, sizeof(response), "err: unknown battery type '%s' (lto2s|lifepo4|liion)", val);
            sendTextReply(mp, response);
            return;
        }
        saveConfig();
        applyChemistryConfig();
        snprintf(response, sizeof(response), "ok bat=%s", val);

    } else if (strncmp(cmd, "set imax ", 9) == 0) {
        int val = atoi(cmd + 9);
        if (val >= 50 && val <= 2000) {
            boardConfig.chargeCurrentMax_mA = (uint16_t)val;
            saveConfig();
            if (bq25798Ok)
                bq25798.setChargeLimitA(boardConfig.chargeCurrentMax_mA / 1000.0f);
            snprintf(response, sizeof(response), "ok imax=%u", boardConfig.chargeCurrentMax_mA);
        } else {
            snprintf(response, sizeof(response), "err: imax out of range (50-2000)");
        }

    } else if (strncmp(cmd, "set mppt ", 9) == 0) {
        boardConfig.mpptEnabled = (cmd[9] == '1');
        saveConfig();
        if (bq25798Ok)
            bq25798.setMPPTenable(boardConfig.mpptEnabled);
        snprintf(response, sizeof(response), "ok mppt=%u", boardConfig.mpptEnabled ? 1 : 0);

    } else if (strncmp(cmd, "set leds ", 9) == 0) {
        boardConfig.ledsEnabled = (cmd[9] == '1');
        saveConfig();
        updateLEDs();
        snprintf(response, sizeof(response), "ok leds=%u", boardConfig.ledsEnabled ? 1 : 0);

    } else if (strncmp(cmd, "set frost ", 10) == 0) {
        boardConfig.frostProtect = (cmd[10] == '1');
        saveConfig();
        applyChemistryConfig();
        snprintf(response, sizeof(response), "ok frost=%u", boardConfig.frostProtect ? 1 : 0);

    } else if (strncmp(cmd, "set cal ", 8) == 0) {
        float val = atof(cmd + 8);
        if (val >= 0.5f && val <= 2.0f) {
            boardConfig.inaCalibration = val;
            saveConfig();
            if (ina228Ok)
                ina228.setCalibrationFactor(val);
            snprintf(response, sizeof(response), "ok cal=%.4f", boardConfig.inaCalibration);
        } else {
            snprintf(response, sizeof(response), "err: cal out of range (0.5-2.0)");
        }

    } else if (strncmp(cmd, "get diag", 8) == 0) {
        uint16_t diagFlags = ina228Ok ? ina228.getDiagnosticFlags() : 0;
        bq_charging_status_t chgStatus = bq25798Ok ? bq25798.getChargingStatus() : BQ_CHARGE_NOT_CHARGING;
        bool pgood = bq25798Ok ? bq25798.getChargerStatusPowerGood() : false;
        snprintf(response, sizeof(response), "ina228=%s bq25798=%s diag=0x%04X chg=%u pgood=%u", ina228Ok ? "ok" : "fail",
                 bq25798Ok ? "ok" : "fail", diagFlags, (uint8_t)chgStatus, pgood ? 1 : 0);

    } else {
        snprintf(response, sizeof(response),
                 "cmds: get status|config|diag, set bat|imax|mppt|leds|frost|cal <val>");
    }

    if (response[0] != '\0') {
        sendTextReply(mp, response);
    }
}

void InheroMr2Module::sendTextReply(const meshtastic_MeshPacket &mp, const char *text)
{
    meshtastic_MeshPacket *p = router->allocForSending();
    p->decoded.portnum = meshtastic_PortNum_PRIVATE_APP;
    p->to = mp.from;
    size_t textLen = strlen(text);
    if (textLen > sizeof(p->decoded.payload.bytes))
        textLen = sizeof(p->decoded.payload.bytes);
    memcpy(p->decoded.payload.bytes, text, textLen);
    p->decoded.payload.size = textLen;
    p->decoded.want_response = false;
    p->priority = meshtastic_MeshPacket_Priority_RELIABLE;

    service->sendToMesh(p, RX_SRC_LOCAL, true);
}

// === Battery Chemistry & Charger Management ===

void InheroMr2Module::applyChemistryConfig()
{
    if (!bq25798Ok)
        return;

    const ChemistryParams &params = getChemistryParams(boardConfig.chemistry);

    // Set charge voltage limit (API expects volts)
    bq25798.setChargeLimitV(params.chargeVoltage_mV / 1000.0f);

    // Set charge current limit (API expects amps)
    bq25798.setChargeLimitA(boardConfig.chargeCurrentMax_mA / 1000.0f);

    // MPPT
    bq25798.setMPPTenable(boardConfig.mpptEnabled);

    // JEITA frost protection
    if (boardConfig.frostProtect) {
        // Below 0°C: reduce charge voltage by 200mV, current to 20%
        bq25798.setJeitaVSet(BQ_JEITA_VSET_MINUS_200MV);
        bq25798.setJeitaISetC(BQ_JEITA_ISETC_20_PERCENT);
        bq25798.setTsCool(BQ_TS_COOL_5C);
        bq25798.setTsIgnore(false);
    } else {
        bq25798.setTsIgnore(true);
    }

    // Configure INA228 UVLO alert based on chemistry
    if (ina228Ok) {
        ina228.setUnderVoltageAlert(params.dangerVoltage_mV);
        ina228.enableAlert(true, false, false); // UVLO, active-low, transparent mode
    }

    LOG_INFO("InheroMr2: Applied chemistry config (charge=%umV, danger=%umV, imax=%umA, mppt=%u)", params.chargeVoltage_mV,
             params.dangerVoltage_mV, boardConfig.chargeCurrentMax_mA, boardConfig.mpptEnabled);
}

void InheroMr2Module::updateLEDs()
{
    if (!boardConfig.ledsEnabled) {
        // Turn off both LEDs
        ledOff(PIN_LED1);
        ledOff(PIN_LED2);
        return;
    }

    // Blue LED: normal operation heartbeat (blink briefly)
    // Red LED: charging indicator
    if (bq25798Ok) {
        bq_charging_status_t status = bq25798.getChargingStatus();
        if (status >= BQ_CHARGE_CC && status <= BQ_CHARGE_CV) {
            ledOn(PIN_LED2); // Red = charging
        } else if (status == BQ_CHARGE_DONE) {
            ledOff(PIN_LED2); // Charge complete
        } else {
            ledOff(PIN_LED2);
        }
    }
}

int InheroMr2Module::estimateSOC()
{
    if (!ina228Ok || batteryData.voltage_mv == 0)
        return -1;

    const ChemistryParams &params = getChemistryParams(boardConfig.chemistry);

    // Simple linear SOC estimation based on voltage
    if (batteryData.voltage_mv >= params.fullVoltage_mV)
        return 100;
    if (batteryData.voltage_mv <= params.emptyVoltage_mV)
        return 0;

    int soc =
        (int)(((float)(batteryData.voltage_mv - params.emptyVoltage_mV) / (params.fullVoltage_mV - params.emptyVoltage_mV)) *
              100.0f);
    return constrain(soc, 0, 100);
}

// === Config Persistence (LittleFS) ===

#define INHERO_CONFIG_DIR "/inhero"

void InheroMr2Module::loadConfig()
{
    if (!InternalFS.begin()) {
        LOG_WARN("InheroMr2: LittleFS init failed");
        return;
    }
    InternalFS.mkdir(INHERO_CONFIG_DIR);

    char buf[32];

    if (readConfigValue("bat", buf, sizeof(buf)) > 0) {
        if (strcmp(buf, "lto2s") == 0)
            boardConfig.chemistry = BatteryChemistry::LTO_2S;
        else if (strcmp(buf, "lifepo4") == 0)
            boardConfig.chemistry = BatteryChemistry::LiFePO4_1S;
        else if (strcmp(buf, "liion") == 0)
            boardConfig.chemistry = BatteryChemistry::Li_Ion_1S;
    }

    if (readConfigValue("imax", buf, sizeof(buf)) > 0)
        boardConfig.chargeCurrentMax_mA = (uint16_t)atoi(buf);

    if (readConfigValue("mppt", buf, sizeof(buf)) > 0)
        boardConfig.mpptEnabled = (buf[0] == '1');

    if (readConfigValue("leds", buf, sizeof(buf)) > 0)
        boardConfig.ledsEnabled = (buf[0] == '1');

    if (readConfigValue("frost", buf, sizeof(buf)) > 0)
        boardConfig.frostProtect = (buf[0] == '1');

    if (readConfigValue("cal", buf, sizeof(buf)) > 0)
        boardConfig.inaCalibration = atof(buf);

    LOG_INFO("InheroMr2: Config loaded (bat=%u, imax=%u, mppt=%u)", (uint8_t)boardConfig.chemistry,
             boardConfig.chargeCurrentMax_mA, boardConfig.mpptEnabled);
}

void InheroMr2Module::saveConfig()
{
    const char *chemStr = "unknown";
    switch (boardConfig.chemistry) {
    case BatteryChemistry::LTO_2S:
        chemStr = "lto2s";
        break;
    case BatteryChemistry::LiFePO4_1S:
        chemStr = "lifepo4";
        break;
    case BatteryChemistry::Li_Ion_1S:
        chemStr = "liion";
        break;
    default:
        chemStr = "unknown";
        break;
    }
    writeConfigValue("bat", chemStr);

    char buf[32];
    snprintf(buf, sizeof(buf), "%u", boardConfig.chargeCurrentMax_mA);
    writeConfigValue("imax", buf);

    writeConfigValue("mppt", boardConfig.mpptEnabled ? "1" : "0");
    writeConfigValue("leds", boardConfig.ledsEnabled ? "1" : "0");
    writeConfigValue("frost", boardConfig.frostProtect ? "1" : "0");

    snprintf(buf, sizeof(buf), "%.4f", boardConfig.inaCalibration);
    writeConfigValue("cal", buf);

    LOG_INFO("InheroMr2: Config saved");
}

void InheroMr2Module::writeConfigValue(const char *key, const char *value)
{
    char path[64];
    snprintf(path, sizeof(path), "%s/%s.txt", INHERO_CONFIG_DIR, key);
    InternalFS.remove(path);
    using namespace Adafruit_LittleFS_Namespace;
    File file(InternalFS);
    if (file.open(path, FILE_O_WRITE)) {
        file.print(value);
        file.close();
    }
}

size_t InheroMr2Module::readConfigValue(const char *key, char *buffer, size_t maxLen, const char *defaultValue)
{
    char path[64];
    snprintf(path, sizeof(path), "%s/%s.txt", INHERO_CONFIG_DIR, key);

    if (!InternalFS.exists(path)) {
        if (defaultValue) {
            strncpy(buffer, defaultValue, maxLen);
            buffer[maxLen - 1] = '\0';
            return strlen(buffer);
        }
        buffer[0] = '\0';
        return 0;
    }

    using namespace Adafruit_LittleFS_Namespace;
    File file(InternalFS);
    if (!file.open(path, FILE_O_READ)) {
        buffer[0] = '\0';
        return 0;
    }

    size_t bytesRead = file.readBytes(buffer, maxLen - 1);
    buffer[bytesRead] = '\0';

    // Trim trailing whitespace
    while (bytesRead > 0 && (buffer[bytesRead - 1] == '\r' || buffer[bytesRead - 1] == '\n' || buffer[bytesRead - 1] == ' ')) {
        buffer[--bytesRead] = '\0';
    }

    file.close();
    return bytesRead;
}
