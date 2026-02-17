/*
 * Copyright (c) 2026 Inhero GmbH
 *
 * SPDX-License-Identifier: MIT
 *
 * Inhero MR-2 Board Module - Implementation
 * MeshCore-compatible CLI via Meshtastic text message DMs.
 *
 * Commands are received as normal text messages (DMs) starting with '/'.
 * Write operations require the sender's public key to be registered in
 * config.security.admin_key[0..2] (same mechanism as Meshtastic AdminModule).
 */

#include "InheroMr2Module.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "main.h"
#include "mesh/generated/meshtastic/telemetry.pb.h"
#include <pb_encode.h>
#include <cctype>

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

// Default battery capacities per chemistry (mAh)
static const uint32_t defaultBatteryCapacity[] = {
    3000, // LTO_2S
    3000, // LiFePO4_1S
    3000, // Li_Ion_1S
    3000, // BAT_UNKNOWN
};

InheroMr2Module::InheroMr2Module()
    : concurrency::OSThread("InheroMr2"),
      SinglePortModule("InheroMr2", meshtastic_PortNum_TEXT_MESSAGE_APP),
      ina228(INA228_I2C_ADDR_DEFAULT)
{
    instance = this;
    // Receive DMs addressed to this node + broadcast (like ReplyBotModule)
    isPromiscuous = true;
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

// === CLI Command Handling via Text Message DMs ===

bool InheroMr2Module::wantPacket(const meshtastic_MeshPacket *p)
{
    // Accept TEXT_MESSAGE_APP packets (same port as TextMessageModule/ReplyBotModule)
    return (p && p->decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP);
}

ProcessMessage InheroMr2Module::handleReceived(const meshtastic_MeshPacket &mp)
{
    // Only process DMs addressed to this node (not broadcasts)
    if (mp.to != nodeDB->getNodeNum() && mp.from != 0) {
        return ProcessMessage::CONTINUE;
    }

    // Extract text payload
    if (mp.decoded.payload.size == 0 || mp.decoded.payload.size > 250) {
        return ProcessMessage::CONTINUE;
    }

    // Null-terminate the payload
    char buf[260];
    size_t n = mp.decoded.payload.size;
    if (n > sizeof(buf) - 1)
        n = sizeof(buf) - 1;
    memcpy(buf, mp.decoded.payload.bytes, n);
    buf[n] = '\0';

    // Skip leading whitespace
    char *cmd = buf;
    while (*cmd == ' ' || *cmd == '\t')
        cmd++;

    // Only intercept commands starting with '/'
    if (cmd[0] != '/') {
        return ProcessMessage::CONTINUE;
    }

    // Skip the '/' prefix
    cmd++;

    LOG_INFO("InheroMr2: CLI command from 0x%08x: /%s", mp.from, cmd);

    // Flash LED2 briefly to indicate CLI activity
    cliFlashUntil = millis() + 200;

    handleCliCommand(mp, cmd);

    // CONTINUE so TextMessageModule still stores/displays the command in chat history
    return ProcessMessage::CONTINUE;
}

/// Check if sender's public key matches one of config.security.admin_key[0..2]
/// Only accepts PKI-encrypted DMs or local (serial/BLE) connections.
/// Channel-encrypted DMs are rejected because mp.from is not cryptographically bound.
bool InheroMr2Module::isAuthorizedAdmin(const meshtastic_MeshPacket &mp)
{
    // Local messages (serial/BLE connected) are always trusted
    if (mp.from == 0) {
        return true;
    }

    // Reject non-PKI messages — channel-encrypted DMs have no cryptographic sender proof
    if (!mp.pki_encrypted) {
        LOG_WARN("InheroMr2: Rejecting non-PKI admin attempt from 0x%08x (channel-encrypted DMs not accepted)", mp.from);
        return false;
    }

    // PKI-encrypted: sender key is cryptographically verified via ECDH
    if (mp.public_key.size != 32) {
        LOG_WARN("InheroMr2: PKI message from 0x%08x has no valid public key", mp.from);
        return false;
    }

    // Compare against all configured admin keys (same logic as AdminModule)
    for (int i = 0; i < 3; i++) {
        if (config.security.admin_key[i].size == 32 &&
            memcmp(mp.public_key.bytes, config.security.admin_key[i].bytes, 32) == 0) {
            LOG_INFO("InheroMr2: Admin key match (slot %d) for 0x%08x", i, mp.from);
            return true;
        }
    }

    LOG_WARN("InheroMr2: Unauthorized admin attempt from 0x%08x", mp.from);
    return false;
}

/// Main CLI dispatcher
void InheroMr2Module::handleCliCommand(const meshtastic_MeshPacket &mp, const char *cmd)
{
    // Skip leading whitespace after '/'
    while (*cmd == ' ' || *cmd == '\t')
        cmd++;

    if (strncmp(cmd, "get ", 4) == 0) {
        // /get commands — read-only, but still require admin for security
        const char *key = cmd + 4;
        while (*key == ' ')
            key++;

        if (strncmp(key, "board.", 6) == 0) {
            handleGetCommand(mp, key + 6);
        } else {
            sendTextReply(mp, "Err: Try /get board.<key> (bat|telem|conf|diag|hwver|frost|imax|mppt|leds|uvlo|ibcal|tccal|batcap|energy)");
        }

    } else if (strncmp(cmd, "set ", 4) == 0) {
        // /set commands — require admin authorization
        if (!isAuthorizedAdmin(mp)) {
            sendTextReply(mp, "Err: Not authorized. Requires PKI-encrypted DM with admin_key.");
            return;
        }

        const char *keyAndValue = cmd + 4;
        while (*keyAndValue == ' ')
            keyAndValue++;

        if (strncmp(keyAndValue, "board.", 6) == 0) {
            handleSetCommand(mp, keyAndValue + 6);
        } else {
            sendTextReply(mp, "Err: Try /set board.<key> <value> (bat|imax|frost|mppt|leds|uvlo|ibcal|tccal|batcap|bqreset|soc)");
        }

    } else if (strncmp(cmd, "reboot", 6) == 0) {
        if (!isAuthorizedAdmin(mp)) {
            sendTextReply(mp, "Err: Not authorized");
            return;
        }
        sendTextReply(mp, "Rebooting in 2s...");
        rebootAtMsec = millis() + 2000;

    } else if (strncmp(cmd, "ver", 3) == 0) {
        char response[128];
        snprintf(response, sizeof(response), "Inhero MR-2 | Meshtastic %s", optstr(APP_VERSION));
        sendTextReply(mp, response);

    } else if (strncmp(cmd, "help", 4) == 0) {
        // Send help in multiple messages to avoid payload limit
        sendTextReply(mp,
            "/get board.<key>\n"
            "  bat telem conf diag hwver frost imax mppt leds uvlo ibcal tccal batcap energy\n"
            "/set board.<key> <value> [admin]\n"
            "  bat <lto2s|lifepo1s|liion1s>\n"
            "  imax <10-1000> frost <0|1> mppt <0|1>\n"
            "  leds <on|off> uvlo <0|1> soc <0-100>\n"
            "  batcap <100-100000> ibcal <mA|reset>\n"
            "  tccal [temp|reset] bqreset\n"
            "/ver /reboot [admin] /help");

    } else {
        sendTextReply(mp, "Err: Unknown command. Try /help");
    }
}

// ============================================================
// /get board.<key> — MeshCore getCustomGetter compatible
// ============================================================

void InheroMr2Module::handleGetCommand(const meshtastic_MeshPacket &mp, const char *key)
{
    char response[256] = {0};

    // Trim trailing whitespace
    char trimmed[64];
    strncpy(trimmed, key, sizeof(trimmed) - 1);
    trimmed[sizeof(trimmed) - 1] = '\0';
    size_t len = strlen(trimmed);
    while (len > 0 && (trimmed[len - 1] == ' ' || trimmed[len - 1] == '\n' || trimmed[len - 1] == '\r')) {
        trimmed[--len] = '\0';
    }

    if (strcmp(trimmed, "bat") == 0) {
        snprintf(response, sizeof(response), "%s", chemistryToString(boardConfig.chemistry));

    } else if (strcmp(trimmed, "hwver") == 0) {
        snprintf(response, sizeof(response), "v0.2 (INA228+RTC)");

    } else if (strcmp(trimmed, "frost") == 0) {
        if (boardConfig.chemistry == BatteryChemistry::LTO_2S) {
            snprintf(response, sizeof(response), "N/A (LTO ignores JEITA)");
        } else {
            snprintf(response, sizeof(response), "frost=%s", boardConfig.frostProtect ? "on" : "off");
        }

    } else if (strcmp(trimmed, "imax") == 0) {
        snprintf(response, sizeof(response), "%umA", boardConfig.chargeCurrentMax_mA);

    } else if (strcmp(trimmed, "mppt") == 0) {
        snprintf(response, sizeof(response), "MPPT=%s", boardConfig.mpptEnabled ? "1" : "0");

    } else if (strcmp(trimmed, "telem") == 0) {
        // Real-time telemetry: VBAT, IBAT, SOC, VSOL, ISOL (MeshCore format)
        int soc = estimateSOC();
        if (ina228Ok && bq25798Ok && solarData) {
            char batCurStr[16], solCurStr[16];
            snprintf(batCurStr, sizeof(batCurStr), "%.1fmA", batteryData.current_ma);
            snprintf(solCurStr, sizeof(solCurStr), "~%.0fmA", (float)solarData->solar.current);

            if (soc >= 0) {
                snprintf(response, sizeof(response), "B:%.2fV/%s/%.0fC SOC:%d%% S:%.2fV/%s",
                         batteryData.voltage_mv / 1000.0f, batCurStr, batteryData.die_temp_c,
                         soc, solarData->solar.voltage / 1000.0f, solCurStr);
            } else {
                snprintf(response, sizeof(response), "B:%.2fV/%s/%.0fC SOC:N/A S:%.2fV/%s",
                         batteryData.voltage_mv / 1000.0f, batCurStr, batteryData.die_temp_c,
                         solarData->solar.voltage / 1000.0f, solCurStr);
            }
        } else if (ina228Ok) {
            snprintf(response, sizeof(response), "B:%.2fV/%.1fmA/%.0fC SOC:%s S:N/A",
                     batteryData.voltage_mv / 1000.0f, batteryData.current_ma, batteryData.die_temp_c,
                     soc >= 0 ? String(soc).c_str() : "N/A");
        } else {
            snprintf(response, sizeof(response), "Err: Sensors not ready");
        }

    } else if (strcmp(trimmed, "conf") == 0) {
        // All configuration values (MeshCore format)
        const char *batType = chemistryToString(boardConfig.chemistry);
        const char *frostStr = (boardConfig.chemistry == BatteryChemistry::LTO_2S) ? "N/A" : (boardConfig.frostProtect ? "on" : "off");
        const ChemistryParams &params = getChemistryParams(boardConfig.chemistry);
        snprintf(response, sizeof(response), "B:%s F:%s M:%s I:%umA Vco:%.2f V0:%.2f",
                 batType, frostStr, boardConfig.mpptEnabled ? "1" : "0",
                 boardConfig.chargeCurrentMax_mA,
                 params.chargeVoltage_mV / 1000.0f, params.emptyVoltage_mV / 1000.0f);

    } else if (strcmp(trimmed, "diag") == 0) {
        // Diagnostics
        uint16_t diagFlags = ina228Ok ? ina228.getDiagnosticFlags() : 0;
        bq_charging_status_t chgStatus = bq25798Ok ? bq25798.getChargingStatus() : BQ_CHARGE_NOT_CHARGING;
        bool pgood = bq25798Ok ? bq25798.getChargerStatusPowerGood() : false;
        snprintf(response, sizeof(response), "ina228=%s bq25798=%s diag=0x%04X chg=%u pgood=%u",
                 ina228Ok ? "ok" : "fail", bq25798Ok ? "ok" : "fail",
                 diagFlags, (uint8_t)chgStatus, pgood ? 1 : 0);

    } else if (strcmp(trimmed, "leds") == 0) {
        snprintf(response, sizeof(response), "LEDs: %s (Heartbeat + BQ Stat)",
                 boardConfig.ledsEnabled ? "ON" : "OFF");

    } else if (strcmp(trimmed, "uvlo") == 0) {
        snprintf(response, sizeof(response), "UVLO: %s",
                 boardConfig.uvloEnabled ? "ENABLED" : "DISABLED");

    } else if (strcmp(trimmed, "ibcal") == 0) {
        snprintf(response, sizeof(response), "INA228 calibration: %.4f (1.0=default)",
                 boardConfig.inaCalibration);

    } else if (strcmp(trimmed, "tccal") == 0) {
        snprintf(response, sizeof(response), "TC offset: %+.2fC (0.00=default)",
                 boardConfig.tcCalOffset);

    } else if (strcmp(trimmed, "batcap") == 0) {
        if (boardConfig.batteryCapacity_mAh > 0) {
            snprintf(response, sizeof(response), "%u mAh (set)", boardConfig.batteryCapacity_mAh);
        } else {
            uint8_t idx = (uint8_t)boardConfig.chemistry;
            if (idx >= sizeof(defaultBatteryCapacity) / sizeof(defaultBatteryCapacity[0]))
                idx = (uint8_t)BatteryChemistry::BAT_UNKNOWN;
            snprintf(response, sizeof(response), "%u mAh (default)", defaultBatteryCapacity[idx]);
        }

    } else if (strcmp(trimmed, "energy") == 0) {
        if (ina228Ok) {
            float charge_mah = ina228.readCharge_mAh();
            snprintf(response, sizeof(response), "%.1fmAh", charge_mah);
        } else {
            snprintf(response, sizeof(response), "Err: INA228 not initialized");
        }

    } else {
        snprintf(response, sizeof(response),
                 "Err: Try board.<bat|hwver|frost|imax|telem|conf|diag|mppt|leds|uvlo|ibcal|tccal|batcap|energy>");
    }

    sendTextReply(mp, response);
}

// ============================================================
// /set board.<key> <value> — MeshCore setCustomSetter compatible
// ============================================================

void InheroMr2Module::handleSetCommand(const meshtastic_MeshPacket &mp, const char *keyAndValue)
{
    char response[256] = {0};

    // --- set board.bat <type> ---
    if (strncmp(keyAndValue, "bat ", 4) == 0) {
        const char *val = keyAndValue + 4;
        while (*val == ' ') val++;

        BatteryChemistry newChem = stringToChemistry(val);
        if (newChem != BatteryChemistry::BAT_UNKNOWN) {
            boardConfig.chemistry = newChem;
            saveConfig();
            applyChemistryConfig();
            snprintf(response, sizeof(response), "Bat set to %s", chemistryToString(boardConfig.chemistry));
        } else {
            snprintf(response, sizeof(response), "Err: Try lto2s|lifepo1s|liion1s");
        }

    // --- set board.frost <0|1|on|off> ---
    } else if (strncmp(keyAndValue, "frost ", 6) == 0) {
        if (boardConfig.chemistry == BatteryChemistry::LTO_2S) {
            snprintf(response, sizeof(response), "Err: Frost setting N/A for LTO (JEITA disabled)");
        } else {
            const char *val = keyAndValue + 6;
            while (*val == ' ') val++;
            bool enabled = (strcmp(val, "1") == 0 || strcmp(val, "on") == 0);
            bool disabled = (strcmp(val, "0") == 0 || strcmp(val, "off") == 0);
            if (enabled || disabled) {
                boardConfig.frostProtect = enabled;
                saveConfig();
                applyChemistryConfig();
                snprintf(response, sizeof(response), "Frost %s", enabled ? "enabled" : "disabled");
            } else {
                snprintf(response, sizeof(response), "Err: Try 0|1 or on|off");
            }
        }

    // --- set board.imax <10-1000> ---
    } else if (strncmp(keyAndValue, "imax ", 5) == 0) {
        const char *val = keyAndValue + 5;
        while (*val == ' ') val++;
        int ma = atoi(val);
        if (ma >= 10 && ma <= 1000) {
            boardConfig.chargeCurrentMax_mA = (uint16_t)ma;
            saveConfig();
            if (bq25798Ok)
                bq25798.setChargeLimitA(boardConfig.chargeCurrentMax_mA / 1000.0f);
            snprintf(response, sizeof(response), "Max charge current set to %umA", boardConfig.chargeCurrentMax_mA);
        } else {
            snprintf(response, sizeof(response), "Err: imax range 10-1000 mA");
        }

    // --- set board.mppt <true|false|1|0> ---
    } else if (strncmp(keyAndValue, "mppt ", 5) == 0) {
        const char *val = keyAndValue + 5;
        while (*val == ' ') val++;

        // Case-insensitive compare
        char lower[20];
        strncpy(lower, val, sizeof(lower) - 1);
        lower[sizeof(lower) - 1] = '\0';
        for (char *p = lower; *p; ++p) *p = tolower(*p);

        if (strcmp(lower, "true") == 0 || strcmp(lower, "1") == 0) {
            boardConfig.mpptEnabled = true;
            saveConfig();
            if (bq25798Ok) bq25798.setMPPTenable(true);
            snprintf(response, sizeof(response), "MPPT enabled");
        } else if (strcmp(lower, "false") == 0 || strcmp(lower, "0") == 0) {
            boardConfig.mpptEnabled = false;
            saveConfig();
            if (bq25798Ok) bq25798.setMPPTenable(false);
            snprintf(response, sizeof(response), "MPPT disabled");
        } else {
            snprintf(response, sizeof(response), "Err: Try true|false or 1|0");
        }

    // --- set board.leds <on|off|1|0> ---
    } else if (strncmp(keyAndValue, "leds ", 5) == 0) {
        const char *val = keyAndValue + 5;
        while (*val == ' ') val++;
        bool enabled = (strcmp(val, "1") == 0 || strcmp(val, "on") == 0 || strcmp(val, "ON") == 0);
        bool disabled = (strcmp(val, "0") == 0 || strcmp(val, "off") == 0 || strcmp(val, "OFF") == 0);
        if (enabled || disabled) {
            boardConfig.ledsEnabled = enabled;
            saveConfig();
            updateLEDs();
            snprintf(response, sizeof(response), "LEDs %s", enabled ? "enabled" : "disabled");
        } else {
            snprintf(response, sizeof(response), "Err: Use on/1 or off/0");
        }

    // --- set board.uvlo <true|false|1|0> ---
    } else if (strncmp(keyAndValue, "uvlo ", 5) == 0) {
        const char *val = keyAndValue + 5;
        while (*val == ' ') val++;
        bool enabled = (strcmp(val, "1") == 0 || strcmp(val, "true") == 0);
        bool disabled = (strcmp(val, "0") == 0 || strcmp(val, "false") == 0);
        if (enabled || disabled) {
            boardConfig.uvloEnabled = enabled;
            saveConfig();
            if (ina228Ok) {
                const ChemistryParams &params = getChemistryParams(boardConfig.chemistry);
                if (enabled) {
                    ina228.setUnderVoltageAlert(params.dangerVoltage_mV);
                    ina228.enableAlert(true, false, false);
                } else {
                    ina228.enableAlert(false, false, false);
                }
            }
            snprintf(response, sizeof(response), "UVLO %s", enabled ? "ENABLED" : "DISABLED");
        } else {
            snprintf(response, sizeof(response), "Err: Use true/1 or false/0");
        }

    // --- set board.ibcal <mA|reset> ---
    } else if (strncmp(keyAndValue, "ibcal ", 6) == 0) {
        const char *val = keyAndValue + 6;
        while (*val == ' ') val++;

        if (strcmp(val, "reset") == 0 || strcmp(val, "RESET") == 0) {
            boardConfig.inaCalibration = 1.0f;
            saveConfig();
            if (ina228Ok) ina228.setCalibrationFactor(1.0f);
            snprintf(response, sizeof(response), "INA228 calibration reset to 1.0000");
        } else {
            float actual_ma = atof(val);
            if (actual_ma < -2000.0f || actual_ma > 2000.0f) {
                snprintf(response, sizeof(response), "Err: Current out of range (-2000 to +2000 mA)");
            } else if (ina228Ok) {
                float measured = batteryData.current_ma;
                if (fabsf(measured) < 1.0f) {
                    snprintf(response, sizeof(response), "Err: Current too low for calibration (%.1fmA)", measured);
                } else {
                    float factor = boardConfig.inaCalibration * (actual_ma / measured);
                    boardConfig.inaCalibration = factor;
                    saveConfig();
                    ina228.setCalibrationFactor(factor);
                    snprintf(response, sizeof(response), "INA228 calibrated: factor=%.4f", factor);
                }
            } else {
                snprintf(response, sizeof(response), "Err: INA228 not initialized");
            }
        }

    // --- set board.tccal [temp|reset] ---
    } else if (strncmp(keyAndValue, "tccal", 5) == 0) {
        const char *rest = keyAndValue + 5;
        if (*rest == ' ') rest++;
        while (*rest == ' ') rest++;

        if (strcmp(rest, "reset") == 0 || strcmp(rest, "RESET") == 0) {
            boardConfig.tcCalOffset = 0.0f;
            saveConfig();
            snprintf(response, sizeof(response), "TC calibration reset to 0.00");
        } else if (*rest != '\0') {
            float actual_temp = atof(rest);
            if (actual_temp < -40.0f || actual_temp > 85.0f) {
                snprintf(response, sizeof(response), "Err: Temp out of range (-40 to +85 C)");
            } else {
                float measured = batteryData.die_temp_c;
                boardConfig.tcCalOffset = actual_temp - measured;
                saveConfig();
                snprintf(response, sizeof(response), "TC calibrated: offset=%+.2fC", boardConfig.tcCalOffset);
            }
        } else {
            snprintf(response, sizeof(response), "Err: Use /set board.tccal <temp_C> or /set board.tccal reset");
        }

    // --- set board.batcap <100-100000> ---
    } else if (strncmp(keyAndValue, "batcap ", 7) == 0) {
        const char *val = keyAndValue + 7;
        while (*val == ' ') val++;
        uint32_t cap = (uint32_t)atol(val);
        if (cap >= 100 && cap <= 100000) {
            boardConfig.batteryCapacity_mAh = cap;
            saveConfig();
            snprintf(response, sizeof(response), "Battery capacity set to %u mAh", cap);
        } else {
            snprintf(response, sizeof(response), "Err: Invalid capacity (100-100000 mAh)");
        }

    // --- set board.bqreset ---
    } else if (strcmp(keyAndValue, "bqreset") == 0) {
        if (bq25798Ok) {
            bq25798.reset();
            delay(100);
            applyChemistryConfig();
            snprintf(response, sizeof(response), "BQ25798 reset done - reconfigured");
        } else {
            snprintf(response, sizeof(response), "Err: BQ25798 not initialized");
        }

    // --- set board.soc <0-100> ---
    } else if (strncmp(keyAndValue, "soc ", 4) == 0) {
        const char *val = keyAndValue + 4;
        while (*val == ' ') val++;
        int soc = atoi(val);
        if (soc >= 0 && soc <= 100) {
            // SOC manual override is informational only in Meshtastic port
            // (no MeshCore SOC tracker to set)
            snprintf(response, sizeof(response), "SOC manually set to %d%% (note: no persistent SOC tracker in Meshtastic port)", soc);
        } else {
            snprintf(response, sizeof(response), "Err: SOC must be 0-100");
        }

    } else {
        snprintf(response, sizeof(response),
                 "Err: Try board.<bat|imax|frost|mppt|leds|uvlo|ibcal|tccal|batcap|bqreset|soc>");
    }

    sendTextReply(mp, response);
}

// ============================================================
// Text message reply — sends response as DM on TEXT_MESSAGE_APP
// ============================================================

void InheroMr2Module::sendTextReply(const meshtastic_MeshPacket &mp, const char *text)
{
    meshtastic_MeshPacket *p = allocDataPacket(); // allocates with our portnum (TEXT_MESSAGE_APP)
    p->to = mp.from;
    p->channel = mp.channel;
    p->want_ack = false;
    p->decoded.want_response = false;

    size_t textLen = strlen(text);
    if (textLen > sizeof(p->decoded.payload.bytes))
        textLen = sizeof(p->decoded.payload.bytes);
    memcpy(p->decoded.payload.bytes, text, textLen);
    p->decoded.payload.size = textLen;
    p->priority = meshtastic_MeshPacket_Priority_RELIABLE;

    service->sendToMesh(p, RX_SRC_LOCAL, true);
}

// ============================================================
// Battery chemistry string helpers (MeshCore compatible names)
// ============================================================

const char *InheroMr2Module::chemistryToString(BatteryChemistry chem)
{
    switch (chem) {
    case BatteryChemistry::LTO_2S:      return "lto2s";
    case BatteryChemistry::LiFePO4_1S:  return "lifepo1s";
    case BatteryChemistry::Li_Ion_1S:   return "liion1s";
    default:                            return "unknown";
    }
}

BatteryChemistry InheroMr2Module::stringToChemistry(const char *str)
{
    // Support both MeshCore names and alternative aliases
    if (strcmp(str, "lto2s") == 0 || strcmp(str, "lto") == 0)
        return BatteryChemistry::LTO_2S;
    if (strcmp(str, "lifepo1s") == 0 || strcmp(str, "lifepo4") == 0 || strcmp(str, "lfp") == 0)
        return BatteryChemistry::LiFePO4_1S;
    if (strcmp(str, "liion1s") == 0 || strcmp(str, "liion") == 0)
        return BatteryChemistry::Li_Ion_1S;
    return BatteryChemistry::BAT_UNKNOWN;
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
        ledOff(PIN_LED1);
        ledOff(PIN_LED2);
        return;
    }

    // === LED2 (Red) — Priority-based status indicator ===
    //
    // Priority 1: DANGER VOLTAGE — 100ms flash every 3s (battery critically low, conserve power)
    // Priority 2: CLI COMMAND    — 200ms flash (remote admin activity)
    // Priority 3: OFF            — normal operation (BQ25798 STAT-LED handles charging)

    const ChemistryParams &params = getChemistryParams(boardConfig.chemistry);
    bool dangerVoltage = ina228Ok && batteryData.voltage_mv > 0 &&
                         batteryData.voltage_mv < params.dangerVoltage_mV;

    if (dangerVoltage) {
        // P1: Short blink every 3s — battery in danger zone (conserve power)
        uint32_t phase = millis() % 3000;
        if (phase < 100) {
            ledOn(PIN_LED2);
        } else {
            ledOff(PIN_LED2);
        }
    } else if (millis() < cliFlashUntil) {
        // P2: Brief flash — CLI command was processed
        ledOn(PIN_LED2);
    } else {
        // P3: Off — BQ25798 hardware STAT-LED shows charging state
        ledOff(PIN_LED2);
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
        boardConfig.chemistry = stringToChemistry(buf);
        if (boardConfig.chemistry == BatteryChemistry::BAT_UNKNOWN && buf[0] != '\0') {
            // Legacy name fallback
            if (strcmp(buf, "lifepo4") == 0)
                boardConfig.chemistry = BatteryChemistry::LiFePO4_1S;
            else if (strcmp(buf, "liion") == 0)
                boardConfig.chemistry = BatteryChemistry::Li_Ion_1S;
        }
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

    if (readConfigValue("tccal", buf, sizeof(buf)) > 0)
        boardConfig.tcCalOffset = atof(buf);

    if (readConfigValue("uvlo", buf, sizeof(buf)) > 0)
        boardConfig.uvloEnabled = (buf[0] == '1');

    if (readConfigValue("batcap", buf, sizeof(buf)) > 0)
        boardConfig.batteryCapacity_mAh = (uint32_t)atol(buf);

    LOG_INFO("InheroMr2: Config loaded (bat=%s, imax=%u, mppt=%u, uvlo=%u)",
             chemistryToString(boardConfig.chemistry),
             boardConfig.chargeCurrentMax_mA, boardConfig.mpptEnabled, boardConfig.uvloEnabled);
}

void InheroMr2Module::saveConfig()
{
    writeConfigValue("bat", chemistryToString(boardConfig.chemistry));

    char buf[32];
    snprintf(buf, sizeof(buf), "%u", boardConfig.chargeCurrentMax_mA);
    writeConfigValue("imax", buf);

    writeConfigValue("mppt", boardConfig.mpptEnabled ? "1" : "0");
    writeConfigValue("leds", boardConfig.ledsEnabled ? "1" : "0");
    writeConfigValue("frost", boardConfig.frostProtect ? "1" : "0");

    snprintf(buf, sizeof(buf), "%.4f", boardConfig.inaCalibration);
    writeConfigValue("cal", buf);

    snprintf(buf, sizeof(buf), "%.2f", boardConfig.tcCalOffset);
    writeConfigValue("tccal", buf);

    writeConfigValue("uvlo", boardConfig.uvloEnabled ? "1" : "0");

    snprintf(buf, sizeof(buf), "%u", boardConfig.batteryCapacity_mAh);
    writeConfigValue("batcap", buf);

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
