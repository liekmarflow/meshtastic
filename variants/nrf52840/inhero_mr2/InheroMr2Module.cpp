/*
 * Copyright (c) 2026 Inhero GmbH
 *
 * SPDX-License-Identifier: MIT
 *
 * Inhero MR-2 Board Module - Implementation
 * Full MeshCore feature parity via Meshtastic text message DMs.
 *
 * This module implements the complete Inhero MR-2 board support:
 *
 * === Hardware Drivers ===
 *  - INA228 power monitor (24-bit, Coulomb counting)
 *  - BQ25798 solar charger (MPPT, JEITA, multi-chemistry)
 *  - RV-3028 RTC detection (for timestamps + wake-from-sleep)
 *
 * === Power Management ===
 *  - Battery chemistry: LTO 2S / LiFePO4 1S / Li-Ion 1S
 *  - SOC: INA228 hardware Coulomb counting, auto-sync on charge-done
 *  - UVLO: Hardware alert via INA228, danger-zone shutdown
 *  - skipFsWrites: Prevents flash corruption on low-voltage boot
 *
 * === System Reliability ===
 *  - nRF52 hardware watchdog (600s timeout)
 *  - GPREGRET2 shutdown reason tracking (survives SYSTEMOFF)
 *  - Controlled shutdown: SX1262 off → RTC wake → SYSTEMOFF
 *  - Error LED blink for missing I2C components
 *
 * === Solar MPPT Management ===
 *  - Stuck PGOOD fix (HIZ toggle with 5-min cooldown)
 *  - MPPT re-enable after BQ fault events (60s cooldown)
 *
 * === Energy Analytics (168h) ===
 *  - Hourly rolling buffer for charge/discharge/solar (7 days)
 *  - Rolling averages: 24h / 3-day / 7-day
 *  - TTL (Time To Live) estimation
 *  - MPPT statistics (enabled minutes, harvested energy)
 *
 * === CLI over Text-DM ===
 *  - /get board.<key>: bat telem conf diag hwver fmax imax mppt
 *                       leds uvlo ibcal tccal batcap energy
 *                       stats cinfo togglehiz
 *  - /set board.<key> <value>: (admin required)
 *  - /help /ver /reboot
 *  - Auth: config.security.admin_key[] (PKI pubkeys)
 *
 * === Telemetry ===
 *  - PowerMetrics Protobuf: ch1=Battery, ch2=Solar, ch3=System
 *  - HasBatteryLevel integration via InheroMr2BatteryLevel (Power.cpp)
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

#ifdef ARCH_NRF52
#include <nrf_wdt.h>
#include <nrf_soc.h>
#endif

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

    // Check GPREGRET2 for low-voltage boot — skip flash writes to prevent corruption
#ifdef ARCH_NRF52
    uint8_t shutdownReason = NRF_POWER->GPREGRET2;
    if ((shutdownReason & 0x03) == SHUTDOWN_REASON_LOW_VOLTAGE) {
        skipFsWrites = true;
        LOG_WARN("InheroMr2: Low-voltage boot detected (GPREGRET2=0x%02X) — skipping FS writes", shutdownReason);

        // Clear GPREGRET2 so a subsequent warm reset doesn't re-trigger this path
        sd_power_gpregret_clr(1, 0xFF);
    }
#endif

    // Load persistent configuration (respects skipFsWrites for first-boot defaults)
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
        // Apply battery chemistry config to charger (also sets CE pin)
        applyChemistryConfig();
        // Configure interrupts for solar events only
        bq25798.configureSolarOnlyInterrupts();
        // Configure STAT LED based on user preference
        bq25798.setStatPinEnable(boardConfig.ledsEnabled);
    } else {
        LOG_WARN("InheroMr2: BQ25798 FAILED");
        // BQ not found — ensure CE stays HIGH (charging disabled)
#ifdef BQ_CE_PIN
        pinMode(BQ_CE_PIN, OUTPUT);
        digitalWrite(BQ_CE_PIN, HIGH);
#endif
    }

    // Detect RV-3028 RTC for hourly stats timestamps
    Wire.beginTransmission(0x52);
    rtcOk = (Wire.endTransmission() == 0);
    if (rtcOk) {
        LOG_INFO("InheroMr2: RV-3028 RTC OK @ 0x52");
    } else {
        LOG_WARN("InheroMr2: RV-3028 RTC not found");
    }

    // Initialize SOC stats capacity from config
    socStats.capacity_mah = (float)getEffectiveCapacity();

    // After danger zone recovery, set SOC to 0% — battery was critically low,
    // solar just charged it past the threshold. Coulomb counting starts from 0%
    // and auto-corrects to 100% when BQ25798 signals "Charging Done".
    if (skipFsWrites && ina228Ok) {
        setSOCManually(0.0f);
        LOG_INFO("InheroMr2: SOC set to 0%% (danger zone recovery)");
    }

    // Initialize hourly stats timestamp
    uint32_t now_time = getTime();
    if (now_time > 1000000000) {
        socStats.lastHourUpdateTime = (now_time / 3600) * 3600;
    } else {
        socStats.lastHourUpdateTime = 0; // Will init on first RTC availability
    }

    // Error LED: if any critical component missing, enable error blink
    if (!ina228Ok || !bq25798Ok || !rtcOk) {
        errorLedActive = true;
        LOG_WARN("InheroMr2: Missing components — error LED active");
        if (!ina228Ok) LOG_WARN("  - INA228 missing");
        if (!bq25798Ok) LOG_WARN("  - BQ25798 missing");
        if (!rtcOk) LOG_WARN("  - RV-3028 RTC missing");
    }

    // Start hardware watchdog (600s timeout, nRF52 only)
    setupWatchdog();

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

    // Feed hardware watchdog every cycle
    feedWatchdog();

    // Read INA228 battery data
    if (ina228Ok) {
        ina228.readAll(&batteryData);

        // Voltage danger check
        const ChemistryParams &params = getChemistryParams(boardConfig.chemistry);
        if (batteryData.voltage_mv > 0 && batteryData.voltage_mv < params.dangerVoltage_mV) {
            LOG_WARN("InheroMr2: Battery DANGER! %umV < %umV danger threshold", batteryData.voltage_mv,
                     params.dangerVoltage_mV);
            // Initiate controlled shutdown with SX1262 power-off and RTC wake
            initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE);
            // If initiateShutdown returns (shouldn't), fall through
        }
    }

    // Read BQ25798 solar data periodically (has 250ms ADC delay, don't read every cycle)
    if (bq25798Ok && (now - lastSensorRead > 30000 || lastSensorRead == 0)) {
        solarData = bq25798.getTelemetryData();
        lastSensorRead = now;

        // === Solar Power Management (ported from MeshCore) ===
        // Check for stuck PGOOD and MPPT re-enable on each solar data read
        checkAndFixPgoodStuck();
        checkAndFixSolarLogic();
    }

    // Update Coulomb counting SOC (every 10s cycle)
    updateBatterySOC();

    // Update MPPT statistics
    if (bq25798Ok && boardConfig.mpptEnabled) {
        updateMpptStats();
    }

    // Send telemetry periodically
    if (now - lastTelemetrySend > telemetryIntervalMs || lastTelemetrySend == 0) {
        sendPowerTelemetry();
        lastTelemetrySend = now;
    }

    // Minute counter for hourly stats (10s * 6 = 60s)
    minuteCounter++;
    if (minuteCounter >= 6) { // Every ~60 seconds
        minuteCounter = 0;
        // Update hourly statistics (will save when hour boundary crossed)
        updateHourlyStats();
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
            sendTextReply(mp, "Err: Try /get board.<key> (bat|telem|conf|diag|hwver|fmax|imax|mppt|leds|uvlo|ibcal|tccal|batcap|energy|stats|cinfo|togglehiz)");
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
            sendTextReply(mp, "Err: Try /set board.<key> <value> (bat|imax|fmax|mppt|leds|uvlo|ibcal|tccal|batcap|bqreset|soc)");
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
            "  bat telem conf diag hwver fmax imax mppt\n"
            "  leds uvlo ibcal tccal batcap energy\n"
            "  stats cinfo togglehiz\n"
            "/set board.<key> <value> [admin]\n"
            "  bat <lto2s|lifepo1s|liion1s>\n"
            "  imax <10-1000> fmax <0|1> mppt <0|1>\n"
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

    } else if (strcmp(trimmed, "fmax") == 0) {
        if (boardConfig.chemistry == BatteryChemistry::LTO_2S) {
            snprintf(response, sizeof(response), "N/A (LTO ignores JEITA)");
        } else {
            snprintf(response, sizeof(response), "fmax=%s", boardConfig.frostProtect ? "on" : "off");
        }

    } else if (strcmp(trimmed, "imax") == 0) {
        snprintf(response, sizeof(response), "%umA", boardConfig.chargeCurrentMax_mA);

    } else if (strcmp(trimmed, "mppt") == 0) {
        snprintf(response, sizeof(response), "MPPT=%s", boardConfig.mpptEnabled ? "1" : "0");

    } else if (strcmp(trimmed, "telem") == 0) {
        // Real-time telemetry: VBAT, IBAT, SOC, VSOL, ISOL (MeshCore format)
        int soc = estimateSOC();
        const char *socMethod = socStats.soc_valid ? "CC" : "V"; // CC=Coulomb Counting, V=Voltage
        if (ina228Ok && bq25798Ok && solarData) {
            char batCurStr[16], solCurStr[16];
            snprintf(batCurStr, sizeof(batCurStr), "%.1fmA", batteryData.current_ma);
            snprintf(solCurStr, sizeof(solCurStr), "~%.0fmA", (float)solarData->solar.current);

            if (soc >= 0) {
                snprintf(response, sizeof(response), "B:%.2fV/%s/%.0fC SOC:%d%%(%s) S:%.2fV/%s",
                         batteryData.voltage_mv / 1000.0f, batCurStr, batteryData.die_temp_c,
                         soc, socMethod, solarData->solar.voltage / 1000.0f, solCurStr);
            } else {
                snprintf(response, sizeof(response), "B:%.2fV/%s/%.0fC SOC:N/A S:%.2fV/%s",
                         batteryData.voltage_mv / 1000.0f, batCurStr, batteryData.die_temp_c,
                         solarData->solar.voltage / 1000.0f, solCurStr);
            }
        } else if (ina228Ok) {
            snprintf(response, sizeof(response), "B:%.2fV/%.1fmA/%.0fC SOC:%s%s S:N/A",
                     batteryData.voltage_mv / 1000.0f, batteryData.current_ma, batteryData.die_temp_c,
                     soc >= 0 ? String(soc).c_str() : "N/A",
                     soc >= 0 ? (socStats.soc_valid ? "%(CC)" : "%(V)") : "");
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

    } else if (strcmp(trimmed, "stats") == 0) {
        // Rolling energy statistics (24h / 3d / 7d averages + TTL)
        if (socStats.currentIndex == 0 && socStats.hours[0].timestamp == 0) {
            snprintf(response, sizeof(response), "Stats: No data yet (need >= 1 hour)");
        } else {
            char ttlBuf[16];
            if (socStats.ttl_hours >= 24) {
                snprintf(ttlBuf, sizeof(ttlBuf), "%ud%uh", socStats.ttl_hours / 24, socStats.ttl_hours % 24);
            } else {
                snprintf(ttlBuf, sizeof(ttlBuf), "%uh", socStats.ttl_hours);
            }
            snprintf(response, sizeof(response),
                     "24h:%.0f/%.0fmAh 3d:%.0f/%.0f 7d:%.0f/%.0f TTL:%s MPPT:%.0f%%",
                     socStats.last_24h_charged_mah, socStats.last_24h_discharged_mah,
                     socStats.avg_3day_daily_charged_mah, socStats.avg_3day_daily_discharged_mah,
                     socStats.avg_7day_daily_charged_mah, socStats.avg_7day_daily_discharged_mah,
                     ttlBuf, getMpptEnabledPercentage7Day());
        }

    } else if (strcmp(trimmed, "cinfo") == 0) {
        // Charger info: PG status, charging state, HIZ, MPPT
        if (!bq25798Ok) {
            snprintf(response, sizeof(response), "Err: BQ25798 not initialized");
        } else {
            bool pgood = bq25798.getChargerStatusPowerGood();
            bq_charging_status_t chgSt = bq25798.getChargingStatus();
            bool hiz = bq25798.getHIZMode();
            bool mpptEn = bq25798.getMPPTenable();
            uint16_t vbus = solarData ? solarData->solar.voltage : 0;
            snprintf(response, sizeof(response),
                     "PG:%u CHG:%u HIZ:%u MPPT:%u VBUS:%.2fV",
                     pgood ? 1 : 0, (uint8_t)chgSt, hiz ? 1 : 0, mpptEn ? 1 : 0,
                     vbus / 1000.0f);
        }

    } else if (strcmp(trimmed, "togglehiz") == 0) {
        // Manual HIZ toggle for debugging stuck PGOOD
        if (!bq25798Ok) {
            snprintf(response, sizeof(response), "Err: BQ25798 not initialized");
        } else {
            bool wasPG = bq25798.getChargerStatusPowerGood();
            bq25798.setHIZMode(true);
            delay(200);
            bq25798.setHIZMode(false);
            delay(500);
            bool isPG = bq25798.getChargerStatusPowerGood();
            bq_charging_status_t st = bq25798.getChargingStatus();
            snprintf(response, sizeof(response),
                     "HIZ cycled: PG %u->%u CHG:%u",
                     wasPG ? 1 : 0, isPG ? 1 : 0, (uint8_t)st);
        }

    } else {
        snprintf(response, sizeof(response),
                 "Err: Try board.<bat|hwver|fmax|imax|telem|conf|diag|mppt|leds|uvlo|ibcal|tccal|batcap|energy|stats|cinfo|togglehiz>");
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

    // --- set board.fmax <0|1|on|off> ---
    } else if (strncmp(keyAndValue, "fmax ", 5) == 0) {
        if (boardConfig.chemistry == BatteryChemistry::LTO_2S) {
            snprintf(response, sizeof(response), "Err: Fmax setting N/A for LTO (JEITA disabled)");
        } else {
            const char *val = keyAndValue + 5;
            while (*val == ' ') val++;
            bool enabled = (strcmp(val, "1") == 0 || strcmp(val, "on") == 0);
            bool disabled = (strcmp(val, "0") == 0 || strcmp(val, "off") == 0);
            if (enabled || disabled) {
                boardConfig.frostProtect = enabled;
                saveConfig();
                applyChemistryConfig();
                snprintf(response, sizeof(response), "Fmax %s", enabled ? "enabled" : "disabled");
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
                    ina228.enableAlert(true, false, true);  // active-LOW, LATCHED
                } else {
                    ina228.setUnderVoltageAlert(0);  // Clear threshold → disables comparison
                    ina228.enableAlert(false, false, false);  // Clear all DIAG_ALRT config
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
        float soc = atof(val);
        if (soc >= 0.0f && soc <= 100.0f) {
            if (setSOCManually(soc)) {
                snprintf(response, sizeof(response), "SOC set to %.0f%% (Coulomb counting baseline recalculated, cap=%umAh)",
                         soc, getEffectiveCapacity());
            } else {
                snprintf(response, sizeof(response), "Err: INA228 not ready or capacity=0");
            }
        } else {
            snprintf(response, sizeof(response), "Err: SOC must be 0-100");
        }

    } else {
        snprintf(response, sizeof(response),
                 "Err: Try board.<bat|imax|fmax|mppt|leds|uvlo|ibcal|tccal|batcap|bqreset|soc>");
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

    // CE-Pin hardware safety: Only enable charging for known battery chemistry.
    // BAT_UNKNOWN keeps CE HIGH (charging disabled) as hardware-level protection
    // against overcharging an unconfigured battery (e.g. LiFePO4 at 4.2V default).
    bool chargeEnabled = (boardConfig.chemistry != BatteryChemistry::BAT_UNKNOWN);
#ifdef BQ_CE_PIN
    pinMode(BQ_CE_PIN, OUTPUT);
    digitalWrite(BQ_CE_PIN, chargeEnabled ? LOW : HIGH);
    LOG_INFO("InheroMr2: CE pin %s (chemistry=%s)",
             chargeEnabled ? "LOW (charging enabled)" : "HIGH (charging disabled)",
             chemistryToString(boardConfig.chemistry));
#endif

    // Also set the I2C charge enable register (defense in depth)
    bq25798.setChargeEnable(chargeEnabled);

    if (!chargeEnabled) {
        LOG_WARN("InheroMr2: Chemistry UNKNOWN — charging disabled (CE + register)");
        return;
    }

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

    // Configure INA228 UVLO alert based on chemistry (only if UVLO enabled)
    if (ina228Ok && boardConfig.uvloEnabled) {
        ina228.setUnderVoltageAlert(params.dangerVoltage_mV);
        ina228.enableAlert(true, false, true); // UVLO, active-low, latched
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
    // Priority 1: DANGER VOLTAGE  — 100ms flash every 3s (battery critically low, conserve power)
    // Priority 2: ERROR CONDITION — 500ms on/off blink (missing I2C component)
    // Priority 3: CLI COMMAND     — 200ms flash (remote admin activity)
    // Priority 4: OFF             — normal operation (BQ25798 STAT-LED handles charging)

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
    } else if (errorLedActive) {
        // P2: 500ms on/off — missing I2C component (INA228 or BQ25798)
        uint32_t phase = millis() % 1000;
        if (phase < 500) {
            ledOn(PIN_LED2);
        } else {
            ledOff(PIN_LED2);
        }
    } else if (millis() < cliFlashUntil) {
        // P3: Brief flash — CLI command was processed
        ledOn(PIN_LED2);
    } else {
        // P4: Off — BQ25798 hardware STAT-LED shows charging state
        ledOff(PIN_LED2);
    }
}

int InheroMr2Module::estimateSOC()
{
    // Primary: Coulomb counting (if calibrated)
    if (socStats.soc_valid)
        return constrain((int)socStats.current_soc_percent, 0, 100);

    // Fallback: Voltage-based estimation (before first charge-done calibration)
    return estimateSOCFromVoltage();
}

int InheroMr2Module::estimateSOCFromVoltage()
{
    if (!ina228Ok || batteryData.voltage_mv == 0)
        return -1;

    const ChemistryParams &params = getChemistryParams(boardConfig.chemistry);

    if (batteryData.voltage_mv >= params.fullVoltage_mV)
        return 100;
    if (batteryData.voltage_mv <= params.emptyVoltage_mV)
        return 0;

    int soc =
        (int)(((float)(batteryData.voltage_mv - params.emptyVoltage_mV) / (params.fullVoltage_mV - params.emptyVoltage_mV)) *
              100.0f);
    return constrain(soc, 0, 100);
}

uint32_t InheroMr2Module::getEffectiveCapacity()
{
    if (boardConfig.batteryCapacity_mAh > 0)
        return boardConfig.batteryCapacity_mAh;
    uint8_t idx = (uint8_t)boardConfig.chemistry;
    if (idx >= sizeof(defaultBatteryCapacity) / sizeof(defaultBatteryCapacity[0]))
        idx = (uint8_t)BatteryChemistry::BAT_UNKNOWN;
    return defaultBatteryCapacity[idx];
}

void InheroMr2Module::updateBatterySOC()
{
    if (!ina228Ok)
        return;

    // Read INA228 hardware Coulomb counter (mAh)
    // Positive = charging (into battery), Negative = discharging
    float charge_mah = ina228.readCharge_mAh();

    // Delta tracking (runs always, independent of SOC validity)
    if (firstChargeRead) {
        lastChargeMah = charge_mah;
        firstChargeRead = false;
    } else {
        float delta_mah = charge_mah - lastChargeMah;
        lastChargeMah = charge_mah;

        // Ignore huge jumps > 10Ah (counter wrap or reset)
        if (delta_mah > 10000.0f || delta_mah < -10000.0f) {
            LOG_WARN("InheroMr2: Coulomb counter jump ignored (%.1f mAh)", delta_mah);
        }
    }

    // Auto-sync: BQ25798 reports "Charging Done" -> set SOC=100%
    if (bq25798Ok) {
        bq_charging_status_t status = bq25798.getChargingStatus();
        if (status == BQ_CHARGE_DONE) {
            if (!socStats.soc_valid) {
                syncSOCToFull(); // First calibration point
                LOG_INFO("InheroMr2: SOC calibrated to 100%% (first charge-done)");
            } else if (socStats.current_soc_percent < 99.0f) {
                syncSOCToFull(); // Re-sync drift
                LOG_INFO("InheroMr2: SOC re-synced to 100%% (charge-done)");
            }
        }
    }

    if (!socStats.soc_valid)
        return; // Wait for first calibration

    // === Core SOC formula (ported from MeshCore) ===
    // Net charge since baseline reset
    float net_charge_mah = charge_mah - socStats.ina228_baseline_mah;

    // Remaining = capacity + net charge (net is negative when discharging)
    uint32_t capacity = getEffectiveCapacity();
    if (capacity == 0)
        return;

    float remaining_mah = (float)capacity + net_charge_mah;
    socStats.current_soc_percent = (remaining_mah / (float)capacity) * 100.0f;

    // Clamp
    if (socStats.current_soc_percent > 100.0f)
        socStats.current_soc_percent = 100.0f;
    if (socStats.current_soc_percent < 0.0f)
        socStats.current_soc_percent = 0.0f;
}

void InheroMr2Module::syncSOCToFull()
{
    if (!ina228Ok)
        return;

    // Reset INA228 hardware Coulomb counter (clears CHARGE + ENERGY registers)
    ina228.resetCoulombCounter();

    // Set baseline to 0 (counter was just reset)
    socStats.ina228_baseline_mah = 0;

    // Mark as fully charged
    socStats.current_soc_percent = 100.0f;
    socStats.soc_valid = true;
}

bool InheroMr2Module::setSOCManually(float soc_percent)
{
    if (!ina228Ok)
        return false;
    if (soc_percent < 0.0f || soc_percent > 100.0f)
        return false;

    uint32_t capacity = getEffectiveCapacity();
    if (capacity == 0)
        return false;

    // Read current CHARGE register value
    float current_charge_mah = ina228.readCharge_mAh();

    // Calculate baseline so that: remaining = capacity + (charge - baseline)
    // remaining = (soc/100) * capacity
    // baseline = charge - (remaining - capacity)
    float remaining_mah = (soc_percent / 100.0f) * (float)capacity;
    socStats.ina228_baseline_mah = current_charge_mah - (remaining_mah - (float)capacity);

    socStats.current_soc_percent = soc_percent;
    socStats.soc_valid = true;
    return true;
}

bool InheroMr2Module::isBatteryCharging() const
{
    if (!bq25798Ok)
        return false;
    // const_cast needed because getChargingStatus() is not const in the driver
    bq_charging_status_t status = const_cast<BQ25798Driver &>(bq25798).getChargingStatus();
    return (status >= BQ_CHARGE_TRICKLE && status <= BQ_CHARGE_DONE && status != BQ_CHARGE_NOT_CHARGING);
}

// ============================================================
// Watchdog (nRF52 WDT, 600s timeout — ported from MeshCore)
// ============================================================

void InheroMr2Module::setupWatchdog()
{
#ifdef ARCH_NRF52
    // 600s timeout (~10 minutes) — catches firmware hangs
    // CRV = (timeout_seconds * 32768) - 1
    NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) |
                      (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);
    NRF_WDT->CRV = 32768UL * 600UL - 1;
    NRF_WDT->RREN = WDT_RREN_RR0_Msk; // Enable reload register 0
    NRF_WDT->TASKS_START = 1;
    wdtEnabled = true;
    LOG_INFO("InheroMr2: WDT started (600s timeout)");
#endif
}

void InheroMr2Module::feedWatchdog()
{
#ifdef ARCH_NRF52
    if (wdtEnabled) {
        NRF_WDT->RR[0] = WDT_RR_RR_Reload;
    }
#endif
}

// ============================================================
// Danger Zone / Shutdown (ported from MeshCore)
// ============================================================

void InheroMr2Module::initiateShutdown(uint8_t reason)
{
#ifdef ARCH_NRF52
    LOG_WARN("InheroMr2: Initiating shutdown (reason=%u)", reason);

    // Power off the SX1262 radio to minimize current draw during sleep
    #if defined(SX126X_POWER_EN)
    digitalWrite(SX126X_POWER_EN, LOW);
    delay(10);
    #endif

    // Turn off LEDs
    ledOff(PIN_LED1);
    ledOff(PIN_LED2);

    // Configure RTC wake (6 hours default, 2 hours on low-voltage)
    uint32_t wakeHours = (reason == SHUTDOWN_REASON_LOW_VOLTAGE) ? 2 : 6;
    if (rtcOk) {
        configureRTCWake(wakeHours);
    }

    // Store shutdown reason + danger-zone flag in GPREGRET2 (survives warm resets)
    uint8_t regval = (reason & 0x03);
    if (reason == SHUTDOWN_REASON_LOW_VOLTAGE)
        regval |= GPREGRET2_IN_DANGER_ZONE;
    sd_power_gpregret_clr(1, 0xFF);  // GPREGRET2 = index 1
    sd_power_gpregret_set(1, regval);

    if (reason == SHUTDOWN_REASON_LOW_VOLTAGE) {
        // System ON Idle: GPIO outputs remain latched → BQ_CE_PIN stays LOW
        // This allows BQ25798 to continue solar charging autonomously.
        // BQ25798 MPPT/CC/CV runs entirely in hardware — no CPU assistance needed.
        // Power: ~3 µA total (vs ~2 µA System OFF), enables solar recovery.
        //
        // CRITICAL: We check voltage IN the idle loop rather than rebooting
        // after each RTC wake. An intermediate NVIC_SystemReset() would go through
        // begin() → variant.cpp → if still low → sd_power_system_off() which
        // RELEASES GPIO latches → CE pin goes HIGH via pull-up → solar charging
        // blocked! By staying in the idle loop, CE stays LOW the entire time.

        LOG_INFO("InheroMr2: Entering System ON Idle (CE pin preserved for solar charging)");
        delay(50);

        // Get critical voltage threshold for this battery chemistry
        const uint16_t critical_threshold = getChemistryParams(boardConfig.chemistry).dangerVoltage_mV;

        // Feed watchdog one last time before entering idle loop
        feedWatchdog();

        // Idle loop with in-loop voltage check
        // Only NVIC_SystemReset() when voltage has actually recovered.
        // RTC timer is single-shot (TRPT=0), so we reconfigure after each fire.
        while (true) {
            // Wait for any SoftDevice event (RTC interrupt via I2C polling below)
            sd_app_evt_wait();

            // Poll RTC timer flag (TF = bit 3 in Status Register 0x0E)
            if (rtcOk) {
                Wire.beginTransmission(0x52);
                Wire.write(0x0E);
                Wire.endTransmission(false);
                Wire.requestFrom(0x52, 1);
                if (Wire.available()) {
                    uint8_t status = Wire.read();
                    if (status & 0x08) {
                        // RTC timer expired — clear TF flag
                        Wire.beginTransmission(0x52);
                        Wire.write(0x0E);
                        Wire.write(status & ~0x08);
                        Wire.endTransmission();

                        // Check battery voltage via INA228 one-shot read
                        // readVBATDirect() is static, works without full driver init
                        uint16_t vbat_mv = Ina228Driver::readVBATDirect(&Wire, INA228_I2C_ADDR_DEFAULT);

                        if (vbat_mv > 0 && vbat_mv >= critical_threshold) {
                            // Voltage recovered — safe to reboot into normal operation
                            // Clear DANGER_ZONE flag so variant.cpp uses the lower boot
                            // threshold (2800 mV). Keep LOW_VOLTAGE reason bits so
                            // setupDrivers() can restore .noinit stats.
                            sd_power_gpregret_clr(1, GPREGRET2_IN_DANGER_ZONE);
                            LOG_INFO("InheroMr2: Voltage recovered (%u mV >= %u mV) — rebooting",
                                     vbat_mv, critical_threshold);
                            delay(50);
                            NVIC_SystemReset();
                            // Never returns
                        }

                        // Still below critical — reconfigure RTC for another cycle
                        LOG_INFO("InheroMr2: Still in danger zone (%u mV < %u mV) — sleeping %u more hours",
                                 vbat_mv, critical_threshold, wakeHours);
                        delay(50);
                        configureRTCWake(wakeHours);
                    }
                }
            }

            // Feed watchdog during idle to prevent reset
            feedWatchdog();
        }
    }

    // Non-low-voltage shutdown: use System OFF (user request, thermal)
    sd_power_system_off();
    // Does not return — wakes via RTC interrupt or button
#endif
}

void InheroMr2Module::configureRTCWake(uint32_t hours)
{
    if (!rtcOk || hours == 0)
        return;

    // RV-3028 I2C address 0x52
    // Countdown Timer configuration:
    //   0x0A-0x0B: Timer Value (16-bit, little-endian)
    //   0x0E: Status Register — clear timer flag (bit 3 = TF)
    //   0x0F: Control 1 — enable countdown timer interrupt (bit 1 = TIE)
    //   0x10: Control 2 — timer clock 1/60 Hz (bits 1:0 = 10), enable timer (bit 2 = TE)

    Wire.beginTransmission(0x52);
    Wire.write(0x0A);
    // Timer value = hours * 60 minutes (1/60 Hz clock = 1 tick per minute)
    uint16_t timerVal = (uint16_t)(hours * 60);
    Wire.write((uint8_t)(timerVal & 0xFF));        // Timer Value Low
    Wire.write((uint8_t)((timerVal >> 8) & 0xFF)); // Timer Value High
    Wire.endTransmission();

    // Clear timer flag in Status Register
    Wire.beginTransmission(0x52);
    Wire.write(0x0E);
    Wire.write(0x00); // Clear all status flags
    Wire.endTransmission();

    // Control 1: Enable Timer Interrupt (TIE = bit 1)
    Wire.beginTransmission(0x52);
    Wire.write(0x0F);
    Wire.write(0x02); // TIE = 1
    Wire.endTransmission();

    // Control 2: Timer Enable + 1/60 Hz clock
    // TE = bit 2, TD[1:0] = bits 1:0 (10 = 1/60 Hz)
    Wire.beginTransmission(0x52);
    Wire.write(0x10);
    Wire.write(0x06); // TE=1, TD=10b (1/60 Hz)
    Wire.endTransmission();

    LOG_INFO("InheroMr2: RTC wake configured for %u hours (%u minutes)", hours, timerVal);
}

// ============================================================
// Solar MPPT Management (ported from MeshCore)
// ============================================================

void InheroMr2Module::checkAndFixPgoodStuck()
{
    if (!bq25798Ok || !solarData)
        return;

    // Problem: VBUS present (solar panel connected) but PGOOD is low
    // BQ25798 can get stuck where it doesn't recognize the input.
    // Solution: Toggle HIZ mode to force input re-detection.

    bool pgood = bq25798.getChargerStatusPowerGood();
    uint16_t vbus_mv = solarData->solar.voltage;

    if (!pgood && vbus_mv > MIN_VBUS_FOR_CHARGING) {
        // Check PG_FLAG to see if BQ has detected any input event
        bool pgFlag = bq25798.checkAndClearPgFlag();
        if (!pgFlag) {
            // No PG event at all — genuinely stuck. Toggle HIZ with cooldown.
            uint32_t now = millis();
            if (now - lastHizToggleTime >= HIZ_TOGGLE_COOLDOWN_MS) {
                lastHizToggleTime = now;
                LOG_WARN("InheroMr2: PGOOD stuck (VBUS=%.2fV, PG=0, no PG_FLAG) — toggling HIZ", vbus_mv / 1000.0f);
                bq25798.setHIZMode(true);
                delay(200);
                bq25798.setHIZMode(false);
            }
        }
    }
}

void InheroMr2Module::checkAndFixSolarLogic()
{
    if (!bq25798Ok)
        return;

    // When PGOOD=1 (solar is connected), ensure MPPT is actually enabled.
    // BQ25798 can sometimes disable MPPT after certain fault events.

    bool pgood = bq25798.getChargerStatusPowerGood();
    if (!pgood || !boardConfig.mpptEnabled)
        return;

    bool mpptCurrentlyEnabled = bq25798.getMPPTenable();
    if (!mpptCurrentlyEnabled) {
        uint32_t now = millis();
        if (now - lastMpptWriteTime >= MPPT_WRITE_COOLDOWN_MS) {
            lastMpptWriteTime = now;
            bq25798.setMPPTenable(true);
            LOG_WARN("InheroMr2: Re-enabled MPPT (was disabled while PG=1)");
        }
    }
}

// ============================================================
// Energy Analytics — Hourly Stats (ported from MeshCore)
// ============================================================

void InheroMr2Module::updateHourlyStats()
{
    uint32_t now = getTime();
    if (now == 0)
        now = millis() / 1000; // Fallback if no RTC/GPS time

    // Check if we've crossed an hour boundary
    if (socStats.lastHourUpdateTime == 0) {
        socStats.lastHourUpdateTime = now;
        return;
    }

    uint32_t elapsed = now - socStats.lastHourUpdateTime;
    if (elapsed < 3600)
        return; // Not yet 1 hour

    // Save current hour's data to the rolling buffer
    HourlyBatteryStats &slot = socStats.hours[socStats.currentIndex];
    slot.timestamp = socStats.lastHourUpdateTime;
    slot.charged_mah = socStats.current_hour_charged_mah;
    slot.discharged_mah = socStats.current_hour_discharged_mah;
    slot.solar_mah = socStats.current_hour_solar_mah;

    // Advance index (circular buffer)
    socStats.currentIndex = (socStats.currentIndex + 1) % HOURLY_STATS_HOURS;

    // Reset accumulators for next hour
    socStats.current_hour_charged_mah = 0;
    socStats.current_hour_discharged_mah = 0;
    socStats.current_hour_solar_mah = 0;
    socStats.lastHourUpdateTime = now;

    // Recalculate rolling statistics from buffer
    calculateRollingStats();
    calculateTTL();

    if (socStats.ttl_hours >= 24) {
        LOG_DEBUG("InheroMr2: Hourly stats updated (idx=%u, 24h_net=%.0fmAh, TTL=%ud%uh)",
                  socStats.currentIndex, socStats.last_24h_net_mah, socStats.ttl_hours / 24, socStats.ttl_hours % 24);
    } else {
        LOG_DEBUG("InheroMr2: Hourly stats updated (idx=%u, 24h_net=%.0fmAh, TTL=%uh)",
                  socStats.currentIndex, socStats.last_24h_net_mah, socStats.ttl_hours);
    }
}

void InheroMr2Module::calculateRollingStats()
{
    // Calculate 24h, 3-day, and 7-day rolling averages from the circular buffer
    float charged_24h = 0, discharged_24h = 0;
    float charged_3d = 0, discharged_3d = 0;
    float charged_7d = 0, discharged_7d = 0;
    uint16_t count_24h = 0, count_3d = 0, count_7d = 0;

    for (int i = 0; i < HOURLY_STATS_HOURS; i++) {
        const HourlyBatteryStats &h = socStats.hours[i];
        if (h.timestamp == 0)
            continue; // Empty slot

        count_7d++;
        charged_7d += h.charged_mah;
        discharged_7d += h.discharged_mah;

        // 3-day window = 72 hours
        // Use index distance in circular buffer
        int dist = (socStats.currentIndex - 1 - i + HOURLY_STATS_HOURS) % HOURLY_STATS_HOURS;
        if (dist < 72) {
            count_3d++;
            charged_3d += h.charged_mah;
            discharged_3d += h.discharged_mah;
        }
        if (dist < 24) {
            count_24h++;
            charged_24h += h.charged_mah;
            discharged_24h += h.discharged_mah;
        }
    }

    // 24h totals
    socStats.last_24h_charged_mah = charged_24h;
    socStats.last_24h_discharged_mah = discharged_24h;
    socStats.last_24h_net_mah = charged_24h - discharged_24h;

    // 3-day daily averages
    float days_3d = (count_3d > 0) ? (count_3d / 24.0f) : 0;
    if (days_3d > 0) {
        socStats.avg_3day_daily_charged_mah = charged_3d / days_3d;
        socStats.avg_3day_daily_discharged_mah = discharged_3d / days_3d;
        socStats.avg_3day_daily_net_mah = (charged_3d - discharged_3d) / days_3d;
    }

    // 7-day daily averages
    float days_7d = (count_7d > 0) ? (count_7d / 24.0f) : 0;
    if (days_7d > 0) {
        socStats.avg_7day_daily_charged_mah = charged_7d / days_7d;
        socStats.avg_7day_daily_discharged_mah = discharged_7d / days_7d;
        socStats.avg_7day_daily_net_mah = (charged_7d - discharged_7d) / days_7d;
    }

    socStats.living_on_battery = (socStats.last_24h_net_mah < 0);
}

void InheroMr2Module::calculateTTL()
{
    // TTL = remaining_mah / daily_deficit
    // Only meaningful when consuming more than charging (net < 0)

    if (!socStats.soc_valid || socStats.capacity_mah <= 0) {
        socStats.ttl_hours = 0;
        return;
    }

    // Use 3-day average if available, otherwise 24h
    float daily_net = socStats.avg_3day_daily_net_mah;
    if (daily_net == 0 && socStats.last_24h_net_mah != 0) {
        daily_net = socStats.last_24h_net_mah;
    }

    if (daily_net >= 0) {
        // Net positive or zero — battery gaining charge, TTL=∞
        socStats.ttl_hours = 0xFFFF; // Effectively infinite
        return;
    }

    float remaining_mah = (socStats.current_soc_percent / 100.0f) * socStats.capacity_mah;
    float hourly_deficit = (-daily_net) / 24.0f; // Convert daily to hourly (positive number)

    if (hourly_deficit < 0.01f) {
        socStats.ttl_hours = 0xFFFF;
        return;
    }

    uint32_t ttl = (uint32_t)(remaining_mah / hourly_deficit);
    socStats.ttl_hours = (ttl > 0xFFFE) ? 0xFFFE : (uint16_t)ttl;
}

// ============================================================
// MPPT Statistics (ported from MeshCore)
// ============================================================

void InheroMr2Module::updateMpptStats()
{
    if (!bq25798Ok)
        return;

    uint32_t now = getTime();
    if (now == 0)
        now = millis() / 1000;

    // Track whether MPPT is currently enabled
    bool mpptNow = bq25798.getMPPTenable() && bq25798.getChargerStatusPowerGood();

    if (!mpptStatsInitialized) {
        mpptStats.lastUpdateTime = now;
        mpptStats.usingRTC = (getTime() != 0);
        mpptStatsInitialized = true;
        lastMpptStatus = mpptNow;
        return;
    }

    // Accumulate minutes MPPT was enabled
    // runOnce() is called every 10s, so each call ≈ 10s/60 ≈ 0.167 minutes
    // We track as integer minutes — increment every 6 calls (~60s)
    if (mpptNow && lastMpptStatus) {
        // Both previous and current reading show MPPT active: count this interval
        mpptStats.currentHourMinutes++;
    }
    lastMpptStatus = mpptNow;

    // Accumulate energy (solar power * time)
    if (solarData && mpptNow) {
        int32_t power_mW = (int32_t)solarData->solar.voltage * solarData->solar.current / 1000;
        // Each interval is ~10s, energy = power * time (mWh) = power_mW * 10 / 3600000
        mpptStats.currentHourEnergy_mWh += (uint32_t)(power_mW * 10UL / 3600UL);
    }

    // Check hour boundary
    uint32_t elapsed = now - mpptStats.lastUpdateTime;
    if (elapsed >= 3600) {
        // Save to circular buffer
        MpptHourlyStats &slot = mpptStats.hours[mpptStats.currentIndex];
        slot.mpptEnabledMinutes = (uint8_t)min((uint16_t)60, mpptStats.currentHourMinutes);
        slot.timestamp = mpptStats.lastUpdateTime;
        slot.harvestedEnergy_mWh = mpptStats.currentHourEnergy_mWh;

        mpptStats.currentIndex = (mpptStats.currentIndex + 1) % MPPT_STATS_HOURS;
        mpptStats.currentHourMinutes = 0;
        mpptStats.currentHourEnergy_mWh = 0;
        mpptStats.lastUpdateTime = now;
    }
}

float InheroMr2Module::getMpptEnabledPercentage7Day()
{
    uint32_t totalMinutes = 0;
    uint32_t totalSlots = 0;

    for (int i = 0; i < MPPT_STATS_HOURS; i++) {
        if (mpptStats.hours[i].timestamp == 0)
            continue;
        totalMinutes += mpptStats.hours[i].mpptEnabledMinutes;
        totalSlots++;
    }

    if (totalSlots == 0)
        return 0.0f;

    // Perfect = 60 minutes per slot
    return (totalMinutes / (totalSlots * 60.0f)) * 100.0f;
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
    if (skipFsWrites) {
        LOG_WARN("InheroMr2: saveConfig() skipped (low-voltage boot, skipFsWrites=true)");
        return;
    }

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
