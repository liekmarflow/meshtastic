/*
 * Copyright (c) 2026 Inhero GmbH
 *
 * SPDX-License-Identifier: MIT
 *
 * Inhero MR-2 Board Module for Meshtastic
 *
 * Handles:
 * - INA228 battery monitoring (ch1 in PowerTelemetry)
 * - BQ25798 solar charger management (ch2 in PowerTelemetry)
 * - Battery chemistry configuration (LTO/LiFePO4/Li-Ion)
 * - MeshCore-compatible CLI via text message DMs (/get, /set, /help)
 * - Admin authentication via config.security.admin_key pubkeys
 * - LittleFS persistent config storage
 * - Early boot voltage check (anti-motorboating)
 * - LED control
 *
 * CLI Commands (via DM text messages):
 *   /get board.<key>    - Read board configuration/telemetry
 *   /set board.<key> <v> - Write board configuration (requires admin key)
 *   /help               - List available commands
 *   /ver                - Firmware version
 *   /reboot             - Reboot node (requires admin key)
 *
 * Authentication:
 *   Write commands require the sender's public key to be registered in
 *   config.security.admin_key[0..2]. PKI-encrypted DMs use the cryptographically
 *   verified sender key; channel-encrypted DMs fall back to NodeDB lookup.
 */

#pragma once

#include "Ina228Driver.h"
#include "BQ25798Driver.h"
#include "concurrency/OSThread.h"
#include "configuration.h"
#include "mesh/MeshModule.h"
#include "mesh/SinglePortModule.h"

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

/// Battery chemistry types supported by Inhero MR-2
enum class BatteryChemistry : uint8_t {
    LTO_2S = 0,      ///< 2S Lithium Titanate: 5.4V charge, 4.2V danger
    LiFePO4_1S = 1,  ///< 1S LiFePO4: 3.5V charge, 2.9V danger
    Li_Ion_1S = 2,    ///< 1S Li-Ion: 4.1V charge, 3.4V danger
    BAT_UNKNOWN = 3   ///< Unknown - safe defaults
};

/// Chemistry-specific voltage thresholds (in mV)
struct ChemistryParams {
    uint16_t chargeVoltage_mV;   ///< Max charge voltage
    uint16_t dangerVoltage_mV;   ///< Deep discharge danger threshold
    uint16_t nominalVoltage_mV;  ///< Nominal voltage (for SOC estimation)
    uint16_t fullVoltage_mV;     ///< Voltage at ~100% SOC
    uint16_t emptyVoltage_mV;    ///< Voltage at ~0% SOC
};

/// Persistent board configuration
struct InheroMr2Config {
    BatteryChemistry chemistry = BatteryChemistry::BAT_UNKNOWN;
    uint16_t chargeCurrentMax_mA = 500;
    bool mpptEnabled = true;
    bool ledsEnabled = true;
    float inaCalibration = 1.0f;
    float tcCalOffset = 0.0f;
    bool frostProtect = true;
    bool uvloEnabled = true;
    uint32_t batteryCapacity_mAh = 0; // 0 = use chemistry default
};

class InheroMr2Module : private concurrency::OSThread, public SinglePortModule
{
  public:
    InheroMr2Module();

    /// Initialize hardware drivers
    bool setupDrivers();

    /// Get chemistry parameters for current config
    static const ChemistryParams &getChemistryParams(BatteryChemistry chem);

    /// Get the singleton instance
    static InheroMr2Module *getInstance() { return instance; }

    /// Override wantPacket to receive TEXT_MESSAGE_APP packets addressed to us
    virtual bool wantPacket(const meshtastic_MeshPacket *p) override;

  protected:
    /// Periodic task - reads sensors, sends telemetry, monitors voltage
    virtual int32_t runOnce() override;

    /// Handle incoming text messages, intercept /commands
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

  private:
    static InheroMr2Module *instance;

    // Hardware drivers
    Ina228Driver ina228;
    BQ25798Driver bq25798;

    // State
    bool driversInitialized = false;
    bool ina228Ok = false;
    bool bq25798Ok = false;
    InheroMr2Config boardConfig;

    // Timing
    uint32_t lastTelemetrySend = 0;
    uint32_t lastSensorRead = 0;
    uint32_t telemetryIntervalMs = 60000; // Default: 1 minute

    // Cached sensor data
    Ina228BatteryData batteryData = {0};
    const BqTelemetry *solarData = nullptr;

    // LED2 (Red) state: CLI flash timing
    uint32_t cliFlashUntil = 0; // millis() when CLI flash should end (200ms pulse)

    // === CLI Command Handling ===

    /// Check if the sender is an authorized remote admin (pubkey in config.security.admin_key[])
    bool isAuthorizedAdmin(const meshtastic_MeshPacket &mp);

    /// Main CLI dispatcher — parses /get, /set, /help, /ver, /reboot
    void handleCliCommand(const meshtastic_MeshPacket &mp, const char *cmd);

    /// Handle /get board.<key> commands (MeshCore getCustomGetter compatible)
    void handleGetCommand(const meshtastic_MeshPacket &mp, const char *key);

    /// Handle /set board.<key> <value> commands (MeshCore setCustomSetter compatible)
    void handleSetCommand(const meshtastic_MeshPacket &mp, const char *keyAndValue);

    /// Send a text message reply to sender on TEXT_MESSAGE_APP
    void sendTextReply(const meshtastic_MeshPacket &mp, const char *text);

    // === Telemetry ===
    void sendPowerTelemetry();

    // === Charger Management ===
    void applyChemistryConfig();
    void updateLEDs();
    int estimateSOC();

    // === Helper: battery type string conversion ===
    static const char *chemistryToString(BatteryChemistry chem);
    static BatteryChemistry stringToChemistry(const char *str);

    // === Config Persistence (LittleFS) ===
    void loadConfig();
    void saveConfig();
    void writeConfigValue(const char *key, const char *value);
    size_t readConfigValue(const char *key, char *buffer, size_t maxLen, const char *defaultValue = "");
};
