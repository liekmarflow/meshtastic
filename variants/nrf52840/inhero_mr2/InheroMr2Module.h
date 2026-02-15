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
 * - Board config via PortNum 256 (PRIVATE_APP) text protocol
 * - LittleFS persistent config storage
 * - Early boot voltage check (anti-motorboating)
 * - LED control
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
    bool frostProtect = true;
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

  protected:
    /// Periodic task - reads sensors, sends telemetry, monitors voltage
    virtual int32_t runOnce() override;

    /// Handle incoming config commands on PortNum 256
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

    // Config persistence
    void loadConfig();
    void saveConfig();

    // Telemetry
    void sendPowerTelemetry();

    // Config command handling
    void handleConfigCommand(const meshtastic_MeshPacket &mp, const char *payload, size_t len);
    void sendTextReply(const meshtastic_MeshPacket &mp, const char *text);

    // Charger management
    void applyChemistryConfig();
    void updateLEDs();
    int estimateSOC();

    // Config persistence helpers
    void writeConfigValue(const char *key, const char *value);
    size_t readConfigValue(const char *key, char *buffer, size_t maxLen, const char *defaultValue = "");
};
