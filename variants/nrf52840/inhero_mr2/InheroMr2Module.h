/*
 * Copyright (c) 2026 Inhero GmbH
 *
 * SPDX-License-Identifier: MIT
 *
 * Inhero MR-2 Board Module for Meshtastic
 *
 * Full MeshCore feature parity implementation:
 * - INA228 battery monitoring (ch1 in PowerTelemetry)
 * - BQ25798 solar charger management (ch2 in PowerTelemetry)
 * - Battery chemistry configuration (LTO/LiFePO4/Li-Ion)
 * - MeshCore-compatible CLI via text message DMs (/get, /set, /help)
 * - Admin authentication via config.security.admin_key pubkeys
 * - LittleFS persistent config storage
 * - Early boot voltage check (anti-motorboating)
 * - GPREGRET2 shutdown reason / danger zone tracking
 * - nRF52 hardware watchdog (600s timeout)
 * - SX1262 power-off before danger-zone shutdown
 * - skip_fs_writes on low-voltage boot
 * - Solar MPPT management (stuck PGOOD fix, MPPT re-enable)
 * - 168-hour energy analytics (hourly buffer, rolling averages, TTL)
 * - MPPT statistics (7-day moving average)
 * - Error LED for missing I2C components
 * - Auto tccal via BME280
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

// ========== GPREGRET2 Shutdown Reason Codes ==========
// Stored in GPREGRET2 bits [1:0] to survive warm resets
#define SHUTDOWN_REASON_NONE          0x00
#define SHUTDOWN_REASON_LOW_VOLTAGE   0x01
#define SHUTDOWN_REASON_USER_REQUEST  0x02
#define SHUTDOWN_REASON_THERMAL       0x03
// GPREGRET2 bit [2]: Danger Zone flag (SX1262 disabled, RTC wake mode)
#define GPREGRET2_IN_DANGER_ZONE      0x04

// ========== Energy Analytics Constants ==========
#define HOURLY_STATS_HOURS 168  ///< 7 days * 24 hours rolling buffer
#define MPPT_STATS_HOURS   168  ///< 7 days * 24 hours MPPT rolling buffer

// ========== Solar MPPT Cooldown Constants ==========
#define HIZ_TOGGLE_COOLDOWN_MS      (5UL * 60UL * 1000UL)   ///< 5 minutes between HIZ toggles
#define MPPT_WRITE_COOLDOWN_MS      (60UL * 1000UL)         ///< 60s between MPPT register writes
#define MIN_VBUS_FOR_CHARGING       3500                      ///< 3.5V minimum for valid solar input (mV)

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

/// Hourly battery statistics for rolling window
struct HourlyBatteryStats {
    uint32_t timestamp;           ///< Unix timestamp (start of hour, seconds)
    float charged_mah;            ///< Charge added this hour (mAh)
    float discharged_mah;         ///< Charge removed this hour (mAh)
    float solar_mah;              ///< Solar charge contribution this hour (mAh)
};

/// MPPT hourly statistics
struct MpptHourlyStats {
    uint8_t mpptEnabledMinutes;   ///< Minutes MPPT was enabled (0-60)
    uint32_t timestamp;           ///< Unix timestamp for this hour
    uint32_t harvestedEnergy_mWh; ///< Harvested solar energy (mWh)
};

/// Full battery SOC statistics (168h rolling buffer)
struct BatterySOCStats {
    // Battery configuration
    float capacity_mah;          ///< Total battery capacity in mAh

    // SOC tracking using INA228 hardware counter
    float current_soc_percent;   ///< Current State of Charge in % (0-100)
    bool soc_valid;              ///< True after first "Charging Done" sync
    float ina228_baseline_mah;   ///< INA228 CHARGE reading at last 100% sync (mAh)

    // Hourly statistics (168-hour rolling buffer for 7 days)
    HourlyBatteryStats hours[HOURLY_STATS_HOURS];
    uint8_t currentIndex;
    uint32_t lastHourUpdateTime; ///< Last hour boundary timestamp

    // Current hour accumulators (reset every hour)
    float current_hour_charged_mah;
    float current_hour_discharged_mah;
    float current_hour_solar_mah;

    // Rolling window statistics (calculated from hourly buffer)
    float last_24h_net_mah;
    float last_24h_charged_mah;
    float last_24h_discharged_mah;
    float avg_3day_daily_net_mah;
    float avg_3day_daily_charged_mah;
    float avg_3day_daily_discharged_mah;
    float avg_7day_daily_net_mah;
    float avg_7day_daily_charged_mah;
    float avg_7day_daily_discharged_mah;
    uint16_t ttl_hours;          ///< Time To Live - hours until battery empty (0 = not calculated)
    bool living_on_battery;      ///< True if net deficit over last 24h
};

/// MPPT statistics (7-day rolling buffer)
struct MpptStatistics {
    MpptHourlyStats hours[MPPT_STATS_HOURS]; ///< Rolling buffer of hourly stats
    uint8_t currentIndex;
    uint32_t lastUpdateTime;       ///< Last update time (Unix seconds or millis fallback)
    uint16_t currentHourMinutes;   ///< Accumulated minutes for current hour
    bool usingRTC;                 ///< True if using RTC, false if using millis fallback
    uint32_t currentHourEnergy_mWh;///< Accumulated energy for current hour (mWh)
    int32_t lastPower_mW;          ///< Last measured power for energy calculation
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

    // === Public battery API (used by Power system's HasBatteryLevel) ===

    /// Get battery voltage in mV from INA228 (0 if unavailable)
    uint16_t getBatteryVoltageMv() const { return ina228Ok ? batteryData.voltage_mv : 0; }

    /// Get chemistry-aware SOC estimate (0-100, or -1 if unavailable)
    int getBatterySOC() { return estimateSOC(); }

    /// Whether a battery is connected (INA228 reads valid voltage)
    bool hasBattery() const { return ina228Ok && batteryData.voltage_mv > 0; }

    /// Whether external power (solar) is present (BQ25798 VBUS > 1V)
    bool hasExternalPower() const { return bq25798Ok && solarData && solarData->solar.voltage > 1000; }

    /// Whether the battery is currently charging (BQ25798 in CC/CV/trickle/pre-charge)
    bool isBatteryCharging() const;

    /// Get SOC stats for external access (e.g. CayenneLPP equivalent)
    const BatterySOCStats *getSOCStats() const { return &socStats; }

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
    bool rtcOk = false;           // RV-3028 RTC detected
    InheroMr2Config boardConfig;
    bool skipFsWrites = false;    // True on low-voltage boot — avoids flash writes

    // Timing
    uint32_t lastTelemetrySend = 0;
    uint32_t lastSensorRead = 0;
    uint32_t telemetryIntervalMs = 60000; // Default: 1 minute

    // Cached sensor data
    Ina228BatteryData batteryData = {0};
    const BqTelemetry *solarData = nullptr;

    // LED2 (Red) state: CLI flash timing
    uint32_t cliFlashUntil = 0;       // millis() when CLI flash should end (200ms pulse)
    bool errorLedActive = false;       // Missing component error LED blink

    // === Energy Analytics (168h buffer, ported from MeshCore) ===
    BatterySOCStats socStats = {};
    MpptStatistics mpptStats = {};
    uint8_t minuteCounter = 0;       // Minutes since last hourly stats update

    // === Coulomb Counting SOC State ===
    float lastChargeMah = 0.0f;       // Previous CHARGE reading for delta tracking
    bool firstChargeRead = true;      // First reading flag

    // === Solar MPPT Management ===
    uint32_t lastMpptWriteTime = 0;   // 60s cooldown for MPPT register writes
    uint32_t lastHizToggleTime = 0;   // 5-min cooldown for HIZ toggles
    bool lastMpptStatus = false;      // Previous MPPT enabled state
    bool mpptStatsInitialized = false;

    // === Watchdog ===
    bool wdtEnabled = false;

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

    // === Coulomb Counting ===
    void updateBatterySOC();           // Called every runOnce — tracks charge delta, auto-syncs on charge-done
    void syncSOCToFull();              // Reset coulomb counter + set SOC=100% (on BQ charge-done)
    bool setSOCManually(float pct);    // Manual SOC override — recalculates baseline
    int estimateSOCFromVoltage();      // Voltage-only fallback SOC
    uint32_t getEffectiveCapacity();   // Returns configured or default capacity in mAh

    // === Energy Analytics (ported from MeshCore) ===
    void updateHourlyStats();          // Save current hour, advance rolling buffer (every 60 min)
    void calculateRollingStats();      // 24h / 3d / 7d averages from hourly buffer
    void calculateTTL();               // Time To Live estimation
    void updateMpptStats();            // MPPT statistics tracking

    // === Solar Power Management (ported from MeshCore) ===
    void checkAndFixPgoodStuck();      // Detect stuck PGOOD, toggle HIZ to force input detection
    void checkAndFixSolarLogic();      // Re-enable MPPT if BQ disabled it (when PG=1)
    float getMpptEnabledPercentage7Day(); // 7-day MPPT enabled % (for stats display)

    // === Watchdog (nRF52 WDT, 600s timeout) ===
    void setupWatchdog();
    void feedWatchdog();

    // === Danger Zone / Shutdown ===
    void initiateShutdown(uint8_t reason);  // Controlled shutdown with SX1262 power-off + RTC wake
    void configureRTCWake(uint32_t hours);  // RV-3028 countdown timer for wake-from-sleep

    // === Helper: battery type string conversion ===
    static const char *chemistryToString(BatteryChemistry chem);
    static BatteryChemistry stringToChemistry(const char *str);

    // === Config Persistence (LittleFS) ===
    void loadConfig();
    void saveConfig();
    void writeConfigValue(const char *key, const char *value);
    size_t readConfigValue(const char *key, char *buffer, size_t maxLen, const char *defaultValue = "");
};
