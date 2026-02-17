# Inhero MR-2 — Meshtastic Variant

Hardware-Variant für das Inhero MR-2 Board auf Basis des RAK4630 (nRF52840 + SX1262) mit erweitertem Power-Management und Solar-Ladesteuerung.

> Portiert von MeshCore (`variants/inhero_mr2`) nach Meshtastic.  
> Branch: `feature/inhero-mr2-variant`

---

## Inhaltsverzeichnis

1. [Hardware-Übersicht](#hardware-übersicht)
2. [Architektur](#architektur)
3. [Dateistruktur](#dateistruktur)
4. [Treiber](#treiber)
   - [INA228 Power Monitor](#ina228-power-monitor)
   - [BQ25798 Solar Charger](#bq25798-solar-charger)
5. [InheroMr2Module](#inheromr2module)
   - [Telemetrie](#telemetrie)
   - [Konfigurations-Protokoll (CLI über Text-DMs)](#konfigurations-protokoll-cli-über-text-dms)
   - [Batterie-Chemie](#batterie-chemie)
   - [LittleFS Persistenz](#littlefs-persistenz)
   - [LED-Steuerung](#led-steuerung)
6. [System-Zuverlässigkeit](#system-zuverlässigkeit)
   - [Hardware Watchdog (nRF52 WDT)](#hardware-watchdog-nrf52-wdt)
   - [GPREGRET2 Shutdown-Tracking](#gpregret2-shutdown-tracking)
   - [Early Boot Spannungsprüfung](#early-boot-spannungsprüfung)
   - [Danger-Zone Shutdown](#danger-zone-shutdown)
   - [skipFsWrites bei Low-Voltage Boot](#skipfswrites-bei-low-voltage-boot)
7. [Solar-MPPT-Management](#solar-mppt-management)
   - [Stuck PGOOD Fix](#stuck-pgood-fix)
   - [MPPT Re-Enable](#mppt-re-enable)
8. [Energy Analytics (168h)](#energy-analytics-168h)
   - [Hourly Stats Rolling Buffer](#hourly-stats-rolling-buffer)
   - [Rolling Averages (24h / 3d / 7d)](#rolling-averages-24h--3d--7d)
   - [TTL (Time To Live)](#ttl-time-to-live)
   - [MPPT-Statistiken](#mppt-statistiken)
9. [I2C Scanner Erweiterung](#i2c-scanner-erweiterung)
10. [Änderungen am Meshtastic-Hauptcode](#änderungen-am-meshtastic-hauptcode)
11. [Build](#build)
12. [Konfiguration über Text-DMs (CLI)](#konfiguration-über-text-dms-cli)
13. [Portierung von MeshCore](#portierung-von-meshcore)

---

## Hardware-Übersicht

| Komponente | Chip | I2C-Adresse | Funktion |
|---|---|---|---|
| MCU | nRF52840 (RAK4630) | — | Hauptprozessor, BLE 5.0 |
| LoRa | SX1262 | SPI | Sub-GHz LoRa (868/915 MHz) |
| Power Monitor | INA228 | 0x40 | 24-Bit Batterie V/I/P/E/Q |
| Solar Charger | BQ25798 | 0x6B | MPPT, JEITA, Multi-Chemie |
| RTC | RV-3028-C7 | 0x52 | Echtzeituhr (Meshtastic-RTC) |
| Display | SSD1306 OLED | 0x3C | 128×64 Pixel |
| Sensor | BME280 | 0x76/0x77 | Temperatur/Feuchte/Druck |

### Pin-Belegung

| Signal | Pin | Beschreibung |
|---|---|---|
| SX1262 CS | 42 (P1.10) | SPI Chip Select |
| SX1262 DIO1 | 47 (P1.15) | Interrupt |
| SX1262 BUSY | 46 (P1.14) | Busy-Signal |
| SX1262 RESET | 38 (P1.06) | Reset |
| SX1262 POWER_EN | 37 (P1.05) | Stromversorgung |
| I2C SDA | 13 | Datenleitung |
| I2C SCL | 14 | Taktleitung |
| LED Blau | 35 (P1.03) | Status-LED |
| LED Rot | 36 (P1.04) | Lade-LED |
| 3V3 Enable | 34 (P1.02) | Peripherie-Stromversorgung |
| Battery ADC | A0 (Pin 5) | Batteriespannung (ADC_MULTIPLIER=1.73) |
| Buzzer | 21 | Piezo-Summer |

---

## Architektur

Das Design folgt dem Prinzip **"Self-Contained Module"**: Der gesamte board-spezifische Code liegt im Variant-Verzeichnis und wird nur für `inhero_mr2` kompiliert. Es gibt nur **4 minimale Änderungen** am Meshtastic-Hauptcode.

```
┌──────────────────────────────────────────────────────┐
│                  Meshtastic Core                     │
│                                                      │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────┐  │
│  │ Modules.cpp │  │ ScanI2C.h    │  │ ScanI2C    │  │
│  │ +3 Zeilen   │  │ +1 Enum      │  │ TwoWire.cpp│  │
│  │ #ifdef      │  │ INA228       │  │ +Detection │  │
│  └──────┬──────┘  └──────────────┘  └────────────┘  │
│         │         ┌──────────────┐                    │
│         │         │ Power.cpp    │                    │
│         │         │ HasBatLevel  │                    │
│         │         │ #ifdef       │                    │
│         │         └──────┬───────┘                    │
└─────────┼────────────────┼───────────────────────────┘
          │ new Module()   │ getBatterySOC()
          ▼                ▼
┌──────────────────────────────────────────────────────┐
│            variants/nrf52840/inhero_mr2/             │
│                                                      │
│  ┌──────────────────────────┐  ┌──────────────────┐  │
│  │  InheroMr2Module         │  │  variant.h/.cpp  │  │
│  │  ├─ Telemetrie           │  │  ├─ Pins         │  │
│  │  ├─ CLI-over-DM          │  │  ├─ GPREGRET2    │  │
│  │  ├─ LittleFS Config      │  │  └─ Early Boot   │  │
│  │  ├─ LED-Steuerung        │  │     (Danger-Zone)│  │
│  │  ├─ SOC Coulomb Counting │  └──────────────────┘  │
│  │  ├─ 168h Energy Analytics│                        │
│  │  ├─ Solar MPPT Mgmt      │                        │
│  │  ├─ Watchdog (600s)      │                        │
│  │  └─ Danger-Zone Shutdown │                        │
│  └────────┬─────────────────┘                        │
│           │ verwendet                                │
│  ┌────────┴───────┐  ┌────────────────────┐          │
│  │  Ina228Driver  │  │  BQ25798Driver     │          │
│  │  (.h / .cpp)   │  │  (.h / .cpp)       │          │
│  └────────────────┘  └────────────────────┘          │
└──────────────────────────────────────────────────────┘
```

### Design-Entscheidungen

- **Kein Eingriff in die Sensor-Hierarchie**: Die INA228 wird nicht als Standard-`TelemetrySensor` registriert. Stattdessen sendet `InheroMr2Module` direkt `PowerMetrics`-Protobuf-Pakete. Das Standard-`PowerTelemetryModule` deaktiviert sich automatisch, da keine erkannten Standard-Sensoren vorhanden sind.

- **CLI via TEXT_MESSAGE_APP**: Konfigurations-Befehle laufen über normale Text-DMs — keine spezielle App oder Custom-PortNum nötig. Jede Standard-Meshtastic-App (Android, iOS, Web, CLI) kann verwendet werden.

- **FreeRTOS-Tasks → OSThread**: MeshCore nutzt 4 separate FreeRTOS-Tasks (SolarDaemon, Heartbeat, SOCUpdate, VoltageMonitor). In Meshtastic läuft alles in einem `runOnce()` mit internen Zählern für verschiedene Kadenzen (10s Basis, 30s Solar, 60s Hourly-Stats).

- **Compile-Guard `#ifdef INHERO_MR2`**: Alle board-spezifischen Includes und Registrierungen sind hinter diesem Guard, sodass andere Varianten nicht betroffen sind.

- **RAM-only Energy Analytics**: Wie in MeshCore werden die 168h-Statistiken **nicht** persistiert — sie gehen bei Reboot/Shutdown verloren. Das ist bewusst so, um Flash-Wear zu minimieren.

---

## Dateistruktur

### Neue Dateien (Variant-Verzeichnis)

| Datei | Zeilen | Beschreibung |
|---|---|---|
| `Ina228Driver.h` | 154 | INA228 Register-Map, Structs, Klassen-Definition |
| `Ina228Driver.cpp` | 402 | I2C-Kommunikation, ADC-Konfiguration, Kalibrierung |
| `BQ25798Driver.h` | 179 | BQ25798 Erweiterung, JEITA-Enums, Telemetrie-Structs |
| `BQ25798Driver.cpp` | 405 | Steinhart-Hart NTC, One-Shot ADC, Register-Zugriff |
| `InheroMr2Module.h` | ~310 | Modul-Klasse, Config-Struct, Chemistry-Enum, SOC/MPPT/Hourly-Stats-Structs |
| `InheroMr2Module.cpp` | ~1590 | Telemetrie, Config-Kommandos, LittleFS, SOC, Energy Analytics, Solar Mgmt, WDT |
| `variant.h` | ~225 | Pin-Definitionen, Radio-Config, Peripherie |
| `variant.cpp` | ~90 | GPIO-Map, Early-Boot-Spannungsprüfung, GPREGRET2 Danger-Zone |
| `platformio.ini` | 29 | Build-Konfiguration, Library-Dependencies |

### Modifizierte Dateien (Meshtastic-Kern)

| Datei | Änderung |
|---|---|
| `src/detect/ScanI2C.h` | +1 Zeile: `INA228` in `DeviceType`-Enum |
| `src/detect/ScanI2CTwoWire.cpp` | +12 Zeilen: INA228-Erkennung via Register 0x3E |
| `src/modules/Modules.cpp` | +6 Zeilen: `#ifdef INHERO_MR2` Include + Instanziierung |
| `src/Power.cpp` | +35 Zeilen: `InheroMr2BatteryLevel` unter `#ifdef INHERO_MR2_POWER` |

---

## Treiber

### INA228 Power Monitor

**Datei**: `Ina228Driver.h` / `Ina228Driver.cpp`

Texas Instruments INA228 — 24-Bit hochauflösender Leistungsmonitor mit integriertem Coulomb-Zähler.

#### Hardware-Konfiguration

| Parameter | Wert |
|---|---|
| I2C-Adresse | 0x40 |
| Shunt-Widerstand | 20 mΩ |
| ADC-Bereich | ±40.96 mV (ADCRANGE=1) |
| Max. Strom | ±2.048 A |
| Current LSB | ~3.81 µA |
| Averaging | 256 Samples |
| VBUS Conversion | 2074 µs |
| VSHUNT Conversion | 4120 µs |

#### API

```cpp
class Ina228Driver {
    bool begin(float shunt_resistor_mohm);
    
    // Messungen
    uint16_t readVoltage_mV();          // Batteriespannung
    int16_t  readCurrent_mA();          // Strom (+ = Laden, - = Entladen)
    int32_t  readPower_mW();            // Leistung
    int32_t  readEnergy_mWh();          // Akkumulierte Energie
    float    readCharge_mAh();          // Akkumulierte Ladung
    float    readDieTemperature_C();    // Chip-Temperatur
    bool     readAll(Ina228BatteryData*); // Alle Werte auf einmal
    
    // Coulomb-Zähler
    void resetCoulombCounter();
    
    // Alerts (Hardware-UVLO über Alert-Pin)
    bool setUnderVoltageAlert(uint16_t voltage_mv);
    bool setOverVoltageAlert(uint16_t voltage_mv);
    void enableAlert(bool uvlo, bool active_high, bool latch);
    
    // Kalibrierung
    float calibrateCurrent(float actual_mA);  // Auto-Kalibrierung
    void  setCalibrationFactor(float factor);  // Manuell (0.5–2.0)
    
    // Früher Boot (statisch, ohne Treiber-Init)
    static uint16_t readVBATDirect(TwoWire*, uint8_t addr);
};
```

#### Besonderheiten

- **ADCRANGE=1**: Faktor ×4 auf SHUNT_CAL gemäß Datasheet Section 7.3.1.1
- **AVG_256**: Filtert TX-Spannungseinbrüche (verhindert falsche UVLO-Auslösung bei Sendevorgängen)
- **readVBATDirect()**: Statische Methode für den Early-Boot-Check — nutzt One-Shot-Modus ohne vollständige Treiber-Initialisierung
- **Vorzeichen-Konvention**: Strom wird invertiert (Batterie-Perspektive: negativ = Entladen)

---

### BQ25798 Solar Charger

**Datei**: `BQ25798Driver.h` / `BQ25798Driver.cpp`

Texas Instruments BQ25798 — 1–4 Zellen Buck-Boost Solar-Laderegler mit MPPT.

#### Hardware-Konfiguration

| Parameter | Wert |
|---|---|
| I2C-Adresse | 0x6B |
| NTC | NCP15XH103F03RC (10 kΩ, B=3380) |
| Pull-Up | 5.6 kΩ |
| Parallel-Widerstand | 27 kΩ |
| IBUS ADC Offset | 27 mA |

#### API

```cpp
class BQ25798Driver : public Adafruit_BQ25798 {
    bool begin(uint8_t addr, TwoWire* wire);
    
    // Telemetrie (One-Shot ADC, ~250ms)
    const BqTelemetry* getTelemetryData();
    
    // JEITA Temperatursteuerung
    bool setJeitaVSet(bq_jeita_vset_t);     // Spannungsreduktion (0–800 mV)
    bool setJeitaISetH(bq_jeita_iseth_t);   // Strom bei Hitze
    bool setJeitaISetC(bq_jeita_isetc_t);   // Strom bei Kälte
    bool setTsCool(bq_ts_cool_t);            // Kalt-Schwelle (5–20 °C)
    bool setTsWarm(bq_ts_warm_t);            // Warm-Schwelle (40–55 °C)
    bool setTsIgnore(bool);                  // NTC-Überwachung deaktivieren
    
    // Status
    bool getChargerStatusPowerGood();
    bq_charging_status_t getChargingStatus();
    
    // IBAT ADC Steuerung (Stromsparen)
    bool stopIbatADC();
    bool startIbatADC();
};
```

#### Telemetrie-Struktur

```cpp
BqTelemetry {
    BqSolarData solar {
        uint16_t voltage;    // Solar-Spannung (mV)
        int16_t  current;    // Solar-Strom (mA)
        int32_t  power;      // Solar-Leistung (mW)
        bool     mppt;       // MPPT aktiv
    };
    BqBattData battery {
        uint16_t voltage;    // Batteriespannung (mV)
        float    current;    // Batteriestrom (mA)
        int32_t  power;      // Leistung (mW)
        float    temperature;// NTC-Temperatur (°C)
    };
    BqSysData system {
        uint16_t voltage;    // Systemspannung (mV)
    };
};
```

#### Besonderheiten

- **One-Shot ADC**: ADC wird nur bei Bedarf aktiviert (spart ~1.5 mA)
- **Steinhart-Hart NTC**: Präzise Temperaturberechnung über das NTC-Netzwerk (Pull-Up + Parallel-Widerstand berücksichtigt)
- **IBUS Offset**: 27 mA ADC-Offset wird automatisch subtrahiert
- **Erbt von `Adafruit_BQ25798`**: Nutzt die Adafruit-Library für Basisfunktionalität (`setChargeLimitV`, `setChargeLimitA`, `setMPPTenable`, etc.)

---

## InheroMr2Module

**Datei**: `InheroMr2Module.h` / `InheroMr2Module.cpp`

Zentrales Board-Modul das alle inhero-spezifische Funktionalität bündelt.

### Klassen-Hierarchie

```
concurrency::OSThread          → Periodische Tasks (runOnce)
    └─ SinglePortModule        → PortNum 256 Empfang (handleReceived)
         └─ InheroMr2Module    → Board-Logik
```

### Initialisierung

1. Modul wird in `Modules.cpp` instanziiert (`#ifdef INHERO_MR2`)
2. Erster `runOnce()`-Aufruf initialisiert die Hardware-Treiber
3. LittleFS-Konfiguration wird geladen
4. INA228 und BQ25798 werden initialisiert
5. Batterie-Chemie wird auf den Charger angewendet

### Timing

| Task | Intervall | Beschreibung |
|---|---|---|
| `runOnce()` Hauptschleife | 10 Sekunden | Basis-Zyklus |
| INA228 Messung | Jede Iteration (10s) | Batterie V/I/T/Q |
| BQ25798 Telemetrie | 30 Sekunden | Solar-Daten (wegen 250ms ADC-Delay) |
| Solar MPPT Check | 30 Sekunden | Stuck PGOOD + MPPT Re-Enable |
| Power-Telemetrie senden | 60 Sekunden (konfigurierbar) | PowerMetrics Broadcast |
| Hourly Stats Update | ~60 Sekunden (Prüfung) | Schreibt bei Stundenwechsel |
| MPPT Stats Update | 10 Sekunden | Minuten-Akku + Energie-Akku |
| Watchdog Feed | 10 Sekunden | WDT-Reload (600s Timeout) |

---

### Telemetrie

Sendet Standard-Meshtastic `PowerMetrics` Protobuf auf `TELEMETRY_APP` Port:

| Kanal | Quelle | Spannung | Strom |
|---|---|---|---|
| **ch1** | INA228 (Batterie) | Batterie V | Batterie A |
| **ch2** | BQ25798 (Solar) | Solar V | Solar A |
| **ch3** | BQ25798 (System) | System V | — |

Die Telemetrie wird als Broadcast gesendet und ist in jeder Standard-Meshtastic-App sichtbar.

---

### Konfigurations-Protokoll (CLI über Text-DMs)

MeshCore-kompatible CLI-Befehle über normale Meshtastic **Textnachrichten (DMs)**. Befehle beginnen mit `/` und werden als direkte Nachricht an den Node gesendet.

#### Authentifizierung

Schreib-Befehle (`/set`, `/reboot`) erfordern, dass der **Public Key des Absenders** in `config.security.admin_key[0..2]` eingetragen ist — identisch zum Meshtastic AdminModule.

- **PKI-verschlüsselte DMs**: Absender-Key kryptographisch verifiziert
- **Channel-verschlüsselte DMs**: Fallback auf NodeDB-Lookup
- **Lokale Nachrichten** (Serial/BLE): Immer erlaubt

#### Lese-Befehle (`/get board.<key>`)

| Befehl | Beschreibung | Beispiel-Antwort |
|---|---|---|
| `/get board.bat` | Batterie-Chemie | `lifepo1s` |
| `/get board.hwver` | Hardware-Version | `v0.2 (INA228+RTC)` |
| `/get board.telem` | Echtzeit-Telemetrie | `B:3.25V/120.5mA/32C SOC:75% S:5.10V/~200mA` |
| `/get board.conf` | Alle Config-Werte | `B:lifepo1s F:on M:1 I:500mA Vco:3.50 V0:2.95` |
| `/get board.diag` | Diagnostik | `ina228=ok bq25798=ok diag=0x0000 chg=3 pgood=1` |
| `/get board.frost` | Frostschutz-Status | `frost=on` |
| `/get board.imax` | Max. Ladestrom | `500mA` |
| `/get board.mppt` | MPPT-Status | `MPPT=1` |
| `/get board.leds` | LED-Status | `LEDs: ON (Heartbeat + BQ Stat)` |
| `/get board.uvlo` | UVLO-Status | `UVLO: ENABLED` |
| `/get board.ibcal` | INA228-Kalibrierung | `INA228 calibration: 1.0000 (1.0=default)` |
| `/get board.tccal` | Temperatur-Kalibrierung | `TC offset: +0.00C (0.00=default)` |
| `/get board.batcap` | Batterie-Kapazität | `3000 mAh (default)` |
| `/get board.energy` | Coulomb-Counter | `125.3mAh` |
| `/get board.stats` | 168h Energie-Statistik | `24h:150/200mAh 3d:140/195 7d:135/190 TTL:720h MPPT:85%` |
| `/get board.cinfo` | Charger-Info (BQ25798) | `PG:1 CHG:3 HIZ:0 MPPT:1 VBUS:5.12V` |
| `/get board.togglehiz` | HIZ-Zyklus (Debug) | `HIZ cycled: PG 0->1 CHG:3` |

#### Schreib-Befehle (`/set board.<key> <value>`) — Admin erforderlich

| Befehl | Parameter | Bereich | Beschreibung |
|---|---|---|---|
| `/set board.bat <type>` | `lto2s`, `lifepo1s`, `liion1s` | — | Batterie-Chemie |
| `/set board.imax <mA>` | Ganzzahl | 10–1000 | Max. Ladestrom |
| `/set board.mppt <v>` | `true/1`, `false/0` | — | MPPT ein/aus |
| `/set board.frost <v>` | `0/off`, `1/on` | — | Frostschutz (JEITA) |
| `/set board.leds <v>` | `on/1`, `off/0` | — | LEDs ein/aus |
| `/set board.uvlo <v>` | `true/1`, `false/0` | — | UVLO-Alert ein/aus |
| `/set board.ibcal <v>` | mA-Wert oder `reset` | −2000–2000 | INA228 Strom-Kalibrierung |
| `/set board.tccal <v>` | °C-Wert oder `reset` | −40–85 | NTC Temperatur-Kalibrierung |
| `/set board.batcap <v>` | mAh | 100–100000 | Batterie-Kapazität |
| `/set board.bqreset` | — | — | BQ25798 Software-Reset |
| `/set board.soc <v>` | Prozent | 0–100 | SOC manuell setzen |

#### Weitere Befehle

| Befehl | Admin | Beschreibung |
|---|---|---|
| `/help` | Nein | Alle verfügbaren Befehle anzeigen |
| `/ver` | Nein | Firmware-Version anzeigen |
| `/reboot` | Ja | Node neustarten |

#### Antwort-Format

- Erfolg: Beschreibender Text (z.B. `Bat set to lifepo1s`)
- Fehler: `Err: <message>` (z.B. `Err: imax range 10-1000 mA`)
- Nicht autorisiert: `Err: Not authorized. Your public key must be in admin_key config.`

#### Beispiel-Session (Meshtastic Chat)

```
Du:     /get board.telem
MR-2:   B:3.25V/120.5mA/32C SOC:75%(CC) S:5.10V/~200mA

Du:     /set board.bat lifepo1s
MR-2:   Bat set to lifepo1s

Du:     /set board.imax 300
MR-2:   Max charge current set to 300mA

Du:     /get board.stats
MR-2:   24h:150/200mAh 3d:140/195 7d:135/190 TTL:720h MPPT:85%

Du:     /get board.cinfo
MR-2:   PG:1 CHG:3 HIZ:0 MPPT:1 VBUS:5.12V

Du:     /get board.togglehiz
MR-2:   HIZ cycled: PG 0->1 CHG:3

Du:     /help
MR-2:   /get board.<key>
          bat telem conf diag hwver frost imax mppt
          leds uvlo ibcal tccal batcap energy
          stats cinfo togglehiz
        /set board.<key> <value> [admin]
          bat <lto2s|lifepo1s|liion1s>
          imax <10-1000> frost <0|1> mppt <0|1>
          ...
```

---

### Batterie-Chemie

Unterstützte Chemien mit ihren Spannungsschwellen:

| Chemie | Laden (mV) | Gefahr (mV) | Nominal (mV) | Voll (mV) | Leer (mV) |
|---|---|---|---|---|---|
| **LTO 2S** | 5400 | 4200 | 4600 | 5200 | 4400 |
| **LiFePO4 1S** | 3500 | 2900 | 3200 | 3400 | 2950 |
| **Li-Ion 1S** | 4100 | 3400 | 3700 | 4050 | 3500 |
| **Unbekannt** | 3500 | 2900 | 3200 | 3400 | 2950 |

#### Beim Setzen der Chemie passiert:

1. **Ladespannung** wird am BQ25798 gesetzt (`setChargeLimitV`)
2. **Ladestrom** wird gesetzt (`setChargeLimitA`)
3. **MPPT** wird konfiguriert
4. **JEITA-Frostschutz** (wenn aktiviert):
   - Unter 5 °C: Spannung −200 mV, Strom auf 20 %
5. **INA228 UVLO-Alert** wird auf `dangerVoltage_mV` konfiguriert

#### SOC-Schätzung

**Primär: INA228 Hardware Coulomb Counting** (portiert von MeshCore)

Der INA228 zählt kontinuierlich in Hardware die ein-/ausgehende Ladung (CHARGE-Register `0x0A`, 40 Bit).
Das Modul liest dieses Register alle 10 Sekunden und berechnet:

$$SOC = \\frac{Kapazität + (CHARGE_{aktuell} - Baseline)}{Kapazität} \\times 100\\%$$

**Kalibrierung:**
- **Automatisch**: Wenn BQ25798 „Charging Done" meldet → `syncSOCToFull()` setzt SOC=100% und resettet den Coulomb-Counter
- **Manuell**: `/set board.soc 85` berechnet die Baseline aus dem aktuellen CHARGE-Wert rückwärts
- **Kapazität**: `/set board.batcap <mAh>` oder Default pro Chemie (3000 mAh)

**Wichtig:** Nach Reboot ist der SOC zunächst ungültig (INA228 CHARGE-Register ist flüchtig).
Er zeigt `(V)` = Voltage-Fallback bis zum ersten Charge-Done-Event oder `/set board.soc`.
Danach wechselt er zu `(CC)` = Coulomb Counting.

**Fallback: Spannungsbasierte Schätzung** (lineare Interpolation)

```
SOC = (V_aktuell - V_leer) / (V_voll - V_leer) × 100%
```

Wird nur verwendet, wenn noch keine Coulomb-Counting-Kalibrierung stattgefunden hat.

#### Integration in Meshtastic Power-System

Das InheroMr2Module registriert sich als `HasBatteryLevel`-Provider im Meshtastic Power-System
(`INHERO_MR2_POWER` Define in `variant.h`). Dadurch werden die **chemie-korrekten Werte**
automatisch in die Standard-`DeviceMetrics`-Telemetrie eingespeist:

| DeviceMetrics-Feld | Quelle | Beschreibung |
|---|---|---|
| `battery_level` | `estimateSOC()` | Chemie-abhängiger SOC (0–100%) |
| `voltage` | INA228 | Präzise Batteriespannung (nicht ADC) |
| `is_charging` | BQ25798 | Tatsächlicher Ladezustand (CC/CV/Trickle) |

Die Standard-LiIon-OCV-Tabelle (`power.h`) wird **nicht** verwendet — stattdessen werden die
chemie-spezifischen Spannungskurven aus `getChemistryParams()` genutzt. Damit zeigt die
Meshtastic-App bei LiFePO4 und LTO korrekte Prozentwerte an.

---

### LittleFS Persistenz

Konfiguration wird im internen Flash (LittleFS) unter `/inhero/` gespeichert:

| Datei | Inhalt | Beispiel |
|---|---|---|
| `/inhero/bat.txt` | Batterie-Chemie | `lifepo1s` |
| `/inhero/imax.txt` | Max. Ladestrom | `500` |
| `/inhero/mppt.txt` | MPPT-Status | `1` |
| `/inhero/leds.txt` | LED-Status | `1` |
| `/inhero/frost.txt` | Frostschutz | `1` |
| `/inhero/cal.txt` | INA228-Kalibrierung | `1.0000` |
| `/inhero/tccal.txt` | NTC Temp-Kalibrierung | `0.00` |
| `/inhero/uvlo.txt` | UVLO-Status | `1` |
| `/inhero/batcap.txt` | Batterie-Kapazität (mAh) | `0` (0 = Default) |

Die Konfiguration wird beim Booten automatisch geladen und bei jedem `/set`-Befehl gespeichert.
Legacy-Werte (`lifepo4`, `liion`) werden beim Laden automatisch auf die MeshCore-Namen (`lifepo1s`, `liion1s`) gemapped.

---

### LED-Steuerung

| LED | Pin | Funktion |
|---|---|---|
| **Blau** (PIN_LED1 = 35) | P1.03 | Status (von Meshtastic verwaltet) |
| **Rot** (PIN_LED2 = 36) | P1.04 | Prioritätsbasierte Statusanzeige (siehe unten) |

**LED2 (Rot) – Prioritätslogik:**

| Priorität | Zustand | Bedeutung |
|---|---|---|
| 1 (höchste) | **100ms Blitz alle 3s** | Batterie unter Danger-Schwelle (stromsparend) |
| 2 | **500ms on/off Blinken** | Fehlender I2C-Baustein (INA228, BQ25798 oder RTC) |
| 3 | **200ms Blitz** | CLI-Befehl empfangen und verarbeitet |
| 4 (Standard) | **AUS** | Normalbetrieb – BQ25798 STAT-LED zeigt Ladezustand |

LEDs können per `/set leds 0` komplett deaktiviert werden (Stromsparen).

---

## System-Zuverlässigkeit

### Hardware Watchdog (nRF52 WDT)

**Datei**: `InheroMr2Module.cpp` → `setupWatchdog()` / `feedWatchdog()`

nRF52 Hardware-Watchdog mit **600 Sekunden Timeout** (~10 Minuten). Wird bei jedem `runOnce()`-Zyklus (alle 10s) gefüttert. Bei Firmware-Hänger erfolgt automatischer Reset.

```
Boot → setupWatchdog()
         │
         ├── NRF_WDT->CRV = 32768 * 600 - 1
         ├── NRF_WDT->CONFIG = Run in Sleep + Pause in Halt
         └── NRF_WDT->TASKS_START = 1
         
runOnce() → feedWatchdog()
         └── NRF_WDT->RR[0] = WDT_RR_RR_Reload
```

**Wichtig**: Der nRF52 WDT kann nach dem Start **nicht mehr gestoppt** werden — nur durch Reset. Das ist beabsichtigt (Zuverlässigkeit).

---

### GPREGRET2 Shutdown-Tracking

**Register**: `NRF_POWER->GPREGRET2` (überlebt Warm-Resets und System-OFF-Wake)

| Bit | Name | Bedeutung |
|---|---|---|
| [1:0] | Shutdown Reason | `0x00`=None, `0x01`=Low Voltage, `0x02`=User Request, `0x03`=Thermal |
| [2] | Danger Zone Flag | `0x04` — Wenn gesetzt: höhere Boot-Schwelle (3.2V statt 2.8V) |

**Ablauf:**

```
Shutdown (Danger Zone)
    │
    ├── GPREGRET2 = reason | DANGER_ZONE_FLAG
    └── sd_power_system_off()
           │
           ▼
Wake (RTC / VBUS)
    │
    ├── variant.cpp: initVariant()
    │   ├── Liest GPREGRET2
    │   ├── Wenn DANGER_ZONE: Schwelle = 3200 mV (statt 2800 mV)
    │   ├── VBAT < Schwelle → SYSTEMOFF (mit neuem GPREGRET2)
    │   └── VBAT >= Schwelle → Clear DANGER_ZONE, normaler Boot
    │
    └── InheroMr2Module: setupDrivers()
        ├── Liest GPREGRET2
        └── Wenn LOW_VOLTAGE: skipFsWrites = true
```

---

### Early Boot Spannungsprüfung

**Datei**: `variant.cpp` → `initVariant()`

Verhindert **Motorboating** (Boot-Crash-Reboot-Schleife) bei kritisch niedrigem Akkustand:

```
Boot → Wire.begin() → INA228::readVBATDirect()
         │
         ├── Lese GPREGRET2 (Danger-Zone Flag?)
         │
         ├── DANGER_ZONE gesetzt → Schwelle = 3200 mV
         │                         (schützt vor Oszillieren)
         ├── Kein DANGER_ZONE    → Schwelle = 2800 mV
         │
         ├── VBAT > Schwelle → Clear DANGER_ZONE → Normaler Boot
         │
         └── VBAT < Schwelle → GPREGRET2 = LOW_VOLTAGE | DANGER_ZONE
                                → 3V3 OFF → LEDs OFF
                                → NRF_POWER->SYSTEMOFF (~1 µA)
```

- Nutzt `Ina228Driver::readVBATDirect()` (statische Methode, ohne vollständige Treiber-Initialisierung)
- Standard-Schwelle: **2800 mV** (unterhalb aller Chemie-Gefahrenschwellen)
- Danger-Zone-Schwelle: **3200 mV** (hysterese gegen Oszillation)
- System OFF Modus: Aufwachen durch USB-Verbindung (VBUS) oder RTC-Alarm

---

### Danger-Zone Shutdown

**Datei**: `InheroMr2Module.cpp` → `initiateShutdown(reason)`

Kontrollierter Shutdown bei kritischem Batteriestatus (im Gegensatz zum harten `NRF_POWER->SYSTEMOFF` beim Early Boot):

```
batteryData.voltage_mv < dangerVoltage_mV
    │
    ├── 1. SX1262 Radio ausschalten
    │      └── digitalWrite(SX126X_POWER_EN, LOW)
    │
    ├── 2. RTC Wake konfigurieren
    │      └── RV-3028 Countdown Timer (2h bei Low-Voltage, 6h bei User-Request)
    │
    ├── 3. GPREGRET2 speichern
    │      └── reason | DANGER_ZONE_FLAG
    │
    └── 4. sd_power_system_off()
           └── Wacht per RTC-Interrupt auf
```

**RTC-Wake-Konfiguration** (RV-3028 @ 0x52):

| Register | Wert | Beschreibung |
|---|---|---|
| 0x0A-0x0B | hours × 60 | Countdown Timer Value (Minuten) |
| 0x0E | 0x00 | Status — Timer-Flag löschen |
| 0x0F | 0x02 | Control 1 — Timer Interrupt Enable (TIE) |
| 0x10 | 0x06 | Control 2 — Timer Enable + 1/60 Hz Clock |

---

### skipFsWrites bei Low-Voltage Boot

Wenn das Board nach einem Danger-Zone-Shutdown aufwacht (`GPREGRET2 & 0x03 == LOW_VOLTAGE`), wird `skipFsWrites = true` gesetzt. Dadurch werden **keine Flash-Schreibzugriffe** durchgeführt, um:

1. Die Batterie nicht durch Flash-Erase/Write-Zyklen (~10 mA Peaks) weiter zu belasten
2. Dateisystem-Korruption bei instabiler Versorgungsspannung zu vermeiden

Betroffene Funktionen:
- `saveConfig()` — gibt sofort zurück mit Warnung im Log
- LittleFS-Schreibzugriffe werden übersprungen

---

## Solar-MPPT-Management

### Stuck PGOOD Fix

**Datei**: `InheroMr2Module.cpp` → `checkAndFixPgoodStuck()`

**Problem**: Der BQ25798 kann in einen Zustand geraten, in dem VBUS anliegt (Solar-Panel verbunden) aber PGOOD low bleibt — der Charger erkennt den Input nicht.

**Lösung**: HIZ-Modus-Toggle erzwingt eine neue Input-Erkennung.

```
VBUS > 3.5V  &&  PGOOD == 0  &&  PG_FLAG == 0
    │
    ├── Cooldown (5 Minuten seit letztem Toggle)
    │
    └── setHIZMode(true) → delay(200ms) → setHIZMode(false)
        └── BQ25798 re-evaluiert den Input
```

| Parameter | Wert |
|---|---|
| VBUS-Schwelle | 3500 mV |
| Toggle-Cooldown | 5 Minuten (`HIZ_TOGGLE_COOLDOWN_MS`) |
| Prüf-Intervall | Alle 30s (bei jeder Solar-Daten-Lesung) |

---

### MPPT Re-Enable

**Datei**: `InheroMr2Module.cpp` → `checkAndFixSolarLogic()`

**Problem**: Der BQ25798 kann MPPT nach bestimmten Fault-Events deaktivieren, obwohl Solar-Power verfügbar ist.

**Lösung**: Wenn PGOOD=1 und MPPT in der Config aktiviert aber im Chip deaktiviert ist, wird MPPT per Register-Schreibzugriff wiederhergestellt.

```
PGOOD == 1  &&  config.mpptEnabled == true  &&  chip.MPPT == false
    │
    ├── Cooldown (60 Sekunden seit letztem Schreibzugriff)
    │
    └── setMPPTenable(true)
```

| Parameter | Wert |
|---|---|
| Schreib-Cooldown | 60 Sekunden (`MPPT_WRITE_COOLDOWN_MS`) |

---

## Energy Analytics (168h)

### Hourly Stats Rolling Buffer

**Datei**: `InheroMr2Module.cpp` → `updateHourlyStats()`

168-Stunden (= 7 Tage) Ringpuffer für Energie-Statistiken. Jede Stunde wird gespeichert:

| Feld | Typ | Beschreibung |
|---|---|---|
| `timestamp` | `uint32_t` | Unix-Timestamp (Beginn der Stunde) |
| `charged_mah` | `float` | Geladene Energie in dieser Stunde (mAh) |
| `discharged_mah` | `float` | Entladene Energie in dieser Stunde (mAh) |
| `solar_mah` | `float` | Solar-Beitrag in dieser Stunde (mAh) |

**Speicher**: 168 × 16 Bytes = **2.688 Bytes RAM** (kein Flash — flüchtig wie in MeshCore)

**Timing**: `minuteCounter` zählt in 10s-Zyklen, alle 6 Zyklen (~60s) wird `updateHourlyStats()` aufgerufen. Bei Stundenwechsel wird der aktuelle Slot gespeichert und der Index weitergerückt.

---

### Rolling Averages (24h / 3d / 7d)

**Datei**: `InheroMr2Module.cpp` → `calculateRollingStats()`

Berechnet aus dem Ringpuffer gleitende Durchschnitte:

| Metrik | Fenster | Beschreibung |
|---|---|---|
| `last_24h_charged_mah` | 24h | Gesamte Ladung letzte 24 Stunden |
| `last_24h_discharged_mah` | 24h | Gesamte Entladung letzte 24 Stunden |
| `last_24h_net_mah` | 24h | Netto (charged - discharged) |
| `avg_3day_daily_charged_mah` | 72h | Tages-Durchschnitt Ladung (3 Tage) |
| `avg_3day_daily_discharged_mah` | 72h | Tages-Durchschnitt Entladung (3 Tage) |
| `avg_7day_daily_charged_mah` | 168h | Tages-Durchschnitt Ladung (7 Tage) |
| `avg_7day_daily_discharged_mah` | 168h | Tages-Durchschnitt Entladung (7 Tage) |
| `living_on_battery` | 24h | `true` wenn Netto-Bilanz negativ |

**Abruf**: `/get board.stats` zeigt alle Werte kompakt an.

---

### TTL (Time To Live)

**Datei**: `InheroMr2Module.cpp` → `calculateTTL()`

Schätzt die verbleibende Laufzeit in Stunden:

$$TTL = \\frac{SOC \\times Kapazität}{täglicher\\_Netto\\text{-}Defizit / 24}$$

- Nutzt 3-Tage-Durchschnitt (stabiler als 24h)
- Fallback auf 24h-Durchschnitt wenn 3d noch nicht verfügbar
- `0xFFFF` = quasi unendlich (Netto-Bilanz positiv)
- `0` = nicht berechenbar (kein SOC oder keine Daten)

---

### MPPT-Statistiken

**Datei**: `InheroMr2Module.cpp` → `updateMpptStats()` / `getMpptEnabledPercentage7Day()`

Zweiter 168-Stunden-Ringpuffer speziell für MPPT-Effizienz:

| Feld | Typ | Beschreibung |
|---|---|---|
| `mpptEnabledMinutes` | `uint8_t` | Minuten mit aktivem MPPT (0-60 pro Stunde) |
| `timestamp` | `uint32_t` | Unix-Timestamp |
| `harvestedEnergy_mWh` | `uint32_t` | Geerntete Solarenergie (mWh) |

**7-Tage MPPT-Effizienz** = `Σ(mpptEnabledMinutes) / (Σ(slots) × 60) × 100%`

Wird im `/get board.stats`-Output als `MPPT:85%` angezeigt.

---

## I2C Scanner Erweiterung

### Problem

Meshtastic's I2C-Scanner prüft Register `0xFE` für die TI Manufacturer ID (`0x5449`). Die INA228 hat die MFG_ID aber bei Register `0x3E` — ohne Fix würde die INA228 fälschlicherweise als INA219 erkannt.

### Lösung

Erweiterung der Erkennung in `ScanI2CTwoWire.cpp`:

```
Adresse 0x40 erkannt
    │
    ├── Reg 0xFE = 0x5449 → INA226 oder INA260
    │
    └── Reg 0xFE ≠ 0x5449
            │
            ├── Reg 0x3E = 0x5449 → INA228 ✓ (NEU)
            │
            └── Reg 0x3E ≠ 0x5449 → INA219 (Fallback)
```

Dies betrifft **alle** Meshtastic-Builds, da es sich um eine korrekte Geräteerkennung handelt, nicht um board-spezifischen Code.

---

## Änderungen am Meshtastic-Hauptcode

### 1. `src/detect/ScanI2C.h`

```diff
  INA226,
+ INA228,
  NXP_SE050,
```

### 2. `src/detect/ScanI2CTwoWire.cpp`

```diff
  } else {
+     // Check for INA228/INA229 (MFG_ID at register 0x3E instead of 0xFE)
+     registerValue = getRegisterValue(RegisterLocation(addr, 0x3E), 2);
+     if (registerValue == 0x5449) {
+         registerValue = getRegisterValue(RegisterLocation(addr, 0x3F), 2);
+         logFoundDevice("INA228", (uint8_t)addr.address);
+         type = INA228;
+     } else {
          logFoundDevice("INA219", (uint8_t)addr.address);
          type = INA219;
+     }
  }
```

### 3. `src/modules/Modules.cpp`

```diff
+ #ifdef INHERO_MR2
+ #include "InheroMr2Module.h"
+ #endif

  // (in setupModules(), vor RoutingModule)
+ #ifdef INHERO_MR2
+     new InheroMr2Module();
+ #endif
```

### 4. `src/Power.cpp`

```diff
+ #ifdef INHERO_MR2_POWER
+ #include "InheroMr2Module.h"
+
+ class InheroMr2BatteryLevel : public HasBatteryLevel
+ {
+   public:
+     int getBatteryPercent() override;     // → InheroMr2Module::getBatterySOC()
+     uint16_t getVoltage() override;       // → InheroMr2Module::getBatteryVoltageMv()
+     bool isBatteryConnect() override;     // → InheroMr2Module::hasBattery()
+     bool isVbusIn() override;             // → InheroMr2Module::hasExternalPower()
+     bool isCharging() override;           // → InheroMr2Module::isBatteryCharging()
+ };
+ #endif
```

Delegiert alle Power-Abfragen an das InheroMr2Module-Singleton. Dadurch zeigt die Meshtastic-App chemie-korrekte SOC-Werte anstatt der Standard-LiIon-OCV-Tabelle.

---

## Build

### Voraussetzungen

- PlatformIO Core ≥ 6.x
- nRF52840 Toolchain (wird automatisch installiert)

### Build-Befehl

```bash
pio run -e inhero_mr2
```

### Ausgabe

```
RAM:   [====      ]  44.3% (110,320 / 248,832 bytes)
Flash: [========= ]  89.2% (727,000 / 815,104 bytes)
→ firmware-inhero_mr2-2.7.20.xxxxxxx.uf2
```

### Library Dependencies

| Library | Quelle | Verwendung |
|---|---|---|
| Melopero RV3028 @ 1.2.0 | PlatformIO Registry | RTC |
| Adafruit BQ25798 Library | GitHub (HEAD) | Solar-Charger-Basisklasse |

### platformio.ini Highlights

```ini
[env:inhero_mr2]
extends = nrf52840_base
board = inhero_mr2
board_level = extra
build_flags = ...
  -D INHERO_MR2                    # Compile Guard
  -DRADIOLIB_EXCLUDE_SX128X=1      # Nicht benötigte Radios ausschließen
  -DRADIOLIB_EXCLUDE_SX127X=1
  -DRADIOLIB_EXCLUDE_LR11X0=1
build_src_filter = ${nrf52_base.build_src_filter}
  +<../variants/nrf52840/inhero_mr2>  # Variant-Code wird mitkompiliert
```

---

## Konfiguration über Text-DMs (CLI)

Die Konfiguration erfolgt über **normale Meshtastic-Textnachrichten (DMs)** — keine spezielle App erforderlich. Jede Standard-Meshtastic-App (Android, iOS, Web, CLI) kann verwendet werden.

### Voraussetzungen

1. **Public Key** des Admin-Geräts muss in `config.security.admin_key[0..2]` eingetragen sein
2. Nachrichten werden als **DM** (direkte Nachricht) an den MR-2 Node gesendet
3. Befehle beginnen mit `/`

### Protokoll

1. Admin sendet DM mit `/`-Befehl an den MR-2 Node
2. Modul prüft den Public Key des Absenders gegen `config.security.admin_key[]`
3. Bei Lesebefehl: Wert wird gelesen und als DM zurückgesendet
4. Bei Schreibbefehl: Admin-Check → Wert setzen → Bestätigung als DM

### Beispiel (meshtastic CLI)

```bash
# Telemetrie abfragen (erscheint im Chat)
meshtastic --sendtext "/get board.telem" --dest '!aabbccdd'

# Batterie-Chemie setzen (Admin-Key erforderlich)
meshtastic --sendtext "/set board.bat lifepo1s" --dest '!aabbccdd'

# Hilfe anzeigen
meshtastic --sendtext "/help" --dest '!aabbccdd'
```

### Beispiel (Python mit meshtastic-python)

```python
import meshtastic

interface = meshtastic.SerialInterface()

# Telemetrie abfragen (-> normale Textnachricht DM)
interface.sendText("/get board.telem", destinationId="!aabbccdd")

# Batterie-Chemie setzen
interface.sendText("/set board.bat lifepo1s", destinationId="!aabbccdd")
```

### Sicherheitsmodell

- **Lesebefehle** (`/get`, `/help`, `/ver`): Keine Admin-Authentifizierung erforderlich
- **Schreibbefehle** (`/set`, `/reboot`): Public Key muss als `admin_key` konfiguriert sein
- **PKI-verschlüsselte DMs**: Höchste Sicherheit — Absender-Key kryptographisch verifiziert
- **Channel-verschlüsselte DMs**: Fallback auf NodeDB-Lookup (weniger sicher)
- **Lokale Verbindung** (USB/BLE): Immer erlaubt

---

## Portierung von MeshCore

### Mapping MeshCore → Meshtastic

| MeshCore Konzept | Meshtastic Äquivalent |
|---|---|
| `Board::setup()` | `InheroMr2Module::setupDrivers()` |
| `Board::loop()` / FreeRTOS Tasks | `InheroMr2Module::runOnce()` (10s OSThread) |
| `BoardConfigContainer` | `InheroMr2Config` + LittleFS |
| `CLI (get/set board.X)` | `/get board.X` / `/set board.X` via Text-DMs |
| `Admin-Passwort` | `config.security.admin_key[]` (Public Key Auth) |
| `CayenneLPP Telemetrie` | Meshtastic `PowerMetrics` Protobuf |
| `MESH_DEBUG_PRINTLN` | `LOG_INFO` / `LOG_WARN` / `LOG_ERROR` |
| `SimplePreferences` | LittleFS (`/inhero/*.txt`) |
| `NRF52Board::deepSleep()` | `initiateShutdown()` → `sd_power_system_off()` |
| `GPREGRET2 Check (begin)` | `variant.cpp::initVariant()` + `setupDrivers()` |
| `nRF52 WDT (600s)` | `setupWatchdog()` / `feedWatchdog()` |
| `SolarDaemonTask` | `checkAndFixPgoodStuck()` + `checkAndFixSolarLogic()` in `runOnce()` |
| `HeartbeatTask` | Meshtastic-eigenes Heartbeat (kein Port nötig) |
| `SOCUpdateTask` | `updateBatterySOC()` + `updateHourlyStats()` in `runOnce()` |
| `VoltageMonitorTask` | Danger-Check in `runOnce()` → `initiateShutdown()` |
| `HourlyBatteryStats[168]` | `socStats.hours[168]` |
| `MpptHourlyStats[168]` | `mpptStats.hours[168]` |
| `calculateRollingStats()` | `calculateRollingStats()` (identische Logik) |
| `calculateTTL()` | `calculateTTL()` (identische Logik) |
| `getMpptEnabledPercentage7Day()` | `getMpptEnabledPercentage7Day()` (identisch) |
| `configureRTCWake()` | `configureRTCWake()` (RV-3028 Timer) |
| `getCustomGetter("stats")` | `/get board.stats` |
| `getCustomGetter("cinfo")` | `/get board.cinfo` |
| `getCustomGetter("togglehiz")` | `/get board.togglehiz` |
| `skip_fs_writes` | `skipFsWrites` in `saveConfig()` |
| `errorLedTask` | `errorLedActive` Flag → P2 in `updateLEDs()` |
| `BQ Interrupt + Semaphore` | Polling alle 30s in `runOnce()` |

### Unterschiede

1. **Telemetrie-Format**: MeshCore nutzt CayenneLPP, Meshtastic nutzt Protobuf (`PowerMetrics`)
2. **Config-Speicherung**: MeshCore nutzt `Preferences` (ESP-NVS-ähnlich), Meshtastic-Port nutzt LittleFS
3. **Charger API**: MeshCore nutzt direkte Register-Schreibzugriffe, Meshtastic-Port nutzt `Adafruit_BQ25798` als Basisklasse mit `setChargeLimitV()` / `setChargeLimitA()` (Argumente in V/A statt mV/mA)
4. **Modul-System**: MeshCore hat ein eigenes Board-Klassen-System, Meshtastic nutzt `SinglePortModule` + `OSThread`
5. **Task-Modell**: MeshCore nutzt 4 FreeRTOS-Tasks mit Semaphoren, Meshtastic nutzt einen einzigen `runOnce()`-Zyklus mit internen Zählern
6. **BQ-Interrupts**: MeshCore nutzt GPIO21 Interrupt + FreeRTOS Semaphore, Meshtastic pollt alle 30s
7. **Energy Analytics**: Identische Ringpuffer-Logik (168h), identische RAM-only-Speicherung
8. **Watchdog**: Identisch (nRF52 WDT, 600s)
9. **GPREGRET2**: Identische Bit-Zuordnung und Logik
10. **Shutdown**: Identisch (SX1262 aus → RTC Wake → GPREGRET2 → SYSTEMOFF)
