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
   - [Konfigurations-Protokoll](#konfigurations-protokoll)
   - [Batterie-Chemie](#batterie-chemie)
   - [LittleFS Persistenz](#littlefs-persistenz)
   - [LED-Steuerung](#led-steuerung)
6. [Early Boot Spannungsprüfung](#early-boot-spannungsprüfung)
7. [I2C Scanner Erweiterung](#i2c-scanner-erweiterung)
8. [Änderungen am Meshtastic-Hauptcode](#änderungen-am-meshtastic-hauptcode)
9. [Build](#build)
10. [Konfiguration über Admin-App](#konfiguration-über-admin-app)

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

Das Design folgt dem Prinzip **"Self-Contained Module"**: Der gesamte board-spezifische Code liegt im Variant-Verzeichnis und wird nur für `inhero_mr2` kompiliert. Es gibt nur **3 minimale Änderungen** am Meshtastic-Hauptcode.

```
┌──────────────────────────────────────────────────────┐
│                  Meshtastic Core                     │
│                                                      │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────┐  │
│  │ Modules.cpp │  │ ScanI2C.h    │  │ ScanI2C    │  │
│  │ +3 Zeilen   │  │ +1 Enum      │  │ TwoWire.cpp│  │
│  │ #ifdef      │  │ INA228       │  │ +Detection │  │
│  └──────┬──────┘  └──────────────┘  └────────────┘  │
│         │                                            │
└─────────┼────────────────────────────────────────────┘
          │ new InheroMr2Module()
          ▼
┌──────────────────────────────────────────────────────┐
│            variants/nrf52840/inhero_mr2/             │
│                                                      │
│  ┌─────────────────────┐  ┌────────────────────────┐ │
│  │  InheroMr2Module    │  │  variant.h / .cpp      │ │
│  │  ├─ Telemetrie      │  │  ├─ Pin-Definitionen   │ │
│  │  ├─ Config-Protokoll│  │  ├─ Early Boot Check   │ │
│  │  ├─ LittleFS        │  │  └─ GPIO Map           │ │
│  │  └─ LED-Steuerung   │  └────────────────────────┘ │
│  └────────┬────────────┘                             │
│           │ verwendet                                │
│  ┌────────┴───────┐  ┌────────────────────┐          │
│  │  Ina228Driver  │  │  BQ25798Driver     │          │
│  │  (.h / .cpp)   │  │  (.h / .cpp)       │          │
│  └────────────────┘  └────────────────────┘          │
└──────────────────────────────────────────────────────┘
```

### Design-Entscheidungen

- **Kein Eingriff in die Sensor-Hierarchie**: Die INA228 wird nicht als Standard-`TelemetrySensor` registriert. Stattdessen sendet `InheroMr2Module` direkt `PowerMetrics`-Protobuf-Pakete. Das Standard-`PowerTelemetryModule` deaktiviert sich automatisch, da keine erkannten Standard-Sensoren vorhanden sind.

- **PortNum 256 (PRIVATE_APP)** für Konfiguration: Ermöglicht eine eigene Admin-App ohne Änderungen an Meshtastic-Protobufs.

- **Compile-Guard `#ifdef INHERO_MR2`**: Alle board-spezifischen Includes und Registrierungen sind hinter diesem Guard, sodass andere Varianten nicht betroffen sind.

---

## Dateistruktur

### Neue Dateien (Variant-Verzeichnis)

| Datei | Zeilen | Beschreibung |
|---|---|---|
| `Ina228Driver.h` | 154 | INA228 Register-Map, Structs, Klassen-Definition |
| `Ina228Driver.cpp` | 402 | I2C-Kommunikation, ADC-Konfiguration, Kalibrierung |
| `BQ25798Driver.h` | 179 | BQ25798 Erweiterung, JEITA-Enums, Telemetrie-Structs |
| `BQ25798Driver.cpp` | 405 | Steinhart-Hart NTC, One-Shot ADC, Register-Zugriff |
| `InheroMr2Module.h` | 115 | Modul-Klasse, Config-Struct, Chemistry-Enum |
| `InheroMr2Module.cpp` | 526 | Telemetrie, Config-Kommandos, LittleFS, SOC |
| `variant.h` | 225 | Pin-Definitionen, Radio-Config, Peripherie |
| `variant.cpp` | 78 | GPIO-Map, Early-Boot-Spannungsprüfung |
| `platformio.ini` | 29 | Build-Konfiguration, Library-Dependencies |

### Modifizierte Dateien (Meshtastic-Kern)

| Datei | Änderung |
|---|---|
| `src/detect/ScanI2C.h` | +1 Zeile: `INA228` in `DeviceType`-Enum |
| `src/detect/ScanI2CTwoWire.cpp` | +12 Zeilen: INA228-Erkennung via Register 0x3E |
| `src/modules/Modules.cpp` | +6 Zeilen: `#ifdef INHERO_MR2` Include + Instanziierung |

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

| Task | Intervall |
|---|---|
| `runOnce()` Hauptschleife | 10 Sekunden |
| INA228 Messung | Jede Iteration (10s) |
| BQ25798 Telemetrie | 30 Sekunden (wegen 250ms ADC-Delay) |
| Power-Telemetrie senden | 60 Sekunden (konfigurierbar) |

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

### Konfigurations-Protokoll

Textbasierte Kommandos auf **PortNum 256** (`PRIVATE_APP`).

#### Lese-Befehle

| Befehl | Antwort-Format |
|---|---|
| `get status` | `v=3200 i=-120 p=-384 t=32.5 sv=5100 si=200 sp=1020 bt=25.3 sys=3300 soc=75` |
| `get config` | `bat=lifepo4 imax=500 mppt=1 leds=1 frost=1 cal=1.0000` |
| `get diag` | `ina228=ok bq25798=ok diag=0x0000 chg=3 pgood=1` |

#### Schreib-Befehle

| Befehl | Parameter | Bereich | Beschreibung |
|---|---|---|---|
| `set bat <type>` | `lto2s`, `lifepo4`, `liion` | — | Batterie-Chemie |
| `set imax <mA>` | Ganzzahl | 50–2000 | Max. Ladestrom |
| `set mppt <0\|1>` | 0 oder 1 | — | MPPT ein/aus |
| `set leds <0\|1>` | 0 oder 1 | — | LEDs ein/aus |
| `set frost <0\|1>` | 0 oder 1 | — | Frostschutz (JEITA) |
| `set cal <factor>` | Float | 0.5–2.0 | INA228 Strom-Kalibrierung |

#### Antwort-Format

- Erfolg: `ok <key>=<value>` (z.B. `ok bat=lifepo4`)
- Fehler: `err: <message>` (z.B. `err: imax out of range (50-2000)`)
- Hilfe: Unbekannte Befehle geben die Befehlsliste zurück

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

Lineare Interpolation zwischen `emptyVoltage_mV` (0 %) und `fullVoltage_mV` (100 %):

```
SOC = (V_aktuell - V_leer) / (V_voll - V_leer) × 100%
```

---

### LittleFS Persistenz

Konfiguration wird im internen Flash (LittleFS) unter `/inhero/` gespeichert:

| Datei | Inhalt | Beispiel |
|---|---|---|
| `/inhero/bat.txt` | Batterie-Chemie | `lifepo4` |
| `/inhero/imax.txt` | Max. Ladestrom | `500` |
| `/inhero/mppt.txt` | MPPT-Status | `1` |
| `/inhero/leds.txt` | LED-Status | `1` |
| `/inhero/frost.txt` | Frostschutz | `1` |
| `/inhero/cal.txt` | INA228-Kalibrierung | `1.0000` |

Die Konfiguration wird beim Booten automatisch geladen und bei jedem `set`-Befehl gespeichert.

---

### LED-Steuerung

| LED | Pin | Funktion |
|---|---|---|
| **Blau** (PIN_LED1 = 35) | P1.03 | Status (von Meshtastic verwaltet) |
| **Rot** (PIN_LED2 = 36) | P1.04 | Ladeanzeige (CC/CV = AN, Done = AUS) |

LEDs können per `set leds 0` komplett deaktiviert werden (Stromsparen).

---

## Early Boot Spannungsprüfung

**Datei**: `variant.cpp` → `initVariant()`

Verhindert **Motorboating** (Boot-Crash-Reboot-Schleife) bei kritisch niedrigem Akkustand:

```
Boot → Wire.begin() → INA228::readVBATDirect()
         │
         ├── VBAT > 2800 mV → Normaler Boot
         │
         └── VBAT < 2800 mV → 3V3 OFF → LEDs OFF → NRF_POWER->SYSTEMOFF
                                                      (~1 µA Verbrauch)
```

- Nutzt `Ina228Driver::readVBATDirect()` (statische Methode, ohne vollständige Treiber-Initialisierung)
- Schwelle: 2800 mV (unterhalb aller Chemie-Gefahrenschwellen)
- System OFF Modus: Aufwachen durch USB-Verbindung (VBUS) oder RTC-Alarm

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
Flash: [========= ]  87.9% (716,672 / 815,104 bytes)
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

## Konfiguration über Admin-App

Die Konfiguration erfolgt über eine **eigene App** (nicht die Standard-Meshtastic-App), die Nachrichten auf **PortNum 256** (`PRIVATE_APP`) sendet.

### Protokoll

1. App sendet Text-Payload auf PortNum 256 an den Zielknoten
2. Modul parst den Befehl und führt ihn aus
3. Antwort wird als Text auf PortNum 256 zurückgesendet

### Beispiel (Python mit meshtastic-python)

```python
import meshtastic
from meshtastic.protobuf import portnums_pb2

interface = meshtastic.SerialInterface()

# Status abfragen
interface.sendData(
    b"get status",
    portNum=portnums_pb2.PortNum.PRIVATE_APP,
    destinationId="!aabbccdd"
)

# Batterie-Chemie setzen
interface.sendData(
    b"set bat lifepo4",
    portNum=portnums_pb2.PortNum.PRIVATE_APP,
    destinationId="!aabbccdd"
)
```

### Sicherheitshinweis

PortNum 256 ist **nicht verschlüsselt** auf der Meshtastic-Ebene (sofern kein PKI verwendet wird). Für produktive Deployments sollte die Konfiguration nur über direkte Verbindung (USB/BLE) erfolgen.

---

## Portierung von MeshCore

### Mapping MeshCore → Meshtastic

| MeshCore Konzept | Meshtastic Äquivalent |
|---|---|
| `Board::setup()` | `InheroMr2Module::setupDrivers()` |
| `Board::loop()` | `InheroMr2Module::runOnce()` |
| `BoardConfigContainer` | `InheroMr2Config` + LittleFS |
| `CLI (get/set board.X)` | Text-Protokoll auf PortNum 256 |
| `CayenneLPP Telemetrie` | Meshtastic `PowerMetrics` Protobuf |
| `MESH_DEBUG_PRINTLN` | `LOG_INFO` / `LOG_WARN` / `LOG_ERROR` |
| `SimplePreferences` | LittleFS (`/inhero/*.txt`) |
| `NRF52Board::deepSleep()` | `NRF_POWER->SYSTEMOFF` in `initVariant()` |

### Unterschiede

1. **Telemetrie-Format**: MeshCore nutzt CayenneLPP, Meshtastic nutzt Protobuf (`PowerMetrics`)
2. **Config-Speicherung**: MeshCore nutzt `Preferences` (ESP-NVS-ähnlich), Meshtastic-Port nutzt LittleFS
3. **Charger API**: MeshCore nutzt direkte Register-Schreibzugriffe, Meshtastic-Port nutzt `Adafruit_BQ25798` als Basisklasse mit `setChargeLimitV()` / `setChargeLimitA()` (Argumente in V/A statt mV/mA)
4. **Modul-System**: MeshCore hat ein eigenes Board-Klassen-System, Meshtastic nutzt `SinglePortModule` + `OSThread`
