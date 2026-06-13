/**
 * @file BmImu.hpp
 * @brief Header-Datei für die BmImu-Klasse zur Steuerung der Bosch-Sensoren BMI270 und BMM150.
 * * Diese Bibliothek basiert auf der ARDUINO_MBI270_BMM150-Bibliothek und abstrahiert
 * die hardwarenahe $\text{I}^2\text{C}$-Kommunikation, die Registerkonfiguration und das 
 * Auslesen der physikalischen Messwerte (Beschleunigung, Drehrate und Magnetfeld).
 * * @copyright Copyright (c) 2019 Arduino SA. All rights reserved.
 * @license GNU Lesser General Public License v2.1
 * @author Ezechiel Tonkeme
 * @date 2026
 */

#include <Arduino.h>
#include <Wire.h>
#include "bmi270.h"
#include "bmm150.h"

/**
 * @enum CfgBoschSensor_t
 * @brief Konfigurationsmodi für die Sensorinitialisierung.
 * * Bestimmt, welche physikalischen Sensoren beim Systemstart aktiviert werden sollen.
 */
typedef enum {
    BOSCH_ACCELEROMETER_ONLY, /**< Nur die IMU (Beschleunigung/Gyroskop) aktivieren. */
    BOSCH_MAGNETOMETER_ONLY,  /**< Nur das Magnetometer aktivieren. */
    BOSCH_ACCEL_AND_MAGN      /**< Beide Sensoreinheiten (Kombibetrieb) aktivieren. */
} CfgBoschSensor_t;

/**
 * @struct dev_info
 * @brief Interne Struktur zur Zuordnung der $\text{I}^2\text{C}$-Businstanz und der Hardware-Adresse.
 */
struct dev_info {
    TwoWire* _wire;    /**< Zeiger auf die verwendete Wire ($\text{I}^2\text{C}$) Instanz. */
    uint8_t dev_addr;  /**< Die spezifische $\text{I}^2\text{C}$-Geräteadresse des Sensor-ICs. */
};

/**
 * @class BmImu
 * @brief Abstraktionsklasse für die Sensor-ICs Bosch BMI270 (IMU) und BMM150 (Magnetometer).
 * * Bietet eine standardisierte objektorientierte API zum Auslesen der 
 * kinematischen und magnetischen Messdaten auf dem Arduino Nano 33 BLE Sense.
 */
class BmImu {
  public:
    /**
     * @brief Konstruktor der BmImu-Klasse.
     * @param wire Referenz auf das zu verwendende TwoWire ($\text{I}^2\text{C}$) Objekt (Standard: Wire).
     */
    BmImu(TwoWire& wire = Wire);

    /**
     * @brief Destruktor der BmImu-Klasse.
     */
    ~BmImu() {}

    /**
     * @brief Versetzt die Sensoren in den kontinuierlichen Erfassungsmodus (Continuous Mode).
     */
    void setContinuousMode();

    /**
     * @brief Versetzt die Sensoren in den One-Shot-Modus (Messung nur auf explizite Anforderung).
     */
    void oneShotMode();

    /**
     * @brief Initialisiert die Hardware und konfiguriert die Sensor-ICs.
     * @param cfg Der gewünschte Betriebsmodus aus #CfgBoschSensor_t (Standard: Kombibetrieb).
     * @return int `0` bei erfolgreicher Initialisierung, negativer Fehlercode bei Fehlschlag.
     */
    int begin(CfgBoschSensor_t cfg = BOSCH_ACCEL_AND_MAGN);

    /**
     * @brief Beendet den Sensorbetrieb und gibt Ressourcen frei (Hinweis: Aktuell rein deklarativ).
     */
    void end();

    /**
     * @brief Aktiviert die Debug-Ausgabe auf der übergebenen Stream-Schnittstelle.
     * @param stream Referenz auf ein Stream-Objekt (z.B. Serial).
     */
    void debug(Stream& stream);
  
    #ifdef __MBED__
    /**
     * @brief Registriert eine Callback-Funktion für Hardware-Interrupts (nur Mbed OS).
     * @param callback Mbed-Callback ohne Rückgabewert.
     */
    void onInterrupt(mbed::Callback<void()> callback);

    /**
     * @brief Weist dem BMI270-Interrupt 1 einen hardwareseitigen Pinnamen zu.
     * @param irq_pin Mbed-spezifischer PinName.
     */
    void setInterruptPin(PinName irq_pin) {
      BMI270_INT1 = irq_pin;
    }

    /**
     * @brief Weist dem BMI270-Interrupt 1 einen Arduino-Digitalpin zu.
     * @param irq_pin Logische Arduino-Pinnummer.
     */
    void setInterruptPin(pin_size_t irq_pin) {
      BMI270_INT1 = digitalPinToPinName(irq_pin);
    }

    PinName BMI270_INT1 = NC; /**< Speicher für den zugewiesenen Interrupt-Pin. */
    #endif
  
    /* --- Beschleunigungssensor (Accelerometer) --- */

    /**
     * @brief Liest die aktuellen Beschleunigungswerte aus.
     * @param[out] x Referenz für den Ausgabewert der X-Achse in [g].
     * @param[out] y Referenz für den Ausgabewert der Y-Achse in [g].
     * @param[out] z Referenz für den Ausgabewert der Z-Achse in [g].
     * @return int `1` bei Erfolg, `0` bei Fehlern.
     */
    int readAcceleration(float& x, float& y, float& z);

    /**
     * @brief Prüft, ob neue Beschleunigungsdaten im FIFO/Register verfügbar sind.
     * @return int Maskierter Statuswert (`INT_STATUS` verknüpft mit `BMI2_ACC_DRDY_INT_MASK`).
     */
    int accelerationAvailable();

    /**
     * @brief Gibt die aktuell konfigurierte Abtastrate des Beschleunigungssensors zurück.
     * @return float Die Abtastrate (Sampling Rate) in Hz.
     */
    float accelerationSampleRate();

    /**
     * @brief Liest das spezifische Daten-Bereit-Statusbit des Beschleunigungssensors aus.
     * @return uint8_t Statusbit (0 oder 1).
     */
    uint8_t accelerationAvailableStatusbit();

    /* --- Gyroskop (Drehratensensor) --- */

    /**
     * @brief Liest die aktuellen Winkelgeschwindigkeiten aus.
     * @param[out] x Referenz für die Drehrate der X-Achse in [°/s] (dps).
     * @param[out] y Referenz für die Drehrate der Y-Achse in [°/s] (dps).
     * @param[out] z Referenz für die Drehrate der Z-Achse in [°/s] (dps).
     * @return int `1` bei Erfolg, `0` bei Fehlern.
     */
    int readGyroscope(float& x, float& y, float& z);

    /**
     * @brief Gibt die Anzahl der verfügbaren Gyroskop-Messwerte im FIFO-Puffer zurück.
     * @return int Anzahl der Samples.
     */
    int gyroscopeAvailable();

    /**
     * @brief Gibt die aktuell konfigurierte Abtastrate des Gyroskops zurück.
     * @return float Die Abtastrate in Hz.
     */
    float gyroscopeSampleRate();

    /**
     * @brief Liest das spezifische Daten-Bereit-Statusbit des Gyroskops aus.
     * @return uint8_t Statusbit (0 oder 1).
     */
    uint8_t gyroscopeAvailableStatusbit();
  
    /* --- Magnetometer --- */

    /**
     * @brief Liest die aktuelle magnetische Flussdichte aus.
     * @param[out] x Referenz für den Magnetfeldwert der X-Achse in [µT].
     * @param[out] y Referenz für den Magnetfeldwert der Y-Achse in [µT].
     * @param[out] z Referenz für den Magnetfeldwert der Z-Achse in [µT].
     * @return int `1` bei Erfolg, `0` bei Fehlern.
     */
    int readMagneticField(float& x, float& y, float& z);

    /**
     * @brief Gibt die Anzahl der verfügbaren Magnetometer-Messwerte im FIFO-Puffer zurück.
     * @return int Anzahl der Samples.
     */
    int magneticFieldAvailable();

    /**
     * @brief Gibt die aktuell konfigurierte Abtastrate des Magnetometers zurück.
     * @return float Die Abtastrate in Hz.
     */
    float magneticFieldSampleRate();

    /**
     * @brief Prüft das Daten-Bereit-Statusbit des Magnetometers.
     * @return true Neue Magnetometerdaten sind verfügbar.
     * @return false Keine neuen Daten verfügbar.
     */
    bool magneticFieldAvailableStatusbit();

  protected:
    /**
     * @brief Interne hardwarenahe Konfiguration des BMM150 Magnetometer-ICs.
     * @param dev Zeiger auf die Bosch-Treiberstruktur des Magnetometers.
     * @return int8_t Statuscode der Bosch-API.
     */
    int8_t configure_sensor(struct bmm150_dev *dev);

    /**
     * @brief Interne hardwarenahe Konfiguration des BMI270 IMU-ICs.
     * @param dev Zeiger auf die Bosch-Treiberstruktur der IMU.
     * @return int8_t Statuscode der Bosch-API.
     */
    int8_t configure_sensor(struct bmi2_dev *dev);

  private:
    /**
     * @brief Statische Wrapper-Funktion für lesende $\text{I}^2\text{C}$-Zugriffe (erforderlich für den Bosch-C-Treiber).
     * @param reg_addr Zu lesende Registeradresse.
     * @param[out] reg_data Zeiger auf den Zielpuffer für die Daten.
     * @param len Anzahl der zu lesenden Bytes.
     * @param intf_ptr Schnittstellen-Zeiger (enthält die #dev_info).
     * @return int8_t `0` bei Erfolg, ungleich `0` bei Fehlern.
     */
    static int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

    /**
     * @brief Statische Wrapper-Funktion für schreibende $\text{I}^2\text{C}$-Zugriffe (erforderlich für den Bosch-C-Treiber).
     * @param reg_addr Zu beschreibende Registeradresse.
     * @param reg_data Zeiger auf die zu schreibenden Datenbezeichner.
     * @param len Anzahl der zu schreibenden Bytes.
     * @param intf_ptr Schnittstellen-Zeiger (enthält die #dev_info).
     * @return int8_t `0` bei Erfolg, ungleich `0` bei Fehlern.
     */
    static int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

    /**
     * @brief Statische Verzögerungsfunktion (Delay) in Mikrosekunden für die Bosch-Treiberarchitektur.
     * @param period Verzögerungszeit in Mikrosekunden.
     * @param intf_ptr Optionaler Schnittstellen-Zeiger.
     */
    static void bmi2_delay_us(uint32_t period, void *intf_ptr);

    /**
     * @brief Interner zentraler Interrupt-Handler zur Verarbeitung von Datenereignissen.
     */
    void interrupt_handler();

    /**
     * @brief Hilfsfunktion zur textbasierten Formatierung und Ausgabe von Bosch-API-Fehlercodes.
     * @param rslt Der auszuwertende API-Statuscode.
     */
    void print_rslt(int8_t rslt);

  private:
    TwoWire* _wire;             /**< Interner Zeiger auf den $\text{I}^2\text{C}$-Bus. */
    Stream* _debug = nullptr;   /**< Zeiger auf den optionalen Debug-Stream. */
    
    #ifdef __MBED__
    mbed::Callback<void(void)> _cb; /**< Interner Speicher für den registrierten Mbed-Callback. */
    #endif

    bool _initialized = false;  /**< Flag zur Zustandsspeicherung, ob das Gesamtsystem erfolgreich initialisiert wurde. */
    int _interrupts = 0;        /**< Interner Zähler für registrierte Hardware-Interrupts. */
    
    struct dev_info accel_gyro_dev_info; /**< Verbindungsinformationen für das BMI270-Submodul. */
    struct dev_info mag_dev_info;        /**< Verbindungsinformationen für das BMM150-Submodul. */
    
    struct bmi2_dev bmi2;       /**< Native Bosch-API Treiberstruktur für den BMI270. */
    struct bmm150_dev bmm1;     /**< Native Bosch-API Treiberstruktur für den BMM150. */
    
    uint16_t _int_status;       /**< Zwischenspeicher für das ausgelesene Interrupt-Statusregister. */
    uint8_t _status;            /**< Allgemeiner interner Status-Zwischenspeicher. */
    bool continuousMode;        /**< Flag zur Speicherung, ob der Continuous Mode aktiv ist. */
};