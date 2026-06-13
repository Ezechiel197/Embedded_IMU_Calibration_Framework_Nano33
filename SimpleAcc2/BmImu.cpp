/**
 * @file BmImu.cpp
 * @brief Implementierung der BmImu-Klasse zur Abstraktion der Bosch BMI270 und BMM150 Treiber.
 * * Enthält die Initialisierungsroutinen, das FIFO- und Energiemanagement, 
 * die hardwarenahen $\text{I}^2\text{C}$-Wrapper, hardwareabhängige Achsen-Transformationen
 * sowie die Fehlerbehandlung inklusive einer optischen Panic-LED-Schleife.
 * * @copyright Copyright (c) 2019 Arduino SA. All rights reserved.
 * @license GNU Lesser General Public License v2.1
 * @author Ezechiel Tonkeme
 * @date 2026
 */

#include "BmImu.h"
#ifdef __MBED__
#include "mbed_events.h"
#include "mbed_shared_queues.h"
#include "drivers/InterruptIn.h"

/** * @brief Interne Mbed OS EventQueue zur asynchronen Ausführung von Interrupt-Callbacks. 
 * Entlastet die ISR (Interrupt Service Routine).
 */
static events::EventQueue queue(10 * EVENTS_EVENT_SIZE);
#endif
  
/**
 * @brief Konstruktor: Initialisiert die Klassenvariablen und setzt plattformspezifische Pins.
 * @param wire Referenz auf die zu nutzende TwoWire-Instanz ($\text{I}^2\text{C}$).
 */
BmImu::BmImu(TwoWire& wire)
{
  _wire = &wire;
  #ifdef TARGET_ARDUINO_NANO33BLE
  BMI270_INT1 = p11; /**< Standard-Hardware-Interrupt-Pin für den Nano 33 BLE */
  #endif
}  

/**
 * @brief Schaltet die Sensoren in den kontinuierlichen FIFO-Erfassungsmodus (Continuous Mode).
 * * Aktiviert den internen FIFO-Speicher des BMI270 für Beschleunigungs- und Gyroskopdaten.
 */
void BmImu::setContinuousMode() 
{
  bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN, 1, &bmi2);
  continuousMode = true;
}

/**
 * @brief Schaltet die Sensoren in den One-Shot-Modus (Streaming deaktiviert).
 * * Deaktiviert den internen FIFO-Speicher des BMI270. Daten müssen explizit gepollt werden.
 */
void BmImu::oneShotMode() 
{
  bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN, 0, &bmi2);
  continuousMode = false;
}

/**
 * @brief Initialisiert den $\text{I}^2\text{C}$-Bus sowie die Bosch Treiberstrukturen für BMI270 und BMM150.
 * * Konfiguriert die Funktionspointer für Lese-, Schreib- und Delayoperationen, lädt 
 * die Sensor-Standardkonfigurationen, setzt die Taktfrequenz des $\text{I}^2\text{C}$-Busses auf 400 kHz 
 * und überprüft die erfolgreiche Kommunikation mit den ICs.
 * * @param cfg Gewünschte Sensoraktivierung aus #CfgBoschSensor_t.
 * @return int `1` bei erfolgreicher Initialisierung aller gewählten Sensoren, `0` bei Fehlern.
 */
int BmImu::begin(CfgBoschSensor_t cfg) 
{
  _wire->begin();

  /* --- BMI270 Struktur-Setup --- */
  bmi2.chip_id = BMI2_I2C_PRIM_ADDR;
  bmi2.read = bmi2_i2c_read;
  bmi2.write = bmi2_i2c_write;
  bmi2.delay_us = bmi2_delay_us;
  bmi2.intf = BMI2_I2C_INTF;
  bmi2.intf_ptr = &accel_gyro_dev_info;
  bmi2.read_write_len = 30;     // Hardware-Limitierung der Arduino Wire-Bibliothek (Buffer)
  bmi2.config_file_ptr = NULL;  // Nutzt das im C-Treiber integrierte Default-Konfigurationsarray

  accel_gyro_dev_info._wire = _wire;
  accel_gyro_dev_info.dev_addr = bmi2.chip_id;

  /* --- BMM150 Struktur-Setup --- */
  bmm1.chip_id = BMM150_DEFAULT_I2C_ADDRESS;
  bmm1.read = bmi2_i2c_read;
  bmm1.write = bmi2_i2c_write;
  bmm1.delay_us = bmi2_delay_us;
  bmm1.intf = BMM150_I2C_INTF;
  bmm1.intf_ptr = &mag_dev_info;

  mag_dev_info._wire = _wire;
  mag_dev_info.dev_addr = bmm1.chip_id;

  int8_t result = 0;

  /* --- Hardware-Initialisierung --- */
  if(cfg != BOSCH_MAGNETOMETER_ONLY) {
    result |= bmi270_init(&bmi2);
    print_rslt(result);

    result |= configure_sensor(&bmi2);
    print_rslt(result);
  }

  if(cfg != BOSCH_ACCELEROMETER_ONLY) {
    result |= bmm150_init(&bmm1);
    print_rslt(result);

    result = configure_sensor(&bmm1);
    print_rslt(result);
  }

  // Erhöhe den I2C-Bustakt auf Fast Mode (400 kHz) für schnellere Datenübertragung
  _wire->setClock(400000);

  _initialized = (result == 0);
  return _initialized;
}

/**
 * @brief Setzt den Stream-Zeiger für die Ausgabe von Debug- und Fehlermeldungen.
 * @param stream Referenz auf ein gültiges Stream-Objekt (z. B. Serial).
 */
void BmImu::debug(Stream& stream)
{
  _debug = &stream;
}

#ifdef __MBED__
/**
 * @brief Konfiguriert und startet die asynchrone Interrupt-Verarbeitung (nur Mbed OS).
 * * Erstellt einen hochpriorisierten Thread, bindet den Hardware-Pin per InterruptIn an 
 * die steigende Flanke (Rise) des Sensor-Pins und delegiert den Callback an die Event-Queue.
 * * @param cb Benutzerdefinierter Callback, der bei einem Sensor-Event gefeuert wird.
 */
void BmImu::onInterrupt(mbed::Callback<void(void)> cb)
{
  if (BMI270_INT1 == NC) {
    return;
  }
  static mbed::InterruptIn irq(BMI270_INT1, PullDown);
  static rtos::Thread event_t(osPriorityHigh, 768, nullptr, "events");
  _cb = cb;
  event_t.start(callback(&queue, &events::EventQueue::dispatch_forever));
  irq.rise(mbed::callback(this, &BmImu::interrupt_handler));
}
#endif

/** * @brief Konversionsfaktor für den Beschleunigungssensor.
 * * Da der Standardmessbereich auf $\pm 4\,\text{g}$ eingestellt ist, verteilt sich der 
 * Wertebereich eines `int16_t` ($2^{15} = 32768$) auf $4\,\text{g}$. 
 * Daraus folgt: $32768 / 4 = 8192\,\text{LSB/g}$.
 */
#define INT16_to_G   (8192.0f)

/**
 * @brief Holt die aktuellen Beschleunigungswerte und rechnet sie in die physikalische Einheit [g] um.
 * @note Auf dem **Arduino Nano 33 BLE** ist der Sensor mechanisch gedreht verbaut. 
 * Hier findet ein automatisches Remapping der Achsen statt ($X_{\text{out}} = -Y_{\text{raw}}$, $Y_{\text{out}} = -X_{\text{raw}}$), 
 * um die Standard-Rechtssystem-Konvention der Platine zu wahren.
 * * @param[out] x Beschleunigung der X-Achse in [g].
 * @param[out] y Beschleunigung der Y-Achse in [g].
 * @param[out] z Beschleunigung der Z-Achse in [g].
 * @return int `1` bei erfolgreichem Lesen aus der Bosch-API, `0` bei Fehler.
 */
int BmImu::readAcceleration(float& x, float& y, float& z) 
{
  struct bmi2_sens_data sensor_data;
  auto ret = bmi2_get_sensor_data(&sensor_data, &bmi2);
  #ifdef TARGET_ARDUINO_NANO33BLE
  x = -sensor_data.acc.y / INT16_to_G;
  y = -sensor_data.acc.x / INT16_to_G;
  #else
  x = sensor_data.acc.x / INT16_to_G;
  y = sensor_data.acc.y / INT16_to_G;
  #endif
  z = sensor_data.acc.z / INT16_to_G;
  return (ret == 0);
}

/**
 * @brief Überprüft den Hardware-Interrupt-Status auf neue Beschleunigungsdaten (Data Ready).
 * * Liest das Interrupt-Statusregister, maskiert das `BMI2_ACC_DRDY_INT_MASK`-Bit und 
 * löscht das Bit anschließend im internen Zustandscache, um Mehrfachauswertungen zu verhindern.
 * * @return int Maskierter Statuswert ungleich `0`, wenn neue Daten bereitstehen.
 */
int BmImu::accelerationAvailable() 
{
  uint16_t status;
  bmi2_get_int_status(&status, &bmi2);
  int ret = ((status | _int_status) & BMI2_ACC_DRDY_INT_MASK);
  _int_status = status;
  _int_status &= ~BMI2_ACC_DRDY_INT_MASK;
  return ret;
}

/**
 * @brief Berechnet die aktuelle Abtastrate (ODR) des Beschleunigungssensors.
 * * Liest die ODR-Registerkonfiguration aus und berechnet die Frequenz in Hz über die 
 * herstellerspezifische Formel der Bosch-API.
 * * @return float Die konfigurierte Abtastrate in Hz.
 */
float BmImu::accelerationSampleRate() 
{
  struct bmi2_sens_config sens_cfg;
  sens_cfg.type = BMI2_ACCEL;
  bmi2_get_sensor_config(&sens_cfg, 1, &bmi2);
  return (1 << sens_cfg.cfg.acc.odr) * 0.39;
}

/**
 * @brief Abfrage des globalen Daten-Bereit-Statusbits für die Beschleunigungskomponente.
 * @return uint8_t `1` wenn Daten verfügbar, andernfalls `0`.
 */
uint8_t BmImu::accelerationAvailableStatusbit()
{
  uint8_t status;
  bmi2_get_status(&status, &bmi2);
  uint8_t ret = ((status | _status) & BMI2_DRDY_ACC);  
  _status = status;
  _status &= ~BMI2_DRDY_ACC;
  return ret;
}

/** * @brief Konversionsfaktor für das Gyroskop.
 * * Der Messbereich ist auf $\pm 2000\,\text{dps}$ eingestellt. Die $16\text{-Bit}$-Auflösung ($2^{15} = 32768$) 
 * konvertiert sich somit über $32768 / 2000 = 16.384\,\text{LSB/(°/s)}$.
 */
#define INT16_to_DPS   (16.384f)

/**
 * @brief Holt die aktuellen Winkelgeschwindigkeiten und rechnet diese in [°/s] (dps) um.
 * @note Beachtet analog zur Beschleunigung das plattformspezifische Achsen-Remapping des Nano 33 BLE.
 * @param[out] x Drehrate der X-Achse in [°/s].
 * @param[out] y Drehrate der Y-Achse in [°/s].
 * @param[out] z Drehrate der Z-Achse in [°/s].
 * @return int `1` bei Erfolg, `0` bei Fehlern.
 */
int BmImu::readGyroscope(float& x, float& y, float& z) {
  struct bmi2_sens_data sensor_data;
  auto ret = bmi2_get_sensor_data(&sensor_data, &bmi2);
  #ifdef TARGET_ARDUINO_NANO33BLE
  x = -sensor_data.gyr.y / INT16_to_DPS;
  y = -sensor_data.gyr.x / INT16_to_DPS;
  #else
  x = sensor_data.gyr.x / INT16_to_DPS;
  y = sensor_data.gyr.y / INT16_to_DPS;
  #endif
  z = sensor_data.gyr.z / INT16_to_DPS;
  return (ret == 0);
}

/**
 * @brief Überprüft den Hardware-Interrupt-Status auf neue Gyroskopedaten.
 * @return int Maskierter Statuswert ungleich `0`, wenn neue Daten bereitstehen.
 */
int BmImu::gyroscopeAvailable() {
  uint16_t status;
  bmi2_get_int_status(&status, &bmi2);
  int ret = ((status | _int_status) & BMI2_GYR_DRDY_INT_MASK);
  _int_status = status;
  _int_status &= ~BMI2_GYR_DRDY_INT_MASK;
  return ret;
}

/**
 * @brief Abfrage des globalen Daten-Bereit-Statusbits für das Gyroskop.
 * @return uint8_t `1` wenn Daten verfügbar, andernfalls `0`.
 */
uint8_t BmImu::gyroscopeAvailableStatusbit()
{
  uint8_t status;
  bmi2_get_status(&status, &bmi2);
  uint8_t ret = ((status | _status) & BMI2_DRDY_GYR);  
  _status = status;
  _status &= ~BMI2_DRDY_GYR;
  return ret;
}

/**
 * @brief Berechnet die aktuelle Abtastrate (ODR) des Gyroskops in Hz.
 * @return float Die ODR in Hz.
 */
float BmImu::gyroscopeSampleRate() {
  struct bmi2_sens_config sens_cfg;
  sens_cfg.type = BMI2_GYRO;
  bmi2_get_sensor_config(&sens_cfg, 1, &bmi2);
  return (1 << sens_cfg.cfg.gyr.odr) * 0.39;
}

/**
 * @brief Liest die aktuellen Magnetfeldstärken des BMM150 aus.
 * * Die Werte werden vom nativen Bosch-Treiber direkt in Mikrotesla [µT] übergeben.
 * * @param[out] x Magnetische Flussdichte der X-Achse in [µT].
 * @param[out] y Magnetische Flussdichte der Y-Achse in [µT].
 * @param[out] z Magnetische Flussdichte der Z-Achse in [µT].
 * @return int `1` bei erfolgreicher Erfassung (`BMM150_OK`), andernfalls `0`.
 */
int BmImu::readMagneticField(float& x, float& y, float& z) 
{
  struct bmm150_mag_data mag_data;
  int const rc = bmm150_read_mag_data(&mag_data, &bmm1);
  x = mag_data.x;
  y = mag_data.y;
  z = mag_data.z;

  if (rc == BMM150_OK)
    return 1;
  else
    return 0;
}

/**
 * @brief Pollt das dedizierte Statusregister des BMM150, um neue Daten anzuzeigen.
 * * Liest das Register `BMM150_REG_DATA_READY_STATUS` (0x48) direkt aus und isoliert 
 * das Data-Ready-Bit über Makros.
 * * @return true Neue Magnetometerdaten liegen im Register vor.
 * @return false Keine neuen Daten verfügbar oder I2C-Fehler.
 */
bool BmImu::magneticFieldAvailableStatusbit()
{
  int8_t rslt;
  uint8_t data_ready_status;
  
  rslt = bmm150_get_regs(BMM150_REG_DATA_READY_STATUS, &data_ready_status, 1, &bmm1);
  if (rslt == BMM150_OK)
  {
    data_ready_status = BMM150_GET_BITS_POS_0(data_ready_status, BMM150_DRDY_STATUS); 
  }
  return (data_ready_status != 0);
}

/**
 * @brief Überprüft, ob ein Data-Ready-Interrupt vom Magnetometer erzeugt wurde.
 * @return int Ungleich `0`, falls der DRDY-Interrupt aktiv war.
 */
int BmImu::magneticFieldAvailable() 
{
  bmm150_get_interrupt_status(&bmm1);
  return bmm1.int_status & BMM150_INT_ASSERTED_DRDY;
}

/**
 * @brief Gibt die konfigurierte Abtastrate des Magnetometers aus.
 * * Übersetzt das interne Einstellungsregister des BMM150 in diskrete Hz-Werte (2, 6, 8, 10, 15, 20, 25, 30 Hz).
 * * @return float Die Datenrate in Hz.
 */
float BmImu::magneticFieldSampleRate() 
{
  struct bmm150_settings settings;
  bmm150_get_sensor_settings(&settings, &bmm1);
  switch (settings.data_rate) {
    case BMM150_DATA_RATE_10HZ: return 10.0f;
    case BMM150_DATA_RATE_02HZ: return 2.0f;
    case BMM150_DATA_RATE_06HZ: return 6.0f;
    case BMM150_DATA_RATE_08HZ: return 8.0f;
    case BMM150_DATA_RATE_15HZ: return 15.0f;
    case BMM150_DATA_RATE_20HZ: return 20.0f;
    case BMM150_DATA_RATE_25HZ: return 25.0f;
    case BMM150_DATA_RATE_30HZ: return 30.0f;
  }
  return 0.0f;
}

/**
 * @brief Interne Konfigurationsfunktion für das BMI270 IMU-IC.
 * * Richtet den Interrupt-Pin INT1 (Active High, Push-Pull, Output enabled) ein, 
 * setzt die Abtastrate für Beschleunigung und Gyroskop auf 50 Hz, wählt die Arbeitsbereiche 
 * ($\pm 4\,\text{g}$ und $\pm 2000\,\text{dps}$) und deaktiviert den Advanced-Power-Save-Modus 
 * für stabileres Signalverhalten während transienter Messungen.
 * * @param dev Zeiger auf die Bosch Treiberstruktur.
 * @return int8_t Statuscode der Operation (`BMI2_OK` bei Erfolg).
 */
int8_t BmImu::configure_sensor(struct bmi2_dev *dev)
{
  int8_t rslt;
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

  struct bmi2_int_pin_config int_pin_cfg;
  int_pin_cfg.pin_type = BMI2_INT1;
  int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
  int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
  int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
  int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
  int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

  struct bmi2_sens_config sens_cfg[2];
  sens_cfg[0].type = BMI2_ACCEL;
  sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
  sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_50HZ;
  sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
  
  sens_cfg[1].type = BMI2_GYRO;
  sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
  sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
  sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

  rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
  if (rslt != BMI2_OK) return rslt;

  rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
  if (rslt != BMI2_OK) return rslt;

  /* Deaktivierung des Advanced Power Save Modes für deterministisches Timing */
  rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev);
  if (rslt != BMI2_OK) return rslt;

  rslt = bmi2_sensor_enable(sens_list, 2, dev);
  return rslt;
}

/**
 * @brief Interne Konfigurationsfunktion für das BMM150 Magnetometer-IC.
 * * Setzt den Power-Modus auf Normalbetrieb, konfiguriert das vordefinierte Messprofil auf 
 * 'Enhanced' (Erhöhte Präzision durch erweitertes Oversampling) und schaltet den 
 * hardwareseitigen Data-Ready-Pin (DRDY) frei.
 * * @param dev Zeiger auf die Bosch Magnetometer Treiberstruktur.
 * @return int8_t Statuscode der Operation (`BMM150_OK` bei Erfolg).
 */
int8_t BmImu::configure_sensor(struct bmm150_dev *dev)
{
    int8_t rslt;
    struct bmm150_settings settings;

    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, dev);

    if (rslt == BMM150_OK)
    {
        settings.preset_mode = BMM150_PRESETMODE_ENHANCED; 
        rslt = bmm150_set_presetmode(&settings, dev);

        if (rslt == BMM150_OK)
        {
            /* Data-Ready Interrupt auf Hardware-Pin mappen */
            settings.int_settings.drdy_pin_en = 0x01;
            settings.int_settings.int_pin_en = 0x00;
            rslt = bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN, &settings, dev); 
        }
    }
    return rslt;
}

/**
 * @brief Statische I2C-Lesefunktion für die hardwareunabhängige Bosch C-Treiberarchitektur.
 * * Setzt die Registeradresse ab, stößt die I2C-Übertragung an, liest die Daten byteweise aus 
 * und füllt den übergebenen Datenpuffer. Beinhaltet einen Schutz vor Pufferüberläufen (> 32 Bytes).
 * * @param reg_addr Zu lesende Hardware-Registeradresse.
 * @param[out] reg_data Pointer auf den Ziel-Datenpuffer im RAM.
 * @param len Anzahl der anzufordernden Bytes.
 * @param intf_ptr Schnittstellen-Zeiger (wird auf den internen #dev_info-Typ gecastet).
 * @return int8_t `0` bei Erfolg, `-1` bei Kommunikationsfehlern.
 */
int8_t BmImu::bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  if ((reg_data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }
  uint8_t bytes_received;

  struct dev_info* dev_info = (struct dev_info*)intf_ptr;
  uint8_t dev_id = dev_info->dev_addr;

  dev_info->_wire->beginTransmission(dev_id);
  dev_info->_wire->write(reg_addr);
  if (dev_info->_wire->endTransmission() == 0) {
    bytes_received = dev_info->_wire->requestFrom(dev_id, len);
    for (uint16_t i = 0; i < bytes_received; i++)
    {
      reg_data[i] = dev_info->_wire->read();
    }
  } else {
    return -1;
  }
  return 0;
}

/**
 * @brief Statische I2C-Schreibfunktion für die hardwareunabhängige Bosch C-Treiberarchitektur.
 * * Überträgt die Zielregisteradresse gefolgt von den zu schreibenden Datenkonfigurationen über den I2C-Bus.
 * * @param reg_addr Zu beschreibende Hardware-Registeradresse.
 * @param reg_data Pointer auf das zu schreibende Datenarray im RAM.
 * @param len Anzahl der zu schreibenden Bytes.
 * @param intf_ptr Schnittstellen-Zeiger (wird auf den internen #dev_info-Typ gecastet).
 * @return int8_t `0` bei Erfolg, `-1` bei Kommunikationsfehlern oder fehlendem ACK.
 */
int8_t BmImu::bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  if ((reg_data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }

  struct dev_info* dev_info = (struct dev_info*)intf_ptr;
  uint8_t dev_id = dev_info->dev_addr;
  dev_info->_wire->beginTransmission(dev_id);
  dev_info->_wire->write(reg_addr);
  for (uint16_t i = 0; i < len; i++)
  {
    dev_info->_wire->write(reg_data[i]);
  }
  if (dev_info->_wire->endTransmission() != 0) {
    return -1;
  }
  return 0;
}

/**
 * @brief Statischer Verzögerungs-Wrapper (Delay) für den Bosch C-Treiber.
 * @param period Verzögerungszeit in Mikrosekunden ($\mu\text{s}$).
 * @param intf_ptr Optionaler Schnittstellen-Zeiger (unbenutzt).
 */
void BmImu::bmi2_delay_us(uint32_t period, void *intf_ptr)
{
  delayMicroseconds(period);
}

#ifdef __MBED__
/**
 * @brief Interner Interrupt-Handler (ISR-Kontext).
 * * Reiht den registrierten Benutzer-Callback bei erfolgreicher Initialisierung 
 * in die asynchrone Mbed Event-Queue ein, um die ISR kurz zu halten.
 */
void BmImu::interrupt_handler()
{
  if (_initialized && _cb) {
    queue.call(_cb);
  }
}
#endif

/**
 * @brief Lokale private Hilfsfunktion: Fängt kritische Hardwarefehler ab (Endlosschleife).
 * * Initialisiert die Board-eigene LED und versetzt diese in eine schnelle, optische 
 * Blinkfrequenz ($5\,\text{Hz}$), um einen Systemabsturz oder Verbindungsverlust (Panic-State) anzuzeigen.
 */
static void panic_led_trap(void)
{
#if !defined(LED_BUILTIN)
  static int const LED_BUILTIN = 2;
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  while (1)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
}

/**
 * @brief Interpretiert Bosch-API Return-Codes und gibt Fehlermeldungen im Klartext aus.
 * * Evaluiert den Rückgabewert (`rslt`) der Sensor-Klassenfunktionen. Bei kritischen 
 * Fehlern (z.B. `BMI2_E_DEV_NOT_FOUND` oder `BMI2_E_COM_FAIL`) wird eine detaillierte Fehlermeldung 
 * auf den konfigurierten Debug-Stream gedruckt und das System über `panic_led_trap()` dauerhaft blockiert.
 * * @param rslt Der zu interpretierende Fehler- oder Statuscode der Bosch-Treiberbibliothek.
 */
void BmImu::print_rslt(int8_t rslt)
{
  if (!_debug) {
    return;
  }
  switch (rslt)
  {
    case BMI2_OK: return; /* Operation erfolgreich: Frühzeitiger Rücksprung */ break;
    case BMI2_E_NULL_PTR:
      _debug->println("Error [" + String(rslt) + "] : Null pointer");
      panic_led_trap();
      break;
    case BMI2_E_COM_FAIL:
      _debug->println("Error [" + String(rslt) + "] : Communication failure");
      panic_led_trap();
      break;
    case BMI2_E_DEV_NOT_FOUND:
      _debug->println("Error [" + String(rslt) + "] : Device not found");
      panic_led_trap();
      break;
    case BMI2_E_OUT_OF_RANGE:
      _debug->println("Error [" + String(rslt) + "] : Out of range");
      panic_led_trap();
      break;
    case BMI2_E_ACC_INVALID_CFG:
      _debug->println("Error [" + String(rslt) + "] : Invalid accel configuration");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_INVALID_CFG:
      _debug->println("Error [" + String(rslt) + "] : Invalid gyro configuration");
      panic_led_trap();
      break;
    case BMI2_E_ACC_GYR_INVALID_CFG:
      _debug->println("Error [" + String(rslt) + "] : Invalid accel/gyro configuration");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_SENSOR:
      _debug->println("Error [" + String(rslt) + "] : Invalid sensor");
      panic_led_trap();
      break;
    case BMI2_E_CONFIG_LOAD:
      _debug->println("Error [" + String(rslt) + "] : Configuration loading error");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_PAGE:
      _debug->println("Error [" + String(rslt) + "] : Invalid page ");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_FEAT_BIT:
      _debug->println("Error [" + String(rslt) + "] : Invalid feature bit");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_INT_PIN:
      _debug->println("Error [" + String(rslt) + "] : Invalid interrupt pin");
      panic_led_trap();
      break;
    case BMI2_E_SET_APS_FAIL:
      _debug->println("Error [" + String(rslt) + "] : Setting advanced power mode failed");
      panic_led_trap();
      break;
    case BMI2_E_AUX_INVALID_CFG:
      _debug->println("Error [" + String(rslt) + "] : Invalid auxiliary configuration");
      panic_led_trap();
      break;
    case BMI2_E_AUX_BUSY:
      _debug->println("Error [" + String(rslt) + "] : Auxiliary busy");
      panic_led_trap();
      break;
    case BMI2_E_SELF_TEST_FAIL:
      _debug->println("Error [" + String(rslt) + "] : Self test failed");
      panic_led_trap();
      break;
    case BMI2_E_REMAP_ERROR:
      _debug->println("Error [" + String(rslt) + "] : Remapping error");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_USER_GAIN_UPD_FAIL:
      _debug->println("Error [" + String(rslt) + "] : Gyro user gain update failed");
      panic_led_trap();
      break;
    case BMI2_E_SELF_TEST_NOT_DONE:
      _debug->println("Error [" + String(rslt) + "] : Self test not done");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_INPUT:
      _debug->println("Error [" + String(rslt) + "] : Invalid input");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_STATUS:
      _debug->println("Error [" + String(rslt) + "] : Invalid status");
      panic_led_trap();
      break;
    case BMI2_E_CRT_ERROR:
      _debug->println("Error [" + String(rslt) + "] : CRT error");
      panic_led_trap();
      break;
    case BMI2_E_ST_ALREADY_RUNNING:
      _debug->println("Error [" + String(rslt) + "] : Self test already running");
      panic_led_trap();
      break;
    case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
      _debug->println("Error [" + String(rslt) + "] : CRT ready for DL fail abort");
      panic_led_trap();
      break;
    case BMI2_E_DL_ERROR:
      _debug->println("Error [" + String(rslt) + "] : DL error");
      panic_led_trap();
      break;
    case BMI2_E_PRECON_ERROR:
      _debug->println("Error [" + String(rslt) + "] : PRECON error");
      panic_led_trap();
      break;
    case BMI2_E_ABORT_ERROR:
      _debug->println("Error [" + String(rslt) + "] : Abort error");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_SELF_TEST_ERROR:
      _debug->println("Error [" + String(rslt) + "] : Gyro self test error");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_SELF_TEST_TIMEOUT:
      _debug->println("Error [" + String(rslt) + "] : Gyro self test timeout");
      panic_led_trap();
      break;
    case BMI2_E_WRITE_CYCLE_ONGOING:
      _debug->println("Error [" + String(rslt) + "] : Write cycle ongoing");
      panic_led_trap();
      break;
    case BMI2_E_WRITE_CYCLE_TIMEOUT:
      _debug->println("Error [" + String(rslt) + "] : Write cycle timeout");
      panic_led_trap();
      break;
    case BMI2_E_ST_NOT_RUNING:
      _debug->println("Error [" + String(rslt) + "] : Self test not running");
      panic_led_trap();
      break;
    case BMI2_E_DATA_RDY_INT_FAILED:
      _debug->println("Error [" + String(rslt) + "] : Data ready interrupt failed");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_FOC_POSITION:
      _debug->println("Error [" + String(rslt) + "] : Invalid FOC position");
      panic_led_trap();
      break;
    default:
      _debug->println("Error [" + String(rslt) + "] : Unknown error code");
      panic_led_trap();
      break;
  }
}