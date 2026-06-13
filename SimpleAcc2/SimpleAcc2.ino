/**
 * @file main.cpp
 * @brief Hauptprogramm zur sensorischen Datenerfassung und Achsenkalibrierung.
 * * Steuert die Datenakquisition der IMU- und Magnetometer-Hardware (BmImu) 
 * über eine nicht-blockierende Befehlsschnittstelle (cmdInterface). Die Rohdaten 
 * werden zur Laufzeit mittels im Flash hinterlegter sensorischer Kalibrierungsdaten 
 * (Offsets und Skalierungsfaktoren) bereinigt, reorientiert und formatiert ausgegeben.
 * * @copyright Copyright (c) 2019 Arduino SA. All rights reserved.
 * @license GNU Lesser General Public License v2.1
 * @author Ezechiel Tonkeme
 * @date 2026
 */

//#include "Arduino_BMI270_BMM150.h"
#include "BmImu.h"
#include "cmdInterface.h"

/**
 * @brief Instanziierung des BmImu-Sensorobjekts auf dem sekundären I2C-Bus.
 * * Verwendet `Wire1`, was typischerweise für intern verbaute Sensoren auf 
 * fortgeschrittenen Arduino-Boards (wie dem Nano 33 BLE Sense) genutzt wird.
 */
BmImu myIMU(Wire1);

/**
 * @brief Initialisierungsroutine der Arduino-Applikation.
 * * Startet die serielle USB-Kommunikation, wartet auf die Verbindung des Host-PCs,
 * initialisiert die Sensor-ICs über das #myIMU-Objekt, liest die Kalibrierungsparameter
 * aus dem Flash-Speicher und gibt die konfigurierte Abtastrate des Beschleunigungssensors aus.
 */
void setup() {
  Serial.begin(9600);
  while (!Serial); /**< Blockiert, bis der serielle Monitor geöffnet wird */
  Serial.println("Started");

  if (!myIMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1); /**< System-Halt bei kritischem Hardwarefehler */
  }

  /* Laden der sensorbezogenen Abgleichwerte aus dem Flash-Speicher */
  int rc = myFlashPrefs.readPrefs(&prefs, sizeof(prefs));

  Serial.print("Accelerometer sample rate = ");
  Serial.print(myIMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
}

/* --- Globale Variablen für gefilterte/skalierte Sensordaten --- */
float m_x = 20.0f, m_y = 0.0f, m_z = 40.0f;  /**< Aktuelle Magnetfeldstärken in [µT] */
float g_x = 0.0f, g_y = 0.0f, g_z = 0.0f;    /**< Aktuelle Winkelgeschwindigkeiten in [°/s] */
float a_x = 0.0f, a_y = 0.0f, a_z = -1.0f;   /**< Aktuelle Beschleunigungswerte in [g] */

/**
 * @brief Erfasst und kalibriert die aktuellen Gyroskopdaten.
 * * Prüft das Data-Ready-Statusbit des Gyroskops. Falls neue Daten vorliegen, werden
 * diese eingelesen, die Z-Achse mathematisch invertiert und alle Achsen anhand der Formel
 * \f$ G_{\text{cal}} = \frac{G_{\text{raw}} - \text{Offset}}{\text{Skalierung}} \f$ bereinigt.
 * * @return true Neue Gyroskopdaten wurden erfolgreich erfasst und kalibriert.
 * @return false Keine neuen Daten verfügbar.
 */
bool get_gyro_data()
{
  bool result = myIMU.gyroscopeAvailableStatusbit();
  if(result)
  {
    myIMU.readGyroscope(g_x, g_y, g_z);
    
    /* Softwareseitige Anpassung der Rotationsrichtung */
    g_z = -g_z;
    
    /* Anwendung der Nullpunkt- und Empfindlichkeitskorrektur */
    g_x = (g_x - prefs.g_x_ofs) / prefs.g_x_s;
    g_y = (g_y - prefs.g_y_ofs) / prefs.g_y_s;
    g_z = (g_z - prefs.g_z_ofs) / prefs.g_z_s;
  }
  return result;
}

/**
 * @brief Erfasst und kalibriert die aktuellen Beschleunigungsdaten.
 * * Prüft das Data-Ready-Statusbit des Beschleunigungssensors. Falls Daten verfügbar sind,
 * werden diese ausgelesen. Die Z-Achse wird invertiert (Reorientierung zum Fahrzeug-/Gehäusesystem) 
 * und eine statische Linear-Kalibrierung (Offset und Gain) auf alle drei Koordinatenachsen angewendet.
 * * @return true Neue Beschleunigungsdaten wurden erfolgreich verarbeitet.
 * @return false Keine neuen Daten verfügbar.
 */
bool get_acc_data()
{
  bool result = myIMU.accelerationAvailableStatusbit();
  if(result)
  {
    myIMU.readAcceleration(a_x, a_y, a_z);
    
    /* Korrektur der Drehrichtungen und Achsenorientierungen */
    a_z = -a_z;
    
    /* Lineare Achsenkorrektur */
    a_x = (a_x - prefs.a_x_ofs) / prefs.a_x_s;
    a_y = (a_y - prefs.a_y_ofs) / prefs.a_y_s;
    a_z = (a_z - prefs.a_z_ofs) / prefs.a_z_s;
  }
  return result;
}

/**
 * @brief Erfasst und kalibriert die aktuellen Magnetometerdaten des BMM150.
 * * Ruft die Rohdaten des Magnetfeldes ab. Führt ein manuelles 2D-Achsen-Remapping durch
 * ($X_{\text{neu}} = Y_{\text{alt}}$, $Y_{\text{neu}} = -X_{\text{alt}}$), um den Sensor 
 * koaxial an das Koordinatensystem des BMI270 anzupassen. Anschließend erfolgt die Hard-Iron 
 * und Soft-Iron Fehlerkompensation über die Flash-Präferenzen.
 * * @warning Die Funktion führt die Datenverarbeitung und Achsentransformation unabhängig 
 * vom Rückgabewert des Statusbits aus.
 * * @return true Magnetometer-Statusbit signalisiert die erfolgreiche Bereitstellung.
 * @return false Keine neuen Daten gemeldet.
 */
bool get_mag_data()
{
  bool result = myIMU.magneticFieldAvailableStatusbit();
  {
    myIMU.readMagneticField(m_x, m_y, m_z);
    
    /* Koaxiale Ausrichtung des Magnetometers an das IMU-Koordinatensystem */
    float temp = m_x;
    m_x = m_y;
    m_y = -temp;
    
    /* Hard- und Soft-Iron Kalibrierungsabgleich */
    m_x = (m_x - prefs.m_x_ofs) / prefs.m_x_s;
    m_y = (m_y - prefs.m_y_ofs) / prefs.m_y_s;
    m_z = (m_z - prefs.m_z_ofs) / prefs.m_z_s;
  }
  return result;
}

/**
 * @brief Hauptprogrammschleife (Superloop).
 * * Verarbeitet fortlaufend und nicht-blockierend eingehende serielle Befehle über das 
 * Befehlsinterface. Je nach historisch gesetztem Systemzustand (`state`) pollt die Schleife
 * die jeweilige Sensorkomponente (Gyroskop, Beschleunigung oder Magnetfeld) und streamt die 
 * kalibrierten Daten tabulatorgetrennt mit einer Genauigkeit von 4 Nachkommastellen auf die USB-Schnittstelle.
 */
void loop() {
  float x, y, z; // Lokale Variablen (aktuell unbenutzt)

  /* Nicht-blockierende Abfrage und Verarbeitung der seriellen Steuerschnittstelle */
  handleSerialInput();   // Nur Empfang und Pufferung
  processCommand();      // Auswertung der Befehle und Zustandsänderung (state)

  /* Zustandsgesteuertes Sensordaten-Streaming */
  switch(state)
  {
    case SAMPLING_G:
      if(get_gyro_data())
      {
        Serial.print("G");
        Serial.print('\t');
        Serial.print(g_x, 4);
        Serial.print('\t');
        Serial.print(g_y, 4);
        Serial.print('\t');
        Serial.println(g_z, 4);
      }
      break;

    case SAMPLING_A:
      if(get_acc_data())
      {
        Serial.print("A");
        Serial.print('\t');
        Serial.print(a_x, 4);
        Serial.print('\t');
        Serial.print(a_y, 4);
        Serial.print('\t');
        Serial.println(a_z, 4);
      }
      break;

    case SAMPLING_M:
      if(get_mag_data())
      {
        Serial.print("M");
        Serial.print('\t');
        Serial.print(m_x, 4);
        Serial.print('\t');
        Serial.print(m_y, 4);
        Serial.print('\t');
        Serial.println(m_z, 4);
      }
      break; 
  }
}