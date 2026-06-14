/**
 * @file cmdInterface.hpp
 * @brief Header-Datei für das serielle Kommando-Interface des Messsystems.
 * * Diese Datei definiert die Zustandsmaschine, das Befehlsschema sowie die 
 * Datenstrukturen zur permanenten Speicherung von Kalibrierdaten (Offsets 
 * und Steigungen) im Flash-Speicher des Arduino Nano 33 BLE.
 * * @author Ezechiel Tonkeme
 * @date 2026
 */

#ifndef CMDINTERFACE_HPP_
#define CMDINTERFACE_HPP_

#include "stdint.h"
#include "Arduino.h"
#include <NanoBLEFlashPrefs.h>

/**
 * @brief Globales Objekt zum Zugriff auf die Flash-Parameter.
 * * Ermöglicht das Lesen und Schreiben von persistenten Daten über die 
 * NanoBLEFlashPrefs-Bibliothek.
 */
extern NanoBLEFlashPrefs myFlashPrefs;

/**
 * @struct flashStruct
 * @brief Struktur zur Speicherung der Sensor-Kalibrierparameter im Flash-Speicher.
 * * Enthält die Offsets (Nullpunktfehler) und Steigungen (Skalierungsfaktoren) 
 * für alle drei Achsen des Beschleunigungssensors, Magnetometers und Gyroskops.
 * @note Die Struktur darf eine maximale Größe von 1019 Words (4076 Bytes) nicht überschreiten.
 */
typedef struct flashStruct {
    /* --- Beschleunigungssensor (Accelerometer) --- */
    float a_x_ofs;  /**< Offset-Korrekturwert für die X-Achse */
    float a_y_ofs;  /**< Offset-Korrekturwert für die Y-Achse */
    float a_z_ofs;  /**< Offset-Korrekturwert für die Z-Achse */
    float a_x_s;    /**< Skalierungsfaktor (Steigung) für die X-Achse */
    float a_y_s;    /**< Skalierungsfaktor (Steigung) für die Y-Achse */
    float a_z_s;    /**< Skalierungsfaktor (Steigung) für die Z-Achse */

    /* --- Magnetometer --- */
    float m_x_ofs;  /**< Hard-Iron Offset-Korrektur für die X-Achse */
    float m_y_ofs;  /**< Hard-Iron Offset-Korrektur für die Y-Achse */
    float m_z_ofs;  /**< Hard-Iron Offset-Korrektur für die Z-Achse */
    float m_x_s;    /**< Soft-Iron Skalierungsfaktor für die X-Achse */
    float m_y_s;    /**< Soft-Iron Skalierungsfaktor für die Y-Achse */
    float m_z_s;    /**< Soft-Iron Skalierungsfaktor für die Z-Achse */

    /* --- Gyroskop --- */
    float g_x_ofs;  /**< Nullpunktdrift-Korrektur für die X-Achse */
    float g_y_ofs;  /**< Nullpunktdrift-Korrektur für die Y-Achse */
    float g_z_ofs;  /**< Nullpunktdrift-Korrektur für die Z-Achse */
    float g_x_s;    /**< Skalierungsfaktor für die X-Achse */
    float g_y_s;    /**< Skalierungsfaktor für die Y-Achse */
    float g_z_s;    /**< Skalierungsfaktor für die Z-Achse */
} flashPrefs;

/**
 * @brief Globale Instanz der Kalibrierparameter.
 * * Hält die aktuell im RAM geladenen und im System aktiven Offsets und Steigungen.
 */
extern flashPrefs prefs;

/**
 * @enum State
 * @brief Definiert die Betriebszustände der System-Zustandsmaschine (FSM).
 * * Die Zustandssteuerung erfolgt asynchron über das serielle Befehlsinterface.
 */
enum State {
    IDLE,        /**< Wartezustand: System passiv, wartet auf Kommandos. Setzen/Lesen von Parametern möglich. */
    SAMPLING_G,  /**< Kontinuierliche Ausgabe der Gyroskop-Messwerte in [dps] (Degrees per Second). */
    SAMPLING_A,  /**< Kontinuierliche Ausgabe der normierten Beschleunigungswerte in [g]. */
    SAMPLING_M   /**< Kontinuierliche Ausgabe der Magnetometer-Messwerte in [µT]. */
};

/**
 * @brief Aktueller Systemzustand der Zustandsmaschine.
 */
extern State state; 

/* --- Öffentliche Funktionsprototypen --- */

/**
 * @brief Überwacht und liest den seriellen Eingangspuffer aus.
 * * Diese Funktion muss zyklisch in der Hauptschleife (loop) aufgerufen werden. 
 * Sie puffert eingehende ASCII-Zeichen, bis ein vollständiger Befehl vorliegt.
 */
void handleSerialInput();

/**
 * @brief Interpretiert und führt das empfangene serielle Kommando aus.
 * * Analysiert das Befehlszeichen (z.B. 'A', 'G', 'M', 'S', 's', 'r', 'w') 
 * und stößt den entsprechenden Zustandswechsel oder Speicherzugriff an.
 */
void processCommand();

#endif /* CMDINTERFACE_HPP_ */