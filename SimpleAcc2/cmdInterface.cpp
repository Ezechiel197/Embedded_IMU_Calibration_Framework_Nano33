/**
 * @file cmdInterface.cpp
 * @brief Implementierung des seriellen Kommando-Interfaces und der Parameterverwaltung.
 * * Dieses Modul verarbeitet die eingehenden seriellen ASCII-Befehle asynchron,
 * steuert die zentrale Zustandsmaschine (FSM) des Systems und bietet Funktionen
 * zum softwareseitigen Mapping sowie zur persistenten Speicherung der 
 * Sensor-Kalibrierdaten im Flash-Speicher.
 * * @author Ezechiel Tonkeme
 * @date 2026
 */

#include "cmdInterface.h"

/* ----- Globale Parameter (Public) ----------------------------------------*/

/**
 * @brief Instanz der Flash-Struktur zur Verwaltung der aktiven Kalibrierparameter.
 */
flashPrefs prefs;

/**
 * @brief Aktueller Zustand der System-Zustandsmaschine. Initialwert ist IDLE.
 */
State state = IDLE;

/**
 * @brief Globales Objekt zur Abstraktion der Flash-Speicher-Zugriffe.
 */
NanoBLEFlashPrefs myFlashPrefs;


/* ----- Private Datentypen -------------------------------------------------*/

/**
 * @enum ParamType
 * @brief Interne Klassifizierung der unterstützten Parametertypen.
 */
enum ParamType { 
    PARAM_FLOAT,   /**< Parameter ist eine 32-Bit-Fließkommazahl. */
    PARAM_BOOL,    /**< Parameter ist ein boolescher Zustand (Wahrheitswert). */
    PARAM_INVALID  /**< Ungültiger Typ / Index außerhalb des erlaubten Bereichs. */
};


/* ----- Private Konstanten ------------------------------------------------*/

/** @brief Maximale Kapazität des seriellen Empfangspuffers in Bytes. */
const byte CMD_BUF_LEN = 32;

/** @brief Anzahl der registrierten Fließkomma-Parameter (Indizes 0 bis 17). */
const uint8_t NUM_FLOAT_PARAMS = 18;

/** @brief Anzahl der registrierten Bool-Parameter (Indizes 100 bis 115). */
const uint8_t NUM_BOOL_PARAMS  = 16;


/* ----- Private Parameter (statisch / modulglobal) -----------------------*/

/** @brief Primärer Empfangspuffer für die ISR/Byte-weise serielle Erfassung. */
char rxBuf[CMD_BUF_LEN];

/** @brief Aktuelle Schreibposition innerhalb des Empfangspuffers `rxBuf`. */
byte rxPos = 0;

/** @brief Event-Flag: Wird auf `true` gesetzt, sobald eine Zeile komplett empfangen wurde. */
volatile bool cmdReady = false;

/** @brief Entkoppelter Arbeitspuffer zur sicheren Befehlsverarbeitung in `processCommand()`. */
char cmdBuf[CMD_BUF_LEN];

/** @brief Internes Array zur Speicherung der booleschen Systemparameter. */
bool paramB[NUM_BOOL_PARAMS];


/* ----- Private Prototypen ------------------------------------------------*/
bool parseUint8(const char *tok, uint8_t &value);
ParamType classifyParam(uint8_t idx); 
void setParams(char* par);
void readParams(char* par);
float getParamF(uint8_t idx);
void setParamF(uint8_t idx, float v);


/* ----- Funktionimplementierungen -----------------------------------------*/

/**
 * @brief Liest den Hardware-UART-Puffer zeilenweise aus.
 * * Die Funktion liest nicht-blockierend alle verfügbaren Bytes. Bei Erkennung eines
 * Zeilenumbruchs (CR/LF) wird der String nullterminiert, in den Arbeitspuffer `cmdBuf`
 * kopiert und das Verarbeitungsflag `cmdReady` gesetzt. Bei Pufferüberlauf wird
 * der bisherige Inhalt verworfen (Überlaufschutz).
 */
void handleSerialInput()
{
  while (Serial.available() > 0) 
  {
    char c = Serial.read();

    if (c == '\r' || c == '\n') {
      if (rxPos > 0) // Leere Zeilen ignorieren
      {  
        rxBuf[rxPos] = '\0';
        strncpy(cmdBuf, rxBuf, CMD_BUF_LEN);
        cmdBuf[CMD_BUF_LEN - 1] = '\0';
        rxPos = 0;
        cmdReady = true; // Event-Flag setzen
      }
    } else {
      if (rxPos < CMD_BUF_LEN - 1) {
        rxBuf[rxPos++] = c;
      } else {
        rxPos = 0; // Puffer zurücksetzen bei Überlauf
      }
    }
  }
}

/**
 * @brief Klassifiziert einen Parameter anhand seines numerischen Index.
 * @param idx Der zu prüfende Parameter-Index.
 * @return ParamType Der ermittelte Typ (FLOAT, BOOL oder INVALID).
 */
ParamType classifyParam(uint8_t idx) 
{
  if (idx < NUM_FLOAT_PARAMS)                  return PARAM_FLOAT;
  if (idx >= 100 && idx < 100+NUM_BOOL_PARAMS) return PARAM_BOOL;
  return PARAM_INVALID;
}

/**
 * @brief Parser- und Setzfunktion für das serielle Kommando 's'.
 * * Erwartet einen String im Format "<index> <wert>". Tokenisiert die Argumente,
 * prüft die Bereichsgrenzen (Index-Validierung) und schreibt den konvertierten
 * Wert in die entsprechende Datenstruktur (Flash-Struktur oder Bool-Array).
 * Gibt den Erfolg oder Fehler über die serielle Schnittstelle aus.
 * * @param par Zeiger auf den Parameter-Sub-String im Arbeitspuffer.
 */
void setParams(char* par)
{
  char *tok = strtok(par, " \t");
  uint8_t idx;
  if (!parseUint8(tok, idx)) 
  {
    Serial.println(F("ERR s: bad index"));
    return;
  }

  ParamType t = classifyParam(idx);
  if (t == PARAM_INVALID) 
  {
    Serial.println(F("ERR s: index out of range"));
    return;
  }

  tok = strtok(NULL, " \t");
  if (!tok) 
  {
    Serial.println(F("ERR s: missing value"));
    return;
  }
  
  if (t == PARAM_FLOAT) 
  {
    float v = atof(tok);
    setParamF(idx, v);
    Serial.print(F("s "));
    Serial.print(idx);
    Serial.print(F(" "));
    Serial.println(v, 5);
  } 
  else if (t == PARAM_BOOL) 
  {
    bool v;
    if (strcmp(tok, "0") == 0 || strcasecmp(tok, "false") == 0) 
    {
      v = false;
    } 
    else if (strcmp(tok, "1") == 0 || strcasecmp(tok, "true") == 0) 
    {
      v = true;
    } 
    else 
    {
      Serial.println(F("ERR s: bad bool"));
      return;
    }
    uint8_t bIdx = idx - 100;
    paramB[bIdx] = v;
    Serial.print(F("s "));
    Serial.print(idx);
    Serial.print(F(" "));
    Serial.println(v ? F("true") : F("false"));  
  } 
}

/**
 * @brief Liest einen Parameter aus und gibt ihn formatiert über UART aus.
 * * Verarbeitet das serielle Kommando 'r <index>'. Identifiziert den Typ des Parameters,
 * liest den aktuellen Wert aus dem RAM und spiegelt ihn im Format "r <index> <wert>" zurück.
 * * @param par Zeiger auf den Index-Sub-String im Arbeitspuffer.
 */
void readParams(char* par)
{
  char *tok = strtok(par, " \t");
  uint8_t idx;
  if (!parseUint8(tok, idx)) 
  {
    Serial.println(F("ERR r: bad index"));
    return;
  }

  ParamType t = classifyParam(idx);
  if (t == PARAM_INVALID) {
    Serial.println(F("ERR r: index out of range"));
    return;
  }

  Serial.print(F("r "));
  Serial.print(idx);
  Serial.print(F(" "));

  if (t == PARAM_FLOAT) 
     Serial.println(getParamF(idx), 5);
  else if (t == PARAM_BOOL) 
  {
    uint8_t bIdx = idx - 100;
    Serial.println(paramB[bIdx] ? F("true") : F("false"));
  }
}

/**
 * @brief Zentrale State-Machine und Befehls-Interpreter.
 * * Falls `cmdReady` gesetzt ist, wird das Flag sofort zurückgesetzt und das erste
 * Zeichen des Puffers extrahiert. Über ein `switch-case`-Konstrukt werden entweder
 * kontinuierliche Sensor-Samplings (A, G, M) gestartet, das System gestoppt (S),
 * Parameter modifiziert/gelesen (s, r) oder die RAM-Struktur `prefs` persistent
 * in den Flash-Speicher gesichert (w).
 */
void processCommand() 
{
  int rc;
  if (!cmdReady) return;

  cmdReady = false; // Flag für asynchrone Weiterverarbeitung sofort freigeben
  char cmd = cmdBuf[0];

  // Restlichen String als Argumente isolieren (führende Leerzeichen überspringen)
  char *params = cmdBuf + 1;
  while (*params == ' ') params++;

  switch (cmd) {
    case 'G': state = SAMPLING_G; break;
    case 'A': state = SAMPLING_A; break;    
    case 'M': state = SAMPLING_M; break;  
    case 'S': state = IDLE;       break;
    case 's': setParams(params);  break;
    case 'r': readParams(params); break;
    case 'w': 
      // Sichert die gesamte prefs-Struktur im Flash
      rc = myFlashPrefs.writePrefs(&prefs, sizeof(prefs));
      Serial.println(myFlashPrefs.errorString(rc));
      break;
    default:
      Serial.println(F("Unknown command"));
      break;
  }
}

/**
 * @brief Getter-Funktion: Liest eine Fließkommazahl anhand des Index aus der RAM-Struktur.
 * @param idx Parameter-Index (0 bis 17).
 * @return float Der aktuelle Wert aus der `prefs`-Struktur oder `0.0` bei ungültigem Index.
 */
float getParamF(uint8_t idx)
{
  switch (idx)
  {
    case 0:  return prefs.a_x_ofs;
    case 1:  return prefs.a_y_ofs;  
    case 2:  return prefs.a_z_ofs;   
    case 3:  return prefs.a_x_s;
    case 4:  return prefs.a_y_s;  
    case 5:  return prefs.a_z_s;
    case 6:  return prefs.m_x_ofs;
    case 7:  return prefs.m_y_ofs;  
    case 8:  return prefs.m_z_ofs;   
    case 9:  return prefs.m_x_s;
    case 10: return prefs.m_y_s;  
    case 11: return prefs.m_z_s;   
    case 12: return prefs.g_x_ofs;
    case 13: return prefs.g_y_ofs;  
    case 14: return prefs.g_z_ofs;   
    case 15: return prefs.g_x_s;
    case 16: return prefs.g_y_s;  
    case 17: return prefs.g_z_s;  
    default: return 0.0f;         
  }
}

/**
 * @brief Setter-Funktion: Schreibt eine Fließkommazahl direkt in das dedizierte RAM-Strukturglied.
 * @param idx Parameter-Index (0 bis 17).
 * @param v Zuzuweisender Fließkommawert.
 */
void setParamF(uint8_t idx, float v)
{
  switch (idx)
  {
    case 0:  prefs.a_x_ofs = v; break;
    case 1:  prefs.a_y_ofs = v; break;  
    case 2:  prefs.a_z_ofs = v; break;  
    case 3:  prefs.a_x_s   = v; break;   
    case 4:  prefs.a_y_s   = v; break;   
    case 5:  prefs.a_z_s   = v; break; 
    case 6:  prefs.m_x_ofs = v; break;   
    case 7:  prefs.m_y_ofs = v; break;   
    case 8:  prefs.m_z_ofs = v; break;                
    case 9:  prefs.m_x_s   = v; break;   
    case 10: prefs.m_y_s   = v; break;   
    case 11: prefs.m_z_s   = v; break;     
    case 12: prefs.g_x_ofs = v; break;   
    case 13: prefs.g_y_ofs = v; break;   
    case 14: prefs.g_z_ofs = v; break;                
    case 15: prefs.g_x_s   = v; break;   
    case 16: prefs.g_y_s   = v; break;   
    case 17: prefs.g_z_s   = v; break;                                         
  }
}

/**
 * @brief Konvertiert ein textbasiertes Token sicher in eine vorzeichenlose 8-Bit-Ganzzahl.
 * * Überprüft das Token auf Nullpointer, konvertiert den String über `strtol` zur Erkennung
 * von Formatfehlern und führt ein striktes Integritäts- und Bounds-Checking im Wertebereich 
 * von 0 bis 255 durch.
 * * @param tok Zeiger auf das zu parsende Text-Token.
 * @param[out] value Referenz auf die Zielvariable zur Rückgabe des Ergebnisses.
 * @return true Konvertierung erfolgreich und im Wertebereich gültig.
 * @return false Konvertierungsfehler oder Wertebereichsüberschreitung.
 */
bool parseUint8(const char *tok, uint8_t &value)
{
  if (!tok || *tok == '\0') return false;
  long v = strtol(tok, NULL, 10);
  if (v < 0 || v > 255) return false;
  value = (uint8_t)v;
  return true;
}