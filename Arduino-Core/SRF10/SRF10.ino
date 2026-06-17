// SRF10 Ultrasonic Sensor Control
// CMDs:
//   SRF10+HELP      --> zeigt alle verfügbaren Commands
//   SRF10+Ver?      --> liest die Firmware-Version
//   SRF10+ADD       --> neue I2C-Adresse setzen
//   SRF10+GAIN      --> Gain-Wert setzen (0–16)
//   SRF10+START     --> einzelne Messung starten
//   SRF10+DIST?     --> kontinuierliche Messung mit Zeitstempel (q zum Beenden)

#include <Wire.h>

#define SRF10_COMMAND_REG  0x00
#define SRF10_GAIN_REG     0x01
#define SRF10_RANGE_REG    0x02
#define SRF10_RESULT_REG   0x02

#define SRF10_READY_MASK   0xFF   // Sensor meldet "busy" solange Register 0x00 != 0xFF
#define POLL_TIMEOUT_MS    200    // max. Wartezeit auf Messung

// Gültige SRF10-Adressen: 0xE0, 0xE2, ... 0xFE (Schritte von 2, 7-bit: 0x70–0x7F)
// In 7-bit-Schreibweise: 0x70 bis 0x7F (nur gerade Vielfache des Originals)
#define SRF10_ADDR_MIN  0x70
#define SRF10_ADDR_MAX  0x7F

String CMD    = "";
byte old_add  = 0x70;
byte new_add  = 0x00;

// ─────────────────────────── SETUP ───────────────────────────
void setup()
{
  Wire.begin();
  Serial.begin(9600);

  Serial.println("=== SRF10 PROGRAMMER ===");
  delay(200);

  scanI2C();

  Serial.println("Aktuelle I2C-Adresse eingeben (z.B. 0x70):");
  while (Serial.available() == 0) {}

  String input = Serial.readStringUntil('\n');
  old_add = strtol(input.c_str(), NULL, 16);

  Serial.print("Adresse gesetzt: 0x");
  Serial.println(old_add, HEX);
  Serial.println("Bereit. 'SRF10+HELP' fuer eine Befehlsuebersicht.");
}

// ─────────────────────────── LOOP ────────────────────────────
void loop()
{
  if (Serial.available())
  {
    CMD = Serial.readStringUntil('\n');
    CMD.trim();

    Serial.print("> ");
    Serial.println(CMD);

    // ── HELP ──────────────────────────────────────────────────
    if (CMD == "SRF10+HELP")
    {
      Serial.println("-----------------------------");
      Serial.println("SRF10+HELP    Befehlsuebersicht");
      Serial.println("SRF10+Ver?    Firmware-Version lesen");
      Serial.println("SRF10+ADD     I2C-Adresse aendern");
      Serial.println("SRF10+GAIN    Gain setzen (0-16)");
      Serial.println("SRF10+START   Einzelmessung starten");
      Serial.println("SRF10+DIST?   Dauermessung (q = Abbruch)");
      Serial.println("-----------------------------");
    }

    // ── VERSION ───────────────────────────────────────────────
    else if (CMD == "SRF10+Ver?")
    {
      byte ver = SRF10_READ_VERSION(old_add);
      Serial.print("Version: ");
      Serial.println(ver);
    }

    // ── ADRESSE AENDERN ───────────────────────────────────────
    else if (CMD == "SRF10+ADD")
    {
      Serial.println("Neue Adresse eingeben (0x70–0x7F):");
      while (Serial.available() == 0) {}

      String input = Serial.readStringUntil('\n');
      new_add = strtol(input.c_str(), NULL, 16);

      if (!isValidAddress(new_add))
      {
        Serial.println("FEHLER: Ungueltige Adresse! Erlaubt: 0x70–0x7F");
        return;
      }

      SRF10_WRITE_NEWADD(old_add, new_add);
      Serial.print("Adresse geaendert zu: 0x");
      Serial.println(new_add, HEX);
    }

    // ── GAIN SETZEN ───────────────────────────────────────────
    else if (CMD == "SRF10+GAIN")
    {
      Serial.println("Gain eingeben (0–16):");
      while (Serial.available() == 0) {}

      String input = Serial.readStringUntil('\n');
      int gain = input.toInt();

      if (gain < 0 || gain > 16)
      {
        Serial.println("FEHLER: Gain muss zwischen 0 und 16 liegen!");
        return;
      }

      SRF10_SET_GAIN(old_add, (byte)gain);
      Serial.print("Gain gesetzt: ");
      Serial.println(gain);
    }

    // ── EINZELMESSUNG ─────────────────────────────────────────
    else if (CMD == "SRF10+START")
    {
      Serial.println("Starte Messung...");
      SRF10_START_RANGING(old_add);

      if (SRF10_WAIT_READY(old_add))
      {
        int dist = SRF10_READ_RANGE(old_add);
        Serial.print("Distanz (cm): ");
        Serial.print(dist);
        Serial.print("  |  Zeit (ms): ");
        Serial.println(millis());
      }
      else
      {
        Serial.println("FEHLER: Sensor antwortet nicht (Timeout)");
      }
    }

    // ── DAUERMESSUNG ──────────────────────────────────────────
    else if (CMD == "SRF10+DIST?")
    {
      Serial.println("Dauermessung laeuft... 'q' + Enter zum Beenden");
      Serial.println("Zeit(ms), Distanz(cm)");

      while (true)
      {
        SRF10_START_RANGING(old_add);

        if (SRF10_WAIT_READY(old_add))
        {
          int dist = SRF10_READ_RANGE(old_add);
          unsigned long t = millis();

          Serial.print(t);
          Serial.print(", ");
          Serial.println(dist);
        }
        else
        {
          Serial.println("FEHLER: Timeout");
        }

        delay(100);

        if (Serial.available())
        {
          String exitCmd = Serial.readStringUntil('\n');
          exitCmd.trim();
          if (exitCmd == "q")
          {
            Serial.println("Dauermessung beendet.");
            break;
          }
        }
      }
    }

    else
    {
      Serial.println("Unbekannter Befehl. 'SRF10+HELP' fuer Hilfe.");
    }
  }
}

// ─────────────────────── HILFSFUNKTIONEN ─────────────────────

bool isValidAddress(byte addr)
{
  // SRF10 erlaubt 7-bit-Adressen 0x70–0x7F
  return (addr >= SRF10_ADDR_MIN && addr <= SRF10_ADDR_MAX);
}

// Pollt Register 0x00 bis der Sensor "fertig" meldet (Wert != 0xFF bedeutet busy)
// Gibt true zurueck wenn bereit, false bei Timeout
bool SRF10_WAIT_READY(byte address)
{
  unsigned long start = millis();

  while (millis() - start < POLL_TIMEOUT_MS)
  {
    Wire.beginTransmission(address);
    Wire.write(SRF10_COMMAND_REG);
    Wire.endTransmission();

    Wire.requestFrom(address, (byte)1);

    if (Wire.available())
    {
      byte status = Wire.read();
      if (status == 0xFF)  // 0xFF = Messung abgeschlossen
      {
        return true;
      }
    }
    delay(5);
  }

  return false;  // Timeout
}

// ─────────────────────── SRF10 FUNCTIONS ─────────────────────

byte SRF10_READ_VERSION(byte address)
{
  Wire.beginTransmission(address);
  Wire.write(SRF10_COMMAND_REG);
  Wire.endTransmission();

  Wire.requestFrom(address, (byte)1);

  if (Wire.available())
    return Wire.read();

  return 0xFF;
}

void SRF10_WRITE_NEWADD(byte old_address, byte new_address)
{
  // SRF10 Adressänderungs-Sequenz (immer diese 3 Bytes + neue Adresse)
  Wire.beginTransmission(old_address); Wire.write(SRF10_COMMAND_REG); Wire.write(0xA0); Wire.endTransmission(); delay(50);
  Wire.beginTransmission(old_address); Wire.write(SRF10_COMMAND_REG); Wire.write(0xAA); Wire.endTransmission(); delay(50);
  Wire.beginTransmission(old_address); Wire.write(SRF10_COMMAND_REG); Wire.write(0xA5); Wire.endTransmission(); delay(50);
  Wire.beginTransmission(old_address); Wire.write(SRF10_COMMAND_REG); Wire.write(new_address << 1); Wire.endTransmission(); delay(50);

  old_add = new_address;
}

void SRF10_SET_GAIN(byte address, byte gain)
{
  // Gain-Register: 0x00 (min) bis 0x10 (max, default)
  Wire.beginTransmission(address);
  Wire.write(SRF10_GAIN_REG);
  Wire.write(gain);
  Wire.endTransmission();
}

void SRF10_START_RANGING(byte address)
{
  Wire.beginTransmission(address);
  Wire.write(SRF10_COMMAND_REG);
  Wire.write(0x51);  // 0x51 = cm, 0x50 = inch, 0x52 = µs
  Wire.endTransmission();
}

int SRF10_READ_RANGE(byte address)
{
  Wire.beginTransmission(address);
  Wire.write(SRF10_RESULT_REG);
  Wire.endTransmission();

  Wire.requestFrom(address, (byte)2);

  if (Wire.available() >= 2)
  {
    int highByte = Wire.read();
    int lowByte  = Wire.read();
    return (highByte << 8) | lowByte;
  }

  return -1;
}

void scanI2C()
{
  Serial.println("I2C Scan...");

  for (byte i = 1; i < 127; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("Geraet gefunden: 0x");
      Serial.println(i, HEX);
    }
  }

  Serial.println("Scan fertig.");
}
