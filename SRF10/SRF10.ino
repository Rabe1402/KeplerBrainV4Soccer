#include <Wire.h>

#define SRF10_COMMAND_REG 0x00
#define SRF10_GAIN_REG    0x01
#define SRF10_RANGE_REG   0x02
#define SRF10_RESULT_REG  0x02

String CMD = "";
byte old_add = 0x70;  // Default SRF10 Adresse
byte new_add = 0x00;

// ---------------- SETUP ----------------
void setup()
{
  Wire.begin();
  Serial.begin(9600);

  Serial.println("Hello, world");
  delay(200);
  
  scanI2C();

  Serial.println("Enter SRF10 address in HEX (e.g. 0x70):");

  while (Serial.available() == 0) {}

  String input = Serial.readStringUntil('\n');
  old_add = strtol(input.c_str(), NULL, 16);

  Serial.print("Old address set to: 0x");
  Serial.println(old_add, HEX);

  Serial.println("Enter command...");
}

// ---------------- LOOP ----------------
void loop()
{
  if (Serial.available())
  {
    CMD = Serial.readStringUntil('\n');
    CMD.trim();

    Serial.print("Got Cmd: ");
    Serial.println(CMD);

    // -------- VERSION --------
    if (CMD == "SRF10+Ver?")
    {
      byte ver = SRF10_READ_VERSION(old_add);
      Serial.print("Version: ");
      Serial.println(ver);
    }

    // -------- CHANGE ADDRESS --------
    else if (CMD == "SRF10+ADD")
    {
      Serial.println("Enter new address (e.g. 0x71):");

      while (Serial.available() == 0) {}

      String input = Serial.readStringUntil('\n');
      new_add = strtol(input.c_str(), NULL, 16);

      SRF10_WRITE_NEWADD(old_add, new_add);

      Serial.print("New address set to: 0x");
      Serial.println(new_add, HEX);
    }

    // -------- START MEASUREMENT --------
    else if (CMD == "SRF10+START")
    {
      Serial.println("Starting measurement...");
      SRF10_START_RANGING(old_add);
    }

    // -------- READ DISTANCE LOOP --------
    else if (CMD == "SRF10+DIST?")
    {
      Serial.println("Reading distance... press 'q' to quit");

      while (true)
      {
        SRF10_START_RANGING(old_add);
        delay(70);

        int dist = SRF10_READ_RANGE(old_add);

        Serial.print("Distance (cm): ");
        Serial.println(dist);

        delay(300);

        if (Serial.available())
        {
          String exitCmd = Serial.readStringUntil('\n');
          exitCmd.trim();

          if (exitCmd == "q")
          {
            Serial.println("Exit distance mode");
            break;
          }
        }
      }
    }

    else
    {
      Serial.println("Unknown command");
    }
  }
}

// ---------------- SRF10 FUNCTIONS ----------------

byte SRF10_READ_VERSION(byte address)
{
  Wire.beginTransmission(address);
  Wire.write(SRF10_COMMAND_REG);
  Wire.endTransmission();

  Wire.requestFrom(address, (byte)1);

  if (Wire.available())
  {
    return Wire.read();
  }

  return 0xFF;
}

void SRF10_WRITE_NEWADD(byte old_address, byte new_address)
{
  Wire.beginTransmission(old_address);
  Wire.write(SRF10_COMMAND_REG);
  Wire.write(0xA0);
  Wire.endTransmission();
  delay(50);

  Wire.beginTransmission(old_address);
  Wire.write(SRF10_COMMAND_REG);
  Wire.write(0xAA);
  Wire.endTransmission();
  delay(50);

  Wire.beginTransmission(old_address);
  Wire.write(SRF10_COMMAND_REG);
  Wire.write(0xA5);
  Wire.endTransmission();
  delay(50);

  Wire.beginTransmission(old_address);
  Wire.write(SRF10_COMMAND_REG);
  Wire.write(new_address << 1);
  Wire.endTransmission();
  delay(50);

  old_add = new_address;
}

void SRF10_START_RANGING(byte address)
{
  Wire.beginTransmission(address);
  Wire.write(SRF10_COMMAND_REG);
  Wire.write(0x51);
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
    int lowByte = Wire.read();
    return (highByte << 8) | lowByte;
  }

  return -1;
}

void scanI2C() {
  Serial.println("Scanning...");

  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
    }
  }

  Serial.println("Done");
}