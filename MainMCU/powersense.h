// *** I2C INA231 ***
// Verfügbare Funktionen:
// void WRITE_I2C_INA231_INIT(void) - Initialisiert den INA231-Sensor (Konfiguration und Kalibrierung)
// int16_t READ_I2C_INA231_SHUNT_VOLTAGE(void) - Liest die Shunt-Spannung (Rohwert, LSB = 2.5 µV)
// uint16_t READ_I2C_INA231_BUS_VOLTAGE(void) - Liest die Bus-Spannung (Rohwert, LSB = 1.25 mV)
// int16_t READ_I2C_INA231_CURRENT(void) - Liest den Strom (Rohwert, LSB = Current_LSB)
// uint16_t READ_I2C_INA231_POWER(void) - Liest die Leistung (Rohwert, LSB = 25 * Current_LSB)

#ifndef INA231_H
#define INA231_H

#include <Wire.h>
#include <HardwareTimer.h>  // Für Timer-Konfiguration

//ausnahmsweise variablen hier lassen, um dem brandel den code einfach geben zu können// 

#define INA231_ADDR          0x40 //i2c adresse des boards

#define REG_CONFIG           0x00 //i2c register für die werte 
#define REG_SHUNT_VOLTAGE    0x01 //i2c register für die werte 
#define REG_BUS_VOLTAGE      0x02 //i2c register für die werte 
#define REG_POWER            0x03 //i2c register für die werte 
#define REG_CURRENT          0x04 //i2c register für die werte  
#define REG_CALIBRATION      0x05 //i2c register für die werte 

#define R_SHUNT              0.1f   // Shunt-Widerstand in Ohm
#define MAX_CURRENT          5.0f   // Maximal erwarteter Strom in A
#define UPDATE_INTERVAL      100    // Update-Intervall in ms

// Umrechnungskonstanten (aus INA231-Datenblatt)
#define SHUNT_VOLTAGE_LSB    0.0025f  // 2.5 µV pro Bit, in mV
#define BUS_VOLTAGE_LSB      1.25f // 1.25 mV pro Bit, in mV
#define CURRENT_LSB          (MAX_CURRENT / 32768.0f * 10000)  // mA pro Bit
#define POWER_LSB            (25.0f * CURRENT_LSB)     // W pro Bit

// Globale Variablen für Messwerte
static volatile float ina231_shunt_voltage = 0.0f;  // mV
static volatile float ina231_bus_voltage = 0.0f;    // V
static volatile float ina231_current = 0.0f;        // A
static volatile float ina231_power = 0.0f;          // W
static volatile uint32_t ina231_error_count = 0;

extern TwoWire i2c; 
static HardwareTimer timer(TIM9);  // HardwareTimer für TIM9

// Hintergrund-Update der Messwerte (aufgerufen vom Timer-Callback)
static void UPDATE_I2C_INA231(void)
{
  #ifdef INA231_DEBUG
  Serial.println("Updating INA231 measurements...");
  #endif

  // Shunt-Spannung
  i2c.beginTransmission(INA231_ADDR);
  i2c.write(REG_SHUNT_VOLTAGE);
  if (i2c.endTransmission(false) == 0) {
    i2c.requestFrom(INA231_ADDR, 2, true);
    if (i2c.available() >= 2) {
      int16_t raw = (int16_t)((i2c.read() << 8) | i2c.read());
      ina231_shunt_voltage = raw * SHUNT_VOLTAGE_LSB;  // in mV
      #ifdef INA231_DEBUG
      Serial.print("Shunt Voltage raw: ");
      Serial.print(raw);
      Serial.print(", mV: ");
      Serial.println(ina231_shunt_voltage, 3);
      #endif
    } else {
      ina231_error_count++;
    }
  } else {
    ina231_error_count++;
  }

  // Bus-Spannung
  i2c.beginTransmission(INA231_ADDR);
  i2c.write(REG_BUS_VOLTAGE);
  if (i2c.endTransmission(false) == 0) {
    i2c.requestFrom(INA231_ADDR, 2, true);
    if (i2c.available() >= 2) {
      uint8_t msb = i2c.read();
      uint8_t lsb = i2c.read();
      uint16_t raw = (msb << 8) | lsb;  // Big-Endian
      ina231_bus_voltage = raw * BUS_VOLTAGE_LSB;  // in V
      #ifdef INA231_DEBUG
      Serial.print("Bus Voltage raw: ");
      Serial.print(raw);
      Serial.print(", V: ");
      Serial.println(ina231_bus_voltage, 3);
      #endif
    } else {
      ina231_error_count++;
    }
  } else {
    ina231_error_count++;
  }

  // Strom
  i2c.beginTransmission(INA231_ADDR);
  i2c.write(REG_CURRENT);
  if (i2c.endTransmission(false) == 0) {
    i2c.requestFrom(INA231_ADDR, 2, true);
    if (i2c.available() >= 2) {
      int16_t raw = (int16_t)((i2c.read() << 8) | i2c.read());
      ina231_current = raw * CURRENT_LSB;  // in A
      #ifdef INA231_DEBUG
      Serial.print("Current raw: ");
      Serial.print(raw);
      Serial.print(", A: ");
      Serial.println(ina231_current, 3);
      #endif
    } else {
      ina231_error_count++;
    }
  } else {
    ina231_error_count++;
  }

  // Leistung
  i2c.beginTransmission(INA231_ADDR);
  i2c.write(REG_POWER);
  if (i2c.endTransmission(false) == 0) {
    i2c.requestFrom(INA231_ADDR, 2, true);
    if (i2c.available() >= 2) {
      uint16_t raw = (i2c.read() << 8) | i2c.read();
      ina231_power = raw * POWER_LSB;  // in W
      #ifdef INA231_DEBUG
      Serial.print("Power raw: ");
      Serial.print(raw);
      Serial.print(", W: ");
      Serial.println(ina231_power, 3);
      #endif
    } else {
      ina231_error_count++;
    }
  } else {
    ina231_error_count++;
  }

  #ifdef INA231_DEBUG
  Serial.print("Error count: ");
  Serial.println(ina231_error_count);
  Serial.println("INA231 measurements updated");
  #endif
}

// Initialisierung des INA231 und HardwareTimer
void WRITE_I2C_INA231_INIT(void)
{
  uint16_t config_read = 0;
  uint32_t attempts = 0;
  const uint32_t max_attempts = 100;

  #ifdef INA231_DEBUG
  Serial.println("Starting INA231 initialization...");
  #endif
  do
  {
    delay(10);
    i2c.beginTransmission(INA231_ADDR);
    i2c.write(REG_CONFIG);
    if (i2c.endTransmission(false) == 0) {
      i2c.requestFrom(INA231_ADDR, 2, true);
      if (i2c.available() >= 2) {
        config_read = (i2c.read() << 8) | i2c.read();
        #ifdef INA231_DEBUG
        Serial.print("INA231 Config Read: 0x");
        Serial.println(config_read, HEX);
        #endif
      } else {
        ina231_error_count++;
      }
    } else {
      ina231_error_count++;
    }
    attempts++;
  } while (config_read != 0x4127 && attempts < max_attempts);

  if (attempts >= max_attempts) {
    #ifdef INA231_DEBUG
    Serial.println("INA231 Config read timeout: proceeding anyway");
    #endif
  } else {
    #ifdef INA231_DEBUG
    Serial.println("INA231 Config read successful");
    #endif
  }

  i2c.beginTransmission(INA231_ADDR);
  i2c.write(REG_CONFIG);
  i2c.write(0x41);
  i2c.write(0x27);
  if (i2c.endTransmission() != 0) {
    ina231_error_count++;
    #ifdef INA231_DEBUG
    Serial.println("INA231 Config write successful");
    #endif
  } else {
    #ifdef INA231_DEBUG
    Serial.println("INA231 Config write successful");
    #endif
  }

  float current_lsb = MAX_CURRENT / 32768.0f;
  uint16_t calibration = (uint16_t)(0.00512f / (current_lsb * R_SHUNT));
  i2c.beginTransmission(INA231_ADDR);
  i2c.write(REG_CALIBRATION);
  i2c.write((calibration >> 8) & 0xFF);
  i2c.write(calibration & 0xFF);
  if (i2c.endTransmission() != 0) {
    ina231_error_count++;
    #ifdef INA231_DEBUG
    Serial.println("INA231 Calibration write successful");
    #endif
  } else {
    #ifdef INA231_DEBUG
    Serial.println("INA231 Calibration write successful");
    #endif
  }

  #ifdef INA231_DEBUG
  Serial.println("Starting TIM9 initialization...");
  #endif
  timer.setPrescaleFactor(8400);
  timer.setOverflow(1000, TICK_FORMAT);
  timer.attachInterrupt(UPDATE_I2C_INA231);
//timer.setInterruptPriority(5); //weg wegen gibts net oder so 
  timer.resume();
  #ifdef INA231_DEBUG
  Serial.println("TIM9 initialization complete");
  #endif
}

// Shunt-Spannung auslesen (mV)
float READ_I2C_INA231_SHUNT_VOLTAGE(void)
{
  return ina231_shunt_voltage;
}

// Bus-Spannung auslesen (V)
float READ_I2C_INA231_BUS_VOLTAGE(void)
{
  return ina231_bus_voltage;
}

// Strom auslesen (A)
float READ_I2C_INA231_CURRENT(void)
{
  return ina231_current;
}

// Leistung auslesen (W)
float READ_I2C_INA231_POWER(void)
{
  return ina231_power;
}

// Fehlerzähler auslesen
uint32_t READ_I2C_INA231_ERROR_COUNT(void)
{
  return ina231_error_count;
}

#endif // INA231_H