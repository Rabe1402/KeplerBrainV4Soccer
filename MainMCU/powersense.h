// *** I2C INA231 ***
// Verfügbare Funktionen:
// void WRITE_I2C_INA231_INIT(void) - Initialisiert den INA231-Sensor (Konfiguration und Kalibrierung)
// int16_t READ_I2C_INA231_SHUNT_VOLTAGE(void) - Liest die Shunt-Spannung (Rohwert, LSB = 2.5 µV)
// uint16_t READ_I2C_INA231_BUS_VOLTAGE(void) - Liest die Bus-Spannung (in mV)
// int16_t READ_I2C_INA231_CURRENT(void) - Liest den Strom (in mA)
// uint16_t READ_I2C_INA231_POWER(void) - Liest die Leistung (Rohwert, LSB = 25 * Current_LSB)

#ifndef INA231_H
#define INA231_H

//#define INA231_DEBUG  

#include <Wire.h>
//ausnahmsweise variablen hier lassen, um dem brandel den code einfach geben zu können// 

#define INA231_ADDR          0x40 //i2c adresse des boards

#define REG_CONFIG           0x00 //i2c register für die werte 
#define REG_SHUNT_VOLTAGE    0x01 //i2c register für die werte 
#define REG_BUS_VOLTAGE      0x02 //i2c register für die werte 
#define REG_POWER            0x03 //i2c register für die werte 
#define REG_CURRENT          0x04 //i2c register für die werte  
#define REG_CALIBRATION      0x05 //i2c register für die werte 
#define REG_ALERT            

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

int counter; //globaler counter 

// Initialisierung des INA231 
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