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

// Umrechnungskonstanten (aus INA231-Datenblatt)
//#define SHUNT_VOLTAGE_LSB    0.0025f  // 2.5 µV pro Bit, in mV
//#define BUS_VOLTAGE_LSB      1.25f // 1.25 mV pro Bit, in mV
//#define CURRENT_LSB          (MAX_CURRENT / 32768.0f * 10000)  // mA pro Bit
//#define POWER_LSB            (25.0f * CURRENT_LSB)     // W pro Bit


extern TwoWire i2c; 

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
    i2c.beginTransmission(0x40);
    i2c.write(0x00);
    if (i2c.endTransmission(false) == 0) {
      i2c.requestFrom(0x40, 2, true);
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

  i2c.beginTransmission(0x40);
  i2c.write(0x00);
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

  float current_lsb = 5.0f / 32768.0f;
  uint16_t calibration = (uint16_t)(0.00512f / (current_lsb * 0.1f));
  i2c.beginTransmission(0x40);
  i2c.write(0x05);
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