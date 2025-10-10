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

#define INA231_ADDR          0x40 //i2c adresse des boards

#define REG_CONFIG           0x00 //i2c register für die werte 
#define REG_SHUNT_VOLTAGE    0x01 //i2c register für die werte 
#define REG_BUS_VOLTAGE      0x02 //i2c register für die werte 
#define REG_POWER            0x03 //i2c register für die werte 
#define REG_CURRENT          0x04 //i2c register für die werte  
#define REG_CALIBRATION      0x05 //i2c register für die werte 

#define R_SHUNT              0.1f   // Shunt-Widerstand in Ohm
#define MAX_CURRENT          1.0f   // Maximal erwarteter Strom in A

extern TwoWire i2c; 

// Initialisierung des INA231
void WRITE_I2C_INA231_INIT(void)
{
  uint16_t config_read = 0;
  do
  {
    delay(10);
    i2c.beginTransmission(INA231_ADDR);
    i2c.write(REG_CONFIG);
    i2c.endTransmission(false);
    i2c.requestFrom(INA231_ADDR, 2, true);
    if (i2c.available() >= 2) {
      config_read = (i2c.read() << 8) | i2c.read();
    }
  } while (config_read != 0x4127);  // Default-Wert nach Reset

  // Configuration: Continuous Shunt + Bus, 1.1 ms Conv, 1 Average
  i2c.beginTransmission(INA231_ADDR);
  i2c.write(REG_CONFIG);
  i2c.write(0x41);  // MSB
  i2c.write(0x27);  // LSB
  i2c.endTransmission();

  // Calibration setzen
  float current_lsb = MAX_CURRENT / 32768.0f;  // LSB für Current
  uint16_t calibration = (uint16_t)(0.00512f / (current_lsb * R_SHUNT));  // Formel aus Datasheet
  i2c.beginTransmission(INA231_ADDR);
  i2c.write(REG_CALIBRATION);
  i2c.write((calibration >> 8) & 0xFF);  // MSB
  i2c.write(calibration & 0xFF);         // LSB
  i2c.endTransmission();
}

// Shunt-Spannung auslesen (Rohwert, LSB = 2.5 µV)
int16_t READ_I2C_INA231_SHUNT_VOLTAGE(void)
{
  int16_t value = 0;
  i2c.beginTransmission(INA231_ADDR);
  i2c.write(REG_SHUNT_VOLTAGE);
  i2c.endTransmission(false);
  i2c.requestFrom(INA231_ADDR, 2, true);
  if (i2c.available() >= 2) {
    value = (int16_t)((i2c.read() << 8) | i2c.read());
  }
  return value;  // Signed
}

// Bus-Spannung auslesen (Rohwert, LSB = 1.25 mV)
uint16_t READ_I2C_INA231_BUS_VOLTAGE(void)
{
  uint16_t value = 0;
  i2c.beginTransmission(INA231_ADDR);
  i2c.write(REG_BUS_VOLTAGE);
  i2c.endTransmission(false);
  i2c.requestFrom(INA231_ADDR, 2, true);
  if (i2c.available() >= 2) {
    value = (i2c.read() << 8) | i2c.read();
  }
  return value;  // Unsigned
}

// Strom auslesen (Rohwert, LSB = Current_LSB)
int16_t READ_I2C_INA231_CURRENT(void)
{
  int16_t value = 0;
  i2c.beginTransmission(INA231_ADDR);
  i2c.write(REG_CURRENT);
  i2c.endTransmission(false);
  i2c.requestFrom(INA231_ADDR, 2, true);
  if (i2c.available() >= 2) {
    value = (int16_t)((i2c.read() << 8) | i2c.read());
  }
  return value;  // Signed
}

// Leistung auslesen (Rohwert, LSB = 25 * Current_LSB)
uint16_t READ_I2C_INA231_POWER(void)
{
  uint16_t value = 0;
  i2c.beginTransmission(INA231_ADDR);
  i2c.write(REG_POWER);
  i2c.endTransmission(false);
  i2c.requestFrom(INA231_ADDR, 2, true);
  if (i2c.available() >= 2) {
    value = (i2c.read() << 8) | i2c.read();
  }
  return value;  // Unsigned
}

#endif // INA231_H