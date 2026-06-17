#ifndef POWERSENSE_H
#define POWERSENSE_H

#include "mbed.h" // Statt <Wire.h> nutzen wir jz mbed!

// 1. Wir kündigen globale Variablen nur an (extern bedeutet: "gibt es irgendwo anders")
extern volatile float ina231_shunt_voltage;
extern volatile float ina231_bus_voltage;
extern volatile float ina231_current;
extern volatile float ina231_power;
extern volatile uint32_t ina231_error_count;

// 2. Wir kündigen die Funktionen an
void WRITE_I2C_INA231_INIT(void);
int16_t READ_I2C_INA231_SHUNT_VOLTAGE(void);
uint16_t READ_I2C_INA231_BUS_VOLTAGE(void);
int16_t READ_I2C_INA231_CURRENT(void);
uint16_t READ_I2C_INA231_POWER(void);

#endif