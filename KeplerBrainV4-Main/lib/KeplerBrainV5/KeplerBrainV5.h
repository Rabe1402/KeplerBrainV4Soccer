// *********************************************
// ***  KeplerBRAIN V4 — Mbed OS Port        ***
// ***  Migrated from Arduino Core           ***
// *********************************************
#pragma once

#include "mbed.h"
#include <cstdint>
#include <cmath>

// ============================================================
// PIN-KONSTANTEN  (identisch zu Arduino-Version)
// ============================================================
#define L1   0x11
#define L2   0x12
#define L3   0x13

#define B1   0x21
#define B2   0x22
#define B3   0x23

#define M1   0x31
#define M2   0x32
#define M3   0x33
#define M4   0x34

#define A1   0x41
#define A2   0x42
#define A3   0x43
#define A4   0x44
#define A5   0x45
#define A6   0x46
#define A7   0x47
#define A8   0x48

#define IOS1 0x51
#define IOS2 0x52
#define IOS3 0x53
#define IOS4 0x54

// SPI CS-Pins (als uint32_t Pin-IDs in Mbed-Notation)
// Werden in KeplerBRAIN_V4.cpp verwendet

// SRF10 I2C-Adressen
#define SRF10_S1  0x70
#define SRF10_S2  0x71
#define SRF10_S3  0x72
#define SRF10_S4  0x73

// ============================================================
// GLOBALE HARDWARE-OBJEKTE  (extern — definiert in .cpp)
// ============================================================
extern I2C          g_i2c;        // PA8=SCL, PC9=SDA
extern SPI          g_spi;        // PA7=MOSI, PA6=MISO, PA5=SCK  (Linesensors)
extern SPI          g_spi_cam;    // PB15=MOSI,PB14=MISO,PB13=SCK (Kamera)
extern Serial       g_serial;     // USB-Serial (115200)
extern Serial       g_bt_serial;  // PB3=RX, PA9=TX (Bluetooth)

// ============================================================
// COMMON
// ============================================================
void sleep(int ms);
uint32_t MILLIS();

// ============================================================
// LEDs
// ============================================================
void WRITE_LED(int port, int value);

// ============================================================
// LCD  (4-bit, LiquidCrystal-kompatibel via GPIO bit-banging)
// ============================================================
void WRITE_LCD_TEXT(int x, int y, const char* text);
void WRITE_LCD_TEXT(int x, int y, const std::string& text);  // Komfort-Overload
void WRITE_LCD_CLEAR();

// ============================================================
// BUTTONS
// ============================================================
uint8_t READ_BUTTON_CLOSED(uint8_t port);
uint8_t READ_BUTTON_PRESSED(uint8_t port);

// ============================================================
// MOTOREN  (PWM via TIM2/TIM3, direkte Register-Zugriffe)
// ============================================================
void WRITE_MOTOR(int port, double value);
void WRITE_MOTOR_PWM(uint32_t frequency);

// ============================================================
// IOS (Digital I/O Ports)
// ============================================================
uint8_t READ_IOS_HIGH(uint8_t port);
uint8_t READ_IOS_CLOSED(uint8_t port);
uint8_t READ_IOS_PRESSED(uint8_t port);
void    WRITE_IOS_DIGITAL(uint8_t port, int value);

// ============================================================
// KICKER  (optional, per #define USE_KICKER aktivieren)
// ============================================================
#ifdef USE_KICKER
void WRITE_KICKER_INIT(uint8_t port);
void WRITE_KICKER(uint8_t port, uint16_t duration_ms);
#endif

// ============================================================
// SERVO  (optional, per #define USE_SERVO aktivieren)
// ============================================================
#ifdef USE_SERVO
void WRITE_SERVO_INIT(uint8_t port);
void WRITE_SERVO_PWM(uint8_t port, uint32_t pulselength_us);
#endif

// ============================================================
// ADC  (MCP3008 über SPI)
// ============================================================
uint16_t READ_IO_ANALOG(uint8_t sensor);

// ============================================================
// SPI LINESENSOR
// ============================================================
uint16_t READ_SPI_LINE(uint8_t port, uint8_t sensor);

// ============================================================
// I2C BNO055 IMU
// ============================================================
void     WRITE_I2C_BNO055_INIT();
bool     READ_I2C_IS_BNO055_READY();
uint16_t READ_I2C_BNO055_YAW();
int16_t  READ_I2C_BNO055_ROLL();
int16_t  READ_I2C_BNO055_PITCH();
uint16_t READ_I2C_BNO055_ACC_X();
int16_t  READ_I2C_BNO055_ACC_Y();
int16_t  READ_I2C_BNO055_ACC_Z();

// ============================================================
// I2C INA231  Powersensor
// ============================================================
void     WRITE_I2C_INA231_INIT();
float    READ_I2C_INA231_SHUNT_VOLTAGE();
float    READ_I2C_INA231_BUS_VOLTAGE();
float    READ_I2C_INA231_CURRENT();
float    READ_I2C_INA231_POWER();
uint32_t READ_I2C_INA231_ERROR_COUNT();

// ============================================================
// SRF10 Ultraschall
// ============================================================
extern uint8_t SRF10_GAIN;
extern uint8_t SRF10_RANGE;
void WRITE_SRF10_INIT();
void WRITE_SRF10_GAIN(uint8_t address, uint8_t gain);
void WRITE_SRF10_RANGE(uint8_t address, uint8_t rangeVal);
int  READ_SRF10(uint8_t address);

// ============================================================
// HAUPTINITIALISIERUNG
// ============================================================
void KEPLERBRAIN_INIT();