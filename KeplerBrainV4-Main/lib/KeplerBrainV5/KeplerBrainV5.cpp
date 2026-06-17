// *********************************************
// ***  KeplerBRAIN V4 — Mbed OS Port        ***
// ***  .cpp Implementation                  ***
// *********************************************

#include "KeplerBRAIN_V4.h"
#include "stm32f4xx.h"   // Direkter Registerzugriff (TIM2/TIM3 PWM)
#include <string>
#include <cstring>
#include <chrono>
#include "rtos.h"

// ============================================================
// GLOBALE HARDWARE-OBJEKTE
// ============================================================
// I2C:  PA8=SCL, PC9=SDA   (400 kHz)
I2C     g_i2c(PC_9, PA_8);

// SPI für Linesensoren und ADC:  PA7=MOSI, PA6=MISO, PA5=SCK
SPI     g_spi(PA_7, PA_6, PA_5);

// SPI für Kamera (OpenMV):  PB15=MOSI, PB14=MISO, PB13=SCK
SPI     g_spi_cam(PB_15, PB_14, PB_13);

// Serielle Ports
Serial  g_serial(USBTX, USBRX, 115200);
Serial  g_bt_serial(PA_9, PB_3, 9600);   // TX, RX

// ============================================================
// CS-Pin Objekte  (SPI Chip-Select)
// ============================================================
static DigitalOut cs_adc(PB_1,  1);   // MCP3008 ADC
static DigitalOut cs_spi1(PB_12, 1);
static DigitalOut cs_spi2(PA_11, 1);  // Linesensoren
static DigitalOut cs_spi3(PA_12, 1);
static DigitalOut cs_cam(PC_6,  1);   // Kamera SPI

// ============================================================
// INA231 interne Zustandsvariablen
// ============================================================
static volatile float    s_shunt_voltage = 0.0f;
static volatile float    s_bus_voltage   = 0.0f;
static volatile float    s_current       = 0.0f;
static volatile float    s_power         = 0.0f;
static volatile uint32_t s_error_count   = 0;

// ============================================================
// LCD  (Bit-Bang 4-bit, gleiche Pins wie Arduino-Version)
// PA15=RS, PC13=E, PC11=D4, PC10=D5, PC12=D6, PA1=D7
// ============================================================
static DigitalOut lcd_rs(PA_15);
static DigitalOut lcd_e(PC_13);
static DigitalOut lcd_d4(PC_11);
static DigitalOut lcd_d5(PC_10);
static DigitalOut lcd_d6(PC_12);
static DigitalOut lcd_d7(PA_1);

static void _lcd_pulse_enable() {
    lcd_e = 1; wait_us(1);
    lcd_e = 0; wait_us(50);
}

static void _lcd_write4(uint8_t nibble) {
    lcd_d4 = (nibble >> 0) & 1;
    lcd_d5 = (nibble >> 1) & 1;
    lcd_d6 = (nibble >> 2) & 1;
    lcd_d7 = (nibble >> 3) & 1;
    _lcd_pulse_enable();
}

static void _lcd_send(uint8_t val, bool is_data) {
    lcd_rs = is_data ? 1 : 0;
    _lcd_write4(val >> 4);
    _lcd_write4(val & 0x0F);
    wait_us(is_data ? 43 : 37);
}

static void _lcd_init() {
    wait_ms(50);
    lcd_rs = 0; lcd_e = 0;
    // 4-bit init sequence
    _lcd_write4(0x03); wait_ms(5);
    _lcd_write4(0x03); wait_us(150);
    _lcd_write4(0x03); wait_us(150);
    _lcd_write4(0x02); // 4-bit mode
    _lcd_send(0x28, false); // 2 lines, 5x8
    _lcd_send(0x0C, false); // display on, cursor off
    _lcd_send(0x06, false); // entry mode
    _lcd_send(0x01, false); wait_ms(2); // clear
}

// ============================================================
// COMMON
// ============================================================
void sleep(int ms) { 
    ThisThread::sleep_for(std::chrono::milliseconds(ms)); 
}

uint32_t MILLIS() {
    return (uint32_t)(Kernel::get_ms_count());
}

// ============================================================
// LEDs  (PA4=grün, PC2=gelb, PC3=rot)
// ============================================================
static DigitalOut led1(PA_4, 0);
static DigitalOut led2(PC_2, 0);
static DigitalOut led3(PC_3, 0);

void WRITE_LED(int port, int value) {
    switch (port) {
        case L1: led1 = value ? 1 : 0; break;
        case L2: led2 = value ? 1 : 0; break;
        case L3: led3 = value ? 1 : 0; break;
    }
}

// ============================================================
// LCD
// ============================================================
void WRITE_LCD_TEXT(int x, int y, const char* text) {
    if (x < 1) x = 1; if (x > 16) x = 16;
    if (y < 1) y = 1; if (y > 2)  y = 2;
    // DDRAM-Adresse: Zeile1=0x00, Zeile2=0x40
    uint8_t addr = (uint8_t)((x - 1) + (y == 2 ? 0x40 : 0x00));
    _lcd_send(0x80 | addr, false);
    while (*text) { _lcd_send((uint8_t)*text++, true); }
}

void WRITE_LCD_TEXT(int x, int y, const std::string& text) {
    WRITE_LCD_TEXT(x, y, text.c_str());
}

void WRITE_LCD_CLEAR() {
    _lcd_send(0x01, false);
    wait_ms(2);
}

// ============================================================
// BUTTONS  (PA10=B1, PB5=B2, PB4=B3 — INPUT_PULLUP → active LOW)
// ============================================================
static DigitalIn btn1(PA_10, PullUp);
static DigitalIn btn2(PB_5,  PullUp);
static DigitalIn btn3(PB_4,  PullUp);

uint8_t READ_BUTTON_CLOSED(uint8_t port) {
    switch (port) {
        case B1: return btn1.read() == 0 ? 1 : 0;
        case B2: return btn2.read() == 0 ? 1 : 0;
        case B3: return btn3.read() == 0 ? 1 : 0;
    }
    return 0;
}

static bool s_btn1_prev = false;
static bool s_btn2_prev = false;
static bool s_btn3_prev = false;

uint8_t READ_BUTTON_PRESSED(uint8_t port) {
    bool cur; bool* prev;
    switch (port) {
        case B1: cur = (btn1 == 0); prev = &s_btn1_prev; break;
        case B2: cur = (btn2 == 0); prev = &s_btn2_prev; break;
        case B3: cur = (btn3 == 0); prev = &s_btn3_prev; break;
        default: return 0;
    }
    uint8_t pressed = (cur && !(*prev)) ? 1 : 0;
    *prev = cur;
    return pressed;
}

// ============================================================
// MOTOREN  — TIM2/TIM3 direkte Registersteuerung
// Gleiche Logik wie Arduino-Version
// ============================================================
static DigitalOut m1_in1(PC_4, 0);
static DigitalOut m1_in2(PB_2, 0);
static DigitalOut m2_in1(PC_5, 0);
static DigitalOut m2_in2(PC_8, 0);
static DigitalOut m3_in1(PH_1, 0);
static DigitalOut m3_in2(PD_2, 0);
static DigitalOut m4_in1(PC_1, 0);
static DigitalOut m4_in2(PC_0, 0);

void WRITE_MOTOR(int port, double value) {
    if (value >  100.0) value =  100.0;
    if (value < -100.0) value = -100.0;
    uint32_t duty = (uint32_t)fabs(value);

    switch (port) {
        case M1: {
            m1_in1 = (value > 0) ? 1 : 0;
            m1_in2 = (value < 0) ? 1 : 0;
            TIM3->CCR2 = (duty * TIM3->ARR) / 100;
            break;
        }
        case M2: {
            m2_in1 = (value > 0) ? 1 : 0;
            m2_in2 = (value < 0) ? 1 : 0;
            TIM2->CCR3 = (duty * TIM2->ARR) / 100;
            break;
        }
        case M3: {
            m3_in1 = (value > 0) ? 1 : 0;
            m3_in2 = (value < 0) ? 1 : 0;
            TIM3->CCR3 = (duty * TIM3->ARR) / 100;
            break;
        }
        case M4: {
            m4_in1 = (value > 0) ? 1 : 0;
            m4_in2 = (value < 0) ? 1 : 0;
            TIM2->CCR1 = (duty * TIM2->ARR) / 100;
            break;
        }
    }
}

void WRITE_MOTOR_PWM(uint32_t frequency) {
    if (frequency < 500) frequency = 500;
    uint32_t prescaler = 15;
    uint32_t arr = (5000000 / frequency) - 1;
    TIM3->PSC = prescaler; TIM3->ARR = arr;
    TIM2->PSC = prescaler; TIM2->ARR = arr;
    TIM3->EGR = TIM_EGR_UG;
    TIM2->EGR = TIM_EGR_UG;
}

// ============================================================
// IOS  (PB7=IOS1, PB8=IOS2, PB6=IOS3, PB9=IOS4)
// Mbed: DigitalInOut für wechselnde Richtung
// ============================================================
static DigitalInOut ios1(PB_7);
static DigitalInOut ios2(PB_8);
static DigitalInOut ios3(PB_6);
static DigitalInOut ios4(PB_9);

static DigitalInOut& _ios(uint8_t port) {
    switch (port) {
        case IOS1: return ios1;
        case IOS2: return ios2;
        case IOS3: return ios3;
        default:   return ios4;
    }
}

uint8_t READ_IOS_HIGH(uint8_t port) {
    auto& p = _ios(port);
    p.input(); p.mode(PullDown);
    return p.read() ? 1 : 0;
}

uint8_t READ_IOS_CLOSED(uint8_t port) {
    auto& p = _ios(port);
    p.input(); p.mode(PullUp);
    return p.read() == 0 ? 1 : 0;
}

static bool s_ios1_prev = false;
static bool s_ios2_prev = false;
static bool s_ios3_prev = false;
static bool s_ios4_prev = false;

uint8_t READ_IOS_PRESSED(uint8_t port) {
    auto& p = _ios(port);
    p.input(); p.mode(PullUp);
    bool cur = (p.read() == 0);
    bool* prev;
    switch (port) {
        case IOS1: prev = &s_ios1_prev; break;
        case IOS2: prev = &s_ios2_prev; break;
        case IOS3: prev = &s_ios3_prev; break;
        default:   prev = &s_ios4_prev; break;
    }
    uint8_t pressed = (cur && !(*prev)) ? 1 : 0;
    *prev = cur;
    return pressed;
}

void WRITE_IOS_DIGITAL(uint8_t port, int value) {
    auto& p = _ios(port);
    p.output();
    p.write(value ? 1 : 0);
}

// ============================================================
// KICKER
// ============================================================
#ifdef USE_KICKER
static Timeout s_kicker_timeout;
static uint8_t s_kicker_active_port = 0;

static void _kicker_off() {
    switch (s_kicker_active_port) {
        case IOS1: ios1.output(); ios1 = 0; break;
        case IOS2: ios2.output(); ios2 = 0; break;
        case IOS3: ios3.output(); ios3 = 0; break;
        case IOS4: ios4.output(); ios4 = 0; break;
    }
}

void WRITE_KICKER_INIT(uint8_t port) {
    auto& p = _ios(port);
    p.output();
    p.write(0);
}

void WRITE_KICKER(uint8_t port, uint16_t duration_ms) {
    s_kicker_active_port = port;
    auto& p = _ios(port);
    p.output();
    p.write(1);
    s_kicker_timeout.attach(&_kicker_off, (float)duration_ms / 1000.0f);
}
#endif // USE_KICKER

// ============================================================
// SERVO
// ============================================================
#ifdef USE_SERVO
// TIM4 direkte Registersteuerung — gleich wie Arduino-Version
void WRITE_SERVO_INIT(uint8_t port) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    switch (port) {
        case IOS1: GPIO_InitStruct.Pin = GPIO_PIN_7; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); break;
        case IOS2: GPIO_InitStruct.Pin = GPIO_PIN_8; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); break;
        case IOS3: GPIO_InitStruct.Pin = GPIO_PIN_6; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); break;
        case IOS4: GPIO_InitStruct.Pin = GPIO_PIN_9; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); break;
    }
    TIM4->PSC = 98; TIM4->ARR = 19999;
    if (port == IOS1) { TIM4->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos); TIM4->CCER |= TIM_CCER_CC2E; }
    if (port == IOS2) { TIM4->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos); TIM4->CCER |= TIM_CCER_CC3E; }
    if (port == IOS3) { TIM4->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos); TIM4->CCER |= TIM_CCER_CC1E; }
    if (port == IOS4) { TIM4->CCMR2 |= (0x6 << TIM_CCMR2_OC4M_Pos); TIM4->CCER |= TIM_CCER_CC4E; }
    TIM4->CR1 |= TIM_CR1_CEN;
}

void WRITE_SERVO_PWM(uint8_t port, uint32_t pulselength_us) {
    if (pulselength_us <  500) pulselength_us =  500;
    if (pulselength_us > 2500) pulselength_us = 2500;
    switch (port) {
        case IOS1: TIM4->CCR2 = pulselength_us; break;
        case IOS2: TIM4->CCR3 = pulselength_us; break;
        case IOS3: TIM4->CCR1 = pulselength_us; break;
        case IOS4: TIM4->CCR4 = pulselength_us; break;
    }
}
#endif // USE_SERVO

// ============================================================
// ADC  MCP3008 über SPI
// ============================================================
uint16_t READ_IO_ANALOG(uint8_t sensor) {
    static const uint8_t addr_map[] = {
        0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0
    };
    if (sensor < 1 || sensor > 8) return 0;
    uint8_t spi_addr = addr_map[sensor - 1];

    g_spi.format(8, 0);
    g_spi.frequency(1000000);
    cs_adc = 0;
    g_spi.write(0x01);
    uint16_t d1 = g_spi.write(spi_addr);
    uint16_t d2 = g_spi.write(0x00);
    cs_adc = 1;
    return ((d1 << 8) | d2) & 0x03FF;
}

// ============================================================
// SPI LINESENSOR
// ============================================================
uint16_t READ_SPI_LINE(uint8_t port, uint8_t sensor) {
    static const uint8_t addr_map[] = {
        0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0
    };
    if (sensor < 1 || sensor > 8) return 0;
    uint8_t spi_addr = addr_map[sensor - 1];

    g_spi.format(8, 0);
    g_spi.frequency(1000000);

    DigitalOut* cs = nullptr;
    if (port == 0x11 /*SPI1*/) cs = &cs_spi1;  // PB12
    else if (port == 0x12 /*SPI2*/) cs = &cs_spi2; // PA11
    else cs = &cs_spi3;

    *cs = 0;
    g_spi.write(0x01);
    uint16_t d1 = g_spi.write(spi_addr);
    uint16_t d2 = g_spi.write(0x00);
    *cs = 1;
    return ((d1 << 8) | d2) & 0x03FF;
}

// ============================================================
// I2C BNO055  IMU
// ============================================================
static uint8_t _i2c_read_reg(uint8_t addr, uint8_t reg) {
    char buf[1] = { (char)reg };
    g_i2c.write(addr << 1, buf, 1, true);
    g_i2c.read(addr << 1, buf, 1);
    return (uint8_t)buf[0];
}

static void _i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    char buf[2] = { (char)reg, (char)val };
    g_i2c.write(addr << 1, buf, 2);
}

static int16_t _bno_read16(uint8_t reg) {
    char buf[2];
    char r[1] = { (char)reg };
    g_i2c.write(0x28 << 1, r, 1, true);
    g_i2c.read(0x28 << 1, buf, 2);
    return (int16_t)((uint8_t)buf[0] | ((uint8_t)buf[1] << 8));
}

void WRITE_I2C_BNO055_INIT() {
    // Warten bis Chip antwortet (CHIP_ID = 0xA0)
    while (_i2c_read_reg(0x28, 0x00) != 0xA0) { wait_ms(10); }
    wait_ms(50);

    // IMU Mode (ACC+GYRO, kein Magnetometer)
    _i2c_write_reg(0x28, 0x3D, 0x08);
    wait_ms(20);

    // Warten bis Gyro kalibriert (Bits 5:4 = 0b11)
    uint8_t status;
    do {
        wait_ms(10);
        status = _i2c_read_reg(0x28, 0x35);
    } while ((status & 0x30) != 0x30);
}

bool READ_I2C_IS_BNO055_READY() {
    return (_i2c_read_reg(0x28, 0x35) & 0x30) == 0x30;
}

uint16_t READ_I2C_BNO055_YAW() {
    return (uint16_t)(_bno_read16(0x1A) / 16);
}

int16_t READ_I2C_BNO055_ROLL() {
    return (int16_t)(65535 - _bno_read16(0x1C) / 16);
}

int16_t READ_I2C_BNO055_PITCH() {
    return (int16_t)(65535 - _bno_read16(0x1E) / 16);
}

uint16_t READ_I2C_BNO055_ACC_X() {
    return (uint16_t)(_bno_read16(0x08) / 100);
}

int16_t READ_I2C_BNO055_ACC_Y() {
    return (int16_t)(_bno_read16(0x0A) / 100);
}

int16_t READ_I2C_BNO055_ACC_Z() {
    return (int16_t)(_bno_read16(0x0C) / 100);
}

// ============================================================
// I2C INA231  Powersensor
// ============================================================

static uint16_t _ina_read16(uint8_t reg) {
    char cmd[1] = { (char)reg };
    char buf[2];
    if (g_i2c.write(0x40 << 1, cmd, 1, true) != 0) { s_error_count++; return 0; }
    if (g_i2c.read(0x40 << 1, buf, 2) != 0)         { s_error_count++; return 0; }
    return (uint16_t)(((uint8_t)buf[0] << 8) | (uint8_t)buf[1]);
}

static void _ina_write16(uint8_t reg, uint16_t val) {
    char buf[3] = { (char)reg, (char)(val >> 8), (char)(val & 0xFF) };
    if (g_i2c.write(0x40 << 1, buf, 3) != 0) s_error_count++;
}

void WRITE_I2C_INA231_INIT() {
    // Warte bis Config-Register lesbar
    uint16_t cfg = 0;
    uint32_t attempts = 0;
    do {
        wait_ms(10);
        cfg = _ina_read16(0x00);
        attempts++;
    } while (cfg != 0x4127 && attempts < 100);

    _ina_write16(0x00, 0x4127);  // Konfiguration schreiben

    float    current_lsb  = 5.0f / 32768.0f;
    uint16_t calibration  = (uint16_t)(0.00512f / (current_lsb * 0.1f));
    _ina_write16(0x05, calibration);
}

// KEPLER_UPDATE ruft diese intern auf, deshalb Update-Funktion separat
void _ina231_update() {
    int16_t  raw_s = (int16_t)_ina_read16(0x01);
    s_shunt_voltage = raw_s * 0.0025f;

    uint16_t raw_b = _ina_read16(0x02);
    s_bus_voltage = raw_b * 1.25f;

    int16_t  raw_c = (int16_t)_ina_read16(0x04);
    s_current = raw_c * (5.0f / 32768.0f * 1000.0f);

    uint16_t raw_p = _ina_read16(0x03);
    s_power = raw_p * (25.0f * (5.0f / 32768.0f * 10000.0f));
}

float    READ_I2C_INA231_SHUNT_VOLTAGE() { return s_shunt_voltage; }
float    READ_I2C_INA231_BUS_VOLTAGE()   { return s_bus_voltage;   }
float    READ_I2C_INA231_CURRENT()       { return s_current;       }
float    READ_I2C_INA231_POWER()         { return s_power;         }
uint32_t READ_I2C_INA231_ERROR_COUNT()   { return s_error_count;   }

// ============================================================
// SRF10 Ultraschall
// ============================================================
uint8_t SRF10_GAIN  = 10;
uint8_t SRF10_RANGE = 23;

#define _SRF10_CMD_REG   0x00
#define _SRF10_GAIN_REG  0x01
#define _SRF10_RANGE_REG 0x02
#define _SRF10_RES_REG   0x03
#define _SRF10_POLL_MS   65
#define _SRF10_MAX_CM    255

static bool _SRF10_WAIT_READY(uint8_t address) {
    uint32_t start = MILLIS();
    while (MILLIS() - start < _SRF10_POLL_MS) {
        char cmd[1] = { _SRF10_CMD_REG };
        char buf[1];
        g_i2c.write(address << 1, cmd, 1);
        g_i2c.read(address << 1, buf, 1);
        if ((uint8_t)buf[0] != 0xFF) return true;
        wait_ms(5);
    }
    return false;
}

void WRITE_SRF10_GAIN(uint8_t address, uint8_t gain) {
    char buf[2] = { _SRF10_GAIN_REG, (char)gain };
    g_i2c.write(address << 1, buf, 2);
}

void WRITE_SRF10_RANGE(uint8_t address, uint8_t rangeVal) {
    char buf[2] = { _SRF10_RANGE_REG, (char)rangeVal };
    g_i2c.write(address << 1, buf, 2);
}

void WRITE_SRF10_INIT() {
    const uint8_t addrs[4] = { SRF10_S1, SRF10_S2, SRF10_S3, SRF10_S4 };
    for (int i = 0; i < 4; i++) {
        WRITE_SRF10_GAIN(addrs[i],  SRF10_GAIN);
        WRITE_SRF10_RANGE(addrs[i], SRF10_RANGE);
    }
}

int READ_SRF10(uint8_t address) {
    // Messung starten (0x51 = cm)
    char start_cmd[2] = { _SRF10_CMD_REG, 0x51 };
    g_i2c.write(address << 1, start_cmd, 2);

    if (!_SRF10_WAIT_READY(address)) return -1;

    char reg[1] = { _SRF10_RES_REG };
    char buf[1];
    g_i2c.write(address << 1, reg, 1);
    g_i2c.read(address << 1, buf, 1);
    int dist = (uint8_t)buf[0];
    return (dist > _SRF10_MAX_CM) ? -1 : dist;
}

// ============================================================
// HAUPTINITIALISIERUNG
// ============================================================
static void _motor_timer_init() {
    // GPIO Alt-Funktion für PWM-Pins  (identisch zur Arduino-Version)
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    // PC7  → TIM3 CH2
    g.Pin = GPIO_PIN_7; g.Alternate = GPIO_AF2_TIM3; HAL_GPIO_Init(GPIOC, &g);
    // PB0  → TIM3 CH3
    g.Pin = GPIO_PIN_0; g.Alternate = GPIO_AF2_TIM3; HAL_GPIO_Init(GPIOB, &g);
    // PA0  → TIM2 CH1
    g.Pin = GPIO_PIN_0; g.Alternate = GPIO_AF1_TIM2; HAL_GPIO_Init(GPIOA, &g);
    // PB10 → TIM2 CH3
    g.Pin = GPIO_PIN_10; g.Alternate = GPIO_AF1_TIM2; HAL_GPIO_Init(GPIOB, &g);

    // 5 kHz PWM (84 MHz / 16 / 1050)
    TIM3->PSC = 15; TIM3->ARR = 1049;
    TIM2->PSC = 15; TIM2->ARR = 1049;

    // PWM Mode 1 für alle 4 Kanäle
    TIM3->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos);
    TIM3->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos);
    TIM3->CCER  |= TIM_CCER_CC2E | TIM_CCER_CC3E;

    TIM2->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos);
    TIM2->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos);
    TIM2->CCER  |= TIM_CCER_CC1E | TIM_CCER_CC3E;

    TIM3->CR1 |= TIM_CR1_CEN;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void KEPLERBRAIN_INIT() {
    // I2C Frequenz
    g_i2c.frequency(400000);

    // SPI defaults
    g_spi.format(8, 0);
    g_spi.frequency(1000000);
    g_spi_cam.format(8, 0);
    g_spi_cam.frequency(1000000);

    // Motor-Timer
    _motor_timer_init();

    // LCD
    _lcd_init();
    WRITE_LCD_TEXT(1, 1, " KeplerBRAIN V4 ");
    WRITE_LCD_TEXT(1, 2, ":-) WELLCOME (-:");
}