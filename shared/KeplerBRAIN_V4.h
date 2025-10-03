// *********************************************
// ***  KeplerBRAIN V4 Library  V 04.04.2025 ***
// *********************************************

#include "stm32f4xx.h"

// BUTTONS

// value = READ_BUTTON_CLOSED(port)
// port: B1, B2, B3
// value: 0, 1

// value = READ_BUTTON_PRESSED(port)
// port: B1, B2, B3
// value: 0, 1


// LEDs

// void WRITE_LED(int port, int value)
// port: L1, L2, L3
// value: 0, 1


// DISPLAY

// void LCD_DRAW_TEXT(int x, int y, String text)
// x: 1, 2, ..., 16
// y: 1, 2
// text: String mit max 16 Zeichen

// void LCD_CLS();


// MOTOREN

// void WRITE_MOTOR_SPEED(int port, float value)
// port: 1, 2, 3, 4
// value:  -0.9 bis 0.9 (Vorzeichen: Drehrichtung)

// WRITE_MOTOR_STOP(int port)
// port: 1, 2, 3, 4


// IOS

// int READ_IO_CLOSED(int port, value)
// port: IOS1, IOS2, IOS3, IOS4
// value: 0, 1

// int READ_IO_PRESSED(int port, value)
// port: IOS1, IOS2, IOS3, IOS4
// value: 0, 1

// void WRITE_IO_DIGITAL(int port, value)
// port: IOS1, IOS2, IOS3, IOS4
// value: 0, 1


// ADC

// int READ_IO_ANALOG(int port)
// port: A1, A2, ..., A8
// value: 0 .. 1023


// *** Konstanten ***

#define L1 0x11
#define L2 0x12
#define L3 0x13

#define B1 0x21
#define B2 0x22
#define B3 0x23

#define M1 0x31
#define M2 0x32
#define M3 0x33
#define M4 0x34

#define A1 0x41
#define A2 0x42
#define A3 0x43
#define A4 0x44
#define A5 0x45
#define A6 0x46
#define A7 0x47
#define A8 0x48

#define IOS1 0x51
#define IOS2 0x52
#define IOS3 0x53
#define IOS4 0x54

#define SPI1 PB12
#define SPI2 PA11
#define SPI3 PA12
#define SPICAM PC6


// *** INCLUDES ***

#include <LiquidCrystal.h>
#include <Wire.h>
#include <SPI.h>

// *** COMMON ***

void SLEEP(int value)
{
  delay(value);
}


// *** Leds ***

// PA_4 Led gruen
// PC_2 Led gelb
// PC_3 Led rot

void WRITE_LED(int port, int value)
{
   switch(port)
    {
      case L1:
        if (value==0) digitalWrite(PA4,LOW);
        else digitalWrite(PA4,HIGH);
      break;
      case L2:
        if (value==0) digitalWrite(PC2,LOW);
        else digitalWrite(PC2,HIGH);
      break;
      case L3:
        if (value==0) digitalWrite(PC3,LOW);
        else digitalWrite(PC3,HIGH);
      break;
    }
}

// *** Display ***

// PA_15 Display RS
// PC_13 Display E
// PC_11 Display D4
// PC_10 Display D5
// PC_12 Display D6
// PA_1  Display D7

LiquidCrystal lcd(PA15, PC13, PC11, PC10, PC12, PA1);
    
void WRITE_LCD_TEXT(int x, int y, String text)
{
  if (y<1) y = 1;
  if (y>2) y = 2;
  if (x<1) x = 1;
  if (x>16) x = 16;
  y--;
  x--;
  lcd.setCursor(x, y);
  lcd.print(text);
}

void WRITE_LCD_CLEAR()
{
  lcd.clear();
}


// *** TASTER ***

// PA_10 Taster 1
// PB_5  Taster 2
// PB_4  Taster 3

uint8_t READ_BUTTON_CLOSED(uint8_t port)
{
   int valueb = 0;
   
   switch(port)
    {
      case B1:
        if (!digitalRead(PA10)) valueb = 1;
      break;
      case B2:
        if (!digitalRead(PB5)) valueb = 1;
      break;
      case B3:
        if (!digitalRead(PB4)) valueb = 1;
      break;
    }

    return valueb;
}

bool button1_closed; 
bool button2_closed; 
bool button3_closed; 

uint8_t READ_BUTTON_PRESSED(uint8_t port)
{
  int value = 0;
  int value1 = 0;
  int value2 = 0;
  int value3 = 0;
  if (port==B1)
  {
        if (!digitalRead(PA10))
        {
          if (button1_closed==false)
          {
            value1 = 1;
          }
          button1_closed = true;
        }
        else
        {
          button1_closed = false;
        } 
        value=value1;
  }
  if (port==B2)
  {      
        if (!digitalRead(PB5))
        {
          if (button2_closed==false)
          {
            value2 = 1;
          }
          button2_closed = true;
        }
        else
        {
          button2_closed = false;
        } 
        value=value2;
   }
   if (port==B3)
   {
        if (!digitalRead(PB4))
        {
          if (button3_closed==false)
          {
            value3 = 1;
          }
          button3_closed = true;
        }
        else
        {
          button3_closed = false;
        } 
        value=value3;
    }
    
    return value;
}    

// *** PWM Motoren ***

// PC_7 Motor 1 PWM
// PC_4 Motor 1 IN1
// PB_2 Motor 1 IN2

// PB_10 Motor 2 PWM
// PC_5  Motor 2 IN1
// PC_8  Motor 2 IN2

// PB_0 Motor 3 PWM
// PH_1 Motor 3 IN1
// PD_2 Motor 3 IN2

// PA_0 Motor 4 PWM
// PC_1 Motor 4 IN1
// PC_0 Motor 4 IN2

uint32_t M_PWM;

void WRITE_MOTOR(int port, double value)
{

  if (value>100) value = 100;
  if (value<-100) value = -100;
  int dutycycle = abs(value);

  switch(port)
    {
      case M1: // PC_7

        if (value==0) { digitalWrite(PC4,LOW); digitalWrite(PB2,LOW); uint32_t pulseLength = (dutycycle * TIM3->ARR) / 100; TIM3->CCR2 = pulseLength; }
        if (value>0) { digitalWrite(PC4,HIGH); digitalWrite(PB2,LOW); uint32_t pulseLength = (dutycycle * TIM3->ARR) / 100; TIM3->CCR2 = pulseLength; }
        if (value<0) { digitalWrite(PC4,LOW); digitalWrite(PB2,HIGH); uint32_t pulseLength = (dutycycle * TIM3->ARR) / 100; TIM3->CCR2 = pulseLength; }
      break;
      case M2: // PB_10
        if (value==0) { digitalWrite(PC5,LOW); digitalWrite(PC8,LOW); uint32_t pulseLength = (dutycycle * TIM2->ARR) / 100; TIM2->CCR3 = pulseLength; }
        if (value>0) { digitalWrite(PC5,HIGH); digitalWrite(PC8,LOW); uint32_t pulseLength = (dutycycle * TIM2->ARR) / 100; TIM2->CCR3 = pulseLength; }
        if (value<0) { digitalWrite(PC5,LOW); digitalWrite(PC8,HIGH); uint32_t pulseLength = (dutycycle * TIM2->ARR) / 100; TIM2->CCR3 = pulseLength; }
      break;
      case M3: // PB_0
        if (value==0) { digitalWrite(PH1,LOW); digitalWrite(PD2,LOW); uint32_t pulseLength = (dutycycle * TIM3->ARR) / 100; TIM3->CCR3 = pulseLength; }
        if (value>0) { digitalWrite(PH1,HIGH); digitalWrite(PD2,LOW); uint32_t pulseLength = (dutycycle * TIM3->ARR) / 100; TIM3->CCR3 = pulseLength; }
        if (value<0) { digitalWrite(PH1,LOW); digitalWrite(PD2,HIGH); uint32_t pulseLength = (dutycycle * TIM3->ARR) / 100; TIM3->CCR3 = pulseLength; }
      break;
      case M4: // PA_0
        if (value==0) { digitalWrite(PC1,LOW); digitalWrite(PC0,LOW); uint32_t pulseLength = (dutycycle * TIM2->ARR) / 100; TIM2->CCR1 = pulseLength; }
        if (value>0) { digitalWrite(PC1,HIGH); digitalWrite(PC0,LOW); uint32_t pulseLength = (dutycycle * TIM2->ARR) / 100; TIM2->CCR1 = pulseLength; }
        if (value<0) { digitalWrite(PC1,LOW); digitalWrite(PC0,HIGH); uint32_t pulseLength = (dutycycle * TIM2->ARR) / 100; TIM2->CCR1 = pulseLength; }
      break;
    }
}

void WRITE_MOTOR_PWM(uint32_t frequency)
{

    // Frequenz überprüfen, um zu verhindern, dass die Frequenz zu niedrig ist
    if (frequency < 500) frequency=500; 

    // Timer-ARR (Auto-Reload-Register) berechnen
    uint32_t prescaler = 15;  // Prescaler: 84 MHz / (15 + 1) = 5.25 MHz
    uint32_t arr = (5000000 / frequency) - 1;  // Berechnung des ARR-Wertes

    // Timer-ARR und CCR-Werte für beide Timer setzen
    TIM3->PSC = prescaler;
    TIM3->ARR = arr;
    TIM2->PSC = prescaler;
    TIM2->ARR = arr;

    // Timer neu starten, um Änderungen wirksam zu machen
    TIM3->EGR = TIM_EGR_UG;  // Update Generation
    TIM2->EGR = TIM_EGR_UG;  // Update Generation
}

// *** IOS ***
   
// PB_6  Digital In Out Servo IOS1
// PB_7  Digital In Out Servo IOS2
// PB_8  Digital In Out Servo IOS3
// PB_9  Digital In Out Servo IOS4

uint8_t READ_IOS_CLOSED(uint8_t port)
{

  if (port==IOS1)
  {
    pinMode(PB6, INPUT_PULLUP);
    if (digitalRead(PB6)==0) return 1;
    else return 0;
  }
  if (port==IOS2)
  {
    pinMode(PB7, INPUT_PULLUP);
    if (digitalRead(PB7)==0) return 1;
    else return 0;
  }
  if (port==IOS3)
  {
    pinMode(PB8, INPUT_PULLUP);
    if (digitalRead(PB8)==0) return 1;
    else return 0;
  }
  if (port==IOS4)
  {
    pinMode(PB9, INPUT_PULLUP);
    if (digitalRead(PB9)==0) return 1;
    else return 0;
  }
}

bool ios1_closed; 
bool ios2_closed; 
bool ios3_closed; 
bool ios4_closed; 

uint8_t READ_IOS_PRESSED(uint8_t port)
{
  int value = 0;
  int value1 = 0;
  int value2 = 0;
  int value3 = 0;
  int value4 = 0;
  if (port==IOS1)
  {
        if (!digitalRead(PB6))
        {
          if (ios1_closed==false)
          {
            value1 = 1;
          }
          ios1_closed = true;
        }
        else
        {
          ios1_closed = false;
        } 
        value=value1;
  }
  if (port==IOS2)
  {      
        if (!digitalRead(PB7))
        {
          if (ios2_closed==false)
          {
            value2 = 1;
          }
          ios2_closed = true;
        }
        else
        {
          ios2_closed = false;
        } 
        value=value2;
   }
   if (port==IOS3)
   {
        if (!digitalRead(PB8))
        {
          if (ios3_closed==false)
          {
            value3 = 1;
          }
          ios3_closed = true;
        }
        else
        {
          ios3_closed = false;
        } 
        value=value3;
    }
   if (port==IOS4)
   {
        if (!digitalRead(PB9))
        {
          if (ios4_closed==false)
          {
            value4 = 1;
          }
          ios4_closed = true;
        }
        else
        {
          ios4_closed = false;
        } 
        value=value4;
    }
    
    return value;
}    

void WRITE_SERVO_INIT(uint8_t port)
{
  // GPIOB Takt aktivieren
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  // TIM4 Takt aktivieren
  __HAL_RCC_TIM4_CLK_ENABLE();

  // GPIO-Konfiguration für PB6 (TIM4 CH1), PB7 (TIM4 CH2), PB8 (TIM4 CH3), PB9 (TIM4 CH4)
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (port==IOS1)
  {
    // PB7 (TIM4 CH2)
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }

  if (port==IOS2)
  {
    // PB8 (TIM4 CH3)
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }

  if (port==IOS3)
  {
    // PB6 (TIM4 CH1)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }

  if (port==IOS4)
  {
    // PB9 (TIM4 CH4)
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }

  // Timer-Konfiguration für 50 Hz PWM
  TIM4->PSC = 98;  // Prescaler: 1 MHz
  TIM4->ARR = 19999;  // PWM Periode: 1 MHz / (19999+1) = 50 Hz
  
  if (port==IOS1)
  {
    // TIM4: Kanal 2 (PB7)
    TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M);
    TIM4->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos); // PWM Mode 1 für CH2
    TIM4->CCER |= TIM_CCER_CC2E;  // Output Compare aktivieren
  }

  if (port==IOS2)
  {
    // TIM4: Kanal 3 (PB8)
    TIM4->CCMR2 &= ~(TIM_CCMR2_OC3M);
    TIM4->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos); // PWM Mode 1 für CH3
    TIM4->CCER |= TIM_CCER_CC3E;  // Output Compare aktivieren
  }

  if (port==IOS3)
  {
    // TIM4: Kanal 1 (PB6)
    TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M);
    TIM4->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos); // PWM Mode 1 für CH1
    TIM4->CCER |= TIM_CCER_CC1E;  // Output Compare aktivieren
  }

  if (port==IOS4)
  {
    // TIM4: Kanal 4 (PB9)
    TIM4->CCMR2 &= ~(TIM_CCMR2_OC4M);
    TIM4->CCMR2 |= (0x6 << TIM_CCMR2_OC4M_Pos); // PWM Mode 1 für CH4
    TIM4->CCER |= TIM_CCER_CC4E;  // Output Compare aktivieren
  }

  // Timer aktivieren
  TIM4->CR1 |= TIM_CR1_CEN;
}

void WRITE_SERVO_PWM(uint8_t port, uint32_t pulselength)
{
  // Pulslänge überprüfen, um zu verhindern, dass die Pulslänge zu niedrig ist
  if (pulselength < 500) pulselength=500; 

  // Pulslänge überprüfen, um zu verhindern, dass die Pulslänge zu groß ist
  if (pulselength > 2500) pulselength=2500; 

  if (port==IOS1)
  {
    TIM4->CCR2 = pulselength;
  }

  if (port==IOS2)
  {
    TIM4->CCR3 = pulselength;
  }

  if (port==IOS3)
  {
    TIM4->CCR1 = pulselength;
  }

  if (port==IOS4)
  {
    TIM4->CCR4 = pulselength;
  }
}

// *** I2C ***

// PA_8 I2C SCL
// PC_9 I2C SDA

TwoWire i2c(PC9, PA8);


// *** I2C BNO055 ***

void WRITE_I2C_BNO055_INIT()
{
  
  do
  {
    delay(10);
    i2c.beginTransmission(0x28);
    i2c.write(0x00);
    i2c.endTransmission(false);
    i2c.requestFrom(0x28, 1, true);
  } while(i2c.read() != 0xA0);
  i2c.beginTransmission(0x28);
  i2c.write(0x3D);
  i2c.write(0x0C);
  i2c.endTransmission();
}

uint16_t READ_I2C_BNO055_YAW()
{
  uint16_t value = 0;
  i2c.beginTransmission(0x28);
  i2c.write(0x1A);  
  i2c.endTransmission(false);
  i2c.requestFrom(0x28, 2, true);
  value = (int16_t)(i2c.read()|i2c.read()<<8 )/16;  
  return value;  
}

int16_t READ_I2C_BNO055_ROLL()
{
  int16_t value = 0;
  i2c.beginTransmission(0x28);
  i2c.write(0x1C);  
  i2c.endTransmission(false);
  i2c.requestFrom(0x28, 2, true);
  value = 65535-(int16_t)(i2c.read()|i2c.read()<<8 )/16;  
  return value;  
}

int16_t READ_I2C_BNO055_PITCH()
{
  int16_t value = 0;
  i2c.beginTransmission(0x28);
  i2c.write(0x1E);  
  i2c.endTransmission(false);
  i2c.requestFrom(0x28, 2, true);
  value = 65535-(int16_t)(i2c.read()|i2c.read()<<8 )/16;  
  return value;  
}


// *** SPI ***

// PA_6 SPI MISO
// PA_7 SPI MOSI
// PA_5 SPI SCK
// PB_1 SPI ADC CS
// PB_12 SPI 1 CS
// PA_11 SPI 2 CS
// PA_12 SPI 3 CS

SPIClass spi(PA7,PA6,PA5);


// *** SPI MCP 3008 AC ***

uint16_t READ_IO_ANALOG(uint8_t sensor)
{
  uint16_t data1;
  uint16_t data2;
  uint16_t value;
  uint8_t spi_address;

  if (sensor==1) spi_address = 0x80;
  if (sensor==2) spi_address = 0x90;
  if (sensor==3) spi_address = 0xA0;
  if (sensor==4) spi_address = 0xB0;
  if (sensor==5) spi_address = 0xC0;
  if (sensor==6) spi_address = 0xD0;
  if (sensor==7) spi_address = 0xE0;
  if (sensor==8) spi_address = 0xF0;
  
  digitalWrite(PB1, LOW);
 
  spi.transfer(0x01);
  delay(1);
  data1=spi.transfer(spi_address);
  delay(1);
  data2=spi.transfer(0x00);
  delay(1);
   digitalWrite(PB1, HIGH);
 
  value =(data1<<8) | data2;
  value = value & 0x03FF; 
//WRITE_LCD_TEXT(5, 2, String(data1));
//WRITE_LCD_TEXT(9, 2, String(data2));
  return value;
}


// *** SPI Linesensor ***

uint16_t READ_SPI_LINE(uint8_t port, uint8_t sensor)
{
  uint16_t data1;
  uint16_t data2;
  uint16_t value;
  uint8_t spi_address;

  if (sensor==1) spi_address = 0x80;
  if (sensor==2) spi_address = 0x90;
  if (sensor==3) spi_address = 0xA0;
  if (sensor==4) spi_address = 0xB0;
  if (sensor==5) spi_address = 0xC0;
  if (sensor==6) spi_address = 0xD0;
  if (sensor==7) spi_address = 0xE0;
  if (sensor==8) spi_address = 0xF0;
  
  if (port==SPI1) digitalWrite(PB12, LOW);
  if (port==SPI2) digitalWrite(PA11, LOW);
  if (port==SPI3) digitalWrite(PA12, LOW);
  spi.transfer(0x01);
  data1=spi.transfer(spi_address);
  data2=spi.transfer(0x00);
  if (port==SPI1) digitalWrite(PB12, HIGH);
  if (port==SPI2) digitalWrite(PA11, HIGH);
  if (port==SPI3) digitalWrite(PA12, HIGH);
  value =(data1<<8) | data2;
  value = value & 0x03FF; 

  return value;
}


// *** SPI CAM Pixy ***

// PB_14 SPI Pixy MISO
// PB_15 SPI Pixy MOSI
// PB_13 SPI Pixy SCK
// PC_6 SPI Pixy CS

SPIClass spi_cam(PB15,PB14,PB13);



// *** Initialisierung ***

void KEPLERBRAIN_INIT()
{ 

  // *** Leds ***

  // PA_4 Led gruen
  // PC_2 Led gelb
  // PC_3 Led rot

  pinMode(PA4, OUTPUT);
  pinMode(PC2, OUTPUT);
  pinMode(PC3, OUTPUT);

  
  // *** Display ***

  // PA_15 Display RS
  // PC_13 Display E
  // PC_11 Display D4
  // PC_10 Display D5
  // PC_12 Display D6
  // PA_1  Display D7

  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print(" KeplerBRAIN V4 ");
  lcd.setCursor(0,1);
  lcd.print(":-) WELLCOME (-:");


  // *** TASTER ***

  // PA_10 Taster 1
  // PB_5  Taster 2
  // PB_4  Taster 3

  pinMode(PA10, INPUT_PULLUP);
  pinMode(PB5, INPUT_PULLUP);
  pinMode(PB4, INPUT_PULLUP);


  // *** PWM Motoren ***

  // PC_7 Motor 1 PWM
  // PC_4 Motor 1 IN1
  // PB_2 Motor 1 IN2

  // PB_10 Motor 2 PWM
  // PC_5  Motor 2 IN1
  // PC_8  Motor 2 IN2

  // PB_0 Motor 3 PWM
  // PH_1 Motor 3 IN1
  // PD_2 Motor 3 IN2

  // PA_0 Motor 4 PWM
  // PC_1 Motor 4 IN1
  // PC_0 Motor 4 IN2

  // GPIOC, GPIOB und GPIOA Takt aktivieren
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
    
  // TIM3 und TIM2 Takt aktivieren
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
    
  // GPIO-Konfiguration für PC7 (TIM3 CH2), PB0 (TIM3 CH3), PA0 (TIM2 CH1), PB10 (TIM2 CH3)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
    
  // PC7 (TIM3 CH2)
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
  // PB0 (TIM3 CH3)
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
  // PA0 (TIM2 CH1)
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
  // PB10 (TIM2 CH3)
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Prescaler und ARR für beide Timer setzen
  // TIM3 für PC7 und PB0
  TIM3->PSC = 15;  // Prescaler: 84 MHz / (15 + 1) = 5.25 MHz
  TIM3->ARR = 1049;  // PWM Periode: 5.25 MHz / 1050 = 5 kHz
    
  // TIM2 für PA0 und PB10
  TIM2->PSC = 15;  // Prescaler: 84 MHz / (15 + 1) = 5.25 MHz
  TIM2->ARR = 1049;  // PWM Periode: 5.25 MHz / 1050 = 5 kHz
    
  // PWM Mode 1 für alle Kanäle setzen
  // TIM3: Kanal 2 (PC7) und Kanal 3 (PB0)
  TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M);
  TIM3->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos); // PWM Mode 1 für CH2
  TIM3->CCMR2 &= ~(TIM_CCMR2_OC3M);
  TIM3->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos); // PWM Mode 1 für CH3
  TIM3->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3E;  // Output Compare aktivieren
    
  // TIM2: Kanal 1 (PA0) und Kanal 3 (PB10)
  TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M);
  TIM2->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos); // PWM Mode 1 für CH1
  TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M);
  TIM2->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos); // PWM Mode 1 für CH3
  TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3E;  // Output Compare aktivieren
    
  // Timer aktivieren
  TIM3->CR1 |= TIM_CR1_CEN;
  TIM2->CR1 |= TIM_CR1_CEN;

  // IN1 und IN2 konfigurieren
    pinMode(PC4,OUTPUT);
  pinMode(PB2,OUTPUT);
  digitalWrite(PC4,LOW);
  digitalWrite(PB2,LOW);

  pinMode(PC5,OUTPUT);
  pinMode(PC8,OUTPUT);
  digitalWrite(PC5,LOW);
  digitalWrite(PC8,LOW);

  pinMode(PH1,OUTPUT);
  pinMode(PD2,OUTPUT);
  digitalWrite(PH1,LOW);
  digitalWrite(PD2,LOW);

  pinMode(PC1,OUTPUT);
  pinMode(PC0,OUTPUT);
  digitalWrite(PC1,LOW);
  digitalWrite(PC0,LOW);


  // *** I2C ***

  i2c.begin(); 


  // *** SPI ***

  // PA_6 SPI MISO
  // PA_7 SPI MOSI
  // PA_5 SPI SCK
  // PB_1 SPI ADC CS
  // PB_12 SPI 1 CS
  // PA_11 SPI 2 CS
  // PA_12 SPI 3 CS

  pinMode(PB1, OUTPUT);
  digitalWrite(PB1, HIGH);
  pinMode(PB12, OUTPUT);
  digitalWrite(PB12, HIGH);
  pinMode(PA11, OUTPUT);
  digitalWrite(PA11, HIGH);
  pinMode(PA12, OUTPUT);
  digitalWrite(PA12, HIGH);
  
  spi.begin();

  spi.setDataMode(SPI_MODE0);
  spi.setBitOrder(MSBFIRST);
  spi.setClockDivider(SPI_CLOCK_DIV16); // Adjust the clock divider as needed

//spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); 
    


 


  // *** SPI CAM Pixy ***

  // PB_14 SPI Pixy MISO
  // PB_15 SPI Pixy MOSI
  // PB_13 SPI Pixy SCK
  // PC_6 SPI Pixy CS

  pinMode(PC6, OUTPUT);
  digitalWrite(PC6, HIGH);
  
  spi_cam.begin();
   // spi_cam.setDataMode(SPI_MODE0);
   // spi_cam.setBitOrder(MSBFIRST);
   // spi_cam.setClockDivider(SPI_CLOCK_DIV16); // Adjust the clock divider as needed
spi_cam.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
 
  /* 
  // Counter
  //timer.attach(counter_count, 0.1);
  
  // LEDs
  led1 = 0;
  led2 = 0;
  led3 = 0;
  
  // Display
  display_init();
  wait_ms(10);
  char textzeile[16];
  sprintf(textzeile," KeplerBRAIN V4 ");
  LCD_DRAW_TEXT(1, 1, textzeile);
  sprintf(textzeile,":-) WELLCOME (-:");
  LCD_DRAW_TEXT(1, 2, textzeile);

  // Motoren
  m1_pwm.period(0.0002);
  m2_pwm.period(0.0002);
  m3_pwm.period(0.0002);
  m4_pwm.period(0.0002);
    
  // SPI
  spi.format(8,0); 
  spi.frequency(1000000);
  spi_adc_cs = 1;
  spi_1_cs = 1;
  spi_2_cs = 1;
  spi_3_cs = 1;
  
  // SPI Pixy
  spi_pixy.format(8,3); 
  spi_pixy.frequency(2000000);
  spi_pixy_cs = 1;
  
  // UART PC
  UART.baud(19200);
    */
}