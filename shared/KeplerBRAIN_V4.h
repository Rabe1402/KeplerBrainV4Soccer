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


// SRF10 Ultraschall

// void WRITE_SRF10_INIT()
//   Setzt Gain + Range auf allen 4 Sensoren (SRF10_GAIN / SRF10_RANGE anpassen)

// int READ_SRF10(byte address)
//   address: SRF10_S1 .. SRF10_S4
//   value: Distanz in cm, -1 bei Fehler/Ausreisser (>255 cm wird gefiltert)

// Hardcoded Adressen – mit der SRF10-Testbench (SRF10+ADD) konfigurieren
// #define SRF10_S1  0x70
// #define SRF10_S2  0x71
// #define SRF10_S3  0x72
// #define SRF10_S4  0x73

// Einstellbare Parameter (vor WRITE_SRF10_INIT() ändern):
// byte SRF10_GAIN  = 10;   // 0–16
// byte SRF10_RANGE = 23;   // (val+1)*43mm → Wert 23 = ~103 cm


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
uint8_t READ_IOS_HIGH(uint8_t port)
{
  if (port==IOS1)
  {
    pinMode(PB6, INPUT_PULLDOWN);
    if (digitalRead(PB6)==1) return 1;
    else return 0;
  }
  if (port==IOS2)
  {
    pinMode(PB8, INPUT_PULLDOWN);
    if (digitalRead(PB8)==1) return 1;
    else return 0;
  }
  if (port==IOS3)
  {
    pinMode(PB6, INPUT_PULLDOWN);
    if (digitalRead(PB6)==1) return 1;
    else return 0;
  }
  if (port==IOS4)
  {
    pinMode(PB9, INPUT_PULLDOWN);
    if (digitalRead(PB9)==1) return 1;
    else return 0;
  }
}

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
    pinMode(PB8, INPUT_PULLUP);
    if (digitalRead(PB8)==0) return 1;
    else return 0;
  }
  if (port==IOS3)
  {
    pinMode(PB6, INPUT_PULLUP);
    if (digitalRead(PB6)==0) return 1;
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
        if (!digitalRead(PB8))
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
        if (!digitalRead(PB6))
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


// =====================
// KICKER
// =====================
#ifdef USE_KICKER

// PB_6  Digital In Out  IOS1 = PBsiben
// PB_7  Digital In Out  IOS2 = PBacht
// PB_8  Digital In Out  IOS3 = PBsechs
// PB_9  Digital In Out  IOS4 = PBneuhn
int aktiv_kicker_port; //merke kicker um im interrupt den richtigen auszuschalten

void WRITE_KICKER_INIT(uint8_t port) //init kicker pin unf timer, für exatkte kick dauer ohne den main code zu stoppen
{
    #ifdef log 
    _log("KeplerBrain", "KickerINIT", "Initializing kicker on port " + String(port));
    #endif
    __HAL_RCC_TIM4_CLK_ENABLE(); //aktivate timer4 um auf APB1 zu sein (84MHz statt 168Mhz)
    TIM4->PSC = 83; //Prescaler auf 83 (84MHz / (83 + 1) = 1MHz) )
    TIM4->ARR = 999; //timer zählt bis 999+1 also 1ms (1MHz / 1000 = 1ms) 
    TIM4->EGR = TIM_EGR_UG; //Update Generation um die neuen Werte zu laden
    TIM4->DIER |= TIM_DIER_UIE;  // interupt wird ausgelöst wenn timer überlauft, also nach 1ms
    HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0); //set priority 3 es soll vor allem kein SPI oder so zerstören. nach dem es sich da um wenige mikrosek handelt kann der kicker auch bissi länger an sein.
    HAL_NVIC_EnableIRQ(TIM4_IRQn);  // timer aktivieren 

    switch (port)
    {
        case IOS1:
            pinMode(PB7, OUTPUT);   //PB6 als Ausgang für den Kicker konfigurieren
            digitalWrite(PB6, LOW); //sicher stellen, dass der kicker aus is
            aktiv_kicker_port = 1; //merke kicker um im interrupt den richtigen auszuschalten und WRITE_KCIKER 
            #ifdef log 
            _log("KeplerBrain", "KickerINIT", "Kicker initialized on port IOS1 (PB6)");
            #endif
            break;

        case IOS2:
            pinMode(PB8, OUTPUT);   //PB6 als Ausgang für den Kicker konfigurieren
            digitalWrite(PB8, LOW); //sicher stellen, dass der kicker aus is
            aktiv_kicker_port = 2; //merke kicker um im interrupt den richtigen auszuschalten und WRITE_KCIKER
            #ifdef log 
            _log("KeplerBrain", "KickerINIT", "Kicker initialized on port IOS2 (PB8)");
            #endif
            break;

        case IOS3:
            pinMode(PB6, OUTPUT);   //PB6 als Ausgang für den Kicker konfigurieren
            digitalWrite(PB6, LOW); //sicher stellen, dass der kicker aus is
            aktiv_kicker_port = 3; //merke kicker um im interrupt den richtigen auszuschalten und WRITE_KCIKER
            #ifdef log 
            _log("KeplerBrain", "KickerINIT", "Kicker initialized on port IOS3 (PB6)");
            #endif
            break;

        case IOS4:
            pinMode(PB9, OUTPUT);   //PB6 als Ausgang für den Kicker konfigurieren
            digitalWrite(PB9, LOW); //sicher stellen, dass der kicker aus is
            aktiv_kicker_port = 4; //merke kicker um im interrupt den richtigen auszuschalten und WRITE_KCIKER
            #ifdef log 
            _log("KeplerBrain", "KickerINIT", "Kicker initialized on port IOS4 (PB9)");
            #endif
            break;

    }

}

void WRITE_KICKER(uint8_t port, uint16_t duration_ms) //kickt für die angegebene dauer in ms, ohne den main code zu blockieren
{
    #ifdef log 
    _log("KeplerBrain", "Kicker", "Activating kicker on port " + String(port) + " for " + String(duration_ms) + " ms");
    #endif

    TIM4->ARR = duration_ms * 1000 -1; //timer zählt bis dahin mit 1Mhz also 1ms = 1000 counts, 10ms = 10000 counts usw. -1 weil der timer von 0 zählt
    TIM4->EGR = TIM_EGR_UG; //Update Generation um die neuen Werte sofort zu laden    
    TIM4->CNT = 0; //Timer zurücksetzen

    switch (port)  // direkt port verwenden
    {
    case IOS1: digitalWrite(PB7, HIGH); break;
    case IOS2: digitalWrite(PB8, HIGH); break;
    case IOS3: digitalWrite(PB6, HIGH); break;
    case IOS4: digitalWrite(PB9, HIGH); break;
    }

    
    TIM4->CR1 |= TIM_CR1_CEN; //counter aktivieren (CEN = Counter Enable)

}

extern "C" void TIM4_IRQHandler() //wenn TIM4 überläuft, also nach duration_ms, kommt dieser interrupt
{
    _log("KeplerBrain", "KickerInterrupt", "aktiv_kicker_port: " + String(aktiv_kicker_port));
    if (TIM4->SR & TIM_SR_UIF) //überprüfen, ob der Update Interrupt Flag gesetzt ist
    {
        TIM4->SR &= ~TIM_SR_UIF; //Interrupt Flag zurücksetzen

        switch (aktiv_kicker_port) 
        {
        case 1:
            digitalWrite(PB7, LOW); //Kicker aus
            
            break;

        case 2:
            digitalWrite(PB8, LOW); //Kicker aus
            
            break;

        case 3:
            digitalWrite(PB6, LOW); //Kicker aus
            
            break;

        case 4:
            digitalWrite(PB9, LOW); //Kicker aus
            
            break;

        }

        
        TIM4->CR1 &= ~TIM_CR1_CEN;  // Timer stoppen

    }
}


#endif // USE_KICKER


// =====================
// SERVO
// =====================
#ifdef USE_SERVO

void WRITE_SERVO_INIT(uint8_t port)
{
  // GPIOB Takt aktivieren
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  // TIM4 Takt aktivieren
  __HAL_RCC_TIM4_CLK_ENABLE();

  // GPIO-Konfiguration für PB6 (TIM4 CH1), PB8 (TIM4 CH2), PB6 (TIM4 CH3), PB9 (TIM4 CH4)
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (port==IOS1)
  {
    // PB8 (TIM4 CH2)
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }

  if (port==IOS2)
  {
    // PB6 (TIM4 CH3)
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
    // TIM4: Kanal 2 (PB8)
    TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M);
    TIM4->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos); // PWM Mode 1 für CH2
    TIM4->CCER |= TIM_CCER_CC2E;  // Output Compare aktivieren
  }

  if (port==IOS2)
  {
    // TIM4: Kanal 3 (PB6)
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

#endif // USE_SERVO


// *** I2C ***

// PA_8 I2C SCL
// PC_9 I2C SDA

TwoWire i2c(PC9, PA8);


// *** I2C BNO055 ***

void WRITE_I2C_BNO055_INIT()
{
  // ------ Warten bis Chip bereit
  do {
    delay(10);
    i2c.beginTransmission(0x28);
    i2c.write(0x00);
    i2c.endTransmission(false);
    i2c.requestFrom(0x28, 1, true);
  } while(i2c.read() != 0xA0); //read first register till it shows 0xA0
  
  delay(50); //wait to let sensor finish starup 
  
  // ------ IMU Mode (nur Accel + Gyro, KEIN Magnetometer!)
  i2c.beginTransmission(0x28);
  i2c.write(0x3D);
  i2c.write(0x08);  // IMU Mode statt 0x0C (only ACC+GYRO) instead of (MAG+ACC+GYRO)
  i2c.endTransmission(); 
  delay(20); //wait till it switch out of config mode 

  // ------- do not proceed if IMU is not calibrated. May add funktion to show when it is stuck here...
  uint8_t status;
  do {
    delay(10);
    i2c.beginTransmission(0x28);
    i2c.write(0x35);
    i2c.endTransmission(false);
    i2c.requestFrom(0x28, 1, true);
    status = i2c.read();

  } while((status & 0x30) != 0x30);
}

// Gyro-Kalibrierung checken. 
// should not be neccesary anymore if the code blocke above works as expected ... 
bool READ_I2C_IS_BNO055_READY()
{
  i2c.beginTransmission(0x28);
  i2c.write(0x35);
  i2c.endTransmission(false);
  i2c.requestFrom(0x28, 1, true);
  uint8_t status = i2c.read();
  
  // Nur Gyro prüfen (Bit 5-4 müssen = 3 sein)
  return ((status & 0x30) == 0x30);
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


uint16_t READ_I2C_BNO055_ACC_X() 
{
  int16_t value = 0;
  i2c.beginTransmission(0x28);
  i2c.write(0x08);  
  i2c.endTransmission(false);
  i2c.requestFrom(0x28, 2, true);
  value = (int16_t)(i2c.read()|i2c.read()<<8 )/100;  
  return value;  
}

int16_t READ_I2C_BNO055_ACC_Y() 
{
  int16_t value = 0;
  i2c.beginTransmission(0x28);
  i2c.write(0x0A);  
  i2c.endTransmission(false);
  i2c.requestFrom(0x28, 2, true);
  value = (int16_t)(i2c.read()|i2c.read()<<8 )/100;  
  return value;  
}

int16_t READ_I2C_BNO055_ACC_Z() 
{
  int16_t value = 0;
  i2c.beginTransmission(0x28);
  i2c.write(0x0C);  
  i2c.endTransmission(false);
  i2c.requestFrom(0x28, 2, true);
  value = (int16_t)(i2c.read()|i2c.read()<<8 )/100;  
  return value;  
}


// *** SRF10 Ultraschall ***

// PA_8 I2C SCL  (i2c-Objekt oben)
// PC_9 I2C SDA

// Hardcoded Adressen – einmalig mit SRF10-Testbench (SRF10+ADD) konfigurieren
#define SRF10_S1  0x70
#define SRF10_S2  0x71
#define SRF10_S3  0x72
#define SRF10_S4  0x73

// Einstellbare Parameter – vor WRITE_SRF10_INIT() ändern falls nötig
// Gain: 0–16  (höher = empfindlicher, mehr Fehlmessungen bei nahen Objekten)
// Range: (val+1)*43mm → 23 = ~103 cm, 46 = ~204 cm, 255 = ~11 m
byte SRF10_GAIN  = 10;
byte SRF10_RANGE = 23;

#define _SRF10_CMD_REG   0x00
#define _SRF10_GAIN_REG  0x01
#define _SRF10_RANGE_REG 0x02
#define _SRF10_RES_REG   0x03
#define _SRF10_POLL_MS   65    // SRF10 braucht max ~65ms pro Messung
#define _SRF10_MAX_CM    255   // Ausreisser-Filter: Werte > 255 cm → -1

// Intern: warten bis Sensor fertig (gibt false bei Timeout zurück)
bool _SRF10_WAIT_READY(byte address)
{
  unsigned long start = millis();
  while (millis() - start < _SRF10_POLL_MS)
  {
    i2c.beginTransmission(address);
    i2c.write(_SRF10_CMD_REG);
    i2c.endTransmission();
    i2c.requestFrom(address, (byte)1);
    if (i2c.available())
    {
      if (i2c.read() != 0xFF) return true;
    }
    delay(5);
  }
  return false;
}

// Gain setzen (0–16)
void WRITE_SRF10_GAIN(byte address, byte gain)
{
  i2c.beginTransmission(address);
  i2c.write(_SRF10_GAIN_REG);
  i2c.write(gain);
  i2c.endTransmission();
}

// Range setzen – Formel: max_mm = (val+1)*43
// Beispiele: 0=43mm, 23≈103cm, 46≈204cm, 255≈11m
void WRITE_SRF10_RANGE(byte address, byte rangeVal)
{
  i2c.beginTransmission(address);
  i2c.write(_SRF10_RANGE_REG);
  i2c.write(rangeVal);
  i2c.endTransmission();
}

// Alle 4 Sensoren mit SRF10_GAIN und SRF10_RANGE initialisieren
// In KEPLERBRAIN_INIT() aufgerufen, kann aber auch manuell wiederholt werden
void WRITE_SRF10_INIT()
{
  const byte addrs[4] = { SRF10_S1, SRF10_S2, SRF10_S3, SRF10_S4 };
  for (int i = 0; i < 4; i++)
  {
    WRITE_SRF10_GAIN(addrs[i],  SRF10_GAIN);
    WRITE_SRF10_RANGE(addrs[i], SRF10_RANGE);
  }
}

// Distanz lesen – gibt cm zurück, -1 bei Timeout oder Ausreisser (>255 cm)
// address: SRF10_S1 .. SRF10_S4
int READ_SRF10(byte address)
{
  int dist = -1;
  // Messung starten (0x51 = cm, 0x50 = inch, 0x52 = µs)
  i2c.beginTransmission(address);
  i2c.write(_SRF10_CMD_REG);
  i2c.endTransmission();

  i2c.requestFrom(112, 1);

  while (i2c.available()) 
  {
    dist = i2c.read(); // evtl. alte Daten löschen
  }

  if (dist!=0xFF)
  {
    i2c.beginTransmission(address);
    i2c.write(_SRF10_RES_REG);
    i2c.endTransmission();

    i2c.requestFrom(address, 1);

    while (i2c.available())
    { 
      dist = i2c.read();
    
    }
    i2c.beginTransmission(address);
    i2c.write(_SRF10_CMD_REG);
    i2c.write(0x51); // Messung starten (0x51 = cm, 0x50 = inch, 0x52 = µs)
    i2c.endTransmission();
  }
  return dist;
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

  //lcd.begin(16, 2);
  //lcd.setCursor(0,0);
  //lcd.print(" KeplerBRAIN V4 ");
  //lcd.setCursor(0,1);
  //lcd.print(":-) WELLCOME (-:");


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

  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print(" KeplerBRAIN V4 ");
  lcd.setCursor(0,1);
  lcd.print(":-) WELLCOME (-:");

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

  // *** SPI CAM Pixy ***

  // PB_14 SPI Pixy MISO
  // PB_15 SPI Pixy MOSI
  // PB_13 SPI Pixy SCK
  // PC_6 SPI Pixy CS

  pinMode(PC6, OUTPUT);
  digitalWrite(PC6, HIGH);
  
  spi_cam.begin();
  spi_cam.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
}