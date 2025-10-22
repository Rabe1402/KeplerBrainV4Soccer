int counter; //globaler counter 
int counter_now_power; // now power counter variable um power metering alle 100 runs zu ermöglichen 

//pwoer sense variablen
static volatile float ina231_shunt_voltage = 0.0f;  // mV
static volatile float ina231_bus_voltage = 0.0f;    // V
static volatile float ina231_current = 0.0f;        // A
static volatile float ina231_power = 0.0f;          // W
static volatile uint32_t ina231_error_count = 0;    // error count variable für debug zwecke 

void KEPLER_UPDATE()
{
  counter++;  // kepler update counter um stetige messungen wie powersens zu ermöglichen 
  // power redaing und led writing alle 1000 mal 
  if (counter = counter_now_power + 1000) 
  {
    #ifdef INA231_DEBUG
    Serial.println("Updating INA231 measurements...");
    #endif

    // Shunt-Spannung
    i2c.beginTransmission(0x40);
    i2c.write(0x01);
    if (i2c.endTransmission(false) == 0) {
      i2c.requestFrom(0x40, 2, true);
      if (i2c.available() >= 2) {
        int16_t raw = (int16_t)((i2c.read() << 8) | i2c.read());
        ina231_shunt_voltage = raw * 0.0025f;  // in mV
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
    i2c.beginTransmission(0x40);
    i2c.write(0x02);
    if (i2c.endTransmission(false) == 0) {
      i2c.requestFrom(0x40, 2, true);
      if (i2c.available() >= 2) {
        uint8_t msb = i2c.read();
        uint8_t lsb = i2c.read();
        uint16_t raw = (msb << 8) | lsb;  // Big-Endian
        ina231_bus_voltage = raw * 1.25f;  // in V
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
    i2c.beginTransmission(0x40);
    i2c.write(0x04);
    if (i2c.endTransmission(false) == 0) {
      i2c.requestFrom(0x40, 2, true);
      if (i2c.available() >= 2) {
        int16_t raw = (int16_t)((i2c.read() << 8) | i2c.read());
        ina231_current = raw * (5.0f / 32768.0f * 1000);  // in A
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
    i2c.beginTransmission(0x40);
    i2c.write(0x03);
    if (i2c.endTransmission(false) == 0) {
      i2c.requestFrom(0x40, 2, true);
      if (i2c.available() >= 2) {
        uint16_t raw = (i2c.read() << 8) | i2c.read();
        ina231_power = raw * (25.0f * (5.0f / 32768.0f * 10000));  // in W
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

    //seriel schreiben der werte 
    Serial.print("BatVolatge: ");
    Serial.print(ina231_bus_voltage);
    Serial.print(" mV");
    Serial.print("    "); //bissi platz das net alles so aufeinander bickt 
    Serial.print("BatAmp: ");
    Serial.print(ina231_current); 
    Serial.print(" mA");
    Serial.print("    "); //bissi platz das net alles so aufeinander bickt 
    Serial.print(ina231_error_count);
    Serial.println(); //nur für neue zeile so , dass der code gut ausschaut
    SLEEP(100);

    //led steuerung
    if (ina231_bus_voltage > 12000)
    {
    WRITE_LED(L1,0);
    WRITE_LED(L2,0);
    WRITE_LED(L3,1);
    }
    if (ina231_bus_voltage > 9000 && ina231_bus_voltage < 11000)
    {
      WRITE_LED(L1,1); //Wenn batterie über 9V und unter 11V dann Rote led 1 
      WRITE_LED(L2,0);
    }
    if (ina231_bus_voltage > 11000 && ina231_bus_voltage < 12000)
    { 
      WRITE_LED(L1, 0);
      WRITE_LED(L2, 1);
      WRITE_LED(L3, 0);
    }
    while (ina231_bus_voltage < 9000)
    {
      WRITE_LED(L1,1);
      SLEEP(300);      //Fick alles wenn die baterie zu leer ist
      WRITE_LED(L1,0);
      SLEEP(399);
    }

    counter_now_power = counter;  //counter schaltung 
  }
} 


