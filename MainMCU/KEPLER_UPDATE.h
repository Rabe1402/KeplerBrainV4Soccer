int counter; //globaler counter 

//pwoer sense variablen
uint16_t BatV;
int16_t BatA;

void KEPLER_UPDATE();
{
  counter ++ 1;  // kepler update counter um stetige messungen wie powersens zu ermöglichen 
  // power redaing und led writing alle 1000 mal 
  if (counter = counter_now_power + 1000) 
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
  -----------------------------------------------------------------------------------------------------
    BatV = ina231_bus_voltage;
    BatA = ina231_current;

    //seriel schreiben der werte 
    Serial.print("BatVolatge: ");
    Serial.print(BatV);
    Serial.print("    "); //bissi platz das net alles so aufeinander bickt 
    Serial.print("BatAmp: ");
    Serial.print(BatA); 
    Serial.print("    "); //bissi platz das net alles so aufeinander bickt 
    Serial.print(ina231_error_count);
    Serial.println(); //nur für neue zeile so , dass der code gut ausschaut
    SLEEP(100);

    //led steuerung
    if (BatV > 12000)
    {
    WRITE_LED(L1,0);
    WRITE_LED(L2,0);
    WRITE_LED(L3,1);
    }
    if (BatV > 9000 && BatV < 11000)
    {
      WRITE_LED(L1,1); //Wenn batterie über 9V und unter 11V dann Rote led 1 
      WRITE_LED(L2,0);
    }
    if (BatV > 11000 && BatV < 12000)
    { 
      WRITE_LED(L1, 0);
      WRITE_LED(L2, 1);
      WRITE_LED(L3, 0);
    }
    while (BatV < 9000)
    {
      WRITE_LED(L1,1);
      SLEEP(300);      //Fick alles wenn die baterie zu leer ist
      WRITE_LED(L1,0);
      SLEEP(399);
    }

    counter_now_power = counter;  //counter schaltung 

  }
}