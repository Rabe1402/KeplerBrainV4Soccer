#include "KeplerUpdate.h"
#include "KeplerBrainV5.h"
#include <cstdio>
#include <string>

// -------- Counter variables for KeplerUpdate.cpp --------
int counter; //globaler counter 
int counter_now_power; // now power counter variable um power metering alle 100 runs zu ermöglichen 
int counter_BT_check; //BT check counter variable um BT connection check alle 500 runs zu ermöglichen

// -------- External Power sense variables (defined in PowerSense.cpp) --------
extern volatile float ina231_shunt_voltage;  // mV
extern volatile float ina231_bus_voltage;    // V
extern volatile float ina231_current;        // A
extern volatile float ina231_power;          // W
extern volatile uint32_t ina231_error_count; // error count variable für debug zwecke

// -------- External global variables --------
extern I2C g_i2c;  // mbed I2C object from KeplerBrainV5.cpp
extern bool is_bt_connected;



void KEPLER_UPDATE()
{
  counter++;  // kepler update counter um stetige messungen wie powersens zu ermöglichen 
  if (counter > 10000) //counter reset alle 10k runs um overflow zu vermeiden 
  {
    counter -= 10000;
    counter_now_power = 0; //reset power counter damit auch alle 100 runs im ersten teil des programms gemessen wird 
    _log("KEPLER_UPDATE", "COUNTER", "Counter reset to avoid overflow");
  }

  if (counter >= counter_BT_check + 500) //BT connection check alle 500 runs
  {
    is_bt_connected = READ_IOS_PRESSED(IOS1); //check if BT is connected at start, needs to be updated when BT connection changes.
    char log_msg[64];
    snprintf(log_msg, sizeof(log_msg), "Bluetooth connection status: %d", is_bt_connected);
    _log("KEPLER_UPDATE", "BT", std::string(log_msg));
    counter_BT_check = counter; //reset BT check counter
  }

  if (READ_BUTTON_CLOSED(B1) == 1)
  {
    glob_exit_override = true;
    _log("MAIN" , "INTERUPT", "Run set to false by button press, entering menu mode");

    sleep(300);
  }

  // power reading und led writing alle 100 mal und jedes mal in den ersten 100 runs
  if (counter >= counter_now_power + 100 || counter < 100) 
  {
    #ifdef INA231_DEBUG
    printf("Updating INA231 measurements...\n");
    #endif

    // ===== Shunt-Spannung (Register 0x01) =====
    {
      char cmd = 0x01;
      char buf[2];
      if (g_i2c.write(0x40 << 1, &cmd, 1, true) == 0) {  // write with I2C restart
        if (g_i2c.read(0x40 << 1, buf, 2) == 0) {
          int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
          ina231_shunt_voltage = raw * 0.0025f;  // in mV
          #ifdef INA231_DEBUG
          printf("Shunt Voltage raw: %d, mV: %.3f\n", raw, ina231_shunt_voltage);
          #endif
        } else {
          ina231_error_count++;
        }
      } else {
        ina231_error_count++;
      }
    }

    // ===== Bus-Spannung (Register 0x02) =====
    {
      char cmd = 0x02;
      char buf[2];
      if (g_i2c.write(0x40 << 1, &cmd, 1, true) == 0) {  // write with I2C restart
        if (g_i2c.read(0x40 << 1, buf, 2) == 0) {
          uint8_t msb = buf[0];
          uint8_t lsb = buf[1];
          uint16_t raw = (msb << 8) | lsb;  // Big-Endian
          ina231_bus_voltage = raw * 1.25f;  // in V
          #ifdef INA231_DEBUG
          printf("Bus Voltage raw: %u, V: %.3f\n", raw, ina231_bus_voltage);
          #endif
        } else {
          ina231_error_count++;
        }
      } else {
        ina231_error_count++;
      }
    }

    // ===== Strom (Register 0x04) =====
    {
      char cmd = 0x04;
      char buf[2];
      if (g_i2c.write(0x40 << 1, &cmd, 1, true) == 0) {  // write with I2C restart
        if (g_i2c.read(0x40 << 1, buf, 2) == 0) {
          int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
          ina231_current = raw * (5.0f / 32768.0f * 1000);  // in A
          #ifdef INA231_DEBUG
          printf("Current raw: %d, A: %.3f\n", raw, ina231_current);
          #endif
        } else {
          ina231_error_count++;
        }
      } else {
        ina231_error_count++;
      }
    }

    // ===== Leistung (Register 0x03) =====
    {
      char cmd = 0x03;
      char buf[2];
      if (g_i2c.write(0x40 << 1, &cmd, 1, true) == 0) {  // write with I2C restart
        if (g_i2c.read(0x40 << 1, buf, 2) == 0) {
          uint16_t raw = (buf[0] << 8) | buf[1];
          ina231_power = raw * (25.0f * (5.0f / 32768.0f * 10000));  // in W
          #ifdef INA231_DEBUG
          printf("Power raw: %u, W: %.3f\n", raw, ina231_power);
          #endif
        } else {
          ina231_error_count++;
        }
      } else {
        ina231_error_count++;
      }
    }

    #ifdef INA231_DEBUG
    printf("Error count: %lu\n", ina231_error_count);
    printf("INA231 measurements updated\n");
    printf("BatVoltage: %.3f V    BatAmp: %.3f mA    Errors: %lu\n", 
           ina231_bus_voltage, ina231_current, ina231_error_count);
    #endif

    //led steuerung
    if (ina231_bus_voltage > 12000)
    {
      WRITE_LED(L1, 0);
      WRITE_LED(L2, 0);
      WRITE_LED(L3, 1);
    }
    if (ina231_bus_voltage > 9000 && ina231_bus_voltage < 11000)
    {
      WRITE_LED(L1, 1); //Wenn batterie über 9V und unter 11V dann Rote led 1 
      WRITE_LED(L2, 0);
    }
    if (ina231_bus_voltage > 11000 && ina231_bus_voltage < 12000)
    { 
      WRITE_LED(L1, 0);
      WRITE_LED(L2, 1);
      WRITE_LED(L3, 0);
    }
    while (ina231_bus_voltage > 500 && ina231_bus_voltage < 9000)
    {
      WRITE_LED(L1, 1);
      sleep(300);  // Wenn die batterie zu leer ist
      WRITE_LED(L1, 0);
      sleep(399);
    }
    if (ina231_bus_voltage < 500) //zeige das der ina ein problem hat und keine spannung lesen kann
    {
      WRITE_LED(L1, 1);
      WRITE_LED(L2, 1);
      WRITE_LED(L3, 1);
    }
    //POWER READ END
    
    if (ina231_bus_voltage > 500)
    {
      char log_msg[256];
      snprintf(log_msg, sizeof(log_msg), 
               "Readings: %.3fV, %.3f mA, %.3f W, errors: %lu",
               ina231_bus_voltage / 1000.0f, 
               ina231_current, 
               ina231_power / 10000.0f, 
               ina231_error_count);
      _log("KEPLER_UPDATE", "POWER", std::string(log_msg));
    } else {
      char log_msg[128];
      snprintf(log_msg, sizeof(log_msg), "Unable to get Power data... INA231 Errors: %lu", ina231_error_count);
      _log("KEPLER_UPDATE", "POWER", std::string(log_msg), true);
    }

    counter_now_power = counter;  //counter schaltung 
  }
}
