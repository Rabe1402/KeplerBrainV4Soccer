#include <INA.h>

/**************************************************************************************************
** Declare program constants, global variables and instantiate INA class                         **
**************************************************************************************************/
const uint32_t SERIAL_SPEED{115200};     ///< Use fast serial speed
const uint32_t SHUNT_MICRO_OHM{100000};  ///< Shunt resistance in Micro-Ohm, e.g. 100000 is 0.1 Ohm
const uint16_t MAXIMUM_AMPS{2};          ///< Max expected amps, clamped from 1A to a max of 1022A
uint8_t        devicesFound{0};          ///< Number of INAs found
INA_Class      INA;                      ///< INA class instantiation to use EEPROM
// INA_Class      INA(0);                ///< INA class instantiation to use EEPROM
// INA_Class      INA(5);                ///< INA class instantiation to use dynamic memory rather
//                                            than EEPROM. Allocate storage for up to (n) devices

void WRITE_I2C_INA231_INIT() {
 
  /************************************************************************************************
  ** The INA.begin call initializes the device(s) found with an expected ±1 Amps maximum current **
  ** and for a 0.1Ohm resistor, and since no specific device is given as the 3rd parameter all   **
  ** devices are initially set to these values.                                                  **
  ************************************************************************************************/
  devicesFound = INA.begin(MAXIMUM_AMPS, SHUNT_MICRO_OHM);  // Expected max Amp & shunt resistance
  while (devicesFound == 0) {
    Serial.println(F("No INA device found, retrying in 10 seconds..."));
    delay(10000);                                             // Wait 10 seconds before retrying
    devicesFound = INA.begin(MAXIMUM_AMPS, SHUNT_MICRO_OHM);  // Expected max Amp & shunt resistance
  }                                                           // while no devices detected
  Serial.print(F(" - Detected "));
  Serial.print(devicesFound);
  Serial.println(F(" INA devices on the I2C bus"));
  INA.setBusConversion(8500);             // Maximum conversion time 8.244ms
  INA.setShuntConversion(8500);           // Maximum conversion time 8.244ms
  INA.setAveraging(128);                  // Average each reading n-times
  INA.setMode(INA_MODE_CONTINUOUS_BOTH);  // Bus/shunt measured continuously
}  // method setup()

uint16_t READ_I2C_INA231_VOLTAGE() 
{
  for (uint8_t i = 0; i < devicesFound; i++)
  return INA.getBusMilliVolts(i);
}

int32_t READ_I2C_INA231_AMPS()
{
  for (uint8_t i = 0; i < devicesFound; i++)
  return INA.getBusMicroAmps(i);
}

int16_t READ_I2C_INA231_WATT()
{
  for (uint8_t i = 0; i < devicesFound; i++)
  return INA.getBusMicroWatts(i);
}