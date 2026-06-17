#ifndef KEPLERUPDATZE_H
#define KEPLERUPDATZE_H

#include "mbed.h"

// -------- Counter variables for KeplerUpdate.cpp --------
extern int counter; //globaler counter 
extern int counter_now_power; // now power counter variable um power metering alle 100 runs zu ermöglichen 
extern int counter_BT_check; //BT check counter variable um BT connection check alle 500 runs zu ermöglichen


// -------- Power sense variables (defined in PowerSense.cpp) --------
extern volatile float ina231_shunt_voltage;  // mV
extern volatile float ina231_bus_voltage;    // V
extern volatile float ina231_current;        // A
extern volatile float ina231_power;          // W
extern volatile uint32_t ina231_error_count;    // error count variable für debug zwecke

void KEPLER_UPDATE();

#endif