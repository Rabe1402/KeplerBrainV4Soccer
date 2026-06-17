#ifndef LOGGING_H
#define LOGGING_H

#include "mbed.h"
#include <vector>
#include <string>
#include <cstdint>

// ============================================================
// LOGGING SYSTEM - Centralized logging for all modules
// ============================================================

// Debug configuration variables
extern bool debug;
extern bool debug_over_serial;
extern bool debug_over_BT;
extern bool debug_push_log;
extern std::vector<std::string> debug_log;

// Main logging function
void _log(const std::string& name, const std::string& component, 
          const std::string& message, bool error = false);

// Temporary debug logging (directly to serial)
void _log_tmp(const std::string& msg);

// Test logging
void _lt(void);

// Dump all log entries
void _log_Dump(void);

#endif // LOGGING_H
