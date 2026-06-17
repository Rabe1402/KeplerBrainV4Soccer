#include "Logging.h"
#include <cstdio>
#include <cstring>

// ============================================================
// LOGGING VARIABLES
// ============================================================
std::vector<std::string> debug_log;
bool debug = true;
bool debug_over_serial = true;
bool debug_over_BT = false;
bool debug_push_log = true;

// Forward declarations (need to be included from Variables.h at link time)
extern bool is_bt_connected;
extern BufferedSerial bt_serial;

// ============================================================
// MAIN LOGGING FUNCTION
// ============================================================
void _log(const std::string& name, const std::string& component, 
          const std::string& message, bool error)
{
  if (debug) {
    // Format: timestamp . name . component . message . error
    char log_entry[256];
    uint32_t timestamp = Kernel::get_ms_count();
    snprintf(log_entry, sizeof(log_entry), "%lu . %s . %s . %s . %s",
             timestamp, name.c_str(), component.c_str(), message.c_str(),
             error ? "ERROR" : "OK");
    
    std::string log_str(log_entry);

    // Output to serial console
    if (debug_over_serial) {
      printf("%s\n", log_str.c_str());
    }

    // Buffer in memory
    if (debug_push_log) {
      debug_log.push_back(log_str);
    }

    // Send over Bluetooth if connected
    if (debug_over_BT && is_bt_connected) {
      bt_serial.write((const uint8_t*)log_str.c_str(), log_str.length());
      bt_serial.write((const uint8_t*)"\n", 1);
    }
  }
}

// ============================================================
// TEMPORARY LOGGING - Direct serial output
// ============================================================
void _log_tmp(const std::string& msg)
{
  printf("%s\n", msg.c_str());
}

// ============================================================
// TEST LOGGING
// ============================================================
void _lt(void)
{
  printf("test\n");
}

// ============================================================
// LOG DUMP - Print all buffered log entries
// ============================================================
void _log_Dump(void)
{
  printf("========================================\n");
  printf("--- Log Dump Start ---\n");
  printf("Total log entries: %zu\n", debug_log.size());
  printf("========================================\n");
  
  for (size_t i = 0; i < debug_log.size(); i++) {
    printf("[%zu] %s\n", i, debug_log[i].c_str());
  }
  
  printf("========================================\n");
  printf("--- Log Dump End ---\n");
  printf("========================================\n");
}
