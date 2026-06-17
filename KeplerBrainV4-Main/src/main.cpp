#include "mbed.h"
#include <vector>
#include <cstdio>
#include <cstring>
#include <string>
#include <cmath>

// ============================================================
// INCLUDE ALL LIBRARIES
// ============================================================
#include "KeplerBrainV5.h"      // Main hardware abstraction
#include "DataCalculation.h"     // Data & sensor calculations
#include "Move.h"                // Motor movement control
#include "KeplerUpdate.h"        // Background updates
#include "PowerSense.h"          // Power monitoring
#include "Logging.h"             // Logging system
#include "Variables.h"           // Global variables
#include "Tests.h"               // Test functions

// ============================================================
// RCJ MODULE - Robot Competition Japan module
// ============================================================
bool RCJ_MODULE(int mode, const std::string& comms ="")
{
  switch (mode) {
    case 0:  // Check IO status
      return (READ_IOS_PRESSED(IOS2) == 1) ? true : false;

    case 1:  // Robot-to-Robot communications
      _log("RCJ_MODULE", "RtoR_comms", "NOT IMPLEMENTED YET! DONT USE.", true);
      return false;

    default:
      return false;
  }
}

// ============================================================
// MENU SYSTEM
// ============================================================
extern int selection;

int Menu(std::string items[])
{
  bool run = false;

  WRITE_LCD_TEXT(1, 1, "|");
  WRITE_LCD_TEXT(1, 2, "|");

  while (!run) {
    // Check RCJ selector
    if (RCJ_MODULE(0)) {
      run = true;
      selection = 0;
      break;
    }

    // Button B1: Previous item
    if (READ_BUTTON_CLOSED(B1) == 1 && selection > 0) {
      selection--;
      WRITE_LCD_TEXT(2, 2, "=");
    } else {
      if (selection > 0) {
        WRITE_LCD_TEXT(2, 2, "<");
      } else {
        WRITE_LCD_TEXT(2, 2, "|");
      }
      sleep(100);
    }

    // Button B2: Next item
    if (READ_BUTTON_CLOSED(B2) == 1 && selection < 10) {
      selection++;
      WRITE_LCD_TEXT(4, 2, "=");
    } else {
      if (selection < 10) {
        WRITE_LCD_TEXT(4, 2, ">");
      } else {
        WRITE_LCD_TEXT(4, 2, "|");
      }
      sleep(100);
    }

    // Display current item
    char display_text[32];
    snprintf(display_text, sizeof(display_text), "%-16s", items[selection].c_str());
    WRITE_LCD_TEXT(2, 1, display_text);

    // Button B3: Confirm selection
    if (READ_BUTTON_CLOSED(B3) == 1) {
      WRITE_LCD_TEXT(6, 2, "V");
      while (READ_BUTTON_CLOSED(B3) == 1) {}
      WRITE_LCD_TEXT(6, 2, "v");

      char log_msg[64];
      snprintf(log_msg, sizeof(log_msg), "Selected Program: %d", selection);
      _log("MENU FUNC", "MENU", log_msg);

      WRITE_LCD_TEXT(1, 2, "|              ");
      for (int i = 2; i < 16; i++) {
        WRITE_LCD_TEXT(i, 2, "=");
        sleep(30);
      }

      sleep(300);
      return selection;
    } else {
      WRITE_LCD_TEXT(6, 2, "v");
    }
  }

  return selection;
}

// ============================================================
// SETUP - Initialize all hardware
// ============================================================
void setup(void)
{
  // Initialize serial communication
  printf("========================================\n");
  printf("KeplerBRAIN V4 - Mbed OS Initialization\n");
  printf("========================================\n");

  // Initialize Bluetooth serial
  bt_serial.set_baud(9600);

  // Initialize KeplerBRAIN hardware
  KEPLERBRAIN_INIT();
  _log("SETUP", "KeplerBRAIN", "Kepler INIT DONE");

  // Initialize IMU
  WRITE_I2C_BNO055_INIT();
  sleep(50);
  _log("SETUP", "IMU", "IMU INIT DONE");
  WRITE_LCD_TEXT(1, 2, "IMU Ready!");
  sleep(50);

  // Initialize Power Sensor
  WRITE_I2C_INA231_INIT();
  _log("SETUP", "POWER", "POWERSENS INIT DONE");

  WRITE_LCD_CLEAR();
  KEPLER_UPDATE();  // First update cycle
}

// ============================================================
// PROGRAM: Dummy Test 1 - Simple movement test
// ============================================================
void dum_shii1(void)
{
  _log("PROGRAM", "DUM_SHII1", "Starting movement test");
  move_angle(0, 60);
  
  while (!glob_exit_override) {
    update_motors();
    KEPLER_UPDATE();
    sleep(10);
  }
  
  _log("PROGRAM", "DUM_SHII1", "Test complete");
}

// ============================================================
// PROGRAM: Dummy Test 2
// ============================================================
void dum_shii2(void)
{
  _log("PROGRAM", "DUM_SHII2", "Starting test 2");
  
  while (!glob_exit_override) {
    KEPLER_UPDATE();
    sleep(10);
  }
  
  _log("PROGRAM", "DUM_SHII2", "Test complete");
}

// ============================================================
// PROGRAM: Soccer Main - Main competition behavior
// ============================================================
void soccer_main(void)
{
  _log("PROGRAM", "SOCCER_MAIN", "Starting soccer program");

  while (!glob_exit_override) {
    // Do soccer things here
    KEPLER_UPDATE();
    update_motors();
    _SPIs();  // Read sensor data
    
    sleep(10);
  }

  _log("PROGRAM", "SOCCER_MAIN", "Soccer program ended");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop(void)
{
  KEPLER_UPDATE();  // Background tasks

  char bt_status[32];
  snprintf(bt_status, sizeof(bt_status), "BT: %s", is_bt_connected ? "ON" : "OFF");
  _log("VOID LOOP", "KeplerBRAIN", bt_status);

  // Program selection strings
  std::string PROG_SELEC_STR[11];
  PROG_SELEC_STR[0]  = "DEFAULT  ";
  PROG_SELEC_STR[1]  = "READOUTS ";
  PROG_SELEC_STR[2]  = "T. IMU   ";
  PROG_SELEC_STR[3]  = "T. GROUND";
  PROG_SELEC_STR[4]  = "T. CAM   ";
  PROG_SELEC_STR[5]  = "BATT RO. ";
  PROG_SELEC_STR[6]  = "PID TUNE ";
  PROG_SELEC_STR[7]  = "PID MAN. ";
  PROG_SELEC_STR[8]  = "LOG DUMP ";
  PROG_SELEC_STR[9]  = "DUMB SHI ";
  PROG_SELEC_STR[10] = "DUM SHI 2";

  bool run = true;

  WRITE_LCD_TEXT(1, 1, "|");
  WRITE_LCD_TEXT(1, 2, "|");

  // Menu loop
  while (run) {
    // Check RCJ selector
    if (RCJ_MODULE(0)) {
      run = true;
      selection = 0;
      break;
    }

    // Button navigation
    if (READ_BUTTON_CLOSED(B1) == 1 && selection > 0) {
      selection--;
      WRITE_LCD_TEXT(2, 2, "=");
    } else {
      if (selection > 0) {
        WRITE_LCD_TEXT(2, 2, "<");
      } else {
        WRITE_LCD_TEXT(2, 2, "|");
      }
      sleep(100);
    }

    if (READ_BUTTON_CLOSED(B2) == 1 && selection < 10) {
      selection++;
      WRITE_LCD_TEXT(4, 2, "=");
    } else {
      if (selection < 10) {
        WRITE_LCD_TEXT(4, 2, ">");
      } else {
        WRITE_LCD_TEXT(4, 2, "|");
      }
      sleep(100);
    }

    // Display menu item
    char display_text[32];
    snprintf(display_text, sizeof(display_text), "%-16s", PROG_SELEC_STR[selection].c_str());
    WRITE_LCD_TEXT(2, 1, display_text);

    // Button B3: Confirm selection
    if (READ_BUTTON_CLOSED(B3) == 1) {
      WRITE_LCD_TEXT(6, 2, "V");
      while (READ_BUTTON_CLOSED(B3) == 1) {}
      WRITE_LCD_TEXT(6, 2, "v");

      WRITE_LCD_TEXT(1, 2, "|              ");
      for (int i = 2; i < 16; i++) {
        WRITE_LCD_TEXT(i, 2, "=");
        sleep(30);
      }

      sleep(300);
      WRITE_LCD_CLEAR();

      run = false;
      glob_exit_override = false;
    } else {
      WRITE_LCD_TEXT(6, 2, "v");
    }
  }

  // Log selected program
  char log_msg[64];
  snprintf(log_msg, sizeof(log_msg), "User selected: %s", PROG_SELEC_STR[selection].c_str());
  _log("MAIN LOOP", "MENU", log_msg);

  // Execute selected program
  switch (selection) {
    case 0:
      soccer_main();
      break;
    case 1:
      // Readouts program
      break;
    case 2:
      // Test IMU
      break;
    case 3:
      // Test Ground sensors
      break;
    case 4:
      // Test Camera
      break;
    case 5:
      // Battery readout
      break;
    case 6:
      // PID tuning
      break;
    case 7:
      // PID manual
      break;
    case 8:
      _log_Dump();
      break;
    case 9:
      dum_shii1();
      break;
    case 10:
      dum_shii2();
      break;
  }
}

// ============================================================
// MBED MAIN ENTRY POINT
// ============================================================
int main(void)
{
  setup();

  while (true) {
    loop();
  }

  return 0;
}
