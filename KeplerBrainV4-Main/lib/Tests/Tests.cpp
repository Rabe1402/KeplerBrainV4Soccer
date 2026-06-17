#include "KeplerUpdate.h"
#include "KeplerBrainV5.h"
#include "Tests.h"
#include "DataCalculation.h"      // Sensor data: _SPIs(), camera data, ground sensors
#include "KeplerUpdate.h"         // Background: KEPLER_UPDATE()
#include "PowerSense.h"           // Power: ina231_* variables
#include "Variables.h"            // Global variables: yaw, pitch, roll, etc.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// ============================================================
// MOTOR TEST
// ============================================================
void _motor_test(void)
{
  WRITE_LCD_CLEAR();
  WRITE_LCD_TEXT(1, 1, "Now Testing:");

  // Stop all motors
  WRITE_MOTOR(M1, 0);
  WRITE_MOTOR(M2, 0);
  WRITE_MOTOR(M3, 0);
  WRITE_MOTOR(M4, 0);

  sleep(1000);

  // Test each motor sequentially
  WRITE_LCD_TEXT(2, 1, "M1");
  WRITE_MOTOR(M1, 50);
  sleep(1000);

  WRITE_LCD_TEXT(2, 1, "M2");
  WRITE_MOTOR(M2, 50);
  sleep(1000);

  WRITE_LCD_TEXT(2, 1, "M3");
  WRITE_MOTOR(M3, 50);
  sleep(1000);

  WRITE_LCD_TEXT(2, 1, "M4");
  WRITE_MOTOR(M4, 50);
  sleep(1000);

  // Stop all motors
  WRITE_MOTOR(M1, 0);
  WRITE_MOTOR(M2, 0);
  WRITE_MOTOR(M3, 0);
  WRITE_MOTOR(M4, 0);

  WRITE_LCD_CLEAR();
  WRITE_LCD_TEXT(1, 1, "Test Complete :)");
  _log("TEST", "MOTOR", "Test Complete - returning to menu in 1s");
  sleep(1000);
}

// ============================================================
// IMU TEST
// ============================================================
void _imu_test(void)
{
  WRITE_LCD_TEXT(1, 2, "x       ");

  uint32_t old_time = 0;
  bool full_speed = false;

  while (READ_BUTTON_CLOSED(B1) != 1) {
    WRITE_LCD_CLEAR();
    KEPLER_UPDATE();
    old_time = Kernel::get_ms_count();

    yaw = READ_I2C_BNO055_YAW();
    pitch = READ_I2C_BNO055_PITCH();
    roll = READ_I2C_BNO055_ROLL();

    uint32_t dt = Kernel::get_ms_count() - old_time;
    char log_msg[128];
    snprintf(log_msg, sizeof(log_msg), "Y=%d P=%d R=%d DT=%lums (Press B1 to exit)", 
             yaw, pitch, roll, dt);
    _log("TEST", "IMU", log_msg);

    char lcd_line[32];
    snprintf(lcd_line, sizeof(lcd_line), "Y%-4d|P%-4d|R%-4d", yaw, pitch, roll);
    WRITE_LCD_TEXT(1, 1, lcd_line);

    if (!full_speed) {
      sleep(100);
    }

    // Toggle full speed mode
    if (READ_BUTTON_CLOSED(B3) == 1) {
      full_speed = !full_speed;
      if (full_speed) {
        WRITE_LCD_TEXT(11, 2, "fs:on ");
      } else {
        WRITE_LCD_TEXT(11, 2, "fs:off");
      }
      sleep(1000);

      snprintf(lcd_line, sizeof(lcd_line), "DT: %lu", Kernel::get_ms_count() - old_time);
      WRITE_LCD_TEXT(3, 2, lcd_line);
    }
  }

  WRITE_LCD_TEXT(1, 2, "o");
  while (READ_BUTTON_CLOSED(B1) == 1) {}
  WRITE_LCD_CLEAR();
}

// ============================================================
// GROUND SENSOR TEST
// ============================================================
void _ground_test(void)
{
  WRITE_LCD_TEXT(1, 2, "x       ");

  uint32_t old_time = 0;
  bool full_speed = false;
  int mode = 0;

  _log("TEST", "GROUND", "Starting ground sensor test (B1=exit, B2=mode, B3=speed)");
  WRITE_LCD_TEXT(1, 2, "fc fl fr lc rc");

  while (READ_BUTTON_CLOSED(B1) != 1) {
    char log_msg[256];
    snprintf(log_msg, sizeof(log_msg),
             "fc=%u fr=%u rc=%u br=%u bc=%u bl=%u lc=%u fl=%u",
             fc, fr, rc, br, bc, bl, lc, fl);
    _log("TEST", "GROUND", log_msg);

    KEPLER_UPDATE();
    old_time = Kernel::get_ms_count();

    _SPIs();

    char lcd_line[32];
    switch (mode) {
      case 0:  // Front sensors
        snprintf(lcd_line, sizeof(lcd_line), "%-3u %-3u %-3u %-3u %-3u", fc, fl, fr, lc, rc);
        break;
      case 1:  // Back sensors + average
      {
        uint16_t avg = (fc + fr + rc + br + bc + bl + lc + fl) / 8;
        snprintf(lcd_line, sizeof(lcd_line), "%-3u %-3u %-3u AVG:%-3u", bl, br, bc, avg);
      }
        break;
    }
    WRITE_LCD_TEXT(1, 1, lcd_line);

    if (!full_speed) {
      sleep(100);
    }

    // Toggle full speed
    if (READ_BUTTON_CLOSED(B3) == 1) {
      full_speed = !full_speed;
      if (full_speed) {
        WRITE_LCD_TEXT(11, 2, "fs:on ");
      } else {
        WRITE_LCD_TEXT(11, 2, "fs:off");
      }
      sleep(1000);
    }

    // Switch mode
    if (READ_BUTTON_CLOSED(B2) == 1) {
      WRITE_LCD_CLEAR();
      mode++;
      if (mode > 1) {
        mode = 0;
      }

      WRITE_LCD_TEXT(1, 1, "              ");

      switch (mode) {
        case 0:
          WRITE_LCD_TEXT(1, 2, "fc fl fr lc rc ");
          break;
        case 1:
          WRITE_LCD_TEXT(1, 2, "bl br bc avg   ");
          break;
      }

      sleep(1000);
    }
  }

  WRITE_LCD_TEXT(1, 2, "o");
  while (READ_BUTTON_CLOSED(B1) == 1) {}
  WRITE_LCD_CLEAR();
}

// ============================================================
// BATTERY TEST
// ============================================================
void _batt_test(void)
{
  _log("TEST", "BAT", "Battery test: refer to KEPLER_UPDATE messages (B1=exit, B2=mode, B3=speed)");
  WRITE_LCD_TEXT(1, 2, "x       ");

  uint32_t old_time = 0;
  bool full_speed = false;
  int mode = 0;

  while (READ_BUTTON_CLOSED(B1) != 1) {
    KEPLER_UPDATE();
    old_time = Kernel::get_ms_count();

    char lcd_line[32];
    switch (mode) {
      case 0:  // Bus voltage
      {
        float voltage = ina231_bus_voltage / 1000.0f;
        snprintf(lcd_line, sizeof(lcd_line), "%.2f V         ", voltage);
      }
        break;
      case 1:  // Shunt voltage
        snprintf(lcd_line, sizeof(lcd_line), "%.3f mV        ", ina231_shunt_voltage);
        break;
      case 2:  // Current
        snprintf(lcd_line, sizeof(lcd_line), "%.3f mA        ", ina231_current);
        break;
      case 3:  // Power
      {
        float power = ina231_power / 10000.0f;
        snprintf(lcd_line, sizeof(lcd_line), "%.3f W         ", power);
      }
        break;
    }
    WRITE_LCD_TEXT(1, 1, lcd_line);

    if (!full_speed) {
      sleep(100);
    }

    // Toggle full speed
    if (READ_BUTTON_CLOSED(B3) == 1) {
      full_speed = !full_speed;
      if (full_speed) {
        WRITE_LCD_TEXT(11, 2, "fs:on ");
      } else {
        WRITE_LCD_TEXT(11, 2, "fs:off");
      }
      sleep(1000);

      uint32_t dt = Kernel::get_ms_count() - old_time;
      char dt_str[32];
      snprintf(dt_str, sizeof(dt_str), "DT: %lu", dt);
      WRITE_LCD_TEXT(3, 2, dt_str);
    }

    // Switch mode
    if (READ_BUTTON_CLOSED(B2) == 1) {
      mode++;
      if (mode > 3) {
        mode = 0;
      }

      char mode_str[16];
      snprintf(mode_str, sizeof(mode_str), "%d", mode);
      WRITE_LCD_TEXT(11, 2, mode_str);

      sleep(1000);
    }

    uint32_t dt = Kernel::get_ms_count() - old_time;
    char dt_str[32];
    snprintf(dt_str, sizeof(dt_str), "DT: %lu              ", dt);
    WRITE_LCD_TEXT(3, 2, dt_str);
  }

  WRITE_LCD_TEXT(1, 2, "o");
  while (READ_BUTTON_CLOSED(B1) == 1) {}
  WRITE_LCD_CLEAR();
}

// ============================================================
// CAMERA TEST
// ============================================================
void _camera_test(void)
{
  WRITE_LCD_TEXT(1, 2, "x       ");
  _log("TEST", "CAM", "Starting camera test (B1=exit, B2=mode, B3=speed)");

  uint32_t old_time = 0;
  bool full_speed = false;
  int mode = 0;

  while (READ_BUTTON_CLOSED(B1) != 1) {
    char log_msg[256];
    snprintf(log_msg, sizeof(log_msg),
             "D0=%u D1=%u D2=%u D3=%u D4=%u D5=%u D6=%u D7=%u",
             SPICAM_Data0, SPICAM_Data1, SPICAM_Data2, SPICAM_Data3,
             SPICAM_Data4, SPICAM_Data5, SPICAM_Data6, SPICAM_Data7);
    _log("TEST", "CAM", log_msg);

    KEPLER_UPDATE();
    old_time = Kernel::get_ms_count();

    _SPIs();

    char lcd_line[32];
    switch (mode) {
      case 0:  // Trigger
        snprintf(lcd_line, sizeof(lcd_line), "%u        ", SPICAM_Data0);
        break;
      case 1:  // Ball in sight
        snprintf(lcd_line, sizeof(lcd_line), "%u        ", SPICAM_Data1);
        break;
      case 2:  // Angle to ball
        snprintf(lcd_line, sizeof(lcd_line), "%d deg   ", _cam_data_calculation());
        break;
      case 3:  // MSB
        snprintf(lcd_line, sizeof(lcd_line), "%u        ", SPICAM_Data3);
        break;
      case 4:  // LSB
        snprintf(lcd_line, sizeof(lcd_line), "%u        ", SPICAM_Data4);
        break;
      case 5:  // Distance
        snprintf(lcd_line, sizeof(lcd_line), "%d mm   ", _cam_distance_calculation());
        break;
    }
    WRITE_LCD_TEXT(1, 1, lcd_line);

    if (!full_speed) {
      sleep(100);
    }

    // Toggle full speed
    if (READ_BUTTON_CLOSED(B3) == 1) {
      full_speed = !full_speed;
      if (full_speed) {
        WRITE_LCD_TEXT(11, 2, "fs:on ");
      } else {
        WRITE_LCD_TEXT(11, 2, "fs:off");
      }
      sleep(1000);
    }

    // Switch mode
    if (READ_BUTTON_CLOSED(B2) == 1) {
      mode++;
      if (mode > 5) {
        mode = 0;
      }

      WRITE_LCD_TEXT(1, 1, "              ");

      const char* mode_strings[] = {
        "0: trigger      ",
        "1: Ball in sight",
        "2: ang to ball  ",
        "3: MSB in mm    ",
        "4: LSB in mm    ",
        "5: Dist in mm   "
      };

      WRITE_LCD_TEXT(1, 2, mode_strings[mode]);
      sleep(1000);
    }
  }

  WRITE_LCD_TEXT(1, 2, "o");
  while (READ_BUTTON_CLOSED(B1) == 1) {}
  WRITE_LCD_CLEAR();
}
