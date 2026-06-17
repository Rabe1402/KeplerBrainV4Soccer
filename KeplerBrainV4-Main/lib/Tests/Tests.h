#ifndef TESTS_H
#define TESTS_H

#include "mbed.h"

// ============================================================
// TEST FUNCTIONS - Sensor and component testing
// ============================================================

// Motor test - cycles through all 4 motors sequentially
void _motor_test(void);

// IMU test - displays yaw, pitch, roll in real-time
void _imu_test(void);

// Ground sensor test - displays all 8 ground sensors
void _ground_test(void);

// Battery test - cycles through voltage, current, power readings
void _batt_test(void);

// Camera test - displays all camera sensor data
void _camera_test(void);

#endif // TESTS_H
