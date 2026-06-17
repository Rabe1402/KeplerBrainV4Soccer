#ifndef DATA_CALCULATION_H
#define DATA_CALCULATION_H

#include "mbed.h"
#include <cstdint>
#include <chrono>

// ============================================================
// SPI CAMERA VARIABLES
// ============================================================
extern uint8_t SPICAM_Data0;  // trigger (250)
extern uint8_t SPICAM_Data1;  // boolean for ball in sight
extern uint8_t SPICAM_Data2;  // relative angle to ball (0-180, 90=ahead)
extern uint8_t SPICAM_Data3;  // distance MSB (mm)
extern uint8_t SPICAM_Data4;  // distance LSB (mm)
extern uint8_t SPICAM_Data5;  // reserved
extern uint8_t SPICAM_Data6;  // reserved
extern uint8_t SPICAM_Data7;  // reserved

// ============================================================
// GROUND SENSOR VARIABLES (8 sensors)
// ============================================================
extern uint16_t fc;  // front center
extern uint16_t fl;  // front left
extern uint16_t fr;  // front right
extern uint16_t lc;  // left center
extern uint16_t rc;  // right center
extern uint16_t bl;  // back left
extern uint16_t br;  // back right
extern uint16_t bc;  // back center

extern uint16_t ground_sensor[9];  // indexed 1-8
extern int16_t ground_smallest;

// ============================================================
// IMU VARIABLES
// ============================================================
extern uint16_t yaw;
extern int16_t pitch;
extern int16_t roll;
extern int16_t acc_x;
extern int16_t acc_y;
extern int16_t acc_z;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================
void _SPIs(void);
int _cam_data_calculation(void);
int _cam_distance_calculation(void);
int smallest_ground_sensor_id(int base);
void _imu_read(void);
void _imu_acc_read(void);

// ============================================================
// PID CONTROLLER CLASS
// ============================================================
class PIDController {
private:
    double Kp;  // Proportional gain
    double Ki;  // Integral gain
    double Kd;  // Derivative gain
    double prev_error;
    double integral;

public:
    PIDController(double p, double i, double d);
    double calculate(double setpoint, double actual_value, double dt);
};

#endif // DATA_CALCULATION_H
