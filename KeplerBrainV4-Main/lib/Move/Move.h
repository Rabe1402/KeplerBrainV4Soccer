#ifndef MOVE_H
#define MOVE_H

#include "mbed.h"
#include <cmath>
#include <algorithm>

// ============================================================
// MOTOR CONTROL VARIABLES
// ============================================================
extern int trg_rot;      // target rotation
extern int rot;          // current rotation
extern float trg_movX;   // target movement X
extern float trg_movY;   // target movement Y
extern float movX;       // current position X
extern float movY;       // current position Y
extern int drive_m1, drive_m2, drive_m3, drive_m4;  // motor drive values
extern int max_accell;   // maximum acceleration

// ============================================================
// IMU SENSOR VARIABLES (from DataCalculation)
// ============================================================
extern uint16_t yaw;  // current heading from IMU

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================
void update_motors(void);
void set_motors(int Motor1, int Motor2, int Motor3, int Motor4);
void rotate(int speed);
void rotate_to(int target, int precision, int precision_switch, float speed, float speed_precise, int base_speed);
void rotate_to_quadratic(int target, int precision, int precision_switch, float speed, float speed_precise, int base_speed, int max_out);
void rotate_quadratic(int angle, int precision, int precision_switch, int max_out, int min_out);
void move_angle(int target, int Speed);
void move_angle_correction(int target, int Speed, int angle_precision);
void orbit_around(int target, int Speed, int orbit_radius);
void orbit(int speed, int ang, int spe);

#endif // MOVE_H
