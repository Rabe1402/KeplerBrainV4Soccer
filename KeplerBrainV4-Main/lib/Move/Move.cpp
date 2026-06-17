#include "KeplerBrainV5.h"
#include "Move.h"
#include <cstdio>
#include <algorithm>

// ============================================================
// VARIABLE DEFINITIONS
// ============================================================
int trg_rot = 0;
int rot = 0;
float trg_movX = 0.0f;
float trg_movY = 0.0f;
float movX = 0.0f;
float movY = 0.0f;
int drive_m1 = 0, drive_m2 = 0, drive_m3 = 0, drive_m4 = 0;
int max_accell = 10;

// ============================================================
// UPDATE MOTORS - Apply acceleration and ramp movements
// ============================================================
void update_motors(void)
{
  int Motor1, Motor2, Motor3, Motor4;

  // ===== ROTATION RAMPING =====
  if (trg_rot > rot + max_accell) {
    rot += max_accell;
  } else if (trg_rot < rot - max_accell) {
    rot -= max_accell;
  } else {
    rot = trg_rot;
  }

  // ===== MOVEMENT RAMPING =====
  float trg_to_realX = (trg_movX - movX);
  float trg_to_realY = (trg_movY - movY);
  float length = std::sqrt(trg_to_realX * trg_to_realX + trg_to_realY * trg_to_realY);

  if (length > max_accell)
  {
    float evX = trg_to_realX / length;
    float evY = trg_to_realY / length;

    evX *= max_accell;
    evY *= max_accell;

    movX += evX;
    movY += evY;
  }

  // ===== MOTOR DISTRIBUTION =====
  // Motor layout:
  //   1  /|\  2 
  //      |
  //   4       3

  Motor1 = -movY + rot;
  Motor2 = movX + rot;
  Motor3 = movY + rot;
  Motor4 = -movX + rot;

  char log_msg[128];
  snprintf(log_msg, sizeof(log_msg), "M1=%d, M2=%d, M3=%d, M4=%d", Motor1, Motor2, Motor3, Motor4);
  _log("trg_movE", "MOTORS", std::string(log_msg));

  WRITE_MOTOR(M1, Motor1);
  WRITE_MOTOR(M2, Motor2);
  WRITE_MOTOR(M3, Motor3);
  WRITE_MOTOR(M4, Motor4);
}

// ============================================================
// SET MOTORS - Add to current motor commands
// ============================================================
void set_motors(int Motor1, int Motor2, int Motor3, int Motor4)
{
  char log_msg[128];
  snprintf(log_msg, sizeof(log_msg), "SET M1=%d, M2=%d, M3=%d, M4=%d", Motor1, Motor2, Motor3, Motor4);
  _log("trg_movE", "SET_MOTORS", std::string(log_msg));

  drive_m1 += Motor1;
  drive_m2 += Motor2;
  drive_m3 += Motor3;
  drive_m4 += Motor4;
}

// ============================================================
// ROTATE - Increment rotation target
// ============================================================
void rotate(int speed)
{
  trg_rot += speed;
}

// ============================================================
// ROTATE TO - Rotate to target angle with PID-like control
// ============================================================
void rotate_to(int target, int precision, int precision_switch, float speed, float speed_precise, int base_speed)
{
  int out;
  int error = yaw - target;

  // Normalize error to -180..+180 range
  if (error >= 180) {
    error = -(360 - error);
    base_speed = -base_speed;
  }

  char log_msg[64];
  snprintf(log_msg, sizeof(log_msg), "err=%d prec=%d", error, precision);
  _log("trg_movE", "trg_rotATE_TO", std::string(log_msg));

  if (abs(error) < precision) {
    return;
  }

  out = error * error * error;
  if (abs(error) < precision_switch) {
    // Use precise speed
    out *= speed_precise;
  } else {
    // Use general speed
    out *= speed;
    out += base_speed;
  }

  rotate(out);
}

// ============================================================
// ROTATE TO QUADRATIC - Quadratic rotation control
// ============================================================
void rotate_to_quadratic(int target, int precision, int precision_switch, float speed, 
                         float speed_precise, int base_speed, int max_out)
{
  int out;
  int error = yaw - target;

  if (error >= 180) {
    error = -(360 - error);
    base_speed = -base_speed;
  }

  char log_msg[64];
  snprintf(log_msg, sizeof(log_msg), "err=%d prec=%d", error, precision);
  _log("trg_movE", "trg_rotATE_TO_QUADRATIC", std::string(log_msg));

  if (abs(error) < precision) return;

  out = error * error;

  if (abs(error) < precision_switch) {
    out *= speed_precise;
    int max_raw = (int)(precision_switch * precision_switch * speed_precise);
    out = out * max_out / std::max(max_raw, 1);
  } else {
    out *= speed;
    out += base_speed;
    int max_raw = (int)(180 * 180 * speed) + abs(base_speed);
    out = out * max_out / std::max(max_raw, 1);
  }

  if (error < 0) out = -out;
  rotate(out);
}

// ============================================================
// ROTATE QUADRATIC - Simple quadratic rotation
// ============================================================
void rotate_quadratic(int angle, int precision, int precision_switch, int max_out, int min_out)
{
  int error = angle;
  if (error >= 180) error = -(360 - error);
  if (abs(error) < precision) return;

  int out;
  if (abs(error) < precision_switch) {
    // Quadratic normalized to precision_switch
    out = max_out * ((float)(error * error) / (float)(precision_switch * precision_switch));
  } else {
    // Quadratic normalized to 90°
    out = max_out * ((float)(error * error) / (float)(90 * 90));
    if (out > max_out) out = max_out;
  }

  if (out < min_out) out = min_out;  // Minimum speed for movement
  if (error < 0) out = -out;

  rotate(out);
}

// ============================================================
// MOVE ANGLE - Move in direction relative to field
// ============================================================
void move_angle(int target, int Speed)
{
  // Y motors (1,3): cos(target + 45°)
  int YMotors = (int)(cos((target + 45) * (M_PI / 180.0)) * Speed);

  // X motors (2,4): sin(target + 45°)
  int XMotors = (int)(sin((target + 45) * (M_PI / 180.0)) * Speed);

  trg_movX += XMotors;
  trg_movY += YMotors;
}

// ============================================================
// MOVE ANGLE WITH HEADING CORRECTION
// ============================================================
void move_angle_correction(int target, int Speed, int angle_precision)
{
  int error = yaw - target;

  if (!(abs(error) < angle_precision)) {
    char log_msg[64];
    snprintf(log_msg, sizeof(log_msg), "err=%d", error);
    _log("trg_movE", "trg_movE_ANGLE_CORRECTION", std::string(log_msg));

    if (error >= 180) {
      error = -(360 - error);
    }
  }

  target += error;

  // Y motors (1,3): cos(target + 45°)
  int YMotors = (int)(cos((target + 45) * (M_PI / 180.0)) * Speed);

  // X motors (2,4): sin(target + 45°)
  int XMotors = (int)(sin((target + 45) * (M_PI / 180.0)) * Speed);

  trg_movX += XMotors;
  trg_movY += YMotors;
}

// ============================================================
// ORBIT AROUND - Move around target with heading correction
// ============================================================
void orbit_around(int target, int Speed, int orbit_radius)
{
  int error = yaw - target;
  if (error >= 180) {
    error = -(360 - error);
  }

  int YMotors = (int)(cos((target - 45) * (M_PI / 180.0)) * Speed);
  int XMotors = (int)(sin((target - 45) * (M_PI / 180.0)) * Speed);

  int orbit_correction = error * orbit_radius;

  int m1 = YMotors + orbit_correction;
  int m2 = -XMotors + orbit_correction;
  int m3 = -YMotors + orbit_correction;
  int m4 = XMotors + orbit_correction;

  // Find maximum absolute value
  int max_val = std::max({abs(m1), abs(m2), abs(m3), abs(m4)});

  // Normalize to Speed if max_val > 0
  if (max_val > 0) {
    m1 = m1 * Speed / max_val;
    m2 = m2 * Speed / max_val;
    m3 = m3 * Speed / max_val;
    m4 = m4 * Speed / max_val;
  }

  set_motors(m1, m2, m3, m4);
}

// ============================================================
// ORBIT - Move and rotate simultaneously
// ============================================================
void orbit(int speed, int ang, int spe)
{
  move_angle(ang, speed);
  rotate(spe);
}
