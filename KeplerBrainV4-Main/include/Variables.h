#ifndef VARIABLES_H
#define VARIABLES_H

#include "mbed.h"
#include <vector>
#include <string>
#include <cstdint>

//=============================================================================
// VARIABLES.H - KeplerBrainV4 Soccer Robot - Global Variables
//=============================================================================

//-----------------------------------------------------------------------------
// GLOBAL CONTROL VARIABLES
//-----------------------------------------------------------------------------
extern int glob_ctr;                 // Global counter
extern bool glob_exit_override;      // Exit override flag for program termination

//-----------------------------------------------------------------------------
// STRUCTURES
//-----------------------------------------------------------------------------
struct herz {
  int value = 0;
};

//-----------------------------------------------------------------------------
// MOVEMENT VARIABLES
//-----------------------------------------------------------------------------
// Speed parameters
extern int target_speed;             // Target movement speed
extern int shoot_speed;              // Shooting mechanism speed

// PID parameters for rotation
extern float rot_kp;                 // Rotation proportional gain
extern float rot_kd;                 // Rotation derivative gain
extern float rot_ki;                 // Rotation integral gain

// PID parameters for movement
extern float mov_kp;                 // Movement proportional gain
extern float mov_kd;                 // Movement derivative gain
extern float mov_ki;                 // Movement integral gain

// Target movement values
extern float trg_movX;               // Target movement X
extern float trg_movY;               // Target movement Y
extern float trg_rot;                // Target rotation

// Current position/rotation values
extern float movX;                   // Current position X
extern float movY;                   // Current position Y
extern float rot;                    // Current rotation

// Advanced control
extern float vrot;                   // Virtual rotation (for future use)
extern float max_accell;             // Maximum acceleration

// Motor drive commands
extern int drive_m1;
extern int drive_m2;
extern int drive_m3;
extern int drive_m4;

//-----------------------------------------------------------------------------
// LINE DETECTION VARIABLES
//-----------------------------------------------------------------------------
extern int16_t ground_sensor[9];     // Array for 8 ground sensors (indexed 1-8)
extern int16_t ground_smallest;      // Smallest sensor value (darkest line)
extern int16_t ground_avg;           // Average of all sensors
extern int ground_sens_id;           // ID of smallest sensor
extern int line_threshold;           // Line detection threshold

// Line escape logic
extern int line_first_sensor_id;     // First sensor that detected line
extern uint32_t line_escape_start_time;  // Timer start for escape movement
extern int line_escape_duration;     // Escape movement duration (ms)
extern uint32_t line_last_seen_millis;   // Last time line was detected
extern int allow_sens_again;         // Wait time before re-enabling sensors (ms)
extern bool sens_allowed;            // Sensor allowed to read?
extern uint32_t line_first_seen_millis;  // First detection time
extern int16_t orbit_ang;            // Orbit angle

//-----------------------------------------------------------------------------
// BALL TRACKING VARIABLES
//-----------------------------------------------------------------------------
extern int ball_last_seen_ang;       // Last angle where ball was seen
extern int ball_target;              // Target heading for ball
extern bool ball_target_locked;      // Is ball target locked?
extern uint32_t last_ball_locked_time;   // Last lock update time

//-----------------------------------------------------------------------------
// MENU SYSTEM VARIABLES
//-----------------------------------------------------------------------------
extern int selection;                // Current menu selection

//-----------------------------------------------------------------------------
// DEBUG/LOGGING VARIABLES
//-----------------------------------------------------------------------------
extern bool debug;                   // Enable debug output
extern bool debug_over_serial;       // Send logs to serial
extern bool debug_over_BT;           // Send logs over Bluetooth
extern bool debug_push_log;          // Buffer logs in vector
extern bool is_bt_connected;         // Bluetooth connection status
extern BufferedSerial bt_serial;     // Bluetooth UART (PA11=RX, PA12=TX)
extern std::vector<std::string> debug_log;  // Log buffer
extern uint32_t search_start_time;   // Search/detection start time

//-----------------------------------------------------------------------------
// IMU SENSOR VARIABLES
//-----------------------------------------------------------------------------
extern int16_t yaw;                  // IMU yaw angle (heading)
extern int16_t pitch;                // IMU pitch angle
extern int16_t roll;                 // IMU roll angle

extern int16_t acc_x;                // IMU X acceleration
extern int16_t acc_y;                // IMU Y acceleration
extern int16_t acc_z;                // IMU Z acceleration

#endif // VARIABLES_H
