#include "mbed.h"
#include <vector>
#include <string>
#include <cstdint>

//=============================================================================
// VARIABLES.CPP - Global Variable Definitions
//=============================================================================

//-----------------------------------------------------------------------------
// GLOBAL CONTROL VARIABLES
//-----------------------------------------------------------------------------
int glob_ctr = 0;
bool glob_exit_override = false;

//-----------------------------------------------------------------------------
// MOVEMENT VARIABLES
//-----------------------------------------------------------------------------
// Speed parameters
int target_speed = 80;              // Default target speed
int shoot_speed = 70;               // Default shoot speed

// PID parameters (default values, adjust based on testing)
float rot_kp = 1.0f;                // Rotation P gain
float rot_kd = 1.0f;                // Rotation D gain
float rot_ki = 1.0f;                // Rotation I gain

float mov_kp = 1.0f;                // Movement P gain
float mov_kd = 1.0f;                // Movement D gain
float mov_ki = 1.0f;                // Movement I gain

// Target movement values
float trg_movX = 0.0f;
float trg_movY = 0.0f;
float trg_rot = 0.0f;

// Current position/rotation values
float movX = 0.0f;
float movY = 0.0f;
float rot = 0.0f;

// Advanced control
float vrot = 0.0f;                  // Virtual rotation (for future use)
float max_accell = 1.0f;            // Maximum acceleration

// Motor drive commands
int drive_m1 = 0;
int drive_m2 = 0;
int drive_m3 = 0;
int drive_m4 = 0;

//-----------------------------------------------------------------------------
// LINE DETECTION VARIABLES
//-----------------------------------------------------------------------------
int16_t ground_sensor[9] = {0};     // Array for 8 ground sensors (index 1-8)
int16_t ground_smallest = 1023;     // Initialize to max value
int16_t ground_avg = 0;             // Average of all sensors
int ground_sens_id = 0;             // Current smallest sensor ID
int line_threshold = -17;           // Line detection threshold (adjust as needed)

// Line escape logic
int line_first_sensor_id = -1;      // First sensor detecting line
uint32_t line_escape_start_time = 0;    // Escape start time
int line_escape_duration = 400;     // Escape duration in ms (adjust as needed)
uint32_t line_last_seen_millis = 0;     // Last detection time
int allow_sens_again = 10;          // Re-enable sensor wait time (ms)
bool sens_allowed = true;           // Sensor enabled flag
uint32_t line_first_seen_millis = 0;    // First detection timestamp
int16_t orbit_ang = 0;              // Orbit angle

//-----------------------------------------------------------------------------
// BALL TRACKING VARIABLES
//-----------------------------------------------------------------------------
int ball_last_seen_ang = 0;         // Last ball angle
int ball_target = 0;                // Target angle to ball
bool ball_target_locked = false;    // Ball target lock status
uint32_t last_ball_locked_time = 0; // Last lock time

//-----------------------------------------------------------------------------
// MENU SYSTEM VARIABLES
//-----------------------------------------------------------------------------
int selection = 0;                  // Current menu selection

//-----------------------------------------------------------------------------
// DEBUG/LOGGING VARIABLES
//-----------------------------------------------------------------------------
bool debug = true;                  // Debug enabled
bool debug_over_serial = true;      // Serial debug output
bool debug_over_BT = true;          // Bluetooth debug output
bool debug_push_log = false;        // Buffer logs (needs SD card for long logs)
bool is_bt_connected = false;       // Bluetooth connection status
BufferedSerial bt_serial(PA_11, PA_12, 9600);  // RX=PA11, TX=PA12, 9600 baud
std::vector<std::string> debug_log;  // Log buffer
uint32_t search_start_time = 0;     // Search start time

//-----------------------------------------------------------------------------
// IMU SENSOR VARIABLES
//-----------------------------------------------------------------------------
int16_t yaw = 0;                    // IMU heading
int16_t pitch = 0;                  // IMU pitch
int16_t roll = 0;                   // IMU roll

int16_t acc_x = 0;                  // Acceleration X
int16_t acc_y = 0;                  // Acceleration Y
int16_t acc_z = 0;                  // Acceleration Z
