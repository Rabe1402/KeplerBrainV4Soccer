//=============================================================================
// VARIABLES.H - KeplerBrainV4 Soccer - Cleaned up version
//=============================================================================

//-----------------------------------------------------------------------------
// SYSTEM
//-----------------------------------------------------------------------------
int serial_baud = 115200;

//-----------------------------------------------------------------------------
// STATE MACHINE
//-----------------------------------------------------------------------------
int current_state = 0;  // 0=Search, 1=Move to ball, 2=Shoot, 3=Line
int last_state = 0;     // Letzter Zustand, für Rückkehr nach Liniencode
bool run = false;       // Hauptprogramm läuft
int error = 0;          // Fehler für rotate_to Funktionen

//-----------------------------------------------------------------------------
// MOVEMENT
//-----------------------------------------------------------------------------
int target_speed = 40;  // ANPASSEN: Grundgeschwindigkeit

// Motor outputs
int drive_m1;
int drive_m2;
int drive_m3;
int drive_m4;

//-----------------------------------------------------------------------------
// LINE DETECTION
//-----------------------------------------------------------------------------
int ground_sensor[8];           // Array für alle 8 Sensoren
int ground_smallest;            // Kleinster Sensorwert (= dunkelste Linie)
int ground_avg;                 // Durchschnitt aller Sensoren
int ground_sens_id;             // ID des aktuell kleinsten Sensors

// Line escape logic
int line_first_sensor_id = -1;  // Erster Sensor der Linie erkannt hat
int line_escape_start_time = 0; // Timer-Start für Ausweichbewegung
int line_escape_duration = 400; // ANPASSEN: Dauer der Ausweichbewegung (ms)
int line_last_seen_millis = 0;
int allow_sens_again = 10; //time to wait to realow ground sens

bool sens_allowed = true;

//-----------------------------------------------------------------------------
// BALL TRACKING
//-----------------------------------------------------------------------------
int ball_last_seen_ang = 0;     // Letzter Winkel wo Ball gesehen wurde
int ball_target = 0;            // Ziel-Heading für Ball
bool ball_target_locked = false;// Ball-Target eingerastet?
int last_ball_locked_time = 0;  // Letztes Lock-Update

//-----------------------------------------------------------------------------
// MENU SYSTEM
//-----------------------------------------------------------------------------
int selection_cursor = 0;
int selection = 0;

//-----------------------------------------------------------------------------
// DEBUG
//-----------------------------------------------------------------------------
bool debug = true;
bool debug_over_serial = true;
std::vector<String> debug_log = {"Log initialized!"};
String log_message = "";

//-----------------------------------------------------------------------------
// IMU SENSOR
//-----------------------------------------------------------------------------
uint16_t yaw;
int16_t pitch;
int16_t roll;