
//sonstige 
int serial_baud = 115200;

int selection_cursor = 0;
int selection = 0;
bool run = false;

int current_state = 0;

int error = 0;

int ground_sensor[7];

int ground_smallest; 
bool reverse = false;
int ground_avg;
//Default

int target_angle = 45;
int target_speed = 40;
int correction_speed = 70;

int line_timers[8] = {0, 0, 0, 0, 0, 0, 0, 0};
bool line_tmp[8]   = {0, 0, 0, 0, 0, 0, 0, 0};
int groundtimer = 0;
int line_last;
int line_base      = 244;
int line_threshold = 3;

int ball_target = 0;

//Debug
bool debug = true;
bool debug_over_serial = true;
std::vector<String> debug_log = {"Log initialized!"};
String log_message = "";

//IMU Sensor 
uint16_t yaw;
int16_t pitch;
int16_t roll;

// Globale Variablen für den gleitenden Durchschnitt
float yaw_history[5] = {0, 0, 0, 0, 0}; // Array für die letzten 5 Yaw-Werte
int history_index = 0; // Index für das Array
bool history_filled = false; // Flag, ob das Array vollständig gefüllt ist
int radtodeg = (M_PI / 180.0);

// Setup-Variablen
unsigned long last_time = millis(); // Zeitstempel für die Berechnung
int drive_base = 10; // Basisgeschwindigkeit
float yaw_direction = 20; // Ziel-Yaw-Wert
float yaw_difference = 0; // yaw difference zum anfahren an einen bestimmten imu wert 


// Motoren 
int drive_m1;
int drive_m2;
int drive_m3;
int drive_m4;
