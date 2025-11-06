

//sonstige 
int serial_baud = 115200;

int selection_cursor = 0;
int selection = 0;
bool run = false;

int i = 0;
int error =0;

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

//spi1 (bodensens)
uint8_t ff;
uint8_t fl;
uint8_t fr;
uint8_t ll;
uint8_t rr;
uint8_t bl;
uint8_t br;
uint8_t bb;

// Motoren 
int drive_m1;
int drive_m2;
int drive_m3;
int drive_m4;
