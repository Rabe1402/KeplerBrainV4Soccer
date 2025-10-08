
//sonstige 


//IMU Sensor 
uint16_t yaw;
int16_t pitch;
int16_t roll;

//yaw direction automation 
int yaw_direction;
float yaw_difference;
int yaw_raw[5];

float Kp;  // Proportional gain - tune this (start higher for faster response)
float Ki; // Integral gain - tune this (small to avoid windup)
float Kd;  // Derivative gain - tune this (higher for more damping, reduces overshoot)
float error = 0;
float previous_error = 0;
float integral = 0;
float derivative = 0;
unsigned long last_time = 0;
float dt = 0;

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
int drive_base;
int drive_m1;
int drive_m2;
int drive_m3;
int drive_m4;
