#include "KeplerBrainV5.h"
#include "DataCalculation.h"
#include <cstdio>
#include <cstring>

// ============================================================
// VARIABLE DEFINITIONS
// ============================================================
// Camera data
uint8_t SPICAM_Data0 = 255;
uint8_t SPICAM_Data1 = 255;
uint8_t SPICAM_Data2 = 255;
uint8_t SPICAM_Data3 = 255;
uint8_t SPICAM_Data4 = 255;
uint8_t SPICAM_Data5 = 255;
uint8_t SPICAM_Data6 = 255;
uint8_t SPICAM_Data7 = 255;

// Ground sensors
uint16_t fc = 0;
uint16_t fl = 0;
uint16_t fr = 0;
uint16_t lc = 0;
uint16_t rc = 0;
uint16_t bl = 0;
uint16_t br = 0;
uint16_t bc = 0;
uint16_t ground_sensor[9] = {0};
int16_t ground_smallest = 1023;

// IMU
uint16_t yaw = 0;
int16_t pitch = 0;
int16_t roll = 0;
int16_t acc_x = 0;
int16_t acc_y = 0;
int16_t acc_z = 0;

// ============================================================
// EXTERNAL SPI & GPIO OBJECTS (from main KeplerBrainV5)
// ============================================================
extern SPI g_spi_cam;      // SPI for camera (SPICAM)
extern SPI g_spi_ground;   // SPI for ground sensors (SPI2)
extern DigitalOut gpio_spicam_cs;  // CS pin for camera
extern DigitalOut gpio_spi2_cs;    // CS pin for ground sensors

// ============================================================
// SPI READ FUNCTION
// ============================================================
void _SPIs(void)
{
  // ===== READ 8 BYTES FROM CAMERA (OpenMV) =====
  gpio_spicam_cs = 0;  // CS low
  wait_us(100);
  
  if (g_spi_cam.write(1) == 250)  // Trigger byte
  {
    SPICAM_Data1 = g_spi_cam.write(0);
    SPICAM_Data2 = g_spi_cam.write(0);
    SPICAM_Data3 = g_spi_cam.write(0);
    SPICAM_Data4 = g_spi_cam.write(0);
    SPICAM_Data5 = g_spi_cam.write(0);
    SPICAM_Data6 = g_spi_cam.write(0);
    SPICAM_Data7 = g_spi_cam.write(0);
  }
  
  gpio_spicam_cs = 1;  // CS high
  
  // ===== READ 8 UINT16_T FROM GROUND SENSORS =====
  gpio_spi2_cs = 0;  // CS low
  wait_us(100);
  
  if (g_spi_ground.write(0xFF) == 250)  // Sync byte
  {
    fc = (g_spi_ground.write(0xFF) << 8) | g_spi_ground.write(0xFF);
    fl = (g_spi_ground.write(0xFF) << 8) | g_spi_ground.write(0xFF);
    fr = (g_spi_ground.write(0xFF) << 8) | g_spi_ground.write(0xFF);
    lc = (g_spi_ground.write(0xFF) << 8) | g_spi_ground.write(0xFF);
    rc = (g_spi_ground.write(0xFF) << 8) | g_spi_ground.write(0xFF);
    bl = (g_spi_ground.write(0xFF) << 8) | g_spi_ground.write(0xFF);
    br = (g_spi_ground.write(0xFF) << 8) | g_spi_ground.write(0xFF);
    bc = (g_spi_ground.write(0xFF) << 8) | g_spi_ground.write(0xFF);
  }
  
  gpio_spi2_cs = 1;  // CS high
}

// ============================================================
// CAMERA DATA CALCULATION
// ============================================================
int _cam_data_calculation(void)
{
  // Calculate relative angle: -90..+90° (90 = straight ahead)
  return (int8_t)(SPICAM_Data2 - 90);
}

// ============================================================
// CAMERA DISTANCE CALCULATION
// ============================================================
int _cam_distance_calculation(void)
{
  // Combine MSB and LSB to get distance in mm
  return (SPICAM_Data3 << 8) | SPICAM_Data4;
}

// ============================================================
// FIND SMALLEST GROUND SENSOR
// ============================================================
int smallest_ground_sensor_id(int base)
{
  int id = 255;
  ground_smallest = 1023;
  
  // Copy ground sensor values to array
  ground_sensor[1] = fc;
  ground_sensor[2] = fr;
  ground_sensor[3] = rc;
  ground_sensor[4] = br;
  ground_sensor[5] = bc;
  ground_sensor[6] = bl;
  ground_sensor[7] = lc;
  ground_sensor[8] = fl;
  
  // Find smallest sensor value relative to base
  for (int i = 1; i <= 8; i++)
  {
    int diff = ground_sensor[i] - base;
    if (diff < ground_smallest)
    {
      ground_smallest = diff;
      id = i;
    }
  }
  
  return id;
}

// ============================================================
// IMU DATA READ
// ============================================================
void _imu_read(void)
{
  yaw = READ_I2C_BNO055_YAW();
  pitch = READ_I2C_BNO055_PITCH();
  roll = READ_I2C_BNO055_ROLL();
  
  char log_msg[128];
  snprintf(log_msg, sizeof(log_msg), "Y=%d P=%d R=%d", yaw, pitch, roll);
  _log("SENSOR DATA", "IMU", std::string(log_msg));
}

// ============================================================
// IMU ACCELEROMETER READ
// ============================================================
void _imu_acc_read(void)
{
  acc_x = READ_I2C_BNO055_ACC_X();
  acc_y = READ_I2C_BNO055_ACC_Y();
  acc_z = READ_I2C_BNO055_ACC_Z();
  
  char log_msg[128];
  snprintf(log_msg, sizeof(log_msg), "AccX=%d AccY=%d AccZ=%d", acc_x, acc_y, acc_z);
  _log("SENSOR DATA", "IMU", std::string(log_msg));
}

// ============================================================
// PID CONTROLLER IMPLEMENTATION
// ============================================================
PIDController::PIDController(double p, double i, double d)
    : Kp(p), Ki(i), Kd(d), prev_error(0), integral(0)
{
}

double PIDController::calculate(double setpoint, double actual_value, double dt)
{
  // 1. Calculate error
  double error = setpoint - actual_value;
  
  // 2. Proportional term: direct response to error
  double Pout = Kp * error;
  
  // 3. Integral term: accumulates error over time (eliminates steady-state offset)
  integral += error * dt;
  double Iout = Ki * integral;
  
  // 4. Derivative term: predicts future error based on rate of change
  double derivative = (error - prev_error) / dt;
  double Dout = Kd * derivative;
  
  // 5. Total output
  double output = Pout + Iout + Dout;
  
  // Save state for next iteration
  prev_error = error;
  
  return output;
}
