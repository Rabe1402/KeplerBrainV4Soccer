#include "../shared/KeplerBRAIN_V4.h"
#include "Variables.h" //alle variablen usw. sind in dieser datei, um diese ein wenig aufzuräumen 

void setup()
{
  // Initialisierung der Hardwarekomponenten des Controllers
  KEPLERBRAIN_INIT();
  WRITE_I2C_BNO055_INIT();
  Serial.begin(115200);
  WRITE_LCD_CLEAR();
  
  drive_base=10;
  yaw_direction=20;
}
 
int selection_cursor = 0;
int selection = 0;

void loop()
{
  
  if (READ_BUTTON_CLOSED(B1) == 1 && selection_cursor > 0){selection_cursor--;WRITE_LCD_TEXT(1, 2, "o");}
  if (READ_BUTTON_CLOSED(B3) == 1 && selection_cursor < 10){selection_cursor++;WRITE_LCD_TEXT(9, 2, "<>");}
  delay(200);

  //Display
  WRITE_LCD_TEXT(1, 1, String(selection_cursor));
  if (READ_BUTTON_CLOSED(B2) == 1){
    WRITE_LCD_TEXT(4, 2, "<>");
    while(READ_BUTTON_CLOSED(B2) == 1){}
    WRITE_LCD_TEXT(4, 2, "()");
    delay(500);
    selection = selection_cursor;
    switch ( selection )
    {
      case 0:
        WRITE_LCD_TEXT(1, 2, "Default");
        last_time = millis();  // Initialize time for PID
        drive_base=10;
        yaw_direction=20;
        Kp = 1.0;   // Proportional gain - tune this (start higher for faster response)
        Ki = 0.01;  // Integral gain - tune this (small to avoid windup)
        Kd = 0.5;   // Derivative gain - tune this (higher for more damping, reduces overshoot)
        while ( selection = 0) 
        {
        _SPIs();
        _imu_read();
        _default();
        }
      case 1:
        WRITE_LCD_TEXT(1, 2, "Motor-Test");
        while ( selection = 1) 
        {
        _motor_test();
        } 
      case 2:
        WRITE_LCD_TEXT(1, 2, "IMU_TEST");
        while ( selection = 2) {
        _imu_read();
        }
      case 3:
        WRITE_LCD_TEXT(1, 2, "Prog 3");
        while ( selection = 3) {

        }
      case 4:
        WRITE_LCD_TEXT(1, 2, "Prog 4");
        while ( selection = 4) {

        }
      case 5:
        WRITE_LCD_TEXT(1, 2, "Prog 5");
        last_time = millis();  // Initialize time for PID
        drive_base=20;
        yaw_direction=20;
        Kp = 1.5;   // Proportional gain - tune this (start higher for faster response)
        Ki = 0.01;  // Integral gain - tune this (small to avoid windup)
        Kd = 0.1;   // Derivative gain - tune this (higher for more damping, reduces overshoot)
        while ( selection = 5) 
        {
       // _SPIs();
        _imu_read();
        _default();
        }
    }
    delay(5000);
  }
  if (selection_cursor <= 0){WRITE_LCD_TEXT(1, 2, "   ()   +");}  
  else if (selection_cursor >= 5){WRITE_LCD_TEXT(1, 2, "-  ()    ");}  
  else{WRITE_LCD_TEXT(1, 2, "-  ()   +");}  
  
  /*WRITE_LCD_CLEAR();
  WRITE_MOTOR(M1, 0);
  WRITE_MOTOR(M2, 0);
  Serial.println("no program selected");
  Serial.println("press button B1 for normal startup");
  WRITE_LCD_TEXT(1, 1, "Select Program");
  WRITE_LCD_TEXT(1, 2, "B1 for normal startup");*/
}

void _motor_test()
{
  WRITE_MOTOR(M1,00);
  WRITE_MOTOR(M2,00);
  WRITE_MOTOR(M3,00);
  WRITE_MOTOR(M4,00);
  SLEEP(1000);
  WRITE_MOTOR(M1,50);
  SLEEP(1000);
  WRITE_MOTOR(M2,50);
  SLEEP(1000);
  WRITE_MOTOR(M3,50);
  SLEEP(1000);
  WRITE_MOTOR(M4,50);
  SLEEP(1000);
}

void _default()
{
	unsigned long now = millis();
  dt = (now - last_time) / 1000.0;  // Time delta in seconds
  if (dt <= 0) dt = 0.001;  // Prevent division by zero or negative
  last_time = now;
  WRITE_LCD_TEXT(1, 1, String(yaw) + "   ");
  WRITE_LCD_TEXT(1, 2, String(yaw_direction));

  // Calculate error (with wrap-around handling for angles assuming 0-359 degrees)
  error = yaw_direction - (int)yaw;
  error = fmod(error + 180, 360) - 180;  // Normalize to -180 to 180 for shortest turn

  // PID calculations
  integral += error * dt;  // Accumulate integral
  // Optional: Add anti-windup by clamping integral if needed, e.g., integral = constrain(integral, -100, 100);
  derivative = (error - previous_error) / dt;
  yaw_difference = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  drive_m1 = drive_base - yaw_difference;
  drive_m2 = drive_base + yaw_difference;
  drive_m3 = drive_base + yaw_difference;
  drive_m4 = drive_base - yaw_difference;

  drive_m1 = constrain(drive_m1, -20, 20);
  drive_m2 = constrain(drive_m2, -20, 20);
  drive_m3 = constrain(drive_m3, -20, 20);
  drive_m4 = constrain(drive_m4, -20, 20);

  WRITE_MOTOR(M1,  drive_m1);
  WRITE_MOTOR(M2,  drive_m2);
  WRITE_MOTOR(M3, -drive_m3);
  WRITE_MOTOR(M4, -drive_m4);
}

void _imu_read()
{
  yaw = READ_I2C_BNO055_YAW();
  pitch = READ_I2C_BNO055_PITCH();
  roll = READ_I2C_BNO055_ROLL();
  //Serial.println("IMU_Sensor");
  //Serial.println( String(yaw)+" "+String(pitch)+" "+String(roll)+" ");
  
}

void _SPIs()
{
	digitalWrite(SPI1, LOW);
	 
	  if(spi.transfer(0XFF) == 250)
	  { 
	    ff = spi.transfer(0XFF);
	    fl = spi.transfer(0XFF);
	    fr = spi.transfer(0XFF);
	    ll = spi.transfer(0XFF);
	    rr = spi.transfer(0XFF);
	    bl = spi.transfer(0XFF);
	    br = spi.transfer(0XFF);
	    bb = spi.transfer(0XFF);
	  }
	 
	  digitalWrite(SPI1, HIGH);
}
