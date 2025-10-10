//#include <INA.h>
#include "../shared/KeplerBRAIN_V4.h"
#include "Variables.h" //alle variablen usw. sind in dieser datei, um diese ein wenig aufzuräumen 
//#include "powersense.h"
#include <vector>
#include <iostream>

void _imu_read()
{
  yaw = READ_I2C_BNO055_YAW();
  pitch = READ_I2C_BNO055_PITCH();
  roll = READ_I2C_BNO055_ROLL();
  _log("imu_read", "Y" + String(yaw)+" P"+String(pitch)+" R"+String(roll));
}
// / \
//  |
// muss dort bleiben ansonsten hat Tests.h keinen zugriff auf imu_read 
#include "Tests.h"


void setup()
{
  // Initialisierung der Hardwarekomponenten des Controllers
  KEPLERBRAIN_INIT();
  WRITE_I2C_BNO055_INIT();
  WRITE_I2C_INA231_INIT();
  Serial.begin(serial_baud);
  WRITE_LCD_CLEAR();
}
 
int selection_cursor = 0;
int selection = 0;

void loop()
{
  
  if (READ_BUTTON_CLOSED(B1) == 1 && selection_cursor > 0){selection_cursor--;WRITE_LCD_TEXT(1, 2, "o");}
  if (READ_BUTTON_CLOSED(B3) == 1 && selection_cursor < 10){selection_cursor++;WRITE_LCD_TEXT(9, 2, "o");}
  delay(200);

  //Display
  WRITE_LCD_TEXT(1, 1, String(selection_cursor));

  if (READ_BUTTON_CLOSED(B2) == 1){

    WRITE_LCD_TEXT(4, 2, "<>");
    while(READ_BUTTON_CLOSED(B2) == 1){}
    WRITE_LCD_TEXT(4, 2, "()");

    delay(500);

    selection = selection_cursor;

    _log("main loop", "Selected Program" + String(selection));

    switch ( selection )
    {
      case 0:
        WRITE_LCD_TEXT(1, 2, "Default");
        delay(700);                             //damit man cancel kann falls man falsch selected hat
        if (READ_BUTTON_CLOSED(B2) == 1){exit;} //


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
        WRITE_LCD_TEXT(1, 2, "Motor Test");
        delay(700);
        if (READ_BUTTON_CLOSED(B2) == 1){exit;}

        _motor_test();

      case 2:
        WRITE_LCD_TEXT(1, 2, "IMU Test");
        delay(700);
        if (READ_BUTTON_CLOSED(B2) == 1){exit;}

        _imu_test();

      case 3:
        WRITE_LCD_TEXT(1, 2, "Input Test");
        delay(700);
        if (READ_BUTTON_CLOSED(B2) == 1){exit;}


      case 4:
        WRITE_LCD_TEXT(1, 2, "Show BATT info");
        delay(700);
        if (READ_BUTTON_CLOSED(B2) == 1){exit;}


      case 5:
        WRITE_LCD_TEXT(1, 2, "Prog 5"); //warum gibts default 2 mal?
        last_time = millis();  // Initialize time for PID
        drive_base=20;
        yaw_direction=20;
        Kp = 1.5;   // Proportional gain - tune this (start higher for faster response)
        Ki = 0.01;  // Integral gain - tune this (small to avoid windup)
        Kd = 0.1;   // Derivative gain - tune this (higher for more damping, reduces overshoot)
        while ( selection = 5) 
        {
       // _SPIs(); //auscommentiert weil ich die INA.h file ned hab
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


void _SPIs()
{
	digitalWrite(SPI1, LOW);
	 
	  if(spi.transfer(0XFF) == 250)
	  { 
	    ff = spi.transfer(0XFF); // was sind die??? pls schreib bei random nummern hin was di machn :3
	    fl = spi.transfer(0XFF); // was sind die??? pls schreib bei random nummern hin was di machn :3
	    fr = spi.transfer(0XFF); // was sind die??? pls schreib bei random nummern hin was di machn :3
	    ll = spi.transfer(0XFF); // was sind die??? pls schreib bei random nummern hin was di machn :3
	    rr = spi.transfer(0XFF); // was sind die??? pls schreib bei random nummern hin was di machn :3
	    bl = spi.transfer(0XFF); // was sind die??? pls schreib bei random nummern hin was di machn :3
	    br = spi.transfer(0XFF); // was sind die??? pls schreib bei random nummern hin was di machn :3
	    bb = spi.transfer(0XFF); // was sind die??? pls schreib bei random nummern hin was di machn :3
	  }
	 
	  digitalWrite(SPI1, HIGH);
}
