//#include "/home/arch/daata/School/6_klasse/IT/KeplerBrainV4Soccer/shared/KeplerBRAIN_V4.h" //idk warum aber ../shared/KE... geht bei mir ned mehr.
#include "../shared/KeplerBRAIN_V4.h"
#include "KEPLER_UPDATE.h" // wird in echte header migrirt wenn randl gut findet 
#include "powersense.h" //powersensor read 
#include "SPIs.h" //alle spi übertrageungen

#include <vector>
#include <iostream>
#include <math.h>

#include "Variables.h" //alle variablen usw. sind in dieser datei, um diese ein wenig aufzuräumen 


void _imu_read()
{
  yaw = READ_I2C_BNO055_YAW();
  pitch = READ_I2C_BNO055_PITCH();
  roll = READ_I2C_BNO055_ROLL();
  _log("imu_read", "Y" + String(yaw)+" P"+String(pitch)+" R"+String(roll));
}
void _log(String name, String message)
{
  if (debug)
  {
    log_message = "\n## Got log from " + String(name) + " at " + String(millis()) + "ms:\n " + message;
    if (debug_over_serial)
    {
      Serial.println(log_message);
    }else
    {
      debug_log.push_back(log_message);
    }
  }
}

#include "Tests.h" // alle test codes 
#include "Move.h"



void setup()
{
  Serial.begin(serial_baud); // Hier lassen um debug im setup zuzulassen 

  // Initialisierung der Hardwarekomponenten des Controllers
  KEPLERBRAIN_INIT();
  Serial.println("KeplerBrainINIT succses -> Moving Forward in Setup");
  SLEEP(100); //um jegliche blockierungen zu vermeiden 
  WRITE_I2C_BNO055_INIT();
  Serial.println("BNO055_INIT succses -> Moving Forward in Setup");
  SLEEP(100); // --"--
  WRITE_I2C_INA231_INIT();
  Serial.println("INA231_INIT succses -> Moving Forward in Setup");
  SLEEP(100); // --"--
  WRITE_LCD_CLEAR();
  Serial.println("Setup finished now jumping in loop(). HAVE FUN!!!");
  KEPLER_UPDATE(); //einmal update
}
 


void loop()
{
  KEPLER_UPDATE(); //um hintergrund aktionen automatisch auszuführen 
  //_log("void loop", "run is " + String(run));
  if (!run)
  {
    if (READ_BUTTON_CLOSED(B1) == 1 && selection_cursor > 0){selection_cursor--;WRITE_LCD_TEXT(1, 2, "o");}
    if (READ_BUTTON_CLOSED(B3) == 1 && selection_cursor < 10){selection_cursor++;WRITE_LCD_TEXT(9, 2, "o");}
    delay(200);

    //Display
    WRITE_LCD_TEXT(1, 1, String(selection_cursor) + "              ");

    if (READ_BUTTON_CLOSED(B2) == 1){

      WRITE_LCD_TEXT(4, 2, "<>");
      while(READ_BUTTON_CLOSED(B2) == 1){}
      WRITE_LCD_TEXT(4, 2, "()");

      delay(500);

      selection = selection_cursor;

      _log("main loop", "Selected Program" + String(selection));

      run = true;
    }
  }else
  {
    //_log("void loop", "run is true");
    switch ( selection )
    {
      case 0:
        while (true)
        {
          //KEPLER_UPDATE();
          _default();
        }

      break;;

      case 1:
        WRITE_LCD_TEXT(1, 2, "Motor Test");
        delay(700);
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _motor_test();

        run = false;

      break;;

      case 2:
        WRITE_LCD_TEXT(1, 2, "IMU Test");
        delay(700);
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _imu_test();

        run = false;

      break;;

      case 3:
        WRITE_LCD_TEXT(1, 2, "Ground Test");
        delay(700);
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _ground_test();
        
        run = false;

      break;;

      case 4:
        WRITE_LCD_TEXT(1, 2, "Show BATT info");
        delay(700);
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _batt_test();
        run = false;

      break;;


      case 5:
        WRITE_LCD_TEXT(1, 2, "IMU READ"); 
        _SPIs(); 
        _imu_read();
        //_default();
      break;;      


      case 6:
        WRITE_LCD_TEXT(1, 2, "BODEN READ"); 
        delay(700);
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}


      break;;


      case 7:
        WRITE_LCD_TEXT(1, 2, "DUMP LOG"); 

      break;;


      case 8:

      break;;   

    }

  }

  if (selection_cursor <= 0){WRITE_LCD_TEXT(1, 2, "   ()   +");}  
  else if (selection_cursor >= 5){WRITE_LCD_TEXT(1, 2, "-  ()    ");}  
  else{WRITE_LCD_TEXT(1, 2, "-  ()   +");}  
}

int check_ground_error_min(int base)
{
  return std::min( { fl - base, fc - base, fr - base, rr - base, br - base, bc - base, bl - base, ll - base } );
}

int smallest_ground_sensor_id(int base)
{
  int id = 255;
  ground_smallest = 1023;

  ground_sensor[0] = fc;
  ground_sensor[1] = fr;
  ground_sensor[2] = rr;
  ground_sensor[3] = br;
  ground_sensor[4] = bc;
  ground_sensor[5] = bl;
  ground_sensor[6] = ll;
  ground_sensor[7] = fl;


  for (int i = 0; i < 8; i++)
  {
    if ( ground_sensor[i] - base < ground_smallest ) {
       ground_smallest = ground_sensor[i] - base;
       id = i; 
    }

  }

  return id;
}

int check_ground_sensor(int base, int threshold)
{
  // fl 0
  // fc 1
  // fr 2
  // rr 3
  // br 4
  // bc 5
  // bl 6
  // ll 7

  if(abs(fc - base) >= threshold) {line_tmp[0] = true;} else {line_tmp[0] = false;}
  if(abs(fr - base) >= threshold) {line_tmp[1] = true;} else {line_tmp[1] = false;}
  if(abs(rr - base) >= threshold) {line_tmp[2] = true;} else {line_tmp[2] = false;}
  if(abs(br - base) >= threshold) {line_tmp[3] = true;} else {line_tmp[3] = false;}
  if(abs(bc - base) >= threshold) {line_tmp[4] = true;} else {line_tmp[4] = false;}
  if(abs(bl - base) >= threshold) {line_tmp[5] = true;} else {line_tmp[5] = false;}
  if(abs(ll - base) >= threshold) {line_tmp[6] = true;} else {line_tmp[6] = false;}
  if(abs(fl - base) >= threshold) {line_tmp[7] = true;} else {line_tmp[7] = false;}

  if( line_tmp[0] == true ) {/*fc */ if( line_tmp[4] == true                        ) {return line_last;} line_last = 180;}
  if( line_tmp[1] == true ) {/*fr */ if( line_tmp[5] == true || line_tmp[7] == true ) {return line_last;} line_last = 135;}
  if( line_tmp[2] == true ) {/*rr */ if( line_tmp[6] == true                        ) {return line_last;} line_last = 90; }
  if( line_tmp[3] == true ) {/*br */ if( line_tmp[7] == true || line_tmp[5] == true ) {return line_last;} line_last = 45; }
  if( line_tmp[4] == true ) {/*bc */ line_last = 0;  }
  if( line_tmp[5] == true ) {/*bl */ line_last = 315;}
  if( line_tmp[6] == true ) {/*ll */ line_last = 270;}
  if( line_tmp[7] == true ) {/*fl */ line_last = 225;}

  return line_last;
}

int check_ground_sensor_time(int base, int threshold, int time_threshold_ms)
{
  //_log("check_ground_sensor_time", String(abs(fl - base)));


  if(abs(fc - base) >= threshold) {
      if( millis() - line_timers[0] >= time_threshold_ms) {
       line_timers[0] = millis(); return 0;}
        line_timers[0] = millis(); 
  }else
  {
    line_timers[0] = 0; 
  }

  if(abs(fr - base) >= threshold) {
      if( millis() - line_timers[1] >= time_threshold_ms) {
       line_timers[1] = millis(); return 45;}
        line_timers[1] = millis(); 
  }else
  {
    line_timers[1] = 0; 
  }

  if(abs(rr - base) >= threshold) {
      if( millis() - line_timers[2] >= time_threshold_ms) {
       line_timers[2] = millis(); return 90;}
        line_timers[2] = millis(); 
  }else
  {
    line_timers[2] = 0; 
  }

  if(abs(br - base) >= threshold) {
      if( millis() - line_timers[4] >= time_threshold_ms) {
       line_timers[4] = millis(); return 135;}
        line_timers[4] = millis(); 
  }else
  {
    line_timers[4] = 0; 
  }

  if(abs(bc - base) >= threshold) {
      if( millis() - line_timers[4] >= time_threshold_ms) {
       line_timers[4] = millis(); return 180;}
        line_timers[4] = millis(); 
  }else
  {
    line_timers[4] = 0; 
  }

  if(abs(bl - base) >= threshold) {
      if( millis() - line_timers[5] >= time_threshold_ms) {
       line_timers[5] = millis(); return 225;}
        line_timers[5] = millis(); 
  }else
  {
    line_timers[5] = 0; 
  }

  if(abs(ll - base) >= threshold) {
      if( millis() - line_timers[8] >= time_threshold_ms) {
       line_timers[6] = millis(); return 270;}
        line_timers[6] = millis(); 
  }else
  {
    line_timers[6] = 0; 
  }
  if(abs(fl - base) >= threshold) {
   if( millis() - line_timers[1] >= time_threshold_ms) {
    line_timers[7] = millis(); return 315;}
     line_timers[7] = millis(); 
  }else
  {
    line_timers[7] = 0; 
  }

  return -1;

}

void _default(){
  _imu_read();
  _SPIs();

  _log("default", "Done SPIs");

  ground_avg = (fc + fr + rr + br + bc + bl + ll + fl)/8;
  
  smallest_ground_sensor_id(ground_avg);

  _log("ground smallest", String(ground_smallest));
  
  

  if (ground_smallest < -2)
  {
    if(!reverse) motors(-drive_m1, -drive_m2, -drive_m3, -drive_m4, true);
    //rotate_to(0, 2, 10, 0.000004, 0.00000006, 7);
    reverse = true;
    WRITE_LCD_TEXT(1, 1, "STOOOP");
    //delay(1000);
    return;
    //motors(0, 0, 0, 0, true);
  }

  reverse = false;

  motors(drive_m1, drive_m2, drive_m3, drive_m4, true);

  rotate_to(0, 2, 10, 0.000004, 0.00000006, 7);
  //move_angle_correction(45, 40, 1);

  
}



