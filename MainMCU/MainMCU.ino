#include <vector>
#include <iostream>
#include <math.h>
#include "Variables.h" //alle variablen usw. sind in dieser datei, um diese ein wenig aufzuräumen 

void _log(String name, String message)
{
  if (debug)
  {
    // Timestamp + Name + Message als kompakten String
    String log_entry = String(millis()) + " [" + name + "] " + message;
    
    if (debug_over_serial)
    {
      Serial.println(log_entry);
    }
    else
    {
      debug_log.push_back(log_entry);
    }
  }
}

void _log_Dump()
{
  Serial.println("--- Log Dump Start ---");
  
  for(int i = 0; i < debug_log.size(); i++)
  {
    Serial.println(debug_log[i]);
  }
}

#include "../shared/KeplerBRAIN_V4.h"


#include "KEPLER_UPDATE.h" // wird in echte header migrirt wenn randl gut findet 
#include "powersense.h" //powersensor read 
#include "data-calculation.h" //alle datenberechnungen wie winkel zu ball usw.



#include "Move.h"
#include "Tests.h" // alle test codes 


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
  _log("VOID LOOP", "run is " + String(run));
  if (!run)
  {
    if (READ_BUTTON_CLOSED(B1) == 1 && selection_cursor > 0){selection_cursor--;WRITE_LCD_TEXT(1, 2, "o");}
    if (READ_BUTTON_CLOSED(B3) == 1 && selection_cursor < 10){selection_cursor++;WRITE_LCD_TEXT(9, 2, "o");}
    delay(100);

    //Display
    WRITE_LCD_TEXT(1, 1, String(selection_cursor) + "              ");

    if (READ_BUTTON_CLOSED(B2) == 1){

      WRITE_LCD_TEXT(4, 2, "<>");
      while(READ_BUTTON_CLOSED(B2) == 1){}
      WRITE_LCD_TEXT(4, 2, "()");

      delay(500);

      selection = selection_cursor;

      _log("MAIN LOOP", "Selected Program" + String(selection));

      run = true;
      WRITE_LCD_CLEAR(); // to clean up dislplay. And hopefully find wehre it hangs 
    }
  }else
  {
    _log("VOID LOOP", "run is true");
    switch ( selection )
    {
      case 0:
        WRITE_LCD_TEXT(1, 2, "Default Code");
        delay(700);
        _log("CASE SWITCH", "Starting default code");
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        // run here the first time to get two set of data 
        ground_avg = (fc + fr + rc + br + bc + bl + lc + fl) / 8;
        ground_sens_id = smallest_ground_sensor_id(ground_avg);

        while (true)
        {
          WRITE_LCD_CLEAR();
          KEPLER_UPDATE();
          _default_statemachiene();
         
        }

      break;;

      case 1:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "Motor Test");
        delay(700);
        _log("CASE SWITCH", "Starting motor test");
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _motor_test();

        run = false;

      break;;

      case 2:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "IMU Test");
        delay(700);
        _log("CASE SWITCH", "Starting IMU test");
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _imu_test();

        run = false;

      break;;

      case 3:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "Ground Test");
        delay(700);
        _log("CASE SWITCH", "Starting ground sensor test");
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _ground_test();
        
        run = false;

      break;;

      case 4:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "Show BATT info");
        delay(700);
        _log("CASE SWITCH", "Starting battery test");
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _batt_test();
        run = false;

      break;;


      case 5:
        WRITE_LCD_TEXT(1, 2, "NOTHING HERE ");
        delay(1000);
        exit;

      break;;      


      case 6:
        WRITE_LCD_TEXT(1, 2, "NOTHING HERE");
        delay(1000);
        exit;


      break;;


      case 7:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "DUMP LOG"); 
        _log("CASE SWITCH", "dumped log with size " + String(debug_log.size()) + " on time; " + String(millis()));
        _log_Dump();
        
      break;;


      case 8:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "Camera Test");
        _log("CASE SWITCH", "Starting camera test");
        delay(700);
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}
        _camera_test();
      break;;   

      case 9:
        WRITE_LCD_TEXT(1, 2, "Camera Test direct SPI read");
        _log("CASE SWITCH", "Starting direct SPI read test");
        delay(700);
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}
        while(true)
        { 
          KEPLER_UPDATE();
          digitalWrite(SPICAM, LOW);
          delay(1);

          if(spi_cam.transfer(1) == 250)
          { 
            SPICAM_Data1 = spi_cam.transfer(0);
            SPICAM_Data2 = spi_cam.transfer(0);
            SPICAM_Data3 = spi_cam.transfer(0);
            SPICAM_Data4 = spi_cam.transfer(0);
            SPICAM_Data5 = spi_cam.transfer(0);
            SPICAM_Data6 = spi_cam.transfer(0);
            SPICAM_Data7 = spi_cam.transfer(0);
          }

          digitalWrite(SPICAM, HIGH);

          // read 8 Bytes from OpenMV END

          WRITE_LCD_TEXT(1, 1, String(SPICAM_Data1)+" "+String(SPICAM_Data2)+" "+String(SPICAM_Data3)+" "+String(SPICAM_Data4));
          WRITE_LCD_TEXT(1, 2, String(SPICAM_Data5)+" "+String(SPICAM_Data6)+" "+String(SPICAM_Data7));
      
          delay(1000);
          if (READ_BUTTON_CLOSED(B1) == 1){exit;}
        }
      break;;
    }

  }

  if (selection_cursor <= 0){WRITE_LCD_TEXT(1, 2, "   ()   +");}  
  else if (selection_cursor >= 5){WRITE_LCD_TEXT(1, 2, "-  ()    ");}  
  else{WRITE_LCD_TEXT(1, 2, "-  ()   +");}  
}


void _default_statemachiene()
{
  _SPIs();
  _imu_read();
  _log("MAIN", "running statemachine with state " + String(current_state));
  WRITE_LCD_TEXT(1, 1, String(current_state) );

  //WRITE_LCD_TEXT(1, 2, String(counter) ); //debug
  switch (current_state)
  {
    case 0: // search
    //rotate (preferabbly in last seen dir) till found
    {
      ball_last_seen_ang = _cam_data_calculation();
      if(ball_last_seen_ang > -90) //MUSS ANGEPASST WERDEN IDK OB SO RICHTIG 
      {
        rotate( 10); //should be ( target_speed /2 ) but for testing only 10
      }else{
        rotate(-10); //should be (-target_speed /2 ) but for testing only -10
      }
      _log("MAIN SWITCH", " [" + "STATE: " + String(current_state) + "] ""ball last seen angle: " + String(ball_last_seen_ang));
    }
    break;; //exit here
    
    //--------------
    
    case 1: // rotate to ball
    {
      int cam_angle = _cam_data_calculation();
      rotate_to_quadratic(ball_target, 2, 23, 0.0000004, 0.00000004, 0); //why so low speed??

      // Target nur neu setzen wenn noch nicht to avoid over steer 
      // oder Ball fast zentriert ist (< 2°)
      // und fix alle 250ms
      if (!ball_target_locked || abs(cam_angle) < 2 || last_ball_locked_time + 250 < millis())
      {
        last_ball_locked_time = millis();
        ball_target = yaw + cam_angle;
        ball_target_locked = true;  
      }

      //WRITE_LCD_TEXT(1, 1, String(ball_target) + " " + String(cam_angle) + "   ");
      _log("MAIN", " [" + "STATE: " + String(current_state) + "] ""ball target: " + String(ball_target) + " cam angle: " + String(cam_angle));
      //rotate_to_quadratic(ball_target, 2, 23, 0.0000004, 0.00000004, 0);
      //move_angle(cam_angle, 40);
    }
    break;; //exit here
    
    //--------------
    
    case 2: // orbit around ball till yaw is 0 
    {
      int yaw_orbit_target = 0; //needs better mechanic once we do look for goal with cam but for now we just want to orbit to 0

      int cam_angle = _cam_data_calculation();
      ball_target = yaw + cam_angle;

      error = yaw - yaw_orbit_target;
      if (error >= 180) {
        error = -(360 - error);
      }
      if (error > 10 || error < -10) //if not in target area
      {
        orbit_to_zero(ball_target, (target_speed/2), 30); //not working right idk why
      }else
      {        
        move_angle(error, target_speed); //move to ball if in target area
      }  
      _log("MAIN", " [" + "STATE: " + String(current_state) + "] ""ball target: " + String(ball_target) + " cam angle: " + String(cam_angle) + " goal-angle: " + String(yaw_orbit_target) );
    }
    break;; //exit here
    
    //--------------
    
    case 3: // line
    {
      // Verwende den gespeicherten Sensor für konsistente Ausweichrichtung
      move_angle((line_first_sensor_id * 45 + 180) % 360, target_speed/2);

      if (millis() - line_last_seen_millis > allow_sens_again)
      {
        sens_allowed = true;
      }

      if ((millis() - line_escape_start_time > line_escape_duration) && ground_smallest > -2) 
      {
        line_first_sensor_id = -1;  // Reset für nächste Erkennung

        current_state = last_state; // Rückkehr zum vorherigen Zustand
        line_escape_start_time = 0; // Reset Timer
        sens_allowed = true; // Sensoren wieder erlauben
      }      

      _log("MAIN", " [" + "STATE: " + String(current_state) + "] ""line sensor: " + String(line_sens_id) + " line smallest: " + String(line_smallest) + " escape timer: " + String(millis() - line_escape_start_time) );
    }
    break;; //exit here (last one not needed ig)
  }

  motors(drive_m1, drive_m2, drive_m3, drive_m4, true);

  // determine state
  
  // run here second time for second set of date and continous readings
  ground_avg = (fc + fr + rc + br + bc + bl + lc + fl) / 8;
  ground_sens_id = smallest_ground_sensor_id(ground_avg);

  //Linie (muss ganz oben bleiben) 
  if (millis() - line_escape_start_time > line_escape_duration && line_escape_start_time != 0) 
  {
    if (last_state != 3) //set last_state for code exit block
    {
      last_state = current_state;
    }
    current_state = 3;
    return;;
  }

  if (ground_smallest < -2)
  {
    line_last_seen_millis = millis();
    
    // Speichere den ersten Sensor, der die Linie erkannt hat
    if (current_state != 3 || (line_first_sensor_id != -1 && line_first_sensor_id != ground_sens_id) && sens_allowed) 
    {
      line_first_sensor_id = ground_sens_id;
      WRITE_LCD_TEXT(2, 1, String(line_first_sensor_id) );
      sens_allowed = false;
      line_escape_start_time = millis();
    }
    if (last_state != 3) //set last_state for code exit block
    {
      last_state = current_state;
    }
    current_state = 3; //line
    return;;
  }

  // search
  if (SPICAM_Data1 == 0)
  {

    if (last_state != 0) //set last_state for code exit block
    {
      last_state = current_state;
    }
    current_state = 0; //search

    return;;
  }
  // move to ball
  if (SPICAM_Data1 == 1 && SPICAM_Data2 < 85 && SPICAM_Data2 > 95 && current_state != 2) //rotate to ball if angle is bigger than 5° and smaller than 175° (also ignore if ball is behind us)
  {
    if (last_state != 1) //set last_state for code exit block
    {
      last_state = current_state;
    }
    current_state = 1; //rotate to ball

    return;;
  }

  if (SPICAM_Data1 == 1 && (SPICAM_Data2 >= 85 || SPICAM_Data2 <= 95)) //shoot if ball is in front of us (also ignore if ball is behind us)
  {
    if (last_state != 2) //set last_state for code exit block
    {
      last_state = current_state;
    }
    current_state = 2; //orbit to zero and "shoot"

    return;;
  }
}