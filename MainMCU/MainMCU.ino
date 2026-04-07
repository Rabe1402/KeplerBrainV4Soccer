#include <vector>
#include <iostream>
#include <math.h>
#include "Variables.h" //alle variablen usw. sind in dieser datei, um diese ein wenig aufzuräumen 

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
  _log("void loop", "run is " + String(run));
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

      _log("main loop", "Selected Program" + String(selection));

      run = true;
      WRITE_LCD_CLEAR(); // to clean up dislplay. And hopefully find wehre it hangs 
    }
  }else
  {
    _log("void loop", "run is true");
    switch ( selection )
    {
      case 0:
        WRITE_LCD_TEXT(1, 2, "Default Code");
        delay(700);
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
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _motor_test();

        run = false;

      break;;

      case 2:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "IMU Test");
        delay(700);
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _imu_test();

        run = false;

      break;;

      case 3:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "Ground Test");
        delay(700);
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _ground_test();
        
        run = false;

      break;;

      case 4:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "Show BATT info");
        delay(700);
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

      break;;


      case 8:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "Camera Test");
        delay(700);
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}
        _camera_test();
      break;;   

      case 9:
        WRITE_LCD_TEXT(1, 2, "Camera Test direct SPI read");
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


void _default(){
  _imu_read();
  _SPIs();

  ground_avg = (fc + fr + rc + br + bc + bl + lc + fl) / 8;
  ground_sens_id = smallest_ground_sensor_id(ground_avg);

  if (ground_smallest < -2)
  {
    //ground_millis = millis() + 200;
    move_angle(ground_sens_id * 45, target_speed);
    
    motors(drive_m1, drive_m2, drive_m3, drive_m4, true);
    delay(200);
    return;;
  }
  
  motors(drive_m1, drive_m2, drive_m3, drive_m4, true);

  if(SPICAM_Data1 == 0)
  {
    ball_target_locked = false;  // Lock zurücksetzen wenn Ball weg
    rotate(8);
    WRITE_LCD_TEXT(1, 1, "NO BALL    ");
  }
  else
  {
    int cam_angle = _cam_data_calculation();

    // Target nur neu setzen wenn noch nicht eingerastet
    // oder Ball fast zentriert ist (< 2°)
    if (!ball_target_locked || abs(cam_angle) < 2 || last_ball_locked_time + 250 < millis())
    {
      last_ball_locked_time = millis();
      ball_target = yaw + cam_angle;
      ball_target_locked = true;
    }

    WRITE_LCD_TEXT(1, 1, String(ball_target) + " " + String(cam_angle) + "   ");
    //rotate_to_quadratic(ball_target, 2, 23, 0.0000004, 0.00000004, 0);
    move_angle(cam_angle, 40);
  }
  WRITE_LCD_TEXT(1, 2, String(counter));
}

void _default_statemachiene(){
  _SPIs();
  _imu_read();

  WRITE_LCD_TEXT(1, 1, String(current_state) );

  //WRITE_LCD_TEXT(1, 2, String(counter) ); //debug
  switch (current_state)
  {
    case 0: // search
    //rotate (preferabbly in last seen dir) till found
      if(ball_last_seen_ang > 180)
      {
        rotate( 10);
      }else{
        rotate(-10);
      }

    break;; //exit here
    
    //--------------
    
    case 1: // rotate to ball
    {
      int cam_angle = _cam_data_calculation();
      rotate_to_quadratic(ball_target, 2, 23, 0.0000004, 0.00000004, 0);

      // Target nur neu setzen wenn noch nicht
      // oder Ball fast zentriert ist (< 2°)
      // und fix alle 250ms
      if (!ball_target_locked || abs(cam_angle) < 2 || last_ball_locked_time + 250 < millis())
      {
        last_ball_locked_time = millis();
        ball_target = yaw + cam_angle;
        ball_target_locked = true;  
      }

      //WRITE_LCD_TEXT(1, 1, String(ball_target) + " " + String(cam_angle) + "   ");
      //rotate_to_quadratic(ball_target, 2, 23, 0.0000004, 0.00000004, 0);
      //move_angle(cam_angle, 40);
    }
    break;; //exit here
    
    //--------------
    
    case 2: // shoot
    {
      int cam_angle = _cam_data_calculation();
      ball_target = yaw + cam_angle;

      orbit_to_zero(ball_target, target_speed/2, 30);
    }
    break;; //exit here
    
    //--------------
    
    case 3: // line
      // Verwende den gespeicherten Sensor für konsistente Ausweichrichtung
      move_angle((line_first_sensor_id * 45 + 180) % 360, target_speed/2);

      if (millis() - line_last_seen_millis > allow_sens_again)
      {
        sens_allowed = true;
      }

      if ((millis() - line_escape_start_time > line_escape_duration) && ground_smallest > -2) 
      {
        line_first_sensor_id = -1;  // Reset für nächste Erkennung

        // Wechsle zurück zu search oder move_to_ball je nach Ball-Status
        /*if (SPICAM_Data1 == 0) 
        {
          current_state = 0; // search
          sens_allowed = true;
          return;;
        } else 
        {
          current_state = 1; // move to ball
          sens_allowed = true;
          return;; 
        }*/
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
    
    current_state = 3; //line
    return;;
  }

  // search
  if (SPICAM_Data1 == 0)
  {
    current_state = 0; //search
    return;;
  }
  // move to ball
  if (SPICAM_Data1 == 1 && SPICAM_Data2 < 85 && SPICAM_Data2 > 95 && current_state != 2) //rotate to ball if angle is bigger than 5° and smaller than 175° (also ignore if ball is behind us)
  {
    current_state = 1; //rotate to ball

    return;;
  }

  if (SPICAM_Data1 == 1 && (SPICAM_Data2 >= 85 || SPICAM_Data2 <= 95)) //shoot if ball is in front of us (also ignore if ball is behind us)
  {
    current_state = 2; //orbit to zero and "shoot"

    return;;
  }
}