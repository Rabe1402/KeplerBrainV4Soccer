#include <vector>
#include <iostream>
#include <math.h>
#include "Variables.h" //alle variablen usw. sind in dieser datei, um diese ein wenig aufzuräumen 

void _log(String name, String component, String message, bool error = false)
{
  if (debug)
  {
    // Timestamp + Name + Message als kompakten String
    String log_entry = String(millis()) + " . " + name + " . " + component + " . " + message + " . " + String(error);
    
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
  Serial.println("got log dump request with log size: " + String(debug_log.size()));
  
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
  Serial.begin(115200); // Hier lassen um debug im setup zuzulassen 

  // Initialisierung der Hardwarekomponenten des Controllers
  KEPLERBRAIN_INIT();
  _log("SETUP", "KeplerBRAIN", "Hardware initialization complete, starting sensor initialization...");
  SLEEP(100); //um jegliche blockierungen zu vermeiden 
  WRITE_LCD_CLEAR();
  WRITE_LCD_TEXT(1, 1, "Imu calib in 1s!!");
  WRITE_LCD_TEXT(1, 2, "please put robot on flat surface");
  _log("SETUP", "IMU", "Starting Imu calibration in 1s, please place robot on flat surface and wait a few seconds");
  SLEEP(1000);
  WRITE_I2C_BNO055_INIT();
  delay(3000);
  WRITE_LCD_TEXT(1, 1, "Imu calib done!"),
  WRITE_LCD_TEXT(1, 2, "check with READ_I2C_IS_BNO055_READY()");
  _log("SETUP", "IMU","BNO055_INIT calib is finsihed, check with IS_BNO055_READY()");
  if (READ_I2C_IS_BNO055_READY()) {
    _log("SETUP", "IMU", "BNO055 is ready!");
    WRITE_LCD_TEXT(1, 2, "IMU Ready!");
  } else 
  {
    _log("SETUP", "IMU", "BNO055 is NOT ready! Check wiring and connections.", true);
    WRITE_LCD_TEXT(1, 2, "IMU NOT Ready!");
    delay(7000);
  }
  SLEEP(100);
  WRITE_I2C_INA231_INIT();
  _log("SETUP", "POWER","INA231_INIT successful");
  SLEEP(100); // --"--
  WRITE_LCD_CLEAR();
  _log("SETUP", "KeplerBRAIN", "Setup finished, jumping to loop(). HAVE FUN!!!");
  KEPLER_UPDATE(); //einmal update
}
 


void loop()
{
  KEPLER_UPDATE(); //um hintergrund aktionen automatisch auszuführen 
  _log("VOID LOOP", "KeplerBRAIN", "run is " + String(run));
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

      _log("VOID LOOP", "MENU", "Selected Program" + String(selection));

      run = true;
      WRITE_LCD_CLEAR(); // to clean up dislplay. And hopefully find wehre it hangs 
    }
  }else
  {
    _log("VOID LOOP", "KeplerBRAIN", "run is true");
    switch ( selection )
    {
      case 0:
        WRITE_LCD_TEXT(1, 2, "Default Code");
        delay(700);
        _log("VOID LOOP", "MENU", "Starting default code");
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
        _log("VOID LOOP", "MENU", "Starting motor test");
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _motor_test();

        run = false;

      break;;

      case 2:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "IMU Test");
        delay(700);
        _log("VOID LOOP", "MENU", "Starting IMU test");
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _imu_test();

        run = false;

      break;;

      case 3:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "Ground Test");
        delay(700);
        _log("VOID LOOP", "MENU", "Starting ground sensor test");
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _ground_test();
        
        run = false;

      break;;

      case 4:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "Show BATT info");
        delay(700);
        _log("VOID LOOP", "MENU", "Starting battery test");
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}

        _batt_test();
        run = false;

      break;;


      case 5:
        WRITE_LCD_TEXT(1, 2, "NOTHING HERE ");
        _log("VOID LOOP", "MENU", "Nothing here, going back to menu in .2s");
        delay(200);
        run = false;

      break;;      


      case 6:
        WRITE_LCD_TEXT(1, 2, "Move 0");
        _log("VOID LOOP", "MENU", "Starting move test, press B1 to exit");
        while((READ_BUTTON_CLOSED(B1) == 0))
        {
        _imu_read();
        rotate_to(0, 3, 3, 10, 10, 0 );
        motors(drive_m1, drive_m2, drive_m3, drive_m4, true);
        }
        

      break;;


      case 7:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "DUMP LOG"); 
        _log("VOID LOOP", "MENU", "dumped log with size " + String(debug_log.size()) + " on time; " + String(millis()));
        _log_Dump();
        
      break;;


      case 8:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "Camera Test");
        _log("VOID LOOP", "MENU", "Starting camera test");
        delay(700);
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}
        _camera_test();
      break;;   

      case 9:
        WRITE_LCD_TEXT(1, 2, "Camera Test direct SPI read");
        _log("VOID LOOP", "MENU", "Starting direct SPI read test, press B1 to exit");
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
          _log("TEST", "SPI_CAM", "SPICAM Raw Data: " + String(SPICAM_Data1) + " " + String(SPICAM_Data2) + " " + String(SPICAM_Data3) + " " + String(SPICAM_Data4) + " " + String(SPICAM_Data5) + " " + String(SPICAM_Data6) + " " + String(SPICAM_Data7));
          delay(1000);
          if (READ_BUTTON_CLOSED(B1) == 1){exit;}
        }
      break;;
    }

  }

  if (selection_cursor <= 0){WRITE_LCD_TEXT(1, 2, "   ()   +");}  
  else{WRITE_LCD_TEXT(1, 2, "-  ()   +");}  
}


void _default_statemachiene()
{
  int cam_angle = _cam_data_calculation();
  _imu_read();
  orbit_ang = yaw;
  if (orbit_ang > 180) {
    orbit_ang = orbit_ang - 360; // 350° wird zu -10°, 270° wird zu -90° usw.
  }
  _SPIs();
  _log("MAIN", "STATE MACHINE", "running state: " + String(current_state));
  WRITE_LCD_TEXT(1, 1, String(current_state) );

  //WRITE_LCD_TEXT(1, 2, String(counter) ); //debug
  switch (current_state)
  {
    case 0: // search
    //rotate (preferabbly in last seen dir) till found
    {
      ball_last_seen_ang = _cam_data_calculation();
      if(ball_last_seen_ang > 0) //MUSS ANGEPASST WERDEN IDK OB SO RICHTIG 
      {
        rotate( 9); //should be ( target_speed /2 ) but for testing only 10
      }else{
        rotate(-9); //should be (-target_speed /2 ) but for testing only -10
      }
      _log("MAIN", "STATE MACHINE", "[STATE: " + String(current_state) + "] ball last seen angle: " + String(ball_last_seen_ang));
    }
    break;; //exit here
    
    //--------------
    
    case 1: // rotate to ball
    {
      
      // Target nur neu setzen wenn noch nicht to avoid over steer 
      // oder Ball fast zentriert ist (< 2°)
      // und fix alle 250ms
      if (!ball_target_locked || abs(cam_angle) < 2 || last_ball_locked_time + 250 < millis())
      {
        
        last_ball_locked_time = millis();
        ball_target_locked = true;  
      }
      //ball_target = yaw + cam_angle;
      //rotate_quadratic(cam_angle, 2, 2, 11, 10, 0, 15); 
      rotate_quadratic(cam_angle, 2, 15, 17, 7);
      WRITE_LCD_TEXT(1, 2, String(cam_angle) + "   ");
      _log("MAIN", "STATE MACHINE", "[STATE: " + String(current_state) + "] ball target: " + String(ball_target) + " cam angle: " + String(cam_angle));
      //rotate_to_quadratic(ball_target, 2, 23, 0.0000004, 0.00000004, 0);
      //move_angle(cam_angle, 40);
    }
    break;; //exit here
    
    //--------------
    
    case 2: // orbit around ball till yaw is 0 
    {
      ball_target = yaw + cam_angle;
      /*if (error > 5 || error < -5) //if not in target area
      {
        orbit_to_zero(ball_target, (target_speed/2), 30); //not working right idk why
      }else
      {        
        move_angle(error, target_speed); //move to ball if in target area
      }  */
      //orbit_around(ball_target, (target_speed/2), 30); //not working right idk why
      
        if(orbit_ang > 0)
        {
          orbit(25, -80, 2.5); 
          
        }
        else
        {
          orbit(25, 80, -2.5);
          //25, 80, -3 geht gut wenn er nah ist, 60 damit er rein spiraliert wenn er weit weg ist.
        }

      WRITE_LCD_TEXT(1, 2, String(orbit_ang) + "   ");
      _log("MAIN", "STATE MACHINE", "[STATE: " + String(current_state) + "] ball target: " + String(ball_target) + " cam angle: " + String(cam_angle) + " goal-angle: " + String(yaw_orbit_target));
    
    }
    break;; //exit here
    
    //--------------
    
    case 3: // line
    {
      // Verwende den gespeicherten Sensor für konsistente Ausweichrichtung
      move_angle((line_first_sensor_id * 45 + 180) % 360, target_speed);

      if ((millis() - line_last_seen_millis) > allow_sens_again)
      {
        sens_allowed = true;
      }

      if (((millis() - line_escape_start_time) > line_escape_duration) && ground_smallest > line_threshold) 
      {
        line_first_sensor_id = -1;  // Reset für nächste Erkennung

        current_state = last_state; // Rückkehr zum vorherigen Zustand
        line_escape_start_time = 0; // Reset Timer
        sens_allowed = true; // Sensoren wieder erlauben
      }      

      _log("MAIN", "STATE MACHINE", "[STATE: " + String(current_state) + "] line sensor: " + String(line_first_sensor_id) + " line smallest: " + String(ground_smallest) + " escape timer: " + String(millis() - line_escape_start_time));
    }
    break;; //exit here (last one not needed ig)

    case 4: // move to ball
    {
      move_angle(_cam_data_calculation(), 30);
      _log("MAIN", "STATE MACHINE", "[STATE: " + String(current_state) + "] ball target: " + String(ball_target) + " cam angle: " + String(cam_angle));
    }
    break;;

    case 5: // shoot
    {
      move_angle(0, 45);
      _log("MAIN", "STATE MACHINE", "[STATE: " + String(current_state) + "] SHOOTING! ball target: " + String(ball_target) + " cam angle: " + String(cam_angle));
    }
    break;;
  }

  motors(drive_m1, drive_m2, drive_m3, drive_m4, true);

  // determine state


  ground_avg = (fc + fr + rc + br + bc + bl + lc + fl) / 8;
  ground_sens_id = smallest_ground_sensor_id(ground_avg);

  //Linie (muss ganz oben bleiben) 
  if (ground_smallest < line_threshold)
  {
    line_last_seen_millis = millis();

    if (current_state != 3) {
        last_state = current_state;          // state retten
        line_first_sensor_id = ground_sens_id; // sensor speichern
        line_escape_start_time = millis();   // timer starten
    }

    current_state = 3;
    return;
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

  // State 1: drehen bis Ball zentriert
  if (SPICAM_Data1 == 1 && (_cam_data_calculation() > 15 || _cam_data_calculation() < -15))
  {
    last_state = current_state;
    current_state = 1;
    return;
  }

  // State 4: Ball zentriert aber zu weit weg → hinfahren
  if (SPICAM_Data1 == 1 && abs(_cam_data_calculation()) <= 15 && _cam_distance_calculation() > 220)
  {
    last_state = current_state;
    current_state = 4;
    return;
  }

  if (SPICAM_Data1 == 1 && abs(_cam_data_calculation()) <= 15 
    && _cam_distance_calculation() <= 220
    && (orbit_ang > -20 && orbit_ang < 20))
  {
    last_state = current_state;
    current_state = 5;
    return;
  }
  // State 2: Ball zentriert und nah genug → orbiten
  if (SPICAM_Data1 == 1 && abs(_cam_data_calculation()) <= 15 && _cam_distance_calculation() <= 220)
  {
    last_state = current_state;
    current_state = 2;
    return;
  }

}