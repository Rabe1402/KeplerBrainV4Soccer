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
        while (true)
        {
          KEPLER_UPDATE();
          _default();
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
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "IMU READ"); 
        _SPIs(); 
        _imu_read();
        //_default();
      break;;      


      case 6:
        KEPLER_UPDATE();
        WRITE_LCD_TEXT(1, 2, "BODEN READ"); 
        delay(700);
        if (READ_BUTTON_CLOSED(B1) == 1){exit;}


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

  _log("default", "Done SPIs");
  ground_avg = (fc + fr + rc + br + bc + bl + lc + fl)/8;
  
  smallest_ground_sensor_id(ground_avg);

  _log("ground smallest", String(ground_smallest));
  
  

  /*if (ground_smallest < -2)
  {
    if(!reverse) motors(-drive_m1, -drive_m2, -drive_m3, -drive_m4, true);
    //rotate_to(0, 2, 10, 0.000004, 0.00000006, 7);
    reverse = true;
    WRITE_LCD_TEXT(1, 1, "STOOOP");
    //delay(1000);
    return;
    //motors(0, 0, 0, 0, true);
  }*/

  //reverse = false;

  motors(drive_m1, drive_m2, drive_m3, drive_m4, true);

  if(SPICAM_Data1 == 0)
  {
    //rotate to dir where ball last seen (future plan)?
    rotate(10);
    reverse = false;
    WRITE_LCD_TEXT(1, 1, "NO BALL");

  }else
  {
   motors(0, 0, 0, 0, true);
    //move to ball
    ball_target = yaw + _cam_data_calculation();
    WRITE_LCD_TEXT(1,1, String(ball_target) + " " + String(_cam_data_calculation()));
    rotate_to_quadratic(ball_target, 2, 23, 0.0000004, 0.00000004, 0);
    move_angle(_cam_data_calculation(), 40);
  }
  WRITE_LCD_TEXT(1, 2, String(counter));
}

