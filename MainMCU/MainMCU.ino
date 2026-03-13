//#include "/home/arch/daata/School/6_klasse/IT/KeplerBrainV4Soccer/shared/KeplerBRAIN_V4.h" //idk warum aber ../shared/KE... geht bei mir ned mehr.
#include "../shared/KeplerBRAIN_V4.h"
#include "KEPLER_UPDATE.h" // wird in echte header migrirt wenn randl gut findet 
#include "powersense.h" //powersensor read 

#include <vector>
#include <iostream>
#include <math.h>
#include "data-calculation.h" //alle datenberechnungen wie winkel zu ball usw.

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
          KEPLER_UPDATE();
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
        _camera_test();
      break;;   

      case 9:

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
      break;;
    }

  }

  if (selection_cursor <= 0){WRITE_LCD_TEXT(1, 2, "   ()   +");}  
  else if (selection_cursor >= 5){WRITE_LCD_TEXT(1, 2, "-  ()    ");}  
  else{WRITE_LCD_TEXT(1, 2, "-  ()   +");}  
}

int smallest_ground_sensor_id(int base)
{
  int id = 255;
  ground_smallest = 1023;

  ground_sensor[0] = fc;
  ground_sensor[1] = fr;
  ground_sensor[2] = rc;
  ground_sensor[3] = br;
  ground_sensor[4] = bc;
  ground_sensor[5] = bl;
  ground_sensor[6] = lc;
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

  }else
  //move_angle_correction(45, 40, 1);
  //rotate_to(0, 2, 10, 0.000004, 0.00000006, 7);
  {
    //move to ball
    if(!reverse) {
    rotate(-50);
    reverse = true;
    delay(672);
    }
    rotate(_cam_data_calculation() / 3);
    ball_target = yaw + _cam_data_calculation();
    WRITE_LCD_TEXT(1,1, String(ball_target) + "   ");
    rotate_to_quadratic(ball_target, 2, 23, 0.0000004, 0.00000004, 0);
  
    //move_angle(_cam_data_calculation(), 40);
  }

}