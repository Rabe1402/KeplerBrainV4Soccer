#include "../shared/KeplerBRAIN_V4.h"
#include "KEPLER_UPDATE.h" // wird in echte header migrirt wenn randl gut findet 
#include "powersense.h" //powersensor read 

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
    log_message = "\n## Got log from " + String(name) + " at " + String(millis()) + "ms:\n " + String(message);
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
    switch ( selection )
    {
      case 0:
        _default();

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
        _default();
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

int check_ground_error(int base)
{
  return std::max( { abs(fl - base), abs(fc - base), abs(fr - base), abs(rr - base), abs(br - base), abs(bc - base), abs(bl - base), abs(ll - base) } );
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

  if(abs(fl - base) >= threshold) {return 0;}
  if(abs(fc - base) >= threshold) {return 1;}
  if(abs(fr - base) >= threshold) {return 2;}
  if(abs(rr - base) >= threshold) {return 3;}
  if(abs(br - base) >= threshold) {return 4;}
  if(abs(bc - base) >= threshold) {return 5;}
  if(abs(bl - base) >= threshold) {return 6;}
  if(abs(ll - base) >= threshold) {return 7;}
}

int check_ground_sensor_time(int base, int threshold, int time_threshold_ms)
{
  if(abs(fl - base) >= threshold) {
     if( millis() - line_timers[1] >= time_threshold_ms) {
      line_timers[1] = millis(); return 1;}
       line_timers[1] = millis(); 
  }else
  {
    line_timers[1] = 0; 
  }

  if(abs(fc - base) >= threshold) {
      if( millis() - line_timers[2] >= time_threshold_ms) {
       line_timers[2] = millis(); return 2;}
        line_timers[2] = millis(); 
  }else
  {
    line_timers[2] = 0; 
  }

  if(abs(fr - base) >= threshold) {
      if( millis() - line_timers[3] >= time_threshold_ms) {
       line_timers[3] = millis(); return 3;}
        line_timers[3] = millis(); 
  }else
  {
    line_timers[3] = 0; 
  }

  if(abs(rr - base) >= threshold) {
      if( millis() - line_timers[4] >= time_threshold_ms) {
       line_timers[4] = millis(); return 4;}
        line_timers[4] = millis(); 
  }else
  {
    line_timers[4] = 0; 
  }

  if(abs(br - base) >= threshold) {
      if( millis() - line_timers[5] >= time_threshold_ms) {
       line_timers[5] = millis(); return 5;}
        line_timers[5] = millis(); 
  }else
  {
    line_timers[5] = 0; 
  }

  if(abs(bc - base) >= threshold) {
      if( millis() - line_timers[6] >= time_threshold_ms) {
       line_timers[6] = millis(); return 6;}
        line_timers[6] = millis(); 
  }else
  {
    line_timers[6] = 0; 
  }

  if(abs(bl - base) >= threshold) {
      if( millis() - line_timers[7] >= time_threshold_ms) {
       line_timers[7] = millis(); return 7;}
        line_timers[7] = millis(); 
  }else
  {
    line_timers[7] = 0; 
  }

  if(abs(ll - base) >= threshold) {
      if( millis() - line_timers[8] >= time_threshold_ms) {
       line_timers[8] = millis(); return 8;}
        line_timers[8] = millis(); 
  }else
  {
    line_timers[8] = 0; 
  }

  return -1;    

}

void _default(){
  _imu_read();
  _SPIs();

  //move_angle_correction(45, correction_speed, 1);
  rotate_to(0, 5, target_speed, 0.000004, 0.00000006, 7);
  //rotate_to(0, 5, 0.00005, 7);

  //if ()

  //_log("default", String(check_ground_error(line_threshold)));

  if (check_ground_sensor_time(24, 1, 50) > -1)
  {
    drive_m1 = 0;
    drive_m2 = 0;
    drive_m3 = 0;
    drive_m4 = 0;


    
    move_angle( (check_ground_sensor(line_threshold, 1) * 45) - 180, target_speed);

    //delay(1000);

    //motors(0, 0, 0, 0);
    //motors(-drive_m1, -drive_m2, -drive_m3, -drive_m4);

  }
  
  motors(drive_m1, drive_m2, drive_m3, drive_m4);
  

}

void _default_old()//  _imu_read();
{
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0; // Zeitdelta in Sekunden
  if (dt <= 0) dt = 0.001; // Verhindern von Division durch Null
  last_time = now;

  // Yaw-Wert ins History-Array einfügen
  yaw_history[history_index] = yaw;
  history_index = (history_index + 1) % 5; // Zyklisch durch das Array
  if (history_index == 0) history_filled = true; // Array ist voll nach 5 Werten

  // Berechne den gleitenden Durchschnitt
  float yaw_average = 0;
  int count = history_filled ? 5 : history_index; // Anzahl der gültigen Werte
  if (count > 0) { // Vermeide Division durch Null
    for (int i = 0; i < count; i++) {
      yaw_average += yaw_history[i];
    }
    yaw_average /= count;
  }

  // Berechne den Fehler (mit Wrap-around für Winkel, 0-359 Grad)
  float error = yaw_direction - yaw_average;
  yaw_difference = fmod(error + 180, 360) - 180; // Normalisiere auf -180 bis 180 für kürzeste Drehung

  // Motorsteuerung
  drive_m1 = drive_base - yaw_difference;
  drive_m2 = drive_base + yaw_difference;
  drive_m3 = drive_base + yaw_difference;
  drive_m4 = drive_base - yaw_difference;

  // Begrenze die Motorwerte
  drive_m1 = constrain(drive_m1, -20, 20);
  drive_m2 = constrain(drive_m2, -20, 20);
  drive_m3 = constrain(drive_m3, -20, 20);
  drive_m4 = constrain(drive_m4, -20, 20);

  // Schreibe die Motorwerte
  WRITE_MOTOR(M1, drive_m1);
  WRITE_MOTOR(M2, drive_m2);
  WRITE_MOTOR(M3, -drive_m3);
  WRITE_MOTOR(M4, -drive_m4);

  // LCD-Ausgabe
  WRITE_LCD_TEXT(1, 1, String(yaw) + "   ");
  WRITE_LCD_TEXT(1, 2, String(yaw_direction));
}

  // alle spi übertragungen der anderen stm32 (e.g. boden; abstand; infrarot)  
void _SPIs()
{      // spi übertageung von den boden sensoren 8 bytes. jeweil der sensor an der boden platte. wie die werte aussehen kann man auf der lbotics website sehen. 
	digitalWrite(SPI2, LOW);
  if(spi.transfer(0XFF) == 250)
  { 
    fc = spi.transfer(0XFF); // front 
    fl = spi.transfer(0XFF); // front left 
    fr = spi.transfer(0XFF); // front right 
    ll = spi.transfer(0XFF); // left
    rr = spi.transfer(0XFF); // right 
    bl = spi.transfer(0XFF); // back left
    br = spi.transfer(0XFF); // back right
    bc = spi.transfer(0XFF); // back back 
  }
 
  digitalWrite(SPI2, HIGH);
}  


