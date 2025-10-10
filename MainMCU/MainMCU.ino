#include "../shared/KeplerBRAIN_V4.h"
#include "Variables.h" //alle variablen usw. sind in dieser datei, um diese ein wenig aufzuräumen 
#include "powersense.h" //powersensor read 
#include <vector>
#include <iostream>
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
}
 
int selection_cursor = 0;
int selection = 0;

void loop()
{
  _powerread();

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
        //if (READ_BUTTON_CLOSED(B2) == 1){exit;} //

        //setup
        last_time = millis();  // Initialize time for PID
        drive_base=10;
        yaw_direction=20;
        Kp = 1.0;   // Proportional gain - tune this (start higher for faster response)
        Ki = 0.01;  // Integral gain - tune this (small to avoid windup)
        Kd = 0.5;   // Derivative gain - tune this (higher for more damping, reduces overshoot)

        //loop
        while ( selection = 0) 
        {
          _powerread();
          _SPIs(); 
          _imu_read();
          _default();
        }

      case 1:
        WRITE_LCD_TEXT(1, 2, "Motor Test");
        delay(700);
       // if (READ_BUTTON_CLOSED(B2) == 1){exit;}

        _motor_test();
        while ( selection = 1) //hier lassen um powerread nicht zu skippen 
        {
          _powerread();
        }
      case 2:
        WRITE_LCD_TEXT(1, 2, "IMU Test");
        delay(700);
        //if (READ_BUTTON_CLOSED(B2) == 1){exit;}

        _imu_test();

        while ( selection = 2) //hier lassen um powerread nicht zu skippen 
        {
          _powerread();
        }
      case 3:
        WRITE_LCD_TEXT(1, 2, "Input Test");
        delay(700);
        //if (READ_BUTTON_CLOSED(B2) == 1){exit;}

        while ( selection = 3)  //hier lassen um powerread nicht zu skippen 
        {
          _powerread(); 
        }
      case 4:
        WRITE_LCD_TEXT(1, 2, "Show BATT info");
        delay(700);
        //if (READ_BUTTON_CLOSED(B2) == 1){exit;}

        while ( selection = 4) //hier lassen um powerread nicht zu skippen 
        {
          _powerread();
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
        _SPIs(); 
        _imu_read();
        _default();
        _powerread();
        }

    }

    delay(5000);

  }

  if (selection_cursor <= 0){WRITE_LCD_TEXT(1, 2, "   ()   +");}  
  else if (selection_cursor >= 5){WRITE_LCD_TEXT(1, 2, "-  ()    ");}  
  else{WRITE_LCD_TEXT(1, 2, "-  ()   +");}  
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

// alle spi übertragungen der anderen stm32 (e.g. boden; abstand; infrarot)  
void _SPIs()
{      // spi übertageung von den boden sensoren 8 bytes. jeweil der sensor an der boden platte. wie die werte aussehen kann man auf der lbotics website sehen. 
	digitalWrite(SPI1, LOW);
	  if(spi.transfer(0XFF) == 250)
	  { 
	    ff = spi.transfer(0XFF); // front 
	    fl = spi.transfer(0XFF); // front left 
	    fr = spi.transfer(0XFF); // front right 
	    ll = spi.transfer(0XFF); // left
	    rr = spi.transfer(0XFF); // right 
	    bl = spi.transfer(0XFF); // back left
	    br = spi.transfer(0XFF); // back right
	    bb = spi.transfer(0XFF); // back back 
	  }
	 
	  digitalWrite(SPI1, HIGH);
}

void _imu_read()
{
  yaw = READ_I2C_BNO055_YAW();
  pitch = READ_I2C_BNO055_PITCH();
  roll = READ_I2C_BNO055_ROLL();
  _log("imu_read", "Y" + String(yaw)+" P"+String(pitch)+" R"+String(roll));
}

// power read code für spannungs üerwachung und ggeb. stromzählung
void _powerread()
{
  //werte leseung 
  BatV = READ_I2C_INA231_BUS_VOLTAGE();
  BatA = READ_I2C_INA231_CURRENT();

  //seriel schreiben der werte 
  Serial.print("BatVolatge: ");
  Serial.print(BatV);
  Serial.print("    "); //bissi platz das net alles so aufeinander bickt 
  Serial.print("BatAmp: ");
  Serial.print(BatA); 
  Serial.print("    "); //bissi platz das net alles so aufeinander bickt 
  Serial.print(ina231_error_count);
  Serial.println(); //nur für neue zeile so , dass der code gut ausschaut
  SLEEP(100);

  //led steuerung
  if (BatV > 12000)
  {
  WRITE_LED(L1,0);
  WRITE_LED(L2,0);
  WRITE_LED(L3,1);
  }
  if (BatV > 9000 && BatV < 11000)
  {
    WRITE_LED(L1,1); //Wenn batterie über 9V und unter 11V dann Rote led 1 
    WRITE_LED(L2,0);
  }
  if (BatV > 11000 && BatV < 12000)
  { 
    WRITE_LED(L1, 0);
    WRITE_LED(L2, 1);
    WRITE_LED(L3, 0);
  }
  while (BatV < 9000)
  {
    WRITE_LED(L1,1);
    SLEEP(300);      //Fick alles wenn die baterie zu leer ist
    WRITE_LED(L1,0);
    SLEEP(399);
  }
}