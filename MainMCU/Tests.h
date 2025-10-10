void _log(String name, String message)
{
  if (debug)
  {
    log_message = "## Got log from" + String(name) + "at" + String(millis() + ":\n" + String(message));
    if (debug_over_serial)
    {
      Serial.println(log_message);
    }else
    {
      debug_log.push_back(log_message);
    }
  }
}

void _motor_test()
{
  WRITE_LCD_CLEAR();
  WRITE_LCD_TEXT(1, 1, "Now Testing:");

  WRITE_MOTOR(M1,00);
  WRITE_MOTOR(M2,00);
  WRITE_MOTOR(M3,00);
  WRITE_MOTOR(M4,00);

  SLEEP(1000);

  WRITE_LCD_TEXT(2, 1, "M1");
  WRITE_MOTOR(M1,50);
  SLEEP(1000);

  WRITE_LCD_TEXT(2, 1, "M2");
  WRITE_MOTOR(M2,50);
  SLEEP(1000);

  WRITE_LCD_TEXT(2, 1, "M3");
  WRITE_MOTOR(M3,50);
  SLEEP(1000);

  WRITE_LCD_TEXT(2, 1, "M4");
  WRITE_MOTOR(M4,50);
  SLEEP(1000);
  
  WRITE_MOTOR(M1,00);
  WRITE_MOTOR(M2,00);
  WRITE_MOTOR(M3,00);
  WRITE_MOTOR(M4,00);
  WRITE_LCD_CLEAR();
  WRITE_LCD_TEXT(1, 1, "Test Complete :)");

  SLEEP(1000);
}

void _imu_test()
{
  WRITE_LCD_TEXT(1, 2, "x       ");

  int old_time;
  bool full_speed = false;

  while(READ_BUTTON_CLOSED(B1) != 1)
  {
    old_time = millis();
    
    yaw = READ_I2C_BNO055_YAW();
    pitch = READ_I2C_BNO055_PITCH();
    roll = READ_I2C_BNO055_ROLL();
    _log("imu_read", "Y" + String(yaw)+" P"+String(pitch)+" R"+String(roll));

    WRITE_LCD_TEXT(1, 2, "Y"   + String(yaw)   );
    WRITE_LCD_TEXT(5, 2, " |P" + String(pitch) );
    WRITE_LCD_TEXT(11,2, " |R" + String(roll)  );

    if (!full_speed){delay(100);}
    if (READ_BUTTON_CLOSED(B3) == 1)
    {
      full_speed = !full_speed;
      if (full_speed)
      {
        WRITE_LCD_TEXT(11, 2, "fs:on ");
      }
      else
      {
        WRITE_LCD_TEXT(11, 2, "fs:off");
      }
      delay(1000);

      WRITE_LCD_TEXT(3, 2, "DT: " + String(millis() - old_time));
    }

  }

  WRITE_LCD_TEXT(1, 2, "o");
  while(READ_BUTTON_CLOSED(B2) == 1){}
  WRITE_LCD_CLEAR();
}