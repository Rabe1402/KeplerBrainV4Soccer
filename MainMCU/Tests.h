

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

void _input_test()
{
  WRITE_LCD_TEXT(1, 2, "x       ");

  int old_time;
  bool full_speed = false;
  int mode = 0;

  while(READ_BUTTON_CLOSED(B1) != 1)
  {
    old_time = millis();
    
    switch (mode)
    {
      case 0:
      break;;

      case 1:
      break;;

      case 2:
      break;;

      case 3:
      break;;
    }
    
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

    }
    if (READ_BUTTON_CLOSED(B2) == 1)
    {
      mode++;
      if (mode > 3)
      {
        mode = 0;
      }

      switch (mode)
      {
        case 0:
        break;;
  
        case 1:
        break;;
  
        case 2:
        break;;
  
        case 3:
        break;;
      }

      WRITE_LCD_TEXT(11, 2, String(mode));

      delay(1000);

    }
    WRITE_LCD_TEXT(3, 2, "DT: " + String(millis() - old_time));

  }

  WRITE_LCD_TEXT(1, 2, "o");
  while(READ_BUTTON_CLOSED(B2) == 1){}
  WRITE_LCD_CLEAR();
}

void _batt_test()
{
  WRITE_LCD_TEXT(1, 2, "x       ");
  int  old_time;
  bool full_speed = false;
  int mode = 0;

  while(READ_BUTTON_CLOSED(B1) != 1)
  {
    old_time = millis();
    
    switch (mode)
    {
      case 0:
        WRITE_LCD_TEXT(1, 1, String(READ_I2C_INA231_BUS_VOLTAGE()) + "mV         ");
      break;;

      case 1:
        WRITE_LCD_TEXT(1, 1, String(READ_I2C_INA231_SHUNT_VOLTAGE()) + "idk 1µV              ");
      break;;

      case 2:
        WRITE_LCD_TEXT(1, 1, String(READ_I2C_INA231_CURRENT()) + "mA                 ");
      break;;

      case 3:
        WRITE_LCD_TEXT(1, 1, String(READ_I2C_INA231_POWER()) + "idk                  ");
      break;;
    }
    

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
    if (READ_BUTTON_CLOSED(B2) == 1)
    {
      mode++;
      if (mode > 3)
      {
        mode = 0;
      }

      WRITE_LCD_TEXT(11, 2, String(mode));

      delay(1000);

    }

    WRITE_LCD_TEXT(3, 2, "DT: " + String(millis() - old_time) + "                  ");

  }

  WRITE_LCD_TEXT(1, 2, "o");
  while(READ_BUTTON_CLOSED(B2) == 1){}
  WRITE_LCD_CLEAR();
}