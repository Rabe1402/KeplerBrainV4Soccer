#include "../shared/KeplerBRAIN_V4.h"

int start_normal;
uint16_t yaw;
int16_t pitch;
int16_t roll;

void setup()
{
  // Initialisierung der Hardwarekomponenten des Controllers
  KEPLERBRAIN_INIT();
  WRITE_I2C_BNO055_INIT();
  Serial.begin(115200);

  WRITE_LCD_CLEAR();

}
 
int selection_cursor = 0;
int selection = 0;

void loop()
{

  if (READ_BUTTON_CLOSED(B1) == 1 && selection_cursor > 0){selection_cursor--;WRITE_LCD_TEXT(1, 2, "o");}
  if (READ_BUTTON_CLOSED(B3) == 1 && selection_cursor < 10){selection_cursor++;WRITE_LCD_TEXT(9, 2, "<>");}
  delay(200);

  //Display
  WRITE_LCD_TEXT(1, 1, String(selection_cursor));
  if (READ_BUTTON_CLOSED(B2) == 1){
    WRITE_LCD_TEXT(4, 2, "<>");
    while(READ_BUTTON_CLOSED(B2) == 1){}
    WRITE_LCD_TEXT(4, 2, "()");
    delay(500);
    selection = selection_cursor;
    switch ( String(selection) )
    {
      case 0:
        WRITE_LCD_TEXT(1, 2, "Prog 0");
        _default();
      case 1:
        WRITE_LCD_TEXT(1, 2, "Prog 1");
        _motor_test(); 
      case 2:
        WRITE_LCD_TEXT(1, 2, "Prog 2");
      case 3:
        WRITE_LCD_TEXT(1, 2, "Prog 3");
      case 4:
        WRITE_LCD_TEXT(1, 2, "Prog 4");
      case 5:
        WRITE_LCD_TEXT(1, 2, "Prog 5");
    }
    delay(5000);
  }
  if (selection_cursor <= 0){WRITE_LCD_TEXT(1, 2, "   ()   +");}  
  else if (selection_cursor >= 5){WRITE_LCD_TEXT(1, 2, "-  ()    ");}  
  else{WRITE_LCD_TEXT(1, 2, "-  ()   +");}  
  
  /*WRITE_LCD_CLEAR();
  WRITE_MOTOR(M1, 0);
  WRITE_MOTOR(M2, 0);
  Serial.println("no program selected");
  Serial.println("press button B1 for normal startup");
  WRITE_LCD_TEXT(1, 1, "Select Program");
  WRITE_LCD_TEXT(1, 2, "B1 for normal startup");*/
}

void _motor_test()
{
  WRITE_MOTOR(M1,00);
  WRITE_MOTOR(M2,00);
  WRITE_MOTOR(M3,00);
  WRITE_MOTOR(M4,00);
  SLEEP(1000);
  WRITE_MOTOR(M1,50);
  WRITE_MOTOR(M2,50);
  SLEEP(1000);
  WRITE_MOTOR(M1,00);
  WRITE_MOTOR(M2,00);
  WRITE_MOTOR(M3,50);
  WRITE_MOTOR(M4,50);
  SLEEP(1000);
}

void _default()
{
	_mcu_read();
	
}

void _mcu_read()
{
  yaw = READ_I2C_BNO055_YAW();
  pitch = READ_I2C_BNO055_PITCH();
  roll = READ_I2C_BNO055_ROLL();
 // WRITE_LCD_TEXT(1,1,String(yaw)+" "+String(pitch)+" "+String(roll)+" ");
}
