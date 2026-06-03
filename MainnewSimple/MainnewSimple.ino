#include "KeplerBRAIN_V4.h"
#include "Vladimir.h"

float speedMultiplier = 1;
int startYaw;
int targetYaw;
bool isDriving = false;

int yaw;
int rawYaw;
float x, y;
float direction;

//0 = no ball
//1 = ball, go to ball
//2 = ball in front
//3 = has ball
int state = 0;

unsigned long startMillis;
unsigned long ballMillis;
unsigned long disMillis;
int dif;

bool searchRight;
bool goRight;
int goalMismatches;

bool isBack;




void setup()
{
  KEPLERBRAIN_INIT();
  WRITE_LCD_CLEAR();
  WRITE_I2C_BNO055_INIT();
  Vladimir_Init();

  startYaw = READ_I2C_BNO055_YAW();
  targetYaw = startYaw;
}

void loop()
{
  if (READ_BUTTON_PRESSED(B1)==1)
  {
    isDriving = !isDriving;
    //while (READ_BUTTON_PRESSED(B1)==1);
    if (isDriving) startYaw = READ_I2C_BNO055_YAW();
    state = 0;
    if (isBack)
      startYaw = (startYaw + 180) % 360;
  }

  if (READ_BUTTON_PRESSED(B2)==1)
  {
    int m = millis();
    while (READ_BUTTON_CLOSED(B2)==1)
    {
      if (millis() - m > 500)
      {
        isBack = !isBack;
        return;
      }
    }

    isYellowGoal = !isYellowGoal;
  }

  if (READ_BUTTON_PRESSED(B3)==1)
  {
    int m = millis();
    while (READ_BUTTON_CLOSED(B3)==1)
    {
      if (millis() - m > 500)
      {
        useForwardSensor = !useForwardSensor;
        return;
      }
    }

    speedMultiplier += 0.1;
    if (speedMultiplier > 1.5)
      speedMultiplier = 0.8;
  }

  CheckLines(SPI2);
  CheckCamera();
  CheckDistance();

  if (!isDriving)
  {
    Drive(0, 0);
    DisplayDraw();
    return;
  }

  for (int i = 0; i < 8; i++)
    if (isLine[i])
      DriveLine(i);
  

  WRITE_LED(L1, seesGoal ? 1 : 0);
  
  if (!seesBall)
  {
    if (state == 1)
      state = 0;
    else if (state == 2)
    {
      state = 3;
      disMillis = millis();
    }
    else if (state == 3)
    {
      if (frontDistance < maxFrontDistance)
        disMillis = millis();
      if (millis() - disMillis > 500)
        state = 0;
    }
  }
  else
  {
    if (state == 0)
    {
      state = 1;

      
      dif = (rawYaw - startYaw);
      searchRight = (dif > 0 && dif < 180 || dif < -180);
    }
    else if (millis() - ballMillis > 600)
    {
      
      state = 2;
      goalMismatches = 0;

    }
  }

  rawYaw = READ_I2C_BNO055_YAW();

  if (state != 1)
    ballMillis = millis();

  switch (state)
  {
    case 0:
      
      if (frontDistance < 14)
        Drive(180, 28);
      else
      {
        targetYaw = rawYaw;

        if (ballX > 80)
          SetMotors(7);
        else
          SetMotors(-7);
      }
      break;
      
    case 1:
      targetYaw = (rawYaw + (80 - ballX) / 3 + 360) % 360;
      dif = rawYaw - startYaw;

      dif = (dif + 540) % 360 - 180;
      
      if (goalMismatches >= maxGoalMismatches)
      {
        searchRight = (dif > 0 && dif < 180 || dif < -180);

        if (abs(dif) > 18)
          ballMillis = millis();
      }
      else
      {
        if (!seesGoal)
        {
          ballMillis = millis();

          if (dif > 50 && !searchRight || dif < -50 && searchRight)
          {
            goalMismatches++;
            searchRight = !searchRight;
          }
          
          dif = 30;
        }
        else
        {
          dif = (goalX - ballX) / 2;
          if (abs(dif * 2) > 30)
            ballMillis = millis();

        }
      }

      //if (dif > 0 && dif < 180 || dif < -180)
      if (seesGoal)
        goRight = dif > 0;
      else
        goRight = searchRight;

      if (goRight)
        Drive(clamp(90 - log(ballY / 15) * 35, 0, 180),        clamp(abs(dif * 0.5), 0, 20));
      else
        Drive(clamp(270 + log(ballY / 15) * 35, 180, 360),          clamp(abs(dif * 0.5), 0, 20));
      break;

    case 2:
      //targetYaw = startYaw;
      Drive((-ballX + 80) / 2, 19);
      break;

    case 3:
      //targetYaw = startYaw;
      Drive(0, 21);
      break;
  }

  DisplayDraw();
  //WRITE_LCD_TEXT(8, 2, String(ballY));
}

void DisplayDraw()
{
  //lcd.clear();

  //WRITE_LCD_TEXT(1, 1, String(state));
  WRITE_LCD_TEXT(1, 1, String(ballX) + "  ");
  WRITE_LCD_TEXT(6, 1, String(speedMultiplier));
  WRITE_LCD_TEXT(12, 1, String(useForwardSensor ? "For  " : "NoFor"));

  WRITE_LCD_TEXT(1, 2, isYellowGoal ? "YELLOW" : "BLUE  ");
  WRITE_LCD_TEXT(9, 2, isBack ? "BACK " : "FRONT");
}

void DriveLine(int i)
{
  WRITE_LED(L2, 1);

  targetYaw = READ_I2C_BNO055_YAW();

  while (isLine[i])
  {
    CheckLines(SPI2);
    Drive(i * 45, 14);
  }

  startMillis = millis();
  bool hasLeft, hasRight;

  while (!isLine[i] && startMillis + 400 > millis())
  {
    CheckLines(SPI2);

    
    if (isLine[(i + 3) % 8]) hasLeft = true;
    if (isLine[(i + 5) % 8]) hasRight = true;

    if ((i == 1 && isLine[7] || i == 7 && isLine[1]) && !useForwardSensor && smartForwardSensor)
    {
      DriveLine(0);
      return;
    }

    if (isLine[(i + 4) % 8])
    {
      DriveLine((i + 4) % 8);
      return;
    }

    for (int i = 0; i < 8; i++)
      if (isLine[i])
        startMillis = millis();

    if (hasLeft && !hasRight) Drive(i * 45 + 130, 30);
    if (!hasLeft && hasRight) Drive(i * 45 + 230, 30);
    else Drive(i * 45 + 180, 30);
  }
  while (isLine[i])
  {
    CheckLines(SPI2);
    Drive(i * 45 + 180, 15);
  }
  while (startMillis + 100 > millis())
  {
    CheckLines(SPI2);
    Drive(i * 45 + 180, 15);
  }
  
  WRITE_LED(L2, 0);
}

float clamp(float d, float min, float max)
{
  const float t = d < min ? min : d;
  return t > max ? max : t;
}

void Drive(int oldDirection, float speed)
{
  yaw = ReadYaw();
  speed *= speedMultiplier;

  oldDirection = (oldDirection + 360) % 360;
  direction = oldDirection / 57.295;
  if (oldDirection < 46) {
    y = 1;
    x = tan(0.785 - direction);
  } else if (oldDirection < 91) {
    y = 1;
    x = -tan(direction - 0.785);
  } else if (oldDirection < 136) {
    y = tan(2.355 - direction);
    x = -1;
  } else if (oldDirection < 181) {
    y = -tan(direction - 2.355);
    x = -1;
  } else if (oldDirection < 226) {
    y = -1;
    x = -tan(3.925 - direction);
  } else if (oldDirection < 271) {
    y = -1;
    x = tan(direction - 3.925);
  } else if (oldDirection < 316) {
    y = -tan(5.495 - direction);
    x = 1;
  } else {
    y = tan(direction - 5.495);
    x = 1;
  }


  if (isDriving)
  {

    WRITE_MOTOR(M1, x * speed + yaw / 2.5 * speedMultiplier);
    WRITE_MOTOR(M2, y * speed + yaw / 2.5 * speedMultiplier);
    WRITE_MOTOR(M3, -x * speed + yaw / 2.5 * speedMultiplier);
    WRITE_MOTOR(M4, -y * speed + yaw / 2.5 * speedMultiplier);
  }
  else
    SetMotors(0);
}

void SetMotors(int speed)
{
  WRITE_MOTOR(M1, speed);
  WRITE_MOTOR(M2, speed);
  WRITE_MOTOR(M3, speed);
  WRITE_MOTOR(M4, speed);
}

int ReadYaw()
{
  return (READ_I2C_BNO055_YAW() - targetYaw + 540) % 360 - 180;
}