#include "KeplerBRAIN_V4.h"
#include "Vladimir.h"

float speedMultiplier = 1.1;
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
    while(READ_BUTTON_PRESSED(B1)==1);
    if (isDriving) startYaw = READ_I2C_BNO055_YAW();
  }

  CheckLines(SPI2);

  if (!isDriving)
  {
    Drive(0, 0);
    return;
  }

  for (int i = 0; i < 8; i++)
    if (isLine[i])
      DriveLine(i);
  

  CheckCamera();
  CheckDistance();
  
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
      if (frontDistance < 15)
        disMillis = millis();
      if (millis() - disMillis > 500)
        state = 0;
    }
  }
  else
  {
    if (state == 0)
      state = 1;
    else if (millis() - ballMillis > 1000)
      state = 2;
  }

  rawYaw = READ_I2C_BNO055_YAW();

  if (state != 1)
    ballMillis = millis();

  switch (state)
  {
    case 0:
      
      if (ballX > 80)
        SetMotors(7);
      else
        SetMotors(-7);
      break;
    case 1:
      targetYaw = (rawYaw + (80 - ballX) / 3 + 360) % 360;
      dif = rawYaw - startYaw;
      if (abs(dif) > 18)
        ballMillis = millis();

      if (dif > 0 && dif < 180 || dif < -180)
        Drive(90 - log(ballY / 20) * 40, clamp(abs(dif * 0.48), 0, 20));
      else
        Drive(270 + log(ballY / 20) * 40, clamp(abs(dif * 0.48), 0, 20));
      break;
    case 2:
      targetYaw = startYaw;
      Drive((-ballX + 80) / 2, 20);
      break;
    case 3:
      targetYaw = startYaw;
      Drive(0, 23);
      break;
  }

  WRITE_LCD_TEXT(1, 1, String(state));
  WRITE_LCD_TEXT(1, 2, String(ballX));
  WRITE_LCD_TEXT(8, 2, String(ballY));
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

  while (!isLine[i] && startMillis + 500 > millis())
  {
    CheckLines(SPI2);

    
    if (isLine[(i + 3) % 8]) hasLeft = true;
    if (isLine[(i + 5) % 8]) hasRight = true;

    if (isLine[(i + 4) % 8])
    {
      DriveLine((i + 4) % 8);
      return;
    }

    for (int i = 0; i < 8; i++)
      if (isLine[i])
        startMillis = millis();

    if (hasLeft && !hasRight) Drive(i * 45 + 130, 25);
    if (!hasLeft && hasRight) Drive(i * 45 + 230, 25);
    else Drive(i * 45 + 180, 32);
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

    WRITE_MOTOR(M1, x * speed + yaw / 2);
    WRITE_MOTOR(M2, y * speed + yaw / 2);
    WRITE_MOTOR(M3, -x * speed + yaw / 2);
    WRITE_MOTOR(M4, -y * speed + yaw / 2);
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