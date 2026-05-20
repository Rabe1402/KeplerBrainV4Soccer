//#include "KeplerBRAIN_V4.h"
#include <Arduino.h>
#include "Vladimir.h"
#include <SPI.h>
#include "stm32f4xx.h"
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SPI.h>

//Declerations
SPIClass spi_cam(PB15,PB14,PB13);
#define SPICAM PC6
SPIClass spi2(PA7,PA6,PA5);


//Line script
#define noiseSample 5
#define averagingSample 3
#define lineThreshold 20
#define lineThresholdForward 30
#define useForwardSensor true

bool isLine[8];
int secondLineValues[8];
int lineValues[8];
int lineValuesTemporary[8][noiseSample];
int lineValuesAv[8][averagingSample];
int averageLineValue = 0;

int compare(const void* a, const void* b)
{
  int int_a = * ( (int*) a );
  int int_b = * ( (int*) b );
     
  if ( int_a == int_b ) return 0;
  else if ( int_a < int_b ) return -1;
  else return 1;
}

void CheckLines(int pin)
{

  digitalWrite(pin, LOW);
  averageLineValue = 0;

  while (spi2.transfer(0XFF) != 250);

  for (int i = 0; i < noiseSample; i++)
  {
    lineValuesTemporary[0][i] = spi2.transfer(0XFF);
    lineValuesTemporary[7][i] = spi2.transfer(0XFF);
    lineValuesTemporary[1][i] = spi2.transfer(0XFF);
    lineValuesTemporary[6][i] = spi2.transfer(0XFF);
    lineValuesTemporary[2][i] = spi2.transfer(0XFF);
    lineValuesTemporary[5][i] = spi2.transfer(0XFF);
    lineValuesTemporary[3][i] = spi2.transfer(0XFF);
    lineValuesTemporary[4][i] = spi2.transfer(0XFF);
    if (i == noiseSample - 1) { break; }
    spi2.transfer(0XFF);
  }
  for (int i = 0; i < 8; i++)
  {
    qsort(lineValuesTemporary[i], noiseSample, sizeof(int), compare);
    lineValues[i] = lineValuesTemporary[i][noiseSample / 2];
  }

  digitalWrite(pin, HIGH);

  for (int i = 0; i < 8; i++)
  {
    for (int g = 0; g < averagingSample - 1; g++)
      lineValuesAv[i][g] = lineValuesAv[i][g + 1];
    lineValuesAv[i][averagingSample - 1] = lineValues[i];

    for (int g = 0; g < averagingSample - 1; g++)
      lineValues[i] += lineValuesAv[i][g];

    lineValues[i] = lineValues[i] / averagingSample;
  }

  for (int i = 0; i < 8; i++){
    secondLineValues[i] = lineValues[i];
  }
  qsort(secondLineValues, 8, sizeof(int), compare);
  averageLineValue = (secondLineValues[3] + secondLineValues[4]) / 2;

  for (int i = 0; i < 8; i++){
    isLine[i] = abs(lineValues[i] - averageLineValue) > lineThreshold && ((useForwardSensor && abs(lineValues[i] - averageLineValue) > lineThresholdForward) || i != 0);
  }
}

void Vladimir_Init()
{
  
  spi2.begin();

  spi2.setDataMode(SPI_MODE0);
  spi2.setBitOrder(MSBFIRST);
  spi2.setClockDivider(SPI_CLOCK_DIV16);


  pinMode(PC6, OUTPUT);
  digitalWrite(PC6, HIGH);

  spi_cam.begin();
   // spi_cam.setDataMode(SPI_MODE0);
   // spi_cam.setBitOrder(MSBFIRST);
   // spi_cam.setClockDivider(SPI_CLOCK_DIV16); // Adjust the clock divider as needed
  spi_cam.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
}


//TOF script

int frontDistance;

void CheckDistance()
{
  digitalWrite(PB12, LOW);

  while (spi2.transfer(0XFF) != 250);
  frontDistance = spi2.transfer(0XFF);
 
  digitalWrite(PB12, HIGH);
}




//Camera script

bool seesBall;
int ballX;
int ballY;
bool isYellowGoal;
int goalX;
int goalY;

int tempGoalBlueX;
int tempGoalBlueY;
int tempGoalYellowX;
int tempGoalYellowY;
int tempBallX;
int tempBallY;

void CheckCamera()
{
  digitalWrite(SPICAM, LOW);
  delayMicroseconds(100);
 
  //if (spi_cam.transfer(1) != 250) return;
  
  spi_cam.transfer(1);
  tempGoalBlueX = spi_cam.transfer(0);
  tempGoalBlueY = spi_cam.transfer(0);
  tempGoalYellowX = spi_cam.transfer(0);
  tempGoalYellowY = spi_cam.transfer(0);
  tempBallX = spi_cam.transfer(0);
  tempBallY = spi_cam.transfer(0);
  spi_cam.transfer(0);

  digitalWrite(SPICAM, HIGH);

  if (tempBallX == 0)
    seesBall = false;
  else
  {
    seesBall = true;
    ballX = tempBallX;
    ballY = tempBallY;
  }

  if (isYellowGoal)
  {
    goalX = tempGoalYellowX;
    goalY = tempGoalYellowY;
  }
  else
  {
    goalX = tempGoalBlueX;
    goalY = tempGoalBlueY;
  }
}