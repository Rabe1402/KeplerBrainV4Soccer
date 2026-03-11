#include "../shared/KeplerBRAIN_V4.h"
//here are all the spi sonnections to the main mcu. 
//This file is included in the main mcu code.
//The spi communication is only used to recieve information
//There are four SPI ports on the mainboard, SPICAM, SPI1, SPI2, and SPI3.
//SPICAM is used to read the camera Data.
//SPI2 is used to read the Ground sensors 

//Example für Winkel von Kamera zurück rechnen. 
    //uint8_t rel_angle  = (int8_t)(SPICAM_Data2 - 90);  // -35..+35°
    //float  target     = imu_heading + rel_angle;

    //if (target <   0) target += 360;
    //if (target >= 360) target -= 360;

//spi Variables: 
uint8_t SPICAM_Data0; //this is the trigger for the reading sequence (250 in cam code) 
uint8_t SPICAM_Data1; // boolean for if there is a Ball in sight
uint8_t SPICAM_Data2; // relative angle to the ball (0-180, 90 is straight ahead, 0 is left, 180 is right)
uint8_t SPICAM_Data3; // distance MSB in mm
uint8_t SPICAM_Data4; // distance LSB in mm
uint8_t SPICAM_Data5; // reserved
uint8_t SPICAM_Data6; // reserved
uint8_t SPICAM_Data7; // reserved

  //Variablen bodensensor
unit8_t fc; // front 
unit8_t fl; // front left 
unit8_t fr; // front right 
unit8_t ll; // left
unit8_t rr; // right 
unit8_t bl; // back left
unit8_t br; // back right
unit8_t bc; // back back 


void _SPIs()
{
  {
  // read 8 Bytes from OpenMV BEGIN
 
  digitalWrite(SPICAM, LOW);
  delay(1);
 
  if(SPICAM_Data0.transfer(1) == 250) #change this number in BOTH codes when 250 coud be one of the other 7 numbers. this number is here as indcation to start the data reader
  { 
    SPICAM_Data1 = spi_cam.transfer(0XFF);
    SPICAM_Data2 = spi_cam.transfer(0XFF);
    SPICAM_Data3 = spi_cam.transfer(0XFF);
    SPICAM_Data4 = spi_cam.transfer(0XFF);
    SPICAM_Data5 = spi_cam.transfer(0XFF);
    SPICAM_Data6 = spi_cam.transfer(0XFF);
    SPICAM_Data7 = spi_cam.transfer(0XFF);
  }
 
  digitalWrite(SPICAM, HIGH);
 
  // read 8 Bytes from OpenMV END
 

	digitalWrite(SPI2, LOW);
 	if(spi.transfer(0XFF) == 250)
 	{
		ff = spi.transfer(0XFF); // front 
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
