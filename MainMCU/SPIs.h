#here are all the spi sonnections to the main mcu. 
#This file is included in the main mcu code.
#The spi communication is only used to recieve information
#There are four SPI ports on the mainboard, SPICAM, SPI1, SPI2, and SPI3.
#SPICAM is used to read the camera Data.
#SPI2 is used to read the Ground sensors 


#spi Variables: 
uint8_t SPICAM_Data0 = 0;
uint8_t SPICAM_Data1 = 0;
uint8_t SPICAM_Data2 = 0;
uint8_t SPICAM_Data3 = 0;
uint8_t SPICAM_Data4 = 0;
uint8_t SPICAM_Data5 = 0;
uint8_t SPICAM_Data6 = 0;
uint8_t SPICAM_Data7 = 0;

uint8_t SPI2_Data0 = 0;
uint8_t SPI2_Data1 = 0;
uint8_t SPI2_Data2 = 0;
uint8_t SPI2_Data3 = 0;
uint8_t SPI2_Data4 = 0; 
uint8_t SPI2_Data5 = 0;
uint8_t SPI2_Data6 = 0;
uint8_t SPI2_Data7 = 0;

void _SPIs()
{
  {
  // read 8 Bytes from OpenMV BEGIN
 
  digitalWrite(SPICAM, LOW);
  delay(1);
 
  if(SPICAM_Data0.transfer(1) == 250) #change this number in BOTH codes when 250 coud be one of the other 7 numbers. this number is here as indcation to start the data reader
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
 
  }
  {
    // read 8 Bytes from Ground Sensors BEGIN

    digitalWrite(SPI2, LOW);
    delay(1);

    if(SPI2_Data0.transfer(1) == 250) #change this number in BOTH codes when 250 coud be one of the other 7 numbers. this number is here as indcation to start the data reader
    { 
      SPI2_Data1 = spi.transfer(0);
      SPI2_Data2 = spi.transfer(0);
      SPI2_Data3 = spi.transfer(0);
      SPI2_Data4 = spi.transfer(0);
      SPI2_Data5 = spi.transfer(0);
      SPI2_Data6 = spi.transfer(0);
      SPI2_Data7 = spi.transfer(0);
    }
  }
}

