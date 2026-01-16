// alle spi übertragungen der anderen stm32 (e.g. boden; "abstand"; infrarot)  

//Variablen bodensensor
unit8_t fc; // front 
unit8_t fl; // front left 
unit8_t fr; // front right 
unit8_t ll; // left
unit8_t rr; // right 
unit8_t bl; // back left
unit8_t br; // back right
unit8_t bc; // back back 

//variable fpr inrarot normal 
unit8_t isN;

//varablen für infrarot debug1 
unit8_t is1;
unit8_t is2;
unit8_t is3;
unit8_t is4;
unit8_t is5:
unit8_t is6;
unit8_t is7;
unit8_t is8;


void _SPIs()

{   // spi übertageung von den boden sensoren 8 bytes. jeweil der sensor an der boden platte. wie die werte aussehen kann man auf der lbotics website sehen. 

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

	// spi übertragung von dem infrarot Sensorkreis
	digitalWrite(SPI1, LOW);
	if(spi.transfar(0XFF) == 250)
	{	
		#ifndef InfrarotDebug
		isN = spi.transfer(0XFF); // number of the sensor with the loweest number that still has a signal
		#endif
		//debug mode for infrared sens. for further information look at the infraMCU code... 
		#ifdef InfrarotDebug 
		is1 = spi.transfer(0XFF);
		is2 = spi.transfer(0XFF);
		is3 = spi.transfer(0XFF);
		is4 = spi.transfer(0XFF);
		is5 = spi.transfer(0XFF);
		is6 = spi.transfer(0XFF);
		is7 = spi.transfer(0XFF);
		is8 = spi.transfer(0XFF);
		#endif

	//der abstand sensor ist immoment gar nicht eingebaut desswegen fehlt auch der code... 
	//aber er wäre HIER 
		
}
