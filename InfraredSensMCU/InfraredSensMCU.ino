// ***************************************************************************
// ***  KeplerBRAIN V4 Sensor STAMP SPI Client IR Ballsensor  V 15.01.2024 ***
// ***************************************************************************
 
 
// Array zum Speichern der detection results
volatile uint8_t arr_sensor[16] = {0};
 
// SPI Data
volatile uint8_t spi_data[9] = {250,1,2,3,4,5,6,7,8};
volatile uint8_t tx_index = 0;
 
void setup()
{
  // Initialize SPI1 as Slave with interrupt
  SPI1_Init();                
  // Enable the SPI1 interrupt in NVIC
  NVIC_EnableIRQ(SPI1_IRQn);  
 
  pinMode(PB8, INPUT);
  pinMode(PB7, INPUT);
  pinMode(PB6, INPUT);
  pinMode(PB5, INPUT);
  pinMode(PA10, INPUT);
  pinMode(PA9, INPUT);
  pinMode(PB15, INPUT);
  pinMode(PB14, INPUT);
  pinMode(PB12, INPUT);
  pinMode(PB13, INPUT);
  pinMode(PB2, INPUT);
  pinMode(PB0, INPUT);
  pinMode(PA1, INPUT);
  pinMode(PA0, INPUT);
  pinMode(PC13, INPUT);
  pinMode(PB9, INPUT);
 
  // User Code
  //Serial.begin(115200);
 
}
 
void loop()
{
 
  // explizite Auswertung
  if (digitalRead(PB5)) arr_sensor[0] = 0; else arr_sensor[0] = 1; 
  if (digitalRead(PB7)) arr_sensor[1] = 0; else arr_sensor[1] = 1; 
  if (digitalRead(PB8)) arr_sensor[2] = 0; else arr_sensor[2] = 1; 
  if (digitalRead(PB6)) arr_sensor[3] = 0; else arr_sensor[3] = 1; 
  if (digitalRead(PB14)) arr_sensor[4] = 0; else arr_sensor[4] = 1; 
  if (digitalRead(PA10)) arr_sensor[5] = 0; else arr_sensor[5] = 1; 
  if (digitalRead(PA9)) arr_sensor[6] = 0; else arr_sensor[6] = 1; 
  if (digitalRead(PB15)) arr_sensor[7] = 0; else arr_sensor[7] = 1; 
 
  if (digitalRead(PB13)) arr_sensor[8] = 0; else arr_sensor[8] = 1; 
  if (digitalRead(PB2)) arr_sensor[9] = 0; else arr_sensor[9] = 1; 
  if (digitalRead(PB0)) arr_sensor[10] = 0; else arr_sensor[10] = 1; 
  if (digitalRead(PB12)) arr_sensor[11] = 0; else arr_sensor[11] = 1; 
  if (digitalRead(PA1)) arr_sensor[12] = 0; else arr_sensor[12] = 1; 
  if (digitalRead(PA0)) arr_sensor[13] = 0; else arr_sensor[13] = 1; 
  if (digitalRead(PC13)) arr_sensor[14] = 0; else arr_sensor[14] = 1; 
  if (digitalRead(PB9)) arr_sensor[15] = 0; else arr_sensor[15] = 1; 
 
 
  // Ausgabe Sensorwerte und Übertragung
  for (int i = 0; i < 16; i++)
  {
    if (arr_sensor[i] == 1) 
    {
      //Serial.print("1 ");
 
      // Die Nummer des Sensors mit der höchsten Zahl der 
      // ein Signal "sieht" wird  übertragen
      // !!! Dies ist zu verbessern und hier ist eine Logik
      // zu entwickeln, was übertragen wird, wenn mehr als ein
      // Sensor das Signal dedektiert
      spi_data[1] = i+1;
    } 
    else 
    {
      //Serial.print("0 ");
    }
  }
 
  //Serial.println("");
}
 
// Initialize SPI1 in slave mode with interrupt
void SPI1_Init() 
{
  // Enable clocks for GPIOA and SPI1
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
 
  // Configure PA4 (NSS), PA5 (SCK), PA6 (MISO), PA7 (MOSI) as alternate function
  GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
  GPIOA->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
  GPIOA->AFR[0] |= (5 << (4 * 4)) | (5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4));  // Set AF5 for SPI1
 
  // Configure SPI1 as a slave
  SPI1->CR1 = 0;                 // Reset CR1
  SPI1->CR1 &= ~SPI_CR1_MSTR;    // Set as slave
  SPI1->CR1 &= ~SPI_CR1_SSM;     // Disable software slave management
  SPI1->CR1 |= SPI_CR1_SPE;      // Enable SPI
  SPI1->CR2 |= SPI_CR2_RXNEIE;   // Enable RXNE interrupt
  SPI1->CR2 |= SPI_CR2_TXEIE;    // Enable TXE interrupt
}
 
extern "C" void SPI1_IRQHandler(void)
{
   // Check if RXNE (Receive Not Empty) flag is set and handle RX first
    if (SPI1->SR & SPI_SR_RXNE)
    {
      uint8_t rx_value = SPI1->DR;  
    }
      // Check if TXE (Transmit Empty) flag is set, but only if not busy
    if (SPI1->SR & SPI_SR_TXE)
    {
        // Send data from tx_buffer if in "send mode"
        if (tx_index < 9)
        {
          // Write data to SPI
          SPI1->DR = spi_data[tx_index];  
          tx_index++;
        }
        else
        {
          // Reset buffer index
          tx_index = 0;                      
        }
    }
}