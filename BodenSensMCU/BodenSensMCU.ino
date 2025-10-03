// *******************************************************************************
// ***  KeplerBRAIN V4 Sensor STAMP SPI Client Soccer Bodensensor V 29.11.2024 ***
// *******************************************************************************
 
// PB1 F  Front
// PC2 FL Front Left
// PC5 FR Front Right
// PC0 L  Left
// PB0 R  Right
// PC3 BL Back Left
// PA1 BR Back Right
// PA0 B  Back
 
// PA15 Transistor LEDs Boden
// PB7 Led left
// PB8 Button left
// PB9 Led right
// PC13 Button right
 
#include "stm32f4xx_hal.h"
 
// ADC1 Handler
ADC_HandleTypeDef hadc1;       
 
// DMA Handler für ADC   
DMA_HandleTypeDef hdma_adc1;      
 
// Array zum Speichern der ADC-Werte
uint32_t adcValues[8];   
 
// SPI Data
volatile uint8_t spi_data[9] = {250,1,2,3,4,5,6,7,8};
volatile uint8_t tx_index = 0;
 
// Funktion zur Initialisierung des ADC mit DMA
void ADC1_Init(void)              
{
  // Takt für ADC1 aktivieren
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();  // DMA2 für ADC1
 
  // ADC-Pins initialisieren
  GPIO_InitTypeDef GPIO_InitStruct = {0};
 
  // PC2, PC0, PC3, PB1, PC5, PB0, PA1, PA0 als Analog-Eingänge konfigurieren
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_5;  // PC-Pins
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
 
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0;  // PB-Pins
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0;  // PA-Pins
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
  // ADC1 konfigurieren
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  // Prescaler für geringere Taktfrequenz
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;  // 12-Bit-Auflösung
  hadc1.Init.ScanConvMode = ENABLE;  // Scan-Modus aktivieren (mehrere Kanäle)
  hadc1.Init.ContinuousConvMode = ENABLE;  // Kontinuierliche Konvertierung
  hadc1.Init.DiscontinuousConvMode = DISABLE;  // Kein diskontinuierlicher Modus
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;  // Keine externe Auslösung
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;  // Daten rechtsbündig
  hadc1.Init.NbrOfConversion = 8;  // Anzahl der Kanäle
  hadc1.Init.DMAContinuousRequests = ENABLE;  // DMA-Anforderungen kontinuierlich
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;  // End-of-Conversion-Sequenz
 
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    // Fehler bei der Initialisierung
  }
 
  // Konfigurieren der ADC-Kanäle
  ADC_ChannelConfTypeDef sConfig = {0};
 
  sConfig.Channel = ADC_CHANNEL_9;  // PB1 front
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 
  sConfig.Channel = ADC_CHANNEL_12;  // PC2 front left
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 
  sConfig.Channel = ADC_CHANNEL_15;  // PC5 front right
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 
  sConfig.Channel = ADC_CHANNEL_10;  // PC0 left
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 
  sConfig.Channel = ADC_CHANNEL_8;  // PB0 right
  sConfig.Rank = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 
  sConfig.Channel = ADC_CHANNEL_13;  // PC3 back left
  sConfig.Rank = 6;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 
  sConfig.Channel = ADC_CHANNEL_1;  // PA1 back right
  sConfig.Rank = 7;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 
  sConfig.Channel = ADC_CHANNEL_0;  // PA0 back
  sConfig.Rank = 8;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 
  // DMA für ADC konfigurieren
  hdma_adc1.Instance = DMA2_Stream0;  // STM32F411RE verwendet DMA2 Stream 0 für ADC1
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;  // Zirkulärer Modus für kontinuierliche Konvertierung
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
 
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
  {
    // Fehler bei der DMA-Initialisierung
  }
 
  // Verknüpfen des DMA-Handles mit ADC
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
 
  // Start des ADC mit DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 8);
}
 
// Initialisierung der Systemuhr für STM32F411RE
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
 
  // Konfigurieren des Oszillators
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
 
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Fehler bei der Oszillator-Konfiguration
    while (1);
  }
 
  // Initialisieren des Taktsystems
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
 
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    // Fehler bei der Taktkonfiguration
    while (1);
  }
}
 
 
void setup()
{
  SPI1_Init();                  // Initialize SPI1 as Slave with interrupt    
  NVIC_EnableIRQ(SPI1_IRQn);    // Enable the SPI1 interrupt in NVIC
  HAL_Init();                   // HAL Initialisierung
  SystemClock_Config();         // Systemuhr konfigurieren
  ADC1_Init();                  // ADC und DMA initialisieren
 
  pinMode(PA15,OUTPUT);   // Transistor LEDs Boden
  digitalWrite(PA15,LOW); // LEDs Boden LOW:ein HIGH:aus
  pinMode(PB7,OUTPUT);    // LED links
  digitalWrite(PB7, LOW); // LED links aus
  pinMode(PB9,OUTPUT);    // LED rechts
  pinMode(PB8,INPUT);     // Taster links
  pinMode(PC13,INPUT);    // Taster rechts
 
  // User Code
  Serial.begin(115200);
}
 
void loop() 
{
  spi_data[1] = adcValues[0] / 10;
  spi_data[2] = adcValues[1] / 10;
  spi_data[3] = adcValues[2] / 10;
  spi_data[4] = adcValues[3] / 10;
  spi_data[5] = adcValues[4] / 10;
  spi_data[6] = adcValues[5] / 10;
  spi_data[7] = adcValues[6] / 10;
  spi_data[8] = adcValues[7] / 10;
 
  // User Code
  
  Serial.print(adcValues[0]);Serial.print(" ");
  Serial.print(adcValues[1]);Serial.print(" ");
  Serial.print(adcValues[2]);Serial.print(" ");
  Serial.print(adcValues[3]);Serial.print(" ");
  Serial.print(adcValues[4]);Serial.print(" ");
  Serial.print(adcValues[5]);Serial.print(" ");
  Serial.print(adcValues[6]);Serial.print(" ");
  Serial.println(adcValues[7]);
  
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
      //Serial.println(rx_value);
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
