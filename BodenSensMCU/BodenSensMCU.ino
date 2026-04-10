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
 
// Array zum Speichern der ADC-Werte (12-bit: 0-4095)
uint32_t adcValues[8];   
 
// SPI Data: 1 trigger byte + 8 sensoren × 2 bytes = 17 bytes
volatile uint8_t spi_data[17] = {250,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;  // 12-Bit Auflösung (0-4095)
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
 
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    // Fehler bei der Initialisierung
  }
 
  // Konfigurieren der ADC-Kanäle
  ADC_ChannelConfTypeDef sConfig = {0};
 
  sConfig.Channel = ADC_CHANNEL_9;   // PB1 front
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
 
  sConfig.Channel = ADC_CHANNEL_8;   // PB0 right
  sConfig.Rank = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 
  sConfig.Channel = ADC_CHANNEL_13;  // PC3 back left
  sConfig.Rank = 6;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 
  sConfig.Channel = ADC_CHANNEL_1;   // PA1 back right
  sConfig.Rank = 7;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 
  sConfig.Channel = ADC_CHANNEL_0;   // PA0 back
  sConfig.Rank = 8;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 
  // DMA für ADC konfigurieren
  hdma_adc1.Instance = DMA2_Stream0;
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
 
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
  {
    // Fehler bei der DMA-Initialisierung
  }
 
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 8);
}
 
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
 
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { while (1); }
 
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
 
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { while (1); }
}
 
void setup()
{
  SPI1_Init();
  NVIC_EnableIRQ(SPI1_IRQn);
  HAL_Init();
  SystemClock_Config();
  ADC1_Init();
 
  pinMode(PA15, OUTPUT);
  digitalWrite(PA15, LOW); // LEDs Boden ein
  pinMode(PB7, OUTPUT);
  digitalWrite(PB7, LOW);
  pinMode(PB9, OUTPUT);
  pinMode(PB8, INPUT);
  pinMode(PC13, INPUT);
 
  Serial.begin(115200);
}
 
void loop() 
{
  // Jeden Sensor als 2 Bytes (MSB + LSB) in spi_data schreiben
  // spi_data[0] = 250 (trigger, nie überschreiben!)
  for (int i = 0; i < 8; i++)
  {
    spi_data[1 + i * 2]     = (adcValues[i] >> 8) & 0xFF; // MSB (obere 4 bit)
    spi_data[1 + i * 2 + 1] = adcValues[i] & 0xFF;        // LSB (untere 8 bit)
  }

  // Debug Serial Output
  Serial.print("F:");  Serial.print(adcValues[0]);
  Serial.print(" FL:"); Serial.print(adcValues[1]);
  Serial.print(" FR:"); Serial.print(adcValues[2]);
  Serial.print(" L:");  Serial.print(adcValues[3]);
  Serial.print(" R:");  Serial.print(adcValues[4]);
  Serial.print(" BL:"); Serial.print(adcValues[5]);
  Serial.print(" BR:"); Serial.print(adcValues[6]);
  Serial.print(" B:");  Serial.println(adcValues[7]);
}
 
void SPI1_Init() 
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
 
  GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
  GPIOA->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
  GPIOA->AFR[0] |= (5 << (4 * 4)) | (5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4));
 
  SPI1->CR1 = 0;
  SPI1->CR1 &= ~SPI_CR1_MSTR;
  SPI1->CR1 &= ~SPI_CR1_SSM;
  SPI1->CR1 |= SPI_CR1_SPE;
  SPI1->CR2 |= SPI_CR2_RXNEIE;
  SPI1->CR2 |= SPI_CR2_TXEIE;
}
 
extern "C" void SPI1_IRQHandler(void)
{
  if (SPI1->SR & SPI_SR_RXNE)
  {
    uint8_t rx_value = SPI1->DR; // lesen um flag zu clearen
    (void)rx_value;
  }

  if (SPI1->SR & SPI_SR_TXE)
  {
    if (tx_index < 17)
    {
      SPI1->DR = spi_data[tx_index];
      tx_index++;
    }
    else
    {
      tx_index = 0;
    }
  }
}
