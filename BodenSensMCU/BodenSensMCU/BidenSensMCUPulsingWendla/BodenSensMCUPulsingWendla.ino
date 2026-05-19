#include "stm32f4xx_hal.h"

// --- Hardware & SPI ---
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;

volatile uint32_t adcValues[8];
volatile uint8_t spi_data[9] = { 250, 0, 0, 1, 0, 0, 0, 0, 0 };
volatile uint8_t tx_index = 0;

volatile uint8_t  phase         = 0;   // 0 = LED war AN, 1 = LED war AUS
volatile uint32_t pulseCount    = 0;
volatile uint32_t val_on[8], val_off[8];


// --- Filter & Dynamik ---
uint32_t impulseCounter[8] = {0};
int finalId = 255;
float ambientNoise = -1000.0; 
int dynamicThreshold = -2540;

// --- Prototypen ---
void SystemClock_Config(void);
void ADC1_Init(void);
void SPI1_Init(void);
int calculateDegree(int id);
void updateSpiData(int id);

// Präzise Mikrosekunden-Verzögerung
void delay_us(uint16_t us) {
    uint32_t start = SysTick->VAL;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((start - SysTick->VAL) < ticks);
}

void setup() {
    HAL_Init();
    SystemClock_Config();
    ADC1_Init();
    TIM2_Init();        // neu
    SPI1_Init();
    NVIC_EnableIRQ(SPI1_IRQn);

    pinMode(PA15, OUTPUT);
    digitalWrite(PA15, LOW); // LED direkt AN für ersten Puls
    pinMode(PB7, OUTPUT);
    Serial.begin(115200);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 8);
}

void loop() {
    // nur noch Debug, CPU ist sonst frei
    Serial.print("Thresh: "); Serial.print(dynamicThreshold);
    Serial.print(" Phase: ");  Serial.println(phase);
    delay(200);
}

void updateSpiData(int id) {
    if (id != 255) {
        int degree = calculateDegree(id);
        spi_data[1] = (uint8_t)(degree / 2);
        spi_data[2] = (uint8_t)(degree - spi_data[1]);
        spi_data[3] = 0; // Linie erkannt
        digitalWrite(PB7, HIGH);
    } else {
        spi_data[1] = 0;
        spi_data[2] = 0;
        spi_data[3] = 1; // Feld frei
        digitalWrite(PB7, LOW);
    }
}

int calculateDegree(int id) {
    switch (id) {
        case 0: return 180; // Front -> Flucht nach hinten
        case 1: return 135; // Front Left
        case 2: return 225; // Front Right
        case 3: return 90;  // Left
        case 4: return 270; // Right
        case 5: return 45;  // Back Left
        case 6: return 315; // Back Right
        case 7: return 0;   // Back -> Flucht nach vorne
        default: return 0;
    }
}


void ADC1_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    // Zuordnung der Pins laut deinem Plan
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_5; // PC2, PC0, PC3, PC5
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0; // PB1, PB0
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0; // PA1, PA0
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 8;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    // Kanäle: PB1(9), PC2(12), PC5(15), PC0(10), PB0(8), PC3(13), PA1(1), PA0(0)
    uint32_t chs[] = {ADC_CHANNEL_9, ADC_CHANNEL_12, ADC_CHANNEL_15, ADC_CHANNEL_10, 
                      ADC_CHANNEL_8, ADC_CHANNEL_13, ADC_CHANNEL_1, ADC_CHANNEL_0};
    for(int i=0; i<8; i++) {
        sConfig.Channel = chs[i];
        sConfig.Rank = i + 1;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    }

    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
    
}


// Initialize SPI1 in slave mode with interrupt
void SPI1_Init() {
  // Enable clocks for GPIOA and SPI1
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  // Configure PA4 (NSS), PA5 (SCK), PA6 (MISO), PA7 (MOSI) as alternate function
  GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
  GPIOA->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
  GPIOA->AFR[0] |= (5 << (4 * 4)) | (5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4));  // Set AF5 for SPI1

  // Configure SPI1 as a slave
  SPI1->CR1 = 0;                // Reset CR1
  SPI1->CR1 &= ~SPI_CR1_MSTR;   // Set as slave
  SPI1->CR1 &= ~SPI_CR1_SSM;    // Disable software slave management
  SPI1->CR1 |= SPI_CR1_SPE;     // Enable SPI
  SPI1->CR2 |= SPI_CR2_RXNEIE;  // Enable RXNE interrupt
  SPI1->CR2 |= SPI_CR2_TXEIE;   // Enable TXE interrupt
}

extern "C" void SPI1_IRQHandler(void) 
{
  // Check if RXNE (Receive Not Empty) flag is set and handle RX first
  if (SPI1->SR & SPI_SR_RXNE) {
    uint8_t rx_value = SPI1->DR;
    //Serial.println(rx_value);
  }
  // Check if TXE (Transmit Empty) flag is set, but only if not busy
  if (SPI1->SR & SPI_SR_TXE) {
    // Send data from tx_buffer if in "send mode"
    if (tx_index < 9) {
      // Write data to SPI
      SPI1->DR = spi_data[tx_index];
      tx_index++;
    } else {
      // Reset buffer index
      tx_index = 0;
    }
  }
}

void TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();

    // APB1 = 42MHz auf STM32F4, TIM-Takt = 84MHz
    // Prescaler 83 → 1MHz Zähltakt
    // Period 49 → alle 50µs ein TRGO-Event (= 20kHz)
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 83;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 49;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    // TRGO bei Update-Event → triggert ADC
    TIM_MasterConfigTypeDef mc = {0};
    mc.MasterOutputTrigger = TIM_TRGO_UPDATE;
    mc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &mc);

    HAL_TIM_Base_Start(&htim2);
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance != ADC1) return;

    if (phase == 0) {
        // ADC hat mit LED AN gemessen → Werte sichern, LED AUS
        for (int i = 0; i < 8; i++) val_on[i] = adcValues[i];
        digitalWrite(PA15, HIGH); // LED aus
        phase = 1;

    } else {
        // ADC hat mit LED AUS gemessen → Delta + Zähler
        for (int i = 0; i < 8; i++) val_off[i] = adcValues[i];
        digitalWrite(PA15, LOW);  // LED an für nächsten Puls
        phase = 0;

        // Delta berechnen
        long sum = 0;
        int deltas[8];
        for (int i = 0; i < 8; i++) {
            deltas[i] = (int)val_off[i] - (int)val_on[i];
            sum += deltas[i];
        }

        // Schwellenwert alle 100 Pulse anpassen
        if (pulseCount % 100 == 0) {
            float avg = sum / 8.0f;
            ambientNoise     = ambientNoise * 0.90f + avg * 0.10f;
            dynamicThreshold = (int)ambientNoise - 73;
        }

        // Impulse zählen
        for (int i = 0; i < 8; i++) {
            if (deltas[i] > dynamicThreshold)
                impulseCounter[i]++;
        }

        pulseCount++;

        // Block fertig?
        if (pulseCount >= 1000) {
            pulseCount = 0;
            int maxHits = 0, winner = 255;
            for (int i = 0; i < 8; i++) {
                if (impulseCounter[i] > maxHits && impulseCounter[i] > 850) {
                    maxHits = impulseCounter[i];
                    winner  = i;
                }
                impulseCounter[i] = 0;
            }
            updateSpiData(winner);
        }
    }
}

void TIM2_Init(void);