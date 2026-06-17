// *******************************************************************************
// *** KeplerBRAIN V4 Sensor STAMP SPI Client Soccer Bodensensor Mbed Port     ***
// *******************************************************************************

#include "mbed.h"
#include "stm32f4xx_hal.h"

// --- Hardware-Schnittstellen (Mbed-Style für GPIO & Serial) ---
DigitalOut led_transistor(PA_15, 0); // Transistor LEDs Boden (Start mit LED AN)
DigitalOut led_left(PB_7, 0);        // LED left
DigitalOut led_right(PB_9, 0);       // LED right

InterruptIn button_left(PB_8);       // Button left (falls benötigt)
InterruptIn button_right(PC_13);     // Button right (falls benötigt)

BufferedSerial pc(USBTX, USBRX, 115200);

// Mbed Ticker für die 2000Hz (500us Intervall)
Ticker timerTicker;

// --- Hardware-Strukturen für HAL ---
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

// --- Globale Variablen (Aus KeplerBRAIN V4 & Logik) ---
volatile uint32_t adcValues[8];
volatile uint8_t spi_data[9] = { 250, 0, 0, 1, 0, 0, 0, 0, 0 };
volatile uint8_t tx_index = 0;

volatile uint8_t  phase         = 0;   // 0 = LED ist AN, 1 = LED ist AUS
volatile uint32_t pulseCount    = 0;
volatile uint32_t val_on[8], val_off[8];

// --- Debug globals ---
volatile uint32_t dbg_callbackCount = 0;  
volatile int      dbg_lastDeltas[8] = {0};
volatile int      dbg_lastWinner    = 255;
volatile int      dbg_lastMaxHits   = 0;

// --- Filter & Dynamik ---
uint32_t impulseCounter[8] = {0};
int finalId = 255;
float ambientNoise = -1000.0; 
int dynamicThreshold = -2540;

// --- Prototypen ---
void ADC1_Init(void);
void SPI1_Init(void);
void Timer_Init(void);
int calculateDegree(int id);
void updateSpiData(int id);
void Timer_Callback(void);

// --- Hilfsfunktion für printf auf BufferedSerial ---
void debug_print(const char* format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (len > 0) {
        pc.write(buffer, len);
    }
}

int main() {
    // HAL_Init() wird von Mbed bereits beim Booten ausgeführt.
    
    ADC1_Init();
    SPI1_Init();
    NVIC_EnableIRQ(SPI1_IRQn);

    Timer_Init();        

    uint32_t lastCbCount = 0;
    uint8_t  stage       = 0;

    while (true) {
        switch (stage) {
        case 0: 
            {
                uint32_t now = dbg_callbackCount;
                uint32_t diff = now - lastCbCount;
                lastCbCount = now;
                debug_print("[CB/s] %lu\r\n", diff * 5); 
            }
            break;

        case 1: 
            debug_print("[DELTA] ");
            for (int i = 0; i < 8; i++) {
                debug_print("%d", dbg_lastDeltas[i]);
                if (i < 7) debug_print(" | ");
            }
            debug_print("\r\n");
            break;

        case 2: 
            debug_print("[THRESH] %d  [WINNER] ", dynamicThreshold);
            if (dbg_lastWinner == 255) {
                debug_print("kein\r\n");
            } else { 
                debug_print("%d (%d hits)\r\n", dbg_lastWinner, dbg_lastMaxHits); 
            }
            break;
        }

        stage = (stage + 1) % 3;
        ThisThread::sleep_for(200ms);
    }
}

void updateSpiData(int id) {
    if (id != 255) {
        int degree = calculateDegree(id);
        spi_data[1] = (uint8_t)(degree / 2);
        spi_data[2] = (uint8_t)(degree - spi_data[1]);
        spi_data[3] = 0; 
        led_left = 1; // Visuelles Feedback auf dem Board: LED links AN bei Treffer
    } else {
        spi_data[1] = 0;
        spi_data[2] = 0;
        spi_data[3] = 1; 
        led_left = 0; // LED links AUS wenn kein Signal
    }
}

int calculateDegree(int id) {
    switch (id) {
        case 0: return 180; 
        case 1: return 135; 
        case 2: return 225; 
        case 3: return 90;  
        case 4: return 270; 
        case 5: return 45;  
        case 6: return 315; 
        case 7: return 0;   
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

    // Zuordnung der analogen Pins laut deinem Kommentar (PC2, PC0, PC3, PC5, PB1, PB0, PA1, PA0)
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_5; 
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0; 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0; 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE; 
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START; 
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 8;
    hadc1.Init.DMAContinuousRequests = DISABLE; 
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV; 
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    // Gemessene Channels (Reihenfolge analog zu den Register-Zuweisungen)
    uint32_t chs[] = {ADC_CHANNEL_9, ADC_CHANNEL_12, ADC_CHANNEL_15, ADC_CHANNEL_10, 
                      ADC_CHANNEL_8, ADC_CHANNEL_13, ADC_CHANNEL_1, ADC_CHANNEL_0};
    for(int i = 0; i < 8; i++) {
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
    hdma_adc1.Init.Mode = DMA_NORMAL; 
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc1);
    
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
    
    HAL_NVIC_DisableIRQ(ADC_IRQn);
}

void SPI1_Init() {
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

extern "C" void SPI1_IRQHandler(void) {
  if (SPI1->SR & SPI_SR_RXNE) {
    volatile uint8_t rx_value = SPI1->DR;
    (void)rx_value;
  }
  if (SPI1->SR & SPI_SR_TXE) {
    if (tx_index < 9) {
      SPI1->DR = spi_data[tx_index];
      tx_index++;
    } else {
      tx_index = 0;
    }
  }
}

void Timer_Init(void) {
    // 2000 Hz Taktung für präzises Phasen-Sampling (500 Mikrosekunden Intervalle)
    timerTicker.attach(&Timer_Callback, 500us);
}

void Timer_Callback(void) {
    // 1. Die Messung für die soeben abgelaufene Phase starten
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 8);

    // 2. Hardware-Pins für die nächste Phase umschalten
    if (phase == 0) {
        led_transistor = 1; // IR-LEDs AUS für die Dunkelphase
        phase = 1;
    } else {
        led_transistor = 0; // IR-LEDs AN für die Leuchtphase
        phase = 0;
    }
}

// C-Linkage für HAL Callbacks in Mbed OS zwingend erforderlich
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance != ADC1) return;
    dbg_callbackCount++;

    if (phase == 1) { 
        for (int i = 0; i < 8; i++) val_on[i] = adcValues[i];
    } else { 
        for (int i = 0; i < 8; i++) val_off[i] = adcValues[i];

        long sum = 0;
        int deltas[8];
        for (int i = 0; i < 8; i++) {
            deltas[i] = (int)val_off[i] - (int)val_on[i];
            sum += deltas[i];
        }
        for (int i = 0; i < 8; i++) dbg_lastDeltas[i] = deltas[i];
        
        if (pulseCount % 100 == 0) {
            float avg = sum / 8.0f;
            ambientNoise     = ambientNoise * 0.90f + avg * 0.10f;
            dynamicThreshold = (int)ambientNoise - 73;
        }

        for (int i = 0; i < 8; i++) {
            if (deltas[i] > dynamicThreshold)
                impulseCounter[i]++;
        }

        pulseCount++;

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
            dbg_lastWinner  = winner;
            dbg_lastMaxHits = maxHits;
            updateSpiData(winner);
        }
    }
}

extern "C" void DMA2_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_adc1);
}