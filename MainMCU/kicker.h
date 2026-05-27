


#define IOS1 0x51
#define IOS2 0x52
#define IOS3 0x53
#define IOS4 0x54

// PB_6  Digital In Out  IOS1
// PB_7  Digital In Out  IOS2
// PB_8  Digital In Out  IOS3
// PB_9  Digital In Out  IOS4
int aktiv_kicker_port; //merke kicker um im interrupt den richtigen auszuschalten

void WRITE_KICKER_INIT(uint8_t port) //init kicker pin unf timer, für exatkte kick dauer ohne den main code zu stoppen
{

    __HAL_RCC_TIM1_CLK_ENABLE(); //aktivate timer1 ? 5 um auf APB1 zu sein (84MHz statt 168Mhz)
    TIM1->PSC = 83; //Prescaler auf 83 (84MHz / (83 + 1) = 1MHz) )
    TIM1->ARR = 999; //timer zählt bis 999+1 also 1ms (1MHz / 1000 = 1ms) 
    TIM1->EGR = TIM_EGR_UG; //Update Generation um die neuen Werte zu laden
    TIM1->DIER |= TIM_DIER_UIE;  // interupt wird ausgelöst wenn timer überlauft, also nach 1ms
    HAL_NVIC_SetPriority(TIM1_IRQn, 3, 0); //set priority 3 es soll vor allem kein SPI oder so zerstören. nach dem es sich da um wenige mikrosek handelt kann der kicker auch bissi länger an sein.

    switch (port)
    {
        case IOS1:
            pinMode(PB6, OUTPUT);   //PB6 als Ausgang für den Kicker konfigurieren
            digitalWrite(PB6, LOW); //sicher stellen, dass der kicker aus is
            aktiv_kicker_port = 1; //merke kicker um im interrupt den richtigen auszuschalten und WRITE_KCIKER 
            break;

        case IOS2:
            pinMode(PB7, OUTPUT);   //PB6 als Ausgang für den Kicker konfigurieren
            digitalWrite(PB7, LOW); //sicher stellen, dass der kicker aus is
            aktiv_kicker_port = 2; //merke kicker um im interrupt den richtigen auszuschalten und WRITE_KCIKER
            break;

        case IOS3:
            pinMode(PB8, OUTPUT);   //PB6 als Ausgang für den Kicker konfigurieren
            digitalWrite(PB8, LOW); //sicher stellen, dass der kicker aus is
            aktiv_kicker_port = 3; //merke kicker um im interrupt den richtigen auszuschalten und WRITE_KCIKER
            break;

        case IOS4:
            pinMode(PB9, OUTPUT);   //PB6 als Ausgang für den Kicker konfigurieren
            digitalWrite(PB9, LOW); //sicher stellen, dass der kicker aus is
            aktiv_kicker_port = 4; //merke kicker um im interrupt den richtigen auszuschalten und WRITE_KCIKER
            break;

    }

}

void WRITE_KICKER(uint8_t port, uint16_t duration_ms) //kickt für die angegebene dauer in ms, ohne den main code zu blockieren
{
    TIM1->ARR = duration_ms * 1000 -1; //timer zählt bis dahin mit 1Mhz also 1ms = 1000 counts, 10ms = 10000 counts usw. -1 weil der timer von 0 zählt
    TIM1->EGR = TIM_EGR_UG; //Update Generation um die neuen Werte sofort zu laden    
    TIM1->CNT = 0; //Timer zurücksetzen

    switch (aktiv_kicker_port) 
    {
        case 1:
            digitalWrite(PB6, HIGH); //Kicker los
            break;

        case 2:
            digitalWrite(PB7, HIGH); //Kicker los
            break;
            
        case 3:
            digitalWrite(PB8, HIGH); //Kicker los
            break;
            
        case 4:
            digitalWrite(PB9, HIGH); //Kicker los
            break;
            
    }

    
    TIM5->CR1 |= TIM_CR1_CEN; //Timer aktivieren (CEN = Counter Enable)
}

extern "C" void TIM5_IRQHandler() //wenn TIM5 überläuft, also nach duration_ms, kommt dieser interrupt
{
    if (TIM5->SR & TIM_SR_UIF) //überprüfen, ob der Update Interrupt Flag gesetzt ist
    {
        TIM5->SR &= ~TIM_SR_UIF; //Interrupt Flag zurücksetzen

        switch (aktiv_kicker_port) 
        {
        case 1:
            digitalWrite(PB6, LOW); //Kicker aus
            break;

        case 2:
            digitalWrite(PB7, LOW); //Kicker aus
            break;

        case 3:
            digitalWrite(PB8, LOW); //Kicker aus
            break;

        case 4:
            digitalWrite(PB9, LOW); //Kicker aus
            break;

        }

        aktiv_kicker_port = 0; //reset (lwk egal)
        TIM5->CR1 &= ~TIM_CR1_CEN;  // Timer stoppen

    }
}
