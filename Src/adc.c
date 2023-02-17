#include <stm32f3xx.h>

void ADC_init() __attribute__((section (".ccmram")));

void ADC_start() __attribute__((section (".ccmram")));

/**
 * ADC1 channel 1 - PA0
 * ADC2 channel 1 - PA4
 */


#define ADC_6BITS  0b11u
#define ADC_8BITS  0b10u
#define ADC_10BITS 0b01u
#define ADC_12BITS 0b00u
uint8_t adcResolution = ADC_8BITS;

// 0b10000: PLL clock divided by 1  // RCC_CFGR2_ADCPRE12_DIV1
// 0b10001: PLL clock divided by 2
// 0b10010: PLL clock divided by 4
// 0b10011: PLL clock divided by 6
// 0b10100: PLL clock divided by 8
// 0b10101: PLL clock divided by 10
// 0b10110: PLL clock divided by 12
// 0b10111: PLL clock divided by 16
// 0b11000: PLL clock divided by 32
// 0b11001: PLL clock divided by 64
// 0b11010: PLL clock divided by 128
// 0b11011: PLL clock divided by 256
uint8_t rccDivider = 0b10000;

// Delay for interleaved mode. Set only when ADEN=0
// circle time = SAMPLE_TIME + CONV. TIME = 1.5 + 8.5 = 10 tics
// Delay = circle time /2 - SAMPLE_TIME = 10 / 2 - 1.5 = 3.5 tics
// 0b0000 - 1
// 0b0001 - 2
// 0b0010 - 3
// 0b0011 - 4   MAX: 1011 - 12
uint8_t interleaveDelay = 0b0010;

//000: 1.5 ADC clock cycles
//001: 2.5 ADC clock cycles
//010: 4.5 ADC clock cycles
//011: 7.5 ADC clock cycles
//100: 19.5 ADC clock cycles
//101: 61.5 ADC clock cycles
//110: 181.5 ADC clock cycles
//111: 601.5 ADC clock cycles
uint8_t sampleTime = 0b000;

// ADC clock
// 00: (Asynchronous clock mode) PLL
// 01: HCLK/1 (Synchronous clock mode) AHB
// 10: HCLK/2
// 11: HCLK/4
uint8_t adcClock = 0b00;

#define BUF_SIZE 4096
uint8_t samplesBuffer[BUF_SIZE];
uint8_t samplesReady;
uint8_t samplesError;

void DWT_Delay_us(uint32_t us);

void ADC_init() __attribute__((section (".ccmram")));

void ADC_start() __attribute__((section (".ccmram")));

void DMA_start() __attribute__((section (".ccmram")));


void ADC_init() {
    // init GPIO, VR and calibration run only once
    // Enable clocks GPIOA and GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Set mode b11 (analog input) for ADC pins
    GPIOA->MODER |= (0b11u << 0u);      // PA0 for ADC1 ch1
    GPIOA->MODER |= (0b11u << (4 * 2)); // PA4 for ADC2 ch1

    RCC->AHBENR |= RCC_AHBENR_ADC12EN; // turn on ADC12 clock

    // Set Prescaler ADC for Asynchronous clock mode
    MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12_Msk, rccDivider << RCC_CFGR2_ADCPRE12_Pos);

    // Set ADC clock
    MODIFY_REG(ADC12_COMMON->CCR, ADC_CCR_CKMODE_Msk, adcClock << ADC_CCR_CKMODE_Pos);

    // enable the ADC voltage regulator
    ADC1->CR &= ~ADC_CR_ADVREGEN_1;
    ADC2->CR &= ~ADC_CR_ADVREGEN_1;

    ADC1->CR |= ADC_CR_ADVREGEN_0;
    ADC2->CR |= ADC_CR_ADVREGEN_0;
    DWT_Delay_us(10); // voltage regulator startup time

    // start ADC calibration cycle
    ADC1->CR |= ADC_CR_ADCAL;
    // wait for calibration to complete
    while (ADC1->CR & ADC_CR_ADCAL);

    // start ADC calibration cycle
    ADC2->CR |= ADC_CR_ADCAL;
    // wait for calibration to complete
    while (ADC2->CR & ADC_CR_ADCAL);

//    ADC_start();
}

void ADC_start() {
    DMA_start();
    // Delay for interleaved mode. Set only when ADEN=0
    ADC12_COMMON->CCR |= (interleaveDelay << ADC_CCR_DELAY_Pos);

    // enable the ADC
    ADC1->CR |= ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADEN;
    // wait till ADC start
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
    while (!(ADC2->ISR & ADC_ISR_ADRDY));

    // Select ADC Channels
    ADC1->SQR1 = (1u << ADC_SQR1_SQ1_Pos); // 1-st channel for ADC1
    ADC2->SQR1 = (1u << ADC_SQR1_SQ1_Pos); // 1-st channel for ADC2
    // Regular channel sequence length - 1
    ADC1->SQR1 &= ~ADC_SQR1_L_Msk;
    ADC2->SQR1 &= ~ADC_SQR1_L_Msk;

    // Set Prescaler ADC for Asynchronous clock mode
    MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12_Msk, rccDivider << RCC_CFGR2_ADCPRE12_Pos);
    // Set ADC clock
    MODIFY_REG(ADC12_COMMON->CCR, ADC_CCR_CKMODE_Msk, adcClock << ADC_CCR_CKMODE_Pos);

    // Set sampling time for channels
    MODIFY_REG(ADC1->SMPR1, ADC_SMPR1_SMP1_Msk, sampleTime << ADC_SMPR1_SMP1_Pos); // ADC1 channel 1
    MODIFY_REG(ADC2->SMPR1, ADC_SMPR1_SMP1_Msk, sampleTime << ADC_SMPR1_SMP1_Pos); // ADC2 channel 1

    // set data resolution
    MODIFY_REG(ADC1->CFGR, ADC_CFGR_RES_Msk, adcResolution << ADC_CFGR_RES_Pos);
    MODIFY_REG(ADC2->CFGR, ADC_CFGR_RES_Msk, adcResolution << ADC_CFGR_RES_Pos);

    // Enable continuous conversion mode. Only on the master
    ADC1->CFGR |= ADC_CFGR_CONT; // Master ADC1 + ADC2

    // dual mode
    // 00000: Independent mode
    // 00110: Regular simultaneous mode only
    // 00111: Interleaved mode only
    MODIFY_REG(ADC12_COMMON->CCR, ADC_CCR_DUAL_Msk, 0b00111u << ADC_CCR_DUAL_Pos);

    // DMA mode.  0 -> One Shot; 1 -> Circular
    // 0 - stop when DMA_CCR_TCIE
    ADC12_COMMON->CCR &= ~ADC_CCR_DMACFG;

    // COMMON DMA mode b11 - for 8-bit resolution
    ADC12_COMMON->CCR |= (0b11u << ADC_CCR_MDMA_Pos);

    // ADC start conversion
    ADC1->CR |= ADC_CR_ADSTART;

//    __WFI(); // stop CPU
}

void DMA_start() {
    // Enable clocks DMA1
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Disable DMA
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    // enable interrupts - Transfer complete and error
    DMA1_Channel1->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;
    // disable Circular mode
    DMA1_Channel1->CCR &= ~DMA_CCR_CIRC;

    // Memory increment mode
    DMA1_Channel1->CCR |= DMA_CCR_MINC;

    // Peripheral size
    // 00: 8-bits
    // 01: 16-bits
    // 10: 32-bits
    DMA1_Channel1->CCR |= (0b01u << DMA_CCR_PSIZE_Pos);
    // Memory size
    DMA1_Channel1->CCR |= (0b01u << DMA_CCR_MSIZE_Pos);

    // Channel Priority level 11 - Very high
    DMA1_Channel1->CCR |= DMA_CCR_PL_Msk;

    // Number of data to transfer. 2 samples in 1 transfer.
    DMA1_Channel1->CNDTR = BUF_SIZE / 2;

    // Peripheral address register
    DMA1_Channel1->CPAR = (uint32_t) &ADC12_COMMON->CDR;

    // Memory address register
    DMA1_Channel1->CMAR = (uint32_t) (&samplesBuffer);

    // clear interrupt flags
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;

    // Enable DMA
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable DMA1 CH1 interrupt
}

uint16_t dmaT = 0;
uint16_t dmaE = 0;

void DMA1_Channel1_IRQHandler() {
    if (DMA1->ISR & DMA_ISR_TCIF1) { // transfer complete
        // ADC stop conversion
        ADC1->CR |= ADC_CR_ADSTP;
        samplesReady = 1;

        dmaT++;
        // toggle PB0
        uint32_t odr = GPIOB->ODR;
        // Set selected pins that were at low level, and reset ones that were high
        GPIOB->BSRR = ((odr & GPIO_ODR_0) << 16U) | (~odr & GPIO_ODR_0);
        DMA1->IFCR = DMA_IFCR_CTCIF1; // clear interrupt flag
    }
    if (DMA1->ISR & DMA_ISR_TEIF1) {
        DMA1->IFCR = DMA_IFCR_CTEIF1;
        dmaE++;
        samplesError = 1;
        // toggle PB1
        uint32_t odr = GPIOB->ODR;
        // Set selected pins that were at low level, and reset ones that were high
        GPIOB->BSRR = ((odr & GPIO_ODR_1) << 16U) | (~odr & GPIO_ODR_1);
    }
    DMA1->IFCR = DMA_IFCR_CGIF1;
}
