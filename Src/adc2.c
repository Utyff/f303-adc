#include <stm32f3xx.h>

void ADC_init() __attribute__((section (".ccmram")));

void ADC_start() __attribute__((section (".ccmram")));

/**
 * ADC3 channel 1 - PB1
 * ADC4 channel 3 - PB12
 */


#define ADC_6BITS  0b11u
#define ADC_8BITS  0b10u
#define ADC_10BITS 0b01u
#define ADC_12BITS 0b00u
//uint8_t adcResolution = ADC_8BITS;
extern uint8_t adcResolution;

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
//uint8_t rccDivider = 0b10000;
extern uint8_t rccDivider;

// Delay for interleaved mode. Set only when ADEN=0
// circle time = SAMPLE_TIME + CONV. TIME = 1.5 + 8.5 = 10 tics
// Delay = circle time /2 - SAMPLE_TIME = 10 / 2 - 1.5 = 3.5 tics
// 0b0000 - 1
// 0b0001 - 2
// 0b0010 - 3
// 0b0011 - 4   MAX: 1011 - 12
//uint8_t interleaveDelay = 0b0001;
extern uint8_t interleaveDelay;

//000: 1.5 ADC clock cycles
//001: 2.5 ADC clock cycles
//010: 4.5 ADC clock cycles
//011: 7.5 ADC clock cycles
//100: 19.5 ADC clock cycles
//101: 61.5 ADC clock cycles
//110: 181.5 ADC clock cycles
//111: 601.5 ADC clock cycles
//uint8_t sampleTime = 0b000;
extern uint8_t sampleTime;

// ADC clock
// 00: (Asynchronous clock mode) PLL
// 01: HCLK/1 (Synchronous clock mode) AHB
// 10: HCLK/2
// 11: HCLK/4
//uint8_t adcClock = 0b00;
extern uint8_t adcClock;

#define BUF_SIZE 4096
uint8_t samplesBuffer2[BUF_SIZE];
uint8_t samplesReady2;
uint8_t samplesError2;

void DWT_Delay_us(uint32_t us);

void ADC_init() __attribute__((section (".ccmram")));

void ADC_start() __attribute__((section (".ccmram")));

void DMA_start() __attribute__((section (".ccmram")));


void ADC_init2() {
    // init GPIO, VR and calibration run only once
    // Enable clocks GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Set mode b11 (analog input) for ADC pins
    GPIOB->MODER |= (0b11u << (1  * 2)); // PB1  for ADC3 ch1
    GPIOB->MODER |= (0b11u << (12 * 2)); // PB12 for ADC4 ch3

    RCC->AHBENR |= RCC_AHBENR_ADC34EN; // turn on ADC34 clock

    // Set Prescaler ADC for Asynchronous clock mode
    MODIFY_REG(RCC->CFGR2, RCC_CFGR2_ADCPRE12_Msk, rccDivider << RCC_CFGR2_ADCPRE12_Pos);

    // Set ADC clock
    MODIFY_REG(ADC34_COMMON->CCR, ADC_CCR_CKMODE_Msk, adcClock << ADC_CCR_CKMODE_Pos);

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
}

void ADC_start2() {
    DMA_start();
    // Delay for interleaved mode. Set only when ADEN=0
    ADC34_COMMON->CCR |= (interleaveDelay << ADC_CCR_DELAY_Pos);

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
    MODIFY_REG(ADC34_COMMON->CCR, ADC_CCR_CKMODE_Msk, adcClock << ADC_CCR_CKMODE_Pos);

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
    // 00110: Regular simultaneous mode
    // 00111: Interleaved mode
    MODIFY_REG(ADC34_COMMON->CCR, ADC_CCR_DUAL_Msk, 0b00111u << ADC_CCR_DUAL_Pos);

    // DMA mode.  0 -> One Shot; 1 -> Circular
    // 0 - stop when DMA_CCR_TCIE
    ADC34_COMMON->CCR &= ~ADC_CCR_DMACFG;

    // COMMON DMA mode b11 - for 8-bit resolution
    ADC34_COMMON->CCR |= (0b11u << ADC_CCR_MDMA_Pos);

    // ADC start conversion
    ADC1->CR |= ADC_CR_ADSTART;

//    __WFI(); // stop CPU
}

#define TIM_SIZE 20
int tim_i2 = 0;
int tim2[TIM_SIZE];
uint32_t st2;

void DMA_start2() {
    // Enable clocks DMA2
    RCC->AHBENR |= RCC_AHBENR_DMA2EN;

    // Disable DMA
    DMA2_Channel5->CCR &= ~DMA_CCR_EN;

    // enable interrupts - Transfer complete and error
    DMA2_Channel5->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;
    // disable Circular mode
    DMA2_Channel5->CCR &= ~DMA_CCR_CIRC;

    // Memory increment mode
    DMA2_Channel5->CCR |= DMA_CCR_MINC;

    // Peripheral size
    // 00: 8-bits
    // 01: 16-bits
    // 10: 32-bits
    DMA2_Channel5->CCR |= (0b01u << DMA_CCR_PSIZE_Pos);
    // Memory size
    DMA2_Channel5->CCR |= (0b01u << DMA_CCR_MSIZE_Pos);

    // Channel Priority level 11 - Very high
    DMA2_Channel5->CCR |= DMA_CCR_PL_Msk;

    // Number of data to transfer. 2 samples in 1 transfer.
    DMA2_Channel5->CNDTR = BUF_SIZE / 2;

    // Peripheral address register
    DMA2_Channel5->CPAR = (uint32_t) &ADC34_COMMON->CDR;

    // Memory address register
    DMA2_Channel5->CMAR = (uint32_t) (&samplesBuffer2);

    // clear interrupt flags
    DMA2->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5;

    // Enable DMA
    DMA2_Channel5->CCR |= DMA_CCR_EN;
    st2 = DWT->CYCCNT;

    NVIC_EnableIRQ(DMA2_Channel5_IRQn); // enable DMA2 CH5 interrupt
}

uint16_t dmaT2 = 0;
uint16_t dmaE2 = 0;

void DMA2_Channel5_IRQHandler() {
    if (DMA2->ISR & DMA_ISR_TCIF5) { // transfer complete

        tim2[tim_i2] = DWT->CYCCNT - st2;
        if(++tim_i2>=TIM_SIZE) tim_i2 = 0;

        // ADC stop conversion
        ADC3->CR |= ADC_CR_ADSTP;
        samplesReady2 = 1;

        dmaT2++;
        GPIOC->ODR ^= GPIO_ODR_14;    // toggle PC14
        DMA2->IFCR = DMA_IFCR_CTCIF5; // clear interrupt flag
    }
    if (DMA2->ISR & DMA_ISR_TEIF5) {
        DMA2->IFCR = DMA_IFCR_CTEIF5;
        dmaE2++;
        samplesError2 = 1;
    }
    DMA2->IFCR = DMA_IFCR_CGIF1;
}
