#include <stm32f3xx.h>


#define SYSTEM_CORE_CLOCK 72000000
#define DWT_IN_MICROSEC (SYSTEM_CORE_CLOCK/1000000)

extern uint8_t samplesReady;
extern uint8_t samplesError;

void ADC_init() __attribute__((section (".ccmram")));

void ADC_start() __attribute__((section (".ccmram")));

int main(void) __attribute__((section (".ccmram")));


void DWT_init() {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

// microseconds
void DWT_Delay_us(uint32_t us) {
    uint32_t t0 = DWT->CYCCNT;
    uint32_t delta = us * DWT_IN_MICROSEC;

    while ((DWT->CYCCNT - t0) < delta) {}
}

void SystemClock_Config(void) {
    // External oscillator (HSE) = 24MHz
    // SYS_CLK = 72MHz
    // APB1 = 36MHz
    RCC->CR |= RCC_CR_HSEON;           // Enable HSE
    while ((RCC->CR & RCC_CR_HSERDY) == 0);

    RCC->CFGR |= RCC_CFGR_PLLMUL3;     // PLL MUL = x9, SYS_CLK=72MHz  (x9 for 8MHz, x3 for 24MHz)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 Prescaler = 2
    RCC->CFGR |= RCC_CFGR_PLLSRC;      // PLL source = HSE

    FLASH->ACR |= FLASH_ACR_LATENCY_1; // Two wait states

    RCC->CR |= RCC_CR_PLLON;           // Enable and wait PLL
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    RCC->CFGR |= RCC_CFGR_SW_PLL;      // Select PLL as system clock
}

int i1=0;
int i2=0;
#define TIME_SIZE 20
int time_i = 0;
int time[TIME_SIZE];
uint32_t start;
uint32_t readFail = 0;
uint32_t temp;

int main(void) {
    SystemClock_Config();

    DWT_init();
    ADC_init();

    // GPIO init
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    // control bits PB0 PB1 PB2 OUT mode & very high speed
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_Msk | GPIO_OSPEEDER_OSPEEDR1_Msk | GPIO_OSPEEDER_OSPEEDR2_Msk;
    MODIFY_REG(GPIOB->MODER, (GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk | GPIO_MODER_MODER2_Msk),
               (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0));

    uint32_t ledTime = 0;
    const uint32_t MASK = 0xFE000000;
    // 0x FE 00 00 00
    // 0x 01 ff ff ff = 2 800 000 us | 33 554 431 > 0,083 ns
    ADC_start();
    start = DWT->CYCCNT;

#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
        i1++;
        if ((DWT->CYCCNT & MASK) != ledTime) {
            GPIOB->ODR ^= GPIO_ODR_2; // LED1 toggle
            ledTime = DWT->CYCCNT & MASK;
        }

        if ((DWT->CYCCNT - start) > 300000) { // 20720
            readFail++;
            temp = ADC1->DR;
        }

        if (samplesReady | samplesError) {

            time[time_i] = DWT->CYCCNT - start;
            if(++time_i>=TIME_SIZE) time_i = 0;

            i2++;

            samplesReady = 0;
            samplesError = 0;

            ADC_start();
            start = DWT->CYCCNT;
        }
    }
}
