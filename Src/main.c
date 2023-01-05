#include <stm32f3xx.h>

void ADC_Init();

#define SYSTEM_CORE_CLOCK 72000000
#define DWT_IN_MICROSEC (SYSTEM_CORE_CLOCK/1000000)


void DWT_Init() {
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

    RCC->CFGR |= RCC_CFGR_PLLMUL3;     // PLL MUL = x3, SYS_CLK=72MHz (x9 for 8MHz)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 Prescaler = 2
    RCC->CFGR |= RCC_CFGR_PLLSRC;      // PLL source = HSE

    FLASH->ACR |= FLASH_ACR_LATENCY_1; // Two wait states

    RCC->CR |= RCC_CR_PLLON;           // Enable and wait PLL
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    RCC->CFGR |= RCC_CFGR_SW_PLL;      // Select PLL as system clock
}

int main(void) {
    SystemClock_Config();

    DWT_Init();
    ADC_Init();

    while (1) {
    }
}
