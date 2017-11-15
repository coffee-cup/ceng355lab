
#include "cmsis/cmsis_device.h"
#include "diag/Trace.h"
#include "lcd.h"
#include <stdio.h>

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

void myGPIOA_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);

// Your global variables...

int main(int argc, char *argv[]) {
  trace_printf("System clock: %u Hz\n", SystemCoreClock);

  myGPIOA_Init(); /* Initialize I/O port PA */
  myTIM2_Init();  /* Initialize timer TIM2 */
  myEXTI_Init();  /* Initialize EXTI */
  myLCD_Init();   /* Initialize LCD Display */

  while (1) {
    // Nothing is going on here...
  }

  return 0;
}

void myGPIOA_Init() {
  /* Enable clock for GPIOA peripheral */
  // Relevant register: RCC->AHBENR
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  /* Configure PA1 as input */
  // Relevant register: GPIOA->MODER
  GPIOA->MODER &= ~(GPIO_MODER_MODER0);

  /* Ensure no pull-up/pull-down for PA1 */
  // Relevant register: GPIOA->PUPDR
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);
}

void myTIM2_Init() {
  /* Enable clock for TIM2 peripheral */
  // Relevant register: RCC->APB1ENR
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  /* Configure TIM2: buffer auto-reload, count up, stop on overflow,
   * enable update events, interrupt on overflow only */
  // Relevant register: TIM2->CR1
  TIM2->CR1 = ((uint16_t)0x008C);

  /* Set clock prescaler value */
  TIM2->PSC = myTIM2_PRESCALER;
  /* Set auto-reloaded delay */
  TIM2->ARR = myTIM2_PERIOD;

  /* Update timer registers */
  // Relevant register: TIM2->EGR
  TIM2->EGR = ((uint16_t)0x0001);

  /* Assign TIM2 interrupt priority = 0 in NVIC */
  // Relevant register: NVIC->IP[3], or use NVIC_SetPriority
  NVIC_SetPriority(TIM2_IRQn, 0);

  /* Enable TIM2 interrupts in NVIC */
  // Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
  NVIC_EnableIRQ(TIM2_IRQn);

  /* Enable update interrupt generation */
  // Relevant register: TIM2->DIER
  TIM2->DIER |= TIM_DIER_UIE;
}

void myEXTI_Init() {
  /* Map EXTI1 line to PA1 */
  // Relevant register: SYSCFG->EXTICR[0]
  SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA;

  /* EXTI1 line interrupts: set rising-edge trigger */
  // Relevant register: EXTI->RTSR
  EXTI->RTSR |= EXTI_RTSR_TR1;

  /* Unmask interrupts from EXTI1 line */
  // Relevant register: EXTI->IMR
  EXTI->IMR |= EXTI_IMR_MR1;

  /* Assign EXTI1 interrupt priority = 0 in NVIC */
  // Relevant register: NVIC->IP[1], or use NVIC_SetPriority
  NVIC_SetPriority(EXTI0_1_IRQn, 0);

  /* Enable EXTI1 interrupts in NVIC */
  // Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
  NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler() {
  /* Check if update interrupt flag is indeed set */
  if ((TIM2->SR & TIM_SR_UIF) != 0) {
    trace_printf("\n*** Overflow! ***\n");

    /* Clear update interrupt flag */
    // Relevant register: TIM2->SR
    TIM2->SR &= ~(TIM_SR_UIF);

    /* Restart stopped timer */
    // Relevant register: TIM2->CR1
    TIM2->CR1 |= TIM_CR1_CEN;
  }
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler() {

  /* Check if EXTI1 interrupt pending flag is indeed set */
  if ((EXTI->PR & EXTI_PR_PR1) != 0) {
    uint16_t timerEnabled = (TIM2->CR1 & TIM_CR1_CEN);

    if (timerEnabled == 1) { /* Timer is running, we need to stop it */
      // Second edge

      /*
       * MAX Frequency
       *
       * 500 kHz
       *
       * By the time the new rising edge comes, we are still
       * processing the first one. Our interrupt handler cannot
       * keep up with the speed of the signal.
       *
       * Theoretical max is the system clock speed (4.8 MHz).
       * But it takes to print and there are only processes running
       * on this computer.
       *
       * MIN Frequency
       *
       * TIM2 counter only has 32 bits. Min frequency will happen
       * when counter value overflows
       *
       * 2^(32-1) = 2147483648
       *
       *
       * 2147483648 / 48000000 (SystemCoreClock) = 44.7392426667 seconds
       * 1 / 44.7392426667 seconds = 0.02235174179 Hz
       *
       */

      uint32_t count = TIM2->CNT; /* Total number of clock cycles */

      /* System Clock = 48 MHz */
      double period = (double)count / ((double)SystemCoreClock);
      double freq = (double)1.0 / period;

      trace_printf("Period: %f s\n", period);
      trace_printf("Freq:   %f Hz\n\n", freq);

      TIM2->CR1 &= ~(TIM_CR1_CEN); /* Stop timer */
    } else {                       /* Timer is not on, we need to start it */
      // First edge

      TIM2->CNT = (uint32_t)0x0; /* Clear count register */
      TIM2->CR1 |= TIM_CR1_CEN;  /* Start Timer */
    }

    EXTI->PR |= EXTI_PR_PR1;
  }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
