#include "uart.h"

// Initialize the two UARTs. One for a prompt,
// one for the fingerprint reader module.
void uarts_init() {
  // Initialize ringbuffer structs.
  prompt_rb.pos = 0;
  prompt_rb.ext = 0;
  fprint_rb.pos = 0;
  fprint_rb.ext = 0;

  // Setup USART1/2 pins.
  GPIO_InitTypeDef gpio_i = {
    .Pin = ( 1 << Pi_U1TX ),
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_HIGH,
    #if !defined( STM32F1 )
      .Alternate = AF_U1TX,
    #endif
  };
  HAL_GPIO_Init( Po_U1TX, &gpio_i );
  gpio_i.Pin = ( 1 << Pi_U2TX );
  #if !defined( STM32F1 )
    gpio_i.Alternate = AF_U2TX;
  #endif
  HAL_GPIO_Init( Po_U2TX, &gpio_i );
  gpio_i.Pin = ( 1 << Pi_U1RX );
  #if !defined( STM32F1 )
    gpio_i.Alternate = AF_U1RX;
  #else
    gpio_i.Mode = GPIO_MODE_INPUT;
  #endif
  HAL_GPIO_Init( Po_U1RX, &gpio_i );
  gpio_i.Pin = ( 1 << Pi_U2RX );
  #if !defined( STM32F1 )
    gpio_i.Alternate = AF_U2RX;
  #endif
  HAL_GPIO_Init( Po_U2RX, &gpio_i );

  // Setup USART1/2.
  #if defined( STM32F1 )
    // ???
  #else
    USART2->BRR  =  ( SystemCoreClock / 115200 );
    USART1->BRR  =  ( SystemCoreClock / 57600 );
  #endif
  USART2->CR1 |= ( USART_CR1_UE |
                   USART_CR1_RE |
                   USART_CR1_RXNEIE |
                   USART_CR1_TE );
  USART1->CR1 |= ( USART_CR1_UE |
                   USART_CR1_RE |
                   USART_CR1_RXNEIE |
                   USART_CR1_TE );

  // Setup interrupts.
  __enable_irq();
  #if   defined( STM32L4 ) || defined( STM32F1 )
    NVIC_SetPriorityGrouping( 0 );
    uint32_t pri_enc = NVIC_EncodePriority( 0, 1, 0 );
    NVIC_SetPriority( USART1_IRQn, pri_enc );
    NVIC_EnableIRQ( USART1_IRQn );
    NVIC_SetPriority( USART2_IRQn, pri_enc );
    NVIC_EnableIRQ( USART2_IRQn );
  #else
    NVIC_SetPriority( USART1_IRQn, 1 );
    NVIC_EnableIRQ( USART1_IRQn );
    NVIC_SetPriority( USART2_IRQn, 1 );
    NVIC_EnableIRQ( USART2_IRQn );
  #endif
}

// Send a string over a UART. (Blocking)
void uart_tx_str( USART_TypeDef *u, const unsigned char *str, int len ) {
  while ( len-- ) {
    #if   defined( STM32L4 )
      while ( ( u->ISR & USART_ISR_TXE ) == 0 ) {};
      u->TDR = *str++;
    #elif defined( STM32F4 ) || defined( STM32F1 )
      while ( ( u->SR & USART_SR_TXE ) == 0 ) {};
      u->DR = *str++;
    #elif defined( STM32G0 )
      while ( ( u->ISR & USART_ISR_TXE_TXFNF ) == 0 ) {};
      u->TDR = *str++;
    #endif
  }
}
