#include "global.h"
#include "uart.h"

// SysTick interrupt handler.
void SysTick_handler( void ) {
  ++millis;
}

// USART interrupt handlers.
void USART1_IRQ_handler( void ) {
  #if defined( STM32F1 )
    if ( USART1->SR & USART_SR_RXNE ) {
      int next_pos = fprint_rb.ext + 1;
      if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
      fprint_rb.buf[ fprint_rb.ext ] = USART1->DR;
      fprint_rb.ext = next_pos;
    }
  #else
    if ( USART1->ISR & USART_ISR_RXNE ) {
      int next_pos = fprint_rb.ext + 1;
      if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
      fprint_rb.buf[ fprint_rb.ext ] = USART1->RDR;
      fprint_rb.ext = next_pos;
    }
  #endif
}

void USART2_IRQ_handler( void ) {
  #if defined( STM32F1 )
    if ( USART2->SR & USART_SR_RXNE ) {
      int next_pos = prompt_rb.ext + 1;
      if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
      prompt_rb.buf[ prompt_rb.ext ] = USART2->DR;
      prompt_rb.ext = next_pos;
    }
  #else
    if ( USART2->ISR & USART_ISR_RXNE ) {
      int next_pos = prompt_rb.ext + 1;
      if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
      prompt_rb.buf[ prompt_rb.ext ] = USART2->RDR;
      prompt_rb.ext = next_pos;
    }
  #endif
}
