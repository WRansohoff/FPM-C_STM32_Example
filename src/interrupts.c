#include "global.h"
#include "uart.h"

// SysTick interrupt handler.
void SysTick_handler( void ) {
  ++millis;
}

// USART interrupt handlers.
void fprint_irq( void ) {
  #if defined( STM32F1 )
    if ( FPM_UART->SR & USART_SR_RXNE ) {
      int next_pos = fprint_rb.ext + 1;
      if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
      fprint_rb.buf[ fprint_rb.ext ] = FPM_UART->DR;
      fprint_rb.ext = next_pos;
    }
  #else
    if ( FPM_UART->ISR & USART_ISR_RXNE ) {
      int next_pos = fprint_rb.ext + 1;
      if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
      fprint_rb.buf[ fprint_rb.ext ] = FPM_UART->RDR;
      fprint_rb.ext = next_pos;
    }
  #endif
}

void prompt_irq( void ) {
  #if defined( STM32F1 )
    if ( P_UART->SR & USART_SR_RXNE ) {
      int next_pos = prompt_rb.ext + 1;
      if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
      prompt_rb.buf[ prompt_rb.ext ] = P_UART->DR;
      prompt_rb.ext = next_pos;
    }
  #else
    if ( P_UART->ISR & USART_ISR_RXNE ) {
      int next_pos = prompt_rb.ext + 1;
      if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
      prompt_rb.buf[ prompt_rb.ext ] = P_UART->RDR;
      prompt_rb.ext = next_pos;
    }
  #endif
}

// TODO: Only supports USART1/2
void USART1_IRQ_handler( void ) {
  if ( P_UART == USART1 ) { prompt_irq(); }
  else { fprint_irq(); }
}

void USART2_IRQ_handler( void ) {
  if ( P_UART == USART2 ) { prompt_irq(); }
  else { fprint_irq(); }
}
