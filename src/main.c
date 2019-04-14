#include "main.h"

// Core value initializations.
volatile uint32_t millis = 0;

// Stub '_sbrk' method.
register char* stack_ptr asm( "sp" );
void* _sbrk( int incr )
{
  extern char end asm("end");
  static char *heap_end;
  char *prev_heap_end;

  if ( heap_end == 0 ) { heap_end = &end; }

  prev_heap_end = heap_end;
  if ( heap_end + incr > stack_ptr ) {
    errno = ENOMEM;
    return ( void* )-1;
  }

  heap_end += incr;

  return ( void* )prev_heap_end;
}

// Simple ring buffer.
#define RINGBUF_LEN ( 256 )
typedef struct {
  volatile char buf[ RINGBUF_LEN ];
  volatile int  pos;
  volatile int  ext;
} ring_buf;
static ring_buf prompt_rb;
static ring_buf fprint_rb;

// TODO: Move these into separate files.
// UART helper methods.
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

// FPM-C driver methods.
// Return the number of milliseconds elapsed since last reset.
uint32_t fpm_millis_func( void ) {
  return millis;
}

// Delay for a given number of milliseconds.
void fpm_delay_func( uint32_t ms_delay ) {
  int next = millis + ms_delay;
  while ( next < millis ) {};
}

// Receive a number of bytes over UART.
uint16_t fpm_uart_read_func( uint8_t *bytes, uint16_t len ) {
  int read;
  for ( read = 0; read < len; ++read ) {
    bytes[ read ] = fprint_rb.buf[ fprint_rb.pos ];
    int next_pos = fprint_rb.pos + 1;
    if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
    fprint_rb.pos = next_pos;
    if ( fprint_rb.pos == fprint_rb.ext ) { break; }
  }
  // Return the number of bytes actually read.
  return read;
}

// Transmit a stream of bytes over UART.
void fmp_uart_write_func( uint8_t *bytes, uint16_t len ) {
  uart_tx_str( USART1, bytes, len );
}

// Return the number of bytes available in the UART buffer.
uint16_t fpm_uart_avail_func( void ) {
  if ( fprint_rb.ext >= fprint_rb.pos ) {
    return ( fprint_rb.ext - fprint_rb.pos );
  }
  else {
    return ( RINGBUF_LEN - ( fprint_rb.pos - fprint_rb.ext ) );
  }
}

/**
 * Main program.
 */
int main(void) {
  // System initialization.
  SystemInit();

  // Initial clock setup.
  clock_setup();

  // Enable SysTick interrupt for 1ms ticks.
  SysTick_Config( SystemCoreClock / 1000 );

  // Enable the GPIOA and GPIOB peripherals.
  #ifdef VVC_F0
    RCC->AHBENR   |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR   |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR   |= RCC_AHBENR_GPIOCEN;
  #elif  VVC_F1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
  #elif  VVC_L0
    RCC->IOPENR   |= RCC_IOPENR_IOPAEN;
    RCC->IOPENR   |= RCC_IOPENR_IOPBEN;
    RCC->IOPENR   |= RCC_IOPENR_IOPCEN;
  #elif  VVC_L4
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR  |= RCC_APB2ENR_USART1EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
  #endif

  // LED output type: Push-pull, low-speed.
  GPIO_InitTypeDef gpio_i = {
    .Pin = ( 1 << Pi_LED ),
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
    #if !defined( STM32F1 )
      .Alternate = 0,
    #endif
  };
  HAL_GPIO_Init( Po_LED, &gpio_i );

  // Initialize ringbuffer structs.
  prompt_rb.pos = 0;
  prompt_rb.ext = 0;
  fprint_rb.pos = 0;
  fprint_rb.ext = 0;

  // Setup USART1/2 pins.
  gpio_i.Pin = ( 1 << Pi_U1TX );
  gpio_i.Mode = GPIO_MODE_AF_PP;
  gpio_i.Speed = GPIO_SPEED_FREQ_HIGH;
  #if !defined( STM32F1 )
    gpio_i.Alternate = AF_U1TX;
  #endif
  HAL_GPIO_Init( Po_U1TX, &gpio_i );
  gpio_i.Pin = ( 1 << Pi_U2TX );
  #if !defined( STM32F1 )
    gpio_i.Alternate = AF_U2TX;
  #endif
  HAL_GPIO_Init( Po_U1TX, &gpio_i );
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
                 #if defined( STM32G0 )
                   USART_CR1_RXNEIE_RXFNEIE |
                 #else
                   USART_CR1_RXNEIE |
                 #endif
                   USART_CR1_TE );
  USART1->CR1 |= ( USART_CR1_UE |
                   USART_CR1_RE |
                 #if defined( STM32G0 )
                   USART_CR1_RXNEIE_RXFNEIE |
                 #else
                   USART_CR1_RXNEIE |
                 #endif
                   USART_CR1_TE );

  // Setup interrupts.
  #if   defined( ARM_CM4F ) || defined( ARM_CM3 )
    NVIC_SetPriorityGrouping( 0 );
    uint32_t pri_enc = NVIC_EncodePriority( 0, 1, 0 );
    NVIC_SetPriority( USART1_IRQn, pri_enc );
    NVIC_EnableIRQ( USART1_IRQn );
    NVIC_SetPriority( USART2_IRQn, pri_enc );
    NVIC_EnableIRQ( USART2_IRQn );
  #elif defined( ARM_CM0PLUS )
    NVIC_SetPriority( USART1_IRQn, 1 );
    NVIC_EnableIRQ( USART1_IRQn );
    NVIC_SetPriority( USART2_IRQn, 1 );
    NVIC_EnableIRQ( USART2_IRQn );
  #endif

  // Print a test string to the 'prompt' USART.
  const char *msg = "test\r\n";
  uart_tx_str( USART2, ( uint8_t* )msg, strlen( msg ) );

  char prompt_buf[ 256 ];
  snprintf( prompt_buf, 256, "Ticks: %ld\r\n", millis );
  uart_tx_str( USART2, ( uint8_t* )prompt_buf, strlen( prompt_buf ) );
  // Main loop.
  while (1) {
    delay_cycles(2000000);
    Po_LED->ODR ^=  (1 << Pi_LED);
    snprintf( prompt_buf, 256, "Ticks: %ld\r\n", millis );
    uart_tx_str( USART2, ( uint8_t* )prompt_buf, strlen( prompt_buf ) );
  }
}

// SysTick interrupt handler.
void SysTick_handler( void ) {
  ++millis;
}

// USART interrupt handlers.
void USART1_IRQ_handler( void ) {
  #if defined( STM32F1 ) || defined( STM32F4 )
    if ( USART1->SR & USART_SR_RXNE ) {
      int next_pos = fprint_rb.ext + 1;
      if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
      fprint_rb.buf[ next_pos ] = USART1->DR;
      fprint_rb.ext = next_pos;
    }
  #else
    if ( USART1->ISR & USART_ISR_RXNE ) {
      int next_pos = fprint_rb.ext + 1;
      if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
      fprint_rb.buf[ next_pos ] = USART1->RDR;
      fprint_rb.ext = next_pos;
    }
  #endif
}

void USART2_IRQ_handler( void ) {
  #if defined( STM32F1 ) || defined( STM32F4 )
    if ( USART2->SR & USART_SR_RXNE ) {
      int next_pos = prompt_rb.ext + 1;
      if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
      prompt_rb.buf[ next_pos ] = USART2->DR;
      prompt_rb.ext = next_pos;
    }
  #else
    if ( USART2->ISR & USART_ISR_RXNE ) {
      int next_pos = prompt_rb.ext + 1;
      if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
      prompt_rb.buf[ next_pos ] = USART2->RDR;
      prompt_rb.ext = next_pos;
    }
  #endif
}
