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
uint32_t fpm_millis( void ) {
  return millis;
}

// Delay for a given number of milliseconds.
void fpm_delay( uint32_t ms_delay ) {
  int next = millis + ms_delay;
  while ( millis < next ) {};
}

// Receive a number of bytes over UART.
uint16_t fpm_uart_read( uint8_t *bytes, uint16_t len ) {
  int read = 0;
  for ( read = 0; read < len; ++read ) {
    if ( fprint_rb.pos == fprint_rb.ext ) { break; }
    bytes[ read ] = fprint_rb.buf[ fprint_rb.pos ];
    int next_pos = fprint_rb.pos + 1;
    if ( next_pos >= RINGBUF_LEN ) { next_pos = 0; }
    fprint_rb.pos = next_pos;
  }
  // Return the number of bytes actually read.
  return read;
}

// Transmit a stream of bytes over UART.
void fpm_uart_write( uint8_t *bytes, uint16_t len ) {
  uart_tx_str( USART1, bytes, len );
}

// Return the number of bytes available in the UART buffer.
uint16_t fpm_uart_avail( void ) {
  if ( fprint_rb.ext >= fprint_rb.pos ) {
    return ( fprint_rb.ext - fprint_rb.pos );
  }
  else {
    return ( RINGBUF_LEN - ( fprint_rb.pos - fprint_rb.ext ) );
  }
}

// FPM library helper methods.
// Return an available ID slot to store a new fingerprint in.
// Return -1 if no slots are available.
int get_free_fprint_id( void ) {
  int16_t fid;
  for ( int page = 0; page < ( fprint_params.capacity / FPM_TEMPLATES_PER_PAGE ) + 1; ++page ) {
    if ( fpm_get_free_index( &fprint, page, &fid ) == FPM_OK ) {
      if ( fid != FPM_NOFREEINDEX ) { return fid; }
    }
    else { return -1; }
  }
  return -1;
}

// Test 'enroll fingerprint' loop. Look for a fingerprint,
// Ask the user to remove it and place it again, then see if
// they match or print an error code.
void test_fingerprint_enroll( void ) {
  int16_t rcode = -1;
  char msg_buf[ 64 ];
  snprintf( msg_buf, 64, "Place finger on reader.\r\n" );
  uart_tx_str( USART2, ( uint8_t* )msg_buf, strlen( msg_buf ) );
  while ( rcode != FPM_OK ) {
    rcode = fpm_get_image( &fprint );
    if ( rcode == FPM_NOFINGER ) {
      msg_buf[ 0 ] = '.';
      uart_tx_str( USART2, ( uint8_t* )msg_buf, 1 );
      fpm_delay( 10 );
    }
    else if ( rcode == FPM_OK ) {
      snprintf( msg_buf, 64, "\r\nImage taken; remove finger.\r\n" );
      uart_tx_str( USART2, ( uint8_t* )msg_buf, strlen( msg_buf ) );
    }
    else {
      snprintf( msg_buf, 64, "\r\nError 0x%02x\r\n", rcode );
      uart_tx_str( USART2, ( uint8_t* )msg_buf, strlen( msg_buf ) );
      return;
    }
  }
  fpm_image2Tz( &fprint, 1 );

  // Wait for the user to remove their finger.
  fpm_delay( 2000 );
  rcode = 0;
  while ( rcode != FPM_NOFINGER ) {
    rcode = fpm_get_image( &fprint );
    fpm_delay( 10 );
  }

  rcode = -1;
  snprintf( msg_buf, 64, "Place the same finger again.\r\n" );
  uart_tx_str( USART2, ( uint8_t* )msg_buf, strlen( msg_buf ) );
  while ( rcode != FPM_OK ) {
    rcode = fpm_get_image( &fprint );
    if ( rcode == FPM_NOFINGER ) {
      msg_buf[ 0 ] = '.';
      uart_tx_str( USART2, ( uint8_t* )msg_buf, 1 );
      fpm_delay( 10 );
    }
    else if ( rcode == FPM_OK ) {
      snprintf( msg_buf, 64, "\r\nImage taken.\r\n" );
      uart_tx_str( USART2, ( uint8_t* )msg_buf, strlen( msg_buf ) );
    }
    else {
      snprintf( msg_buf, 64, "\r\nError 0x%02x\r\n", rcode );
      uart_tx_str( USART2, ( uint8_t* )msg_buf, strlen( msg_buf ) );
      return;
    }
  }
  fpm_image2Tz( &fprint, 2 );

  // Create the new fingerprint model.
  rcode = fpm_create_model( &fprint );
  if ( rcode == FPM_OK ) {
    snprintf( msg_buf, 64, "Prints matched!\r\n" );
    uart_tx_str( USART2, ( uint8_t* )msg_buf, strlen( msg_buf ) );
  }
  else {
    snprintf( msg_buf, 64, "Error 0x%02x\r\n", rcode );
    uart_tx_str( USART2, ( uint8_t* )msg_buf, strlen( msg_buf ) );
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

  // Enable peripherals. TODO: HAL init?
  //__HAL_RCC_GPIOA_CLK_ENABLE();
  //__HAL_RCC_GPIOB_CLK_ENABLE();
  //__HAL_RCC_GPIOC_CLK_ENABLE();
  //__HAL_RCC_USART1_CLK_ENABLE();
  //__HAL_RCC_USART2_CLK_ENABLE();
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

  // Test UART prompt output and define an output buffer.
  // TODO: Use constant for length.
  char prompt_buf[ 256 ];
  snprintf( prompt_buf, 256, "Ticks: %ld\r\n", millis );
  uart_tx_str( USART2, ( uint8_t* )prompt_buf, strlen( prompt_buf ) );

  // Initialize fingerprint reader values.
  fprint.address = 0xFFFFFFFF;
  fprint.password = 0;
  fprint.avail_func = fpm_uart_avail;
  fprint.read_func = fpm_uart_read;
  fprint.write_func = fpm_uart_write;
  int rcode = fpm_begin( &fprint, fpm_millis, fpm_delay );
  if ( rcode == 1 ) {
    fpm_read_params( &fprint, &fprint_params );
    snprintf( prompt_buf, 256, "Found fingerprint sensor!\r\n" );
    uart_tx_str( USART2, ( uint8_t* )prompt_buf, strlen( prompt_buf ) );
    snprintf( prompt_buf, 256, "Capacity: %d\r\n", fprint_params.capacity );
    uart_tx_str( USART2, ( uint8_t* )prompt_buf, strlen( prompt_buf ) );
    snprintf( prompt_buf, 256, "Packet length: %d\r\n", fpm_packet_lengths[ fprint_params.packet_len ] );
    uart_tx_str( USART2, ( uint8_t* )prompt_buf, strlen( prompt_buf ) );
    test_fingerprint_enroll();
  }
  else {
    snprintf( prompt_buf, 256, "Could not find a fingerprint sensor :(\r\nCode: %d\r\n", rcode );
    uart_tx_str( USART2, ( uint8_t* )prompt_buf, strlen( prompt_buf ) );
  }

  // Blink LED and print how many ms have elapsed since boot as a test.
  // Main loop.
  while (1) {
    delay_cycles(2000000);
    Po_LED->ODR ^=  (1 << Pi_LED);
    //snprintf( prompt_buf, 256, "Ticks: %ld\r\n", millis );
    //uart_tx_str( USART2, ( uint8_t* )prompt_buf, strlen( prompt_buf ) );
  }
}

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
