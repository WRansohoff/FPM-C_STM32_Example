#include "main.h"

// Core value initializations.
volatile uint32_t millis = 0;

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

  // Set UART pins and peripherals.
  uarts_init();

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
  }
  else {
    snprintf( prompt_buf, 256, "Could not find a fingerprint sensor :(\r\nCode: %d\r\n", rcode );
    uart_tx_str( USART2, ( uint8_t* )prompt_buf, strlen( prompt_buf ) );
  }

  // Main loop.
  while (1) {
    // Blink the LED
    fpm_delay( 500 );
    Po_LED->ODR ^=  ( 1 << Pi_LED );
    // TODO: Interactive prompt.
    // Just loop the 'test fingerprint enrollment' method for now.
    test_fingerprint_enroll();
  }
}
