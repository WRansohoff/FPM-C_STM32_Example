#include "main.h"

// Core value initializations.
volatile uint32_t millis = 0;

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
