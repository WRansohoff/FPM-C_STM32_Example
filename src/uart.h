#ifndef _VVC_UART_H
#define _VVC_UART_H

#include "global.h"

// Simple ring buffer.
#define RINGBUF_LEN ( 256 )
typedef struct {
  volatile char buf[ RINGBUF_LEN ];
  volatile int  pos;
  volatile int  ext;
} ring_buf;

ring_buf prompt_rb;
ring_buf fprint_rb;

void uarts_init();
void uart_tx_str( USART_TypeDef *u, const unsigned char *str, int len );

#endif
