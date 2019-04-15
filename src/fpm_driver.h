#ifndef _VVC_FPM_DRIVER_H
#define _VVC_FPM_DRIVER_H

#include "global.h"
#include "fpm.h"
#include "uart.h"

// FPM-C driver methods.
// Core FPM methods.
uint32_t fpm_millis( void );
void fpm_delay( uint32_t ms_delay );
uint16_t fpm_uart_read( uint8_t *bytes, uint16_t len );
void fpm_uart_write( uint8_t *bytes, uint16_t len );
uint16_t fpm_uart_avail( void );
// Helper/test methods.
int get_free_fprint_id( void );
void test_fingerprint_enroll( void );

#endif
