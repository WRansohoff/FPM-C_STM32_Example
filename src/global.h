#ifndef _VVC_GLOBAL_H
#define _VVC_GLOBAL_H

#include <errno.h>
#include <stdint.h>
#include <string.h>

#ifdef VVC_F0
  #include "stm32f0xx.h"
  #include "stm32f0xx_hal_conf.h"
#elif VVC_F1
  #include "stm32f1xx.h"
  #include "stm32f1xx_hal_conf.h"
#elif  VVC_L0
  #include "stm32l0xx.h"
  #include "stm32l0xx_hal_conf.h"
#elif  VVC_L4
  #include "stm32l4xx.h"
  #include "stm32l4xx_hal_conf.h"
#endif

// Global defines.
#ifdef VVC_F1
  #define Po_LED   (GPIOC)
  #define Pi_LED   (13)
#else
  #define Po_LED   (GPIOB)
  #define Pi_LED   (3)
  #define Po_U1RX  (GPIOA)
  #define Pi_U1RX  (9)
  #define AF_U1RX  (7)
  #define Po_U1TX  (GPIOA)
  #define Pi_U1TX  (10)
  #define AF_U1TX  (7)
  #define Po_U2RX  (GPIOA)
  #define Pi_U2RX  (2)
  #define AF_U2RX  (7)
  #define Po_U2TX  (GPIOA)
  #define Pi_U2TX  (15)
  #define AF_U2TX  (7)
#endif

// Global variables.
extern uint32_t SystemCoreClock;
extern volatile uint32_t millis;

#endif
