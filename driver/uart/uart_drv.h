#ifndef _UART_DRV_H_
#define _UART_DRV_H_

#include "STC12C5A60S2.h"
#include "compiler.h"
#include "config.h"
#include <string.h>
#include <intrins.h>

void uart_init(void);
void uart1_start_trans(void);
void uart2_start_trans(void);

#endif