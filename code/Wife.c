#include "Wife.h"

void Wife_Init(void)
{
    uart_init(UART_2, 921600, UART2_TX_P10_5, UART2_RX_P10_6);
}