#include "usart.h"
#include <string.h>

#define RX_BUF_SIZE 128
static uint8_t rx_byte;
static char rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_idx = 0;
static volatile int line_ready = 0;

void USART_StartReceive_IT(void)
{
    rx_idx = 0;
    line_ready = 0;
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

int USART_LineAvailable(void)
{
    return line_ready;
}

char *USART_GetLine(void)
{
    if (!line_ready) return NULL;
    line_ready = 0;
    rx_buf[rx_idx] = '\0';
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    return rx_buf;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != huart1.Instance) return;

    if (rx_idx < RX_BUF_SIZE - 1) {
        char c = (char)rx_byte;
        if (c == '\r' || c == '\n') {
            if (rx_idx > 0) {
                line_ready = 1;
            }
        } else {
            rx_buf[rx_idx++] = c;
        }
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}
