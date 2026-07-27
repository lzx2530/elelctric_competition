#ifndef BSP_UART_H
#define BSP_UART_H

#include "common/ringbuf.h"

void bsp_uart_init(void);
/* 当前只打开 K230 RX 中断，调试串口保持阻塞发送 */
void bsp_uart_enable_irqs(void);
void bsp_uart_debug_write_byte(uint8_t byte);
void bsp_uart_debug_write(const uint8_t *data, uint16_t length);
void bsp_uart_debug_write_str(const char *str);
void bsp_uart_debug_printf(const char *fmt, ...);
uint16_t bsp_uart_k230_write(const uint8_t *data, uint16_t length);
void bsp_uart_k230_poll_rx(void);
uint16_t bsp_uart_esp_write(const uint8_t *data, uint16_t length);
void bsp_uart_esp_poll_rx(void);
ringbuf_t *bsp_uart_get_debug_ringbuf(void);
/* 协议层从这里取接收环形缓冲区 */
ringbuf_t *bsp_uart_get_k230_ringbuf(void);
ringbuf_t *bsp_uart_get_esp_ringbuf(void);
void bsp_uart_debug_irq_handler(void);
void bsp_uart_k230_irq_handler(void);
void bsp_uart_esp_irq_handler(void);

#endif
