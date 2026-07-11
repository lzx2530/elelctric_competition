#include "bsp/bsp_uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "ti_msp_dl_config.h"

#define K230_RINGBUF_SIZE    (256U)
#define DEBUG_RX_RINGBUF_SIZE (128U)
#define DEBUG_PRINTF_BUFFER  (192U)

static uint8_t g_debug_rx_ring_storage[DEBUG_RX_RINGBUF_SIZE];
static ringbuf_t g_debug_rx_ringbuf;
static uint8_t g_k230_ring_storage[K230_RINGBUF_SIZE];
static ringbuf_t g_k230_ringbuf;

void bsp_uart_init(void)
{
    ringbuf_init(&g_debug_rx_ringbuf, g_debug_rx_ring_storage, DEBUG_RX_RINGBUF_SIZE);
    ringbuf_init(&g_k230_ringbuf, g_k230_ring_storage, K230_RINGBUF_SIZE);
}

void bsp_uart_enable_irqs(void)
{
    DL_UART_Main_enableInterrupt(UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);

    NVIC_ClearPendingIRQ(UART_K230_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);
}

void bsp_uart_debug_write_byte(uint8_t byte)
{
    DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, byte);
}

void bsp_uart_debug_write(const uint8_t *data, uint16_t length)
{
    uint16_t i;
    for (i = 0U; i < length; i++) {
        bsp_uart_debug_write_byte(data[i]);
    }
}

void bsp_uart_debug_write_str(const char *str)
{
    while (*str != '\0') {
        bsp_uart_debug_write_byte((uint8_t) *str);
        str++;
    }
}

void bsp_uart_debug_printf(const char *fmt, ...)
{
    char buffer[DEBUG_PRINTF_BUFFER];
    va_list args;
    int length;

    va_start(args, fmt);
    length = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (length <= 0) {
        return;
    }

    /* Clamp to the local stack buffer to keep debug output side effects bounded. */
    if (length > (int) sizeof(buffer)) {
        length = (int) sizeof(buffer);
    }

    bsp_uart_debug_write((const uint8_t *) buffer, (uint16_t) length);
}

ringbuf_t *bsp_uart_get_k230_ringbuf(void)
{
    return &g_k230_ringbuf;
}

ringbuf_t *bsp_uart_get_debug_ringbuf(void)
{
    return &g_debug_rx_ringbuf;
}

void bsp_uart_debug_irq_handler(void)
{
    uint32_t iidx;

    iidx = (uint32_t) DL_UART_Main_getPendingInterrupt(UART_DEBUG_INST);
    if (iidx != (uint32_t) DL_UART_MAIN_IIDX_RX) {
        return;
    }

    while (!DL_UART_Main_isRXFIFOEmpty(UART_DEBUG_INST)) {
        (void) ringbuf_push_byte(&g_debug_rx_ringbuf, DL_UART_Main_receiveData(UART_DEBUG_INST));
    }
}

void bsp_uart_k230_irq_handler(void)
{
    uint32_t iidx;

    iidx = (uint32_t) DL_UART_Main_getPendingInterrupt(UART_K230_INST);
    if (iidx != (uint32_t) DL_UART_MAIN_IIDX_RX) {
        return;
    }

    /* IRQ only stages bytes into the ring buffer; frame parsing stays in the main loop. */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_K230_INST)) {
        (void) ringbuf_push_byte(&g_k230_ringbuf, DL_UART_Main_receiveData(UART_K230_INST));
    }
}
