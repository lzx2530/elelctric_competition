#include "bsp/bsp_uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "ti_msp_dl_config.h"

#define K230_RINGBUF_SIZE    (256U)
#define DEBUG_RX_RINGBUF_SIZE (128U)
#define DEBUG_PRINTF_BUFFER  (192U)
#define UART_RX_ERROR_INTERRUPTS (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR | \
    DL_UART_MAIN_INTERRUPT_BREAK_ERROR | DL_UART_MAIN_INTERRUPT_PARITY_ERROR | \
    DL_UART_MAIN_INTERRUPT_FRAMING_ERROR | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR | \
    DL_UART_MAIN_INTERRUPT_NOISE_ERROR)

static uint8_t g_debug_rx_ring_storage[DEBUG_RX_RINGBUF_SIZE];
static ringbuf_t g_debug_rx_ringbuf;
static uint8_t g_k230_ring_storage[K230_RINGBUF_SIZE];
static ringbuf_t g_k230_ringbuf;

static void bsp_uart_service_rx_fifo(UART_Regs *uart, ringbuf_t *ringbuf)
{
    while (!DL_UART_Main_isRXFIFOEmpty(uart)) {
        (void) ringbuf_push_byte(ringbuf, DL_UART_Main_receiveData(uart));
    }
}

static void bsp_uart_clear_error_interrupt(UART_Regs *uart, uint32_t iidx)
{
    uint32_t clear_mask = 0U;

    switch (iidx) {
        case (uint32_t) DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            clear_mask = DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR;
            break;
        case (uint32_t) DL_UART_MAIN_IIDX_BREAK_ERROR:
            clear_mask = DL_UART_MAIN_INTERRUPT_BREAK_ERROR;
            break;
        case (uint32_t) DL_UART_MAIN_IIDX_PARITY_ERROR:
            clear_mask = DL_UART_MAIN_INTERRUPT_PARITY_ERROR;
            break;
        case (uint32_t) DL_UART_MAIN_IIDX_FRAMING_ERROR:
            clear_mask = DL_UART_MAIN_INTERRUPT_FRAMING_ERROR;
            break;
        case (uint32_t) DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
            clear_mask = DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR;
            break;
        case (uint32_t) DL_UART_MAIN_IIDX_NOISE_ERROR:
            clear_mask = DL_UART_MAIN_INTERRUPT_NOISE_ERROR;
            break;
        default:
            break;
    }

    if (clear_mask != 0U) {
        DL_UART_Main_clearInterruptStatus(uart, clear_mask);
    }
}

void bsp_uart_init(void)
{
    ringbuf_init(&g_debug_rx_ringbuf, g_debug_rx_ring_storage, DEBUG_RX_RINGBUF_SIZE);
    ringbuf_init(&g_k230_ringbuf, g_k230_ring_storage, K230_RINGBUF_SIZE);
}

void bsp_uart_enable_irqs(void)
{
    DL_UART_Main_enableInterrupt(UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_RX | UART_RX_ERROR_INTERRUPTS);
    NVIC_ClearPendingIRQ(UART_DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);

    DL_UART_Main_enableInterrupt(UART_K230_INST, UART_RX_ERROR_INTERRUPTS);

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

    while (1) {
        iidx = (uint32_t) DL_UART_Main_getPendingInterrupt(UART_DEBUG_INST);
        if (iidx == (uint32_t) DL_UART_MAIN_IIDX_NO_INTERRUPT) {
            break;
        }

        if ((iidx == (uint32_t) DL_UART_MAIN_IIDX_RX) ||
            (iidx == (uint32_t) DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR)) {
            if (iidx == (uint32_t) DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR) {
                bsp_uart_clear_error_interrupt(UART_DEBUG_INST, iidx);
            }
            bsp_uart_service_rx_fifo(UART_DEBUG_INST, &g_debug_rx_ringbuf);
            continue;
        }

        bsp_uart_clear_error_interrupt(UART_DEBUG_INST, iidx);
        bsp_uart_service_rx_fifo(UART_DEBUG_INST, &g_debug_rx_ringbuf);
    }
}

void bsp_uart_k230_irq_handler(void)
{
    uint32_t iidx;

    while (1) {
        iidx = (uint32_t) DL_UART_Main_getPendingInterrupt(UART_K230_INST);
        if (iidx == (uint32_t) DL_UART_MAIN_IIDX_NO_INTERRUPT) {
            break;
        }

        if ((iidx == (uint32_t) DL_UART_MAIN_IIDX_RX) ||
            (iidx == (uint32_t) DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR)) {
            if (iidx == (uint32_t) DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR) {
                bsp_uart_clear_error_interrupt(UART_K230_INST, iidx);
            }
            /* IRQ only stages bytes into the ring buffer; frame parsing stays in the main loop. */
            bsp_uart_service_rx_fifo(UART_K230_INST, &g_k230_ringbuf);
            continue;
        }

        bsp_uart_clear_error_interrupt(UART_K230_INST, iidx);
        bsp_uart_service_rx_fifo(UART_K230_INST, &g_k230_ringbuf);
    }
}
