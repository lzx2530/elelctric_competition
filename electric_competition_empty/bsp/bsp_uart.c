#include "bsp/bsp_uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "ti_msp_dl_config.h"

#define K230_RINGBUF_SIZE    (256U)
#define ESP_RINGBUF_SIZE     (256U)
#define DEBUG_RX_RINGBUF_SIZE (128U)
#define DEBUG_PRINTF_BUFFER  (192U)
#define UART_K230_TX_WAIT_LIMIT (100000U)
#define UART_ESP_TX_WAIT_LIMIT (100000U)
#define UART_RX_ERROR_INTERRUPTS (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR | \
    DL_UART_MAIN_INTERRUPT_BREAK_ERROR | DL_UART_MAIN_INTERRUPT_PARITY_ERROR | \
    DL_UART_MAIN_INTERRUPT_FRAMING_ERROR | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR | \
    DL_UART_MAIN_INTERRUPT_NOISE_ERROR)

static uint8_t g_debug_rx_ring_storage[DEBUG_RX_RINGBUF_SIZE];
static ringbuf_t g_debug_rx_ringbuf;
static uint8_t g_k230_ring_storage[K230_RINGBUF_SIZE];
static ringbuf_t g_k230_ringbuf;
static uint8_t g_esp_ring_storage[ESP_RINGBUF_SIZE];
static ringbuf_t g_esp_ringbuf;

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
    ringbuf_init(&g_esp_ringbuf, g_esp_ring_storage, ESP_RINGBUF_SIZE);
}

void bsp_uart_enable_irqs(void)
{
    DL_UART_Main_enableInterrupt(UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_RX | UART_RX_ERROR_INTERRUPTS);
    NVIC_ClearPendingIRQ(UART_DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);

    DL_UART_Main_enableInterrupt(UART_K230_INST, DL_UART_MAIN_INTERRUPT_RX | UART_RX_ERROR_INTERRUPTS);

    NVIC_ClearPendingIRQ(UART_K230_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);

    DL_UART_Main_enableInterrupt(UART_ESP_INST, DL_UART_MAIN_INTERRUPT_RX | UART_RX_ERROR_INTERRUPTS);
    NVIC_ClearPendingIRQ(UART_ESP_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_ESP_INST_INT_IRQN);
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

uint16_t bsp_uart_k230_write(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    uint32_t wait_count;

    if (data == NULL) {
        return 0U;
    }
    for (index = 0U; index < length; ++index) {
        wait_count = UART_K230_TX_WAIT_LIMIT;
        while (!DL_UART_Main_transmitDataCheck(UART_K230_INST, data[index])) {
            if (--wait_count == 0U) {
                return index;
            }
        }
    }
    return length;
}

void bsp_uart_k230_poll_rx(void)
{
    /* Covers a missed RX IRQ without changing the interrupt-driven fast path. */
    bsp_uart_service_rx_fifo(UART_K230_INST, &g_k230_ringbuf);
}

uint16_t bsp_uart_esp_write(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    uint32_t wait_count;

    if (data == NULL) {
        return 0U;
    }
    for (index = 0U; index < length; ++index) {
        wait_count = UART_ESP_TX_WAIT_LIMIT;
        while (!DL_UART_Main_transmitDataCheck(UART_ESP_INST, data[index])) {
            if (--wait_count == 0U) {
                return index;
            }
        }
    }
    return length;
}

void bsp_uart_esp_poll_rx(void)
{
    bsp_uart_service_rx_fifo(UART_ESP_INST, &g_esp_ringbuf);
}

ringbuf_t *bsp_uart_get_k230_ringbuf(void)
{
    return &g_k230_ringbuf;
}

ringbuf_t *bsp_uart_get_esp_ringbuf(void)
{
    return &g_esp_ringbuf;
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

void bsp_uart_esp_irq_handler(void)
{
    uint32_t iidx;

    while (1) {
        iidx = (uint32_t) DL_UART_Main_getPendingInterrupt(UART_ESP_INST);
        if (iidx == (uint32_t) DL_UART_MAIN_IIDX_NO_INTERRUPT) {
            break;
        }

        if ((iidx == (uint32_t) DL_UART_MAIN_IIDX_RX) ||
            (iidx == (uint32_t) DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR)) {
            if (iidx == (uint32_t) DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR) {
                bsp_uart_clear_error_interrupt(UART_ESP_INST, iidx);
            }
            bsp_uart_service_rx_fifo(UART_ESP_INST, &g_esp_ringbuf);
            continue;
        }

        bsp_uart_clear_error_interrupt(UART_ESP_INST, iidx);
        bsp_uart_service_rx_fifo(UART_ESP_INST, &g_esp_ringbuf);
    }
}
