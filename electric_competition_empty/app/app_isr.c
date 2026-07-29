#include "app/app_isr.h"
#include "app/app_control_scheduler.h"
#include "bsp/bsp_operator_input.h"
#include "bsp/bsp_uart.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

static encoder_driver_t *g_encoder_driver;

void app_isr_set_encoder_driver(encoder_driver_t *driver)
{
    g_encoder_driver = driver;
}

void GROUP1_IRQHandler(void)
{
    encoder_driver_t *driver = g_encoder_driver;

    /* GROUP1 里复用了 GPIOA/GPIOB 中断，需要先分组再读具体引脚 IIDX。 */
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
            while (1) {
                DL_GPIO_IIDX iidx = DL_GPIO_getPendingInterrupt(GPIOA);
                if (iidx == DL_GPIO_IIDX_NO_INTR) {
                    break;
                }
                if (driver != NULL) {
                    encoder_driver_handle_gpio_interrupt(driver, GPIOA, (uint32_t) iidx);
                }
            }
            break;
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
            while (1) {
                DL_GPIO_IIDX iidx = DL_GPIO_getPendingInterrupt(GPIOB);
                if (iidx == DL_GPIO_IIDX_NO_INTR) {
                    break;
                }
                bsp_operator_input_handle_gpio_interrupt((uint32_t) iidx);
                if (driver != NULL) {
                    encoder_driver_handle_gpio_interrupt(driver, GPIOB, (uint32_t) iidx);
                }
            }
            break;
        default:
            break;
    }
}

void UART2_IRQHandler(void)
{
    bsp_uart_k230_irq_handler();
}

void UART0_IRQHandler(void)
{
    bsp_uart_debug_irq_handler();
}

#if defined(CAPTURE_ABS_PWM_INST)
void TIMA0_IRQHandler(void)
{
    bsp_operator_input_capture_irq_handler();
}
#endif

void TIMA1_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_CTRL_1KHZ_INST) == DL_TIMERA_IIDX_ZERO) {
        /* 1 kHz 定时器中断只做时基推进。 */
        app_control_scheduler_on_tick_isr();
    }
}
