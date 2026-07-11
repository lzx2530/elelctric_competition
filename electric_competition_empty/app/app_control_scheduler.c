#include "app/app_control_scheduler.h"

#include "ti_msp_dl_config.h"

typedef struct {
    volatile uint32_t tick_ms;
    volatile bool control_1khz;
    volatile bool line_5ms;
    volatile bool imu_10ms;
    volatile bool oled_50ms;
    volatile bool debug_100ms;
} scheduler_state_t;

static scheduler_state_t g_scheduler;

void app_control_scheduler_init(void)
{
    g_scheduler.tick_ms = 0U;
    g_scheduler.control_1khz = false;
    g_scheduler.line_5ms = false;
    g_scheduler.imu_10ms = false;
    g_scheduler.oled_50ms = false;
    g_scheduler.debug_100ms = false;

    NVIC_ClearPendingIRQ(TIMER_CTRL_1KHZ_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_CTRL_1KHZ_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_CTRL_1KHZ_INST);
}

void app_control_scheduler_on_tick_isr(void)
{
    g_scheduler.tick_ms++;
    g_scheduler.control_1khz = true;

    /* 用最小代价做软件分频，避免在中断里跑业务逻辑。 */
    if ((g_scheduler.tick_ms % 5U) == 0U) {
        g_scheduler.line_5ms = true;
    }
    if ((g_scheduler.tick_ms % 10U) == 0U) {
        g_scheduler.imu_10ms = true;
    }
    if ((g_scheduler.tick_ms % 50U) == 0U) {
        g_scheduler.oled_50ms = true;
    }
    if ((g_scheduler.tick_ms % 100U) == 0U) {
        g_scheduler.debug_100ms = true;
    }
}

void app_control_scheduler_fetch(scheduler_flags_t *flags)
{
    /* 主循环读取和清零标志时关中断，避免和 1 kHz ISR 竞争。 */
    __disable_irq();
    flags->tick_ms = g_scheduler.tick_ms;
    flags->control_1khz = g_scheduler.control_1khz;
    flags->line_5ms = g_scheduler.line_5ms;
    flags->imu_10ms = g_scheduler.imu_10ms;
    flags->oled_50ms = g_scheduler.oled_50ms;
    flags->debug_100ms = g_scheduler.debug_100ms;

    g_scheduler.control_1khz = false;
    g_scheduler.line_5ms = false;
    g_scheduler.imu_10ms = false;
    g_scheduler.oled_50ms = false;
    g_scheduler.debug_100ms = false;
    __enable_irq();
}
