#ifndef APP_CONTROL_SCHEDULER_H
#define APP_CONTROL_SCHEDULER_H

#include "common/types.h"

typedef struct {
    uint32_t tick_ms;
    bool control_1khz;
    bool line_5ms;
    bool imu_10ms;
    bool oled_50ms;
    bool debug_100ms;
} scheduler_flags_t;

void app_control_scheduler_init(void);
/* 在 1 kHz 定时器中断中调用，只做轻量置位 */
void app_control_scheduler_on_tick_isr(void);
/* 主循环读取并清空本周期事件标志 */
void app_control_scheduler_fetch(scheduler_flags_t *flags);

#endif
