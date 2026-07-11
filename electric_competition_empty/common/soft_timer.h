#ifndef COMMON_SOFT_TIMER_H
#define COMMON_SOFT_TIMER_H

#include "common/types.h"

typedef struct {
    uint32_t period_ticks;
    uint32_t next_deadline;
    bool enabled;
} soft_timer_t;

void soft_timer_init(soft_timer_t *timer, uint32_t period_ticks, uint32_t start_tick);
void soft_timer_start(soft_timer_t *timer, uint32_t start_tick);
void soft_timer_stop(soft_timer_t *timer);
bool soft_timer_is_expired(soft_timer_t *timer, uint32_t now_tick);

#endif
