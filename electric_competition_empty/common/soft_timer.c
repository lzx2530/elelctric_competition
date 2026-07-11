#include "common/soft_timer.h"

void soft_timer_init(soft_timer_t *timer, uint32_t period_ticks, uint32_t start_tick)
{
    timer->period_ticks = period_ticks;
    timer->next_deadline = start_tick + period_ticks;
    timer->enabled = true;
}

void soft_timer_start(soft_timer_t *timer, uint32_t start_tick)
{
    timer->next_deadline = start_tick + timer->period_ticks;
    timer->enabled = true;
}

void soft_timer_stop(soft_timer_t *timer)
{
    timer->enabled = false;
}

bool soft_timer_is_expired(soft_timer_t *timer, uint32_t now_tick)
{
    if ((!timer->enabled) || (timer->period_ticks == 0U)) {
        return false;
    }

    /* Signed subtraction keeps periodic timers robust across tick counter wrap-around. */
    if ((int32_t) (now_tick - timer->next_deadline) >= 0) {
        timer->next_deadline += timer->period_ticks;
        return true;
    }

    return false;
}
