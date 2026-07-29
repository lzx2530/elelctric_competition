#include "app/app_mission.h"

#include "app/app_ball_control.h"
#include "app/app_chassis.h"

#define APP_MISSION_AB_DISTANCE_MM             (1500.0F)
#define APP_MISSION_LOOP_GATE_MM               (5400.0F)
#define APP_MISSION_LOOP_TIMEOUT_DISTANCE_MM   (6600.0F)
#define APP_MISSION_STATIC_LIMIT_MS            (5000U)
#define APP_MISSION_LINE_LIMIT_MS              (20000U)
#define APP_MISSION_BALANCE_LIMIT_MS           (30000U)

typedef struct {
    mission_snapshot_t snapshot;
    uint32_t start_tick_ms;
    uint8_t static_phase;
    uint8_t settled_samples;
} mission_app_t;

static mission_app_t g_mission;

static bool mission_requires_balance(app_mission_mode_t mode)
{
    return mode >= APP_MISSION_STATIC_SWEEP;
}

static void mission_stop_outputs(void)
{
    app_chassis_set_enabled(false);
    app_ball_control_set_enabled(false);
}

static void mission_enter_fault(uint8_t fault_code)
{
    mission_stop_outputs();
    g_mission.snapshot.state = APP_MISSION_FAULT;
    g_mission.snapshot.fault_code = fault_code;
}

static void mission_begin_running(uint32_t tick_ms)
{
    app_mission_mode_t mode = g_mission.snapshot.mode;

    g_mission.start_tick_ms = tick_ms;
    g_mission.snapshot.elapsed_ms = 0U;
    g_mission.snapshot.fault_code = 0U;
    g_mission.static_phase = 0U;
    g_mission.settled_samples = 0U;
    app_chassis_reset_travel();

    if (mode == APP_MISSION_VIDEO_RECORD) {
        mission_stop_outputs();
    } else if (mode == APP_MISSION_LINE_LOOP) {
        app_ball_control_set_enabled(false);
        app_chassis_set_cruise_speed_mps(0.40F);
        app_chassis_set_enabled(true);
    } else {
        g_mission.snapshot.target_mm = 0;
        app_ball_control_set_target_mm(g_mission.snapshot.target_mm);
        app_ball_control_set_enabled(true);
        if (mode != APP_MISSION_STATIC_SWEEP) {
            app_chassis_set_cruise_speed_mps(0.25F);
            app_chassis_set_enabled(true);
        }
    }
    g_mission.snapshot.state = APP_MISSION_RUNNING;
}

void app_mission_init(void)
{
    g_mission.snapshot.mode = APP_MISSION_VIDEO_RECORD;
    g_mission.snapshot.state = APP_MISSION_IDLE;
    mission_stop_outputs();
}

void app_mission_next_mode(void)
{
    if (g_mission.snapshot.state == APP_MISSION_RUNNING ||
        g_mission.snapshot.state == APP_MISSION_ARMING) {
        return;
    }
    g_mission.snapshot.mode = g_mission.snapshot.mode == APP_MISSION_LOOP_HOLD ?
        APP_MISSION_VIDEO_RECORD : (app_mission_mode_t) (g_mission.snapshot.mode + 1U);
    g_mission.snapshot.state = APP_MISSION_IDLE;
    g_mission.snapshot.fault_code = 0U;
}

void app_mission_start(uint32_t tick_ms)
{
    if (g_mission.snapshot.state == APP_MISSION_RUNNING ||
        g_mission.snapshot.state == APP_MISSION_ARMING) {
        return;
    }
    g_mission.start_tick_ms = tick_ms;
    g_mission.snapshot.state = APP_MISSION_ARMING;
    g_mission.snapshot.fault_code = 0U;
}

void app_mission_start_from_k230(uint8_t task_flag, uint32_t tick_ms)
{
    if ((task_flag < APP_MISSION_VIDEO_RECORD) || (task_flag > APP_MISSION_LOOP_HOLD)) {
        return;
    }

    mission_stop_outputs();
    app_ball_control_reset_vision();
    g_mission.snapshot.mode = (app_mission_mode_t) task_flag;
    g_mission.snapshot.state = APP_MISSION_ARMING;
    g_mission.snapshot.elapsed_ms = 0U;
    g_mission.snapshot.target_mm = 0;
    g_mission.snapshot.fault_code = 0U;
    g_mission.start_tick_ms = tick_ms;
}

void app_mission_abort(void)
{
    mission_stop_outputs();
    g_mission.snapshot.state = APP_MISSION_IDLE;
    g_mission.snapshot.elapsed_ms = 0U;
    g_mission.snapshot.fault_code = 0U;
}

void app_mission_task(const imu_snapshot_t *imu, uint32_t tick_ms)
{
    const ball_control_snapshot_t *ball = app_ball_control_get_snapshot();
    const chassis_snapshot_t *chassis = app_chassis_get_snapshot();
    if (g_mission.snapshot.state == APP_MISSION_ARMING) {
        if (!mission_requires_balance(g_mission.snapshot.mode) ||
            (ball->vision_valid && ball->vision_stable && ball->actuator_feedback_valid &&
                (g_mission.snapshot.mode != APP_MISSION_LOOP_HOLD || ball->reference_ready))) {
            mission_begin_running(tick_ms);
        }
        return;
    }
    if (g_mission.snapshot.state != APP_MISSION_RUNNING) {
        return;
    }

    g_mission.snapshot.elapsed_ms = tick_ms - g_mission.start_tick_ms;
    if (g_mission.snapshot.mode == APP_MISSION_VIDEO_RECORD) {
        return;
    }
    if (mission_requires_balance(g_mission.snapshot.mode)) {
        app_ball_control_outer_task(imu, tick_ms);
        if (app_ball_control_get_snapshot()->fault != APP_BALL_FAULT_NONE) {
            mission_enter_fault((uint8_t) (10U + app_ball_control_get_snapshot()->fault));
            return;
        }
    }

    if (g_mission.snapshot.mode == APP_MISSION_STATIC_SWEEP) {
        if (ball->stop_requested) {
            mission_stop_outputs();
            g_mission.snapshot.state = APP_MISSION_FINISHED;
        } else if (g_mission.snapshot.elapsed_ms > APP_MISSION_STATIC_LIMIT_MS) {
            mission_enter_fault(1U);
        }
        return;
    }

    if (g_mission.snapshot.mode == APP_MISSION_AB_CENTER &&
        chassis->travel_mm >= APP_MISSION_AB_DISTANCE_MM) {
        mission_stop_outputs();
        g_mission.snapshot.state = APP_MISSION_FINISHED;
        return;
    }

    if (g_mission.snapshot.mode != APP_MISSION_AB_CENTER &&
        chassis->travel_mm >= APP_MISSION_LOOP_GATE_MM && chassis->start_line_detected) {
        mission_stop_outputs();
        g_mission.snapshot.state = APP_MISSION_FINISHED;
        return;
    }

    if (chassis->travel_mm > APP_MISSION_LOOP_TIMEOUT_DISTANCE_MM ||
        g_mission.snapshot.elapsed_ms > (mission_requires_balance(g_mission.snapshot.mode) ?
            APP_MISSION_BALANCE_LIMIT_MS : APP_MISSION_LINE_LIMIT_MS)) {
        mission_enter_fault(2U);
    }
}

const mission_snapshot_t *app_mission_get_snapshot(void)
{
    return &g_mission.snapshot;
}
