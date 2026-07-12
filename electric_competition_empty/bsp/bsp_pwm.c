#include "bsp/bsp_pwm.h"

#include "common/math_util.h"
#include "ti_msp_dl_config.h"

#define MOTOR_PWM_PERIOD_COUNTS    (3200U)
#define STEP_PWM_MIN_COUNTS        (64U)
#define TIMER_CLOCK_HZ             (32000000.0f)

static float g_step_frequency_hz[2];

static void bsp_pwm_set_motor_compare(GPTIMER_Regs *inst, DL_TIMER_CC_INDEX cc_index, uint32_t compare)
{
    DL_TimerG_setCaptureCompareValue(inst, compare, cc_index);
}

static void bsp_pwm_set_step_counts_yaw(uint32_t period_counts)
{
    /* STEP 信号用 50% 占空比，便于外部步进驱动器稳定识别脉冲。 */
    DL_TimerA_setLoadValue(PWM_STEP_YAW_INST, period_counts - 1U);
    DL_TimerA_setCaptureCompareValue(PWM_STEP_YAW_INST, period_counts / 2U, DL_TIMER_CC_0_INDEX);
}

static void bsp_pwm_set_step_counts_pitch(uint32_t period_counts)
{
    DL_TimerG_setLoadValue(PWM_STEP_PITCH_INST, period_counts - 1U);
    DL_TimerG_setCaptureCompareValue(PWM_STEP_PITCH_INST, period_counts / 2U, DL_TIMER_CC_0_INDEX);
}

void bsp_pwm_init(void)
{
    g_step_frequency_hz[BSP_STEPPER_AXIS_YAW] = 0.0f;
    g_step_frequency_hz[BSP_STEPPER_AXIS_PITCH] = 0.0f;
    bsp_pwm_set_motor_bridge(BSP_MOTOR_PWM_LEFT, 0.0f, 0.0f);
    bsp_pwm_set_motor_bridge(BSP_MOTOR_PWM_RIGHT, 0.0f, 0.0f);
    bsp_pwm_stop_step(BSP_STEPPER_AXIS_YAW);
    bsp_pwm_stop_step(BSP_STEPPER_AXIS_PITCH);
}

void bsp_pwm_start_all(void)
{
    DL_TimerG_startCounter(PWM_MOTOR_LEFT_INST);
    DL_TimerG_startCounter(PWM_MOTOR_RIGHT_INST);
}

void bsp_pwm_set_motor_bridge(bsp_motor_pwm_t motor, float in1_duty, float in2_duty)
{
    float duty1 = math_clampf(in1_duty, 0.0f, 1.0f);
    float duty2 = math_clampf(in2_duty, 0.0f, 1.0f);
    uint32_t compare_in1 = (uint32_t) ((1.0f - duty1) * (float) MOTOR_PWM_PERIOD_COUNTS);
    uint32_t compare_in2 = (uint32_t) ((1.0f - duty2) * (float) MOTOR_PWM_PERIOD_COUNTS);

    switch (motor) {
        case BSP_MOTOR_PWM_LEFT:
            bsp_pwm_set_motor_compare(PWM_MOTOR_LEFT_INST, DL_TIMER_CC_0_INDEX, compare_in1);
            bsp_pwm_set_motor_compare(PWM_MOTOR_LEFT_INST, DL_TIMER_CC_1_INDEX, compare_in2);
            break;
        case BSP_MOTOR_PWM_RIGHT:
            bsp_pwm_set_motor_compare(PWM_MOTOR_RIGHT_INST, DL_TIMER_CC_0_INDEX, compare_in1);
            bsp_pwm_set_motor_compare(PWM_MOTOR_RIGHT_INST, DL_TIMER_CC_1_INDEX, compare_in2);
            break;
        default:
            break;
    }
}

void bsp_pwm_set_motor_duty(bsp_motor_pwm_t motor, float duty)
{
    bsp_pwm_set_motor_bridge(motor, duty, 0.0f);
}

void bsp_pwm_set_step_frequency(bsp_stepper_axis_t axis, float frequency_hz)
{
    uint32_t period_counts;

    if (frequency_hz <= 0.0f) {
        bsp_pwm_stop_step(axis);
        return;
    }

    /* 通过改 period 直接改 STEP 频率，方向交给 GPIO 管。 */
    period_counts = (uint32_t) (TIMER_CLOCK_HZ / frequency_hz);
    if (period_counts < STEP_PWM_MIN_COUNTS) {
        period_counts = STEP_PWM_MIN_COUNTS;
    }

    switch (axis) {
        case BSP_STEPPER_AXIS_YAW:
            bsp_pwm_set_step_counts_yaw(period_counts);
            DL_TimerA_startCounter(PWM_STEP_YAW_INST);
            g_step_frequency_hz[axis] = TIMER_CLOCK_HZ / (float) period_counts;
            break;
        case BSP_STEPPER_AXIS_PITCH:
            bsp_pwm_set_step_counts_pitch(period_counts);
            DL_TimerG_startCounter(PWM_STEP_PITCH_INST);
            g_step_frequency_hz[axis] = TIMER_CLOCK_HZ / (float) period_counts;
            break;
        default:
            break;
    }
}

void bsp_pwm_stop_step(bsp_stepper_axis_t axis)
{
    switch (axis) {
        case BSP_STEPPER_AXIS_YAW:
            DL_TimerA_stopCounter(PWM_STEP_YAW_INST);
            break;
        case BSP_STEPPER_AXIS_PITCH:
            DL_TimerG_stopCounter(PWM_STEP_PITCH_INST);
            break;
        default:
            break;
    }

    g_step_frequency_hz[axis] = 0.0f;
}

float bsp_pwm_get_step_frequency(bsp_stepper_axis_t axis)
{
    return g_step_frequency_hz[axis];
}
