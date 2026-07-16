#include "app/app_ui.h"

#include "drivers/drv_oled_ssd1306.h"

static oled_handle_t g_oled;
static bool g_oled_ready;

void app_ui_init(void)
{
    g_oled_ready = (oled_init(&g_oled, 0x3CU) == STATUS_OK);
}

void app_ui_refresh(const chassis_snapshot_t *chassis,
    const turret_snapshot_t *turret,
    const imu_snapshot_t *imu)
{
    if (!g_oled_ready) {
        return;
    }

    /* Redraw the whole page each refresh to keep the UI path simple and deterministic. */
    oled_clear(&g_oled);
    oled_printf(&g_oled, 0U, 0U, "L%.1f R%.1f", chassis->left_speed_rps, chassis->right_speed_rps);
    oled_printf(&g_oled, 0U, 8U, "E%.2f B%02X", chassis->line_error, chassis->line_bits);
    oled_printf(&g_oled, 0U, 16U, "T%u X%.1f Y%.1f",
        turret->target_valid ? 1U : 0U,
        turret->x_error_cm,
        turret->y_error_cm);
    oled_printf(&g_oled, 0U, 24U, "RP %.1f %.1f", imu->roll_deg, imu->pitch_deg);
    oled_printf(&g_oled, 0U, 32U, "Y %.1f WAI%02X", imu->yaw_deg, imu->who_am_i);
    (void) oled_flush(&g_oled);
}
