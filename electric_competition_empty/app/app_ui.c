#include "app/app_ui.h"

#include "drivers/drv_oled_ssd1306.h"

static oled_handle_t g_oled;
static bool g_oled_ready;

#define APP_UI_BIG_DIGIT_SCALE       (8U)
#define APP_UI_BIG_TIME_Y            (24U)

static const uint8_t g_big_digit_rows[10][5] = {
    {0x07U, 0x05U, 0x05U, 0x05U, 0x07U},
    {0x02U, 0x06U, 0x02U, 0x02U, 0x07U},
    {0x07U, 0x01U, 0x07U, 0x04U, 0x07U},
    {0x07U, 0x01U, 0x07U, 0x01U, 0x07U},
    {0x05U, 0x05U, 0x07U, 0x01U, 0x01U},
    {0x07U, 0x04U, 0x07U, 0x01U, 0x07U},
    {0x07U, 0x04U, 0x07U, 0x05U, 0x07U},
    {0x07U, 0x01U, 0x02U, 0x02U, 0x02U},
    {0x07U, 0x05U, 0x07U, 0x05U, 0x07U},
    {0x07U, 0x05U, 0x07U, 0x01U, 0x07U},
};

static void app_ui_set_pixel(uint8_t x, uint8_t y)
{
    if ((x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT)) {
        return;
    }

    g_oled.buffer[(uint16_t) (y / 8U) * SSD1306_WIDTH + x] |= (uint8_t) (1U << (y % 8U));
}

static void app_ui_draw_big_digit(uint8_t x, uint8_t y, uint8_t digit)
{
    uint8_t row;

    if (digit > 9U) {
        return;
    }

    for (row = 0U; row < 5U; row++) {
        uint8_t column;

        for (column = 0U; column < 3U; column++) {
            uint8_t vertical_offset;

            if ((g_big_digit_rows[digit][row] & (uint8_t) (1U << (2U - column))) == 0U) {
                continue;
            }

            for (vertical_offset = 0U; vertical_offset < APP_UI_BIG_DIGIT_SCALE; vertical_offset++) {
                uint8_t horizontal_offset;

                for (horizontal_offset = 0U; horizontal_offset < APP_UI_BIG_DIGIT_SCALE;
                    horizontal_offset++) {
                    app_ui_set_pixel((uint8_t) (x + column * APP_UI_BIG_DIGIT_SCALE + horizontal_offset),
                        (uint8_t) (y + row * APP_UI_BIG_DIGIT_SCALE + vertical_offset));
                }
            }
        }
    }
}

static void app_ui_draw_big_colon(uint8_t x, uint8_t y)
{
    uint8_t vertical_offset;

    for (vertical_offset = 0U; vertical_offset < APP_UI_BIG_DIGIT_SCALE; vertical_offset++) {
        uint8_t horizontal_offset;

        for (horizontal_offset = 0U; horizontal_offset < APP_UI_BIG_DIGIT_SCALE; horizontal_offset++) {
            app_ui_set_pixel((uint8_t) (x + horizontal_offset),
                (uint8_t) (y + APP_UI_BIG_DIGIT_SCALE + vertical_offset));
            app_ui_set_pixel((uint8_t) (x + horizontal_offset),
                (uint8_t) (y + 3U * APP_UI_BIG_DIGIT_SCALE + vertical_offset));
        }
    }
}

static void app_ui_draw_big_time(uint32_t minutes, uint32_t seconds)
{
    app_ui_draw_big_digit(7U, APP_UI_BIG_TIME_Y, (uint8_t) ((minutes / 10U) % 10U));
    app_ui_draw_big_digit(34U, APP_UI_BIG_TIME_Y, (uint8_t) (minutes % 10U));
    app_ui_draw_big_colon(62U, APP_UI_BIG_TIME_Y);
    app_ui_draw_big_digit(74U, APP_UI_BIG_TIME_Y, (uint8_t) (seconds / 10U));
    app_ui_draw_big_digit(101U, APP_UI_BIG_TIME_Y, (uint8_t) (seconds % 10U));
}

void app_ui_init(void)
{
    g_oled_ready = (oled_init(&g_oled, 0x3CU) == STATUS_OK);
}

void app_ui_process(void)
{
    if (g_oled_ready && oled_flush_async_active(&g_oled)) {
        (void) oled_flush_async_process(&g_oled);
    }
}

void app_ui_refresh(const chassis_snapshot_t *chassis,
    const ball_control_snapshot_t *ball,
    const mission_snapshot_t *mission)
{
    uint32_t tenths;
    uint32_t minutes;
    uint32_t seconds;

    (void) chassis;
    (void) ball;
    if (!g_oled_ready || oled_flush_async_active(&g_oled)) {
        return;
    }

    tenths = mission->elapsed_ms / 100U;
    minutes = tenths / 600U;
    seconds = (tenths / 10U) % 60U;

    oled_clear(&g_oled);
    oled_printf(&g_oled, 46U, 5U, "TASK %u", mission->mode);
    app_ui_draw_big_time(minutes, seconds);
    (void) oled_flush_async_begin(&g_oled);
}
