#include "drivers/drv_oled_ssd1306.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bsp/bsp_i2c.h"
#include <ti/driverlib/dl_common.h>
#include <ti/driverlib/m0p/dl_core.h>

static const uint8_t g_oled_init_seq[] = {
    0xAE,
    0xD5, 0x80,
    0xA8, 0x3F,
    0xD3, 0x00,
    0x40,
    0xA1,
    0xC8,
    0xDA, 0x12,
    0x81, 0xCF,
    0xD9, 0xF1,
    0xDB, 0x30,
    0xA4,
    0xA6,
    0x8D, 0x14,
    0xAF
};

static const uint8_t g_font_5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x08,0x14,0x54,0x54,0x3C},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},{0x08,0x08,0x2A,0x1C,0x08},{0x08,0x1C,0x2A,0x08,0x08}
};

static status_t g_oled_last_status = STATUS_OK;
static uint8_t g_oled_last_failed_command = 0U;
static uint8_t g_oled_last_failed_index = 0U;

#define OLED_ASYNC_STATE_COMMAND (0U)
#define OLED_ASYNC_STATE_DATA    (1U)
#define OLED_ASYNC_STATE_COMPLETE (2U)
#define OLED_ASYNC_DATA_BYTES    (16U)

static status_t oled_send_command(oled_handle_t *handle, uint8_t command)
{
    uint8_t packet[2] = {0x00, command};
    return bsp_i2c_write_bytes(handle->i2c_addr, packet, 2U);
}

static status_t oled_send_data(oled_handle_t *handle, const uint8_t *data, uint16_t length)
{
    uint8_t packet[2] = {0x40U, 0x00U};

    for (uint16_t offset = 0U; offset < length; offset++) {
        packet[1] = data[offset];
        /* 分块发送，避免一次性临时缓冲过大。 */
        if (bsp_i2c_write_bytes(handle->i2c_addr, packet, sizeof(packet)) != STATUS_OK) {
            return STATUS_ERROR;
        }
    }

    return STATUS_OK;
}

static void oled_draw_glyph(oled_handle_t *handle, char ch)
{
    uint8_t index;
    uint16_t page_offset;

    /* 当前字体是 5x7，按页寻址直接落到本地 framebuffer。 */
    if ((ch < 32) || (ch > 127)) {
        ch = '?';
    }

    index = (uint8_t) (ch - 32);
    page_offset = (uint16_t) ((handle->cursor_y / 8U) * SSD1306_WIDTH + handle->cursor_x);
    if ((page_offset + 6U) >= SSD1306_BUF_SIZE) {
        return;
    }

    for (uint8_t i = 0U; i < 5U; i++) {
        handle->buffer[page_offset + i] = g_font_5x7[index][i];
    }
    handle->buffer[page_offset + 5U] = 0x00;
    handle->cursor_x = (uint8_t) (handle->cursor_x + 6U);
    handle->dirty = true;
}

status_t oled_init(oled_handle_t *handle, uint8_t i2c_addr)
{
    memset(handle, 0, sizeof(*handle));
    handle->i2c_addr = i2c_addr;
    handle->width = SSD1306_WIDTH;
    handle->height = SSD1306_HEIGHT;
    g_oled_last_status = STATUS_OK;
    g_oled_last_failed_command = 0U;
    g_oled_last_failed_index = 0U;

    /* Match the known-good CubeMX bring-up path: give the panel time to settle after power-up. */
    delay_cycles(3200000U);

    for (uint8_t i = 0U; i < sizeof(g_oled_init_seq); i++) {
        status_t ret = oled_send_command(handle, g_oled_init_seq[i]);
        if (ret != STATUS_OK) {
            g_oled_last_status = ret;
            g_oled_last_failed_command = g_oled_init_seq[i];
            g_oled_last_failed_index = i;
            return ret;
        }
    }

    oled_clear(handle);
    g_oled_last_status = oled_flush(handle);
    return g_oled_last_status;
}

void oled_clear(oled_handle_t *handle)
{
    memset(handle->buffer, 0, sizeof(handle->buffer));
    handle->cursor_x = 0U;
    handle->cursor_y = 0U;
    handle->dirty = true;
}

void oled_set_cursor(oled_handle_t *handle, uint8_t x, uint8_t y)
{
    handle->cursor_x = x;
    handle->cursor_y = y;
}

void oled_write_char(oled_handle_t *handle, char ch)
{
    oled_draw_glyph(handle, ch);
}

void oled_write_str(oled_handle_t *handle, const char *str)
{
    while (*str != '\0') {
        oled_write_char(handle, *str++);
    }
}

void oled_printf(oled_handle_t *handle, uint8_t x, uint8_t y, const char *fmt, ...)
{
    char buffer[32];
    va_list args;

    oled_set_cursor(handle, x, y);
    va_start(args, fmt);
    (void) vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    oled_write_str(handle, buffer);
}

status_t oled_flush(oled_handle_t *handle)
{
    return oled_flush_pages(handle, 0U, SSD1306_HEIGHT / 8U);
}

status_t oled_flush_pages(oled_handle_t *handle, uint8_t first_page, uint8_t page_count)
{
    uint8_t end_page;

    if ((handle == NULL) || (first_page >= (SSD1306_HEIGHT / 8U)) ||
        (page_count == 0U)) {
        return STATUS_INVALID_ARG;
    }

    end_page = first_page + page_count;
    if (end_page > (SSD1306_HEIGHT / 8U)) {
        return STATUS_INVALID_ARG;
    }

    /* The timer display changes in only two pages. */
    for (uint8_t page = first_page; page < end_page; page++) {
        if (oled_send_command(handle, (uint8_t) (0xB0U + page)) != STATUS_OK) {
            return STATUS_ERROR;
        }
        if (oled_send_command(handle, 0x00U) != STATUS_OK) {
            return STATUS_ERROR;
        }
        if (oled_send_command(handle, 0x10U) != STATUS_OK) {
            return STATUS_ERROR;
        }
        if (oled_send_data(handle, &handle->buffer[page * SSD1306_WIDTH], SSD1306_WIDTH) != STATUS_OK) {
            return STATUS_ERROR;
        }
    }

    if ((first_page == 0U) && (page_count == (SSD1306_HEIGHT / 8U))) {
        handle->dirty = false;
    }
    return STATUS_OK;
}

status_t oled_flush_async_begin(oled_handle_t *handle)
{
    if (handle == NULL) {
        return STATUS_INVALID_ARG;
    }
    if (handle->async_flush_active || bsp_i2c_async_write_active()) {
        return STATUS_BUSY;
    }
    if (!handle->dirty) {
        return STATUS_OK;
    }

    handle->async_page = 0U;
    handle->async_column = 0U;
    handle->async_state = OLED_ASYNC_STATE_COMMAND;
    handle->async_flush_active = true;
    return STATUS_OK;
}

status_t oled_flush_async_process(oled_handle_t *handle)
{
    status_t status;

    if ((handle == NULL) || !handle->async_flush_active) {
        return STATUS_NOT_READY;
    }

    if (bsp_i2c_async_write_active()) {
        status = bsp_i2c_async_poll_write();
        if (status == STATUS_BUSY) {
            return STATUS_BUSY;
        }
        if (status != STATUS_OK) {
            handle->async_flush_active = false;
            g_oled_last_status = status;
            return status;
        }
    }

    if (handle->async_state == OLED_ASYNC_STATE_COMPLETE) {
        handle->async_flush_active = false;
        handle->dirty = false;
        return STATUS_OK;
    }

    if (handle->async_state == OLED_ASYNC_STATE_COMMAND) {
        handle->async_tx[0] = 0x00U;
        handle->async_tx[1] = (uint8_t) (0xB0U + handle->async_page);
        handle->async_tx[2] = 0x00U;
        handle->async_tx[3] = 0x10U;
        status = bsp_i2c_async_start_write(handle->i2c_addr, handle->async_tx, 4U);
        if (status == STATUS_OK) {
            handle->async_state = OLED_ASYNC_STATE_DATA;
        }
        return status;
    }

    handle->async_tx[0] = 0x40U;
    memcpy(&handle->async_tx[1], &handle->buffer[(uint16_t) handle->async_page * SSD1306_WIDTH +
        handle->async_column], OLED_ASYNC_DATA_BYTES);
    status = bsp_i2c_async_start_write(handle->i2c_addr, handle->async_tx,
        OLED_ASYNC_DATA_BYTES + 1U);
    if (status != STATUS_OK) {
        return status;
    }

    handle->async_column += OLED_ASYNC_DATA_BYTES;
    if (handle->async_column >= SSD1306_WIDTH) {
        handle->async_column = 0U;
        handle->async_page++;
        handle->async_state = OLED_ASYNC_STATE_COMMAND;
        if (handle->async_page >= (SSD1306_HEIGHT / 8U)) {
            handle->async_state = OLED_ASYNC_STATE_COMPLETE;
        }
    }
    return STATUS_BUSY;
}

bool oled_flush_async_active(const oled_handle_t *handle)
{
    return (handle != NULL) && handle->async_flush_active;
}

status_t oled_get_last_status(void)
{
    return g_oled_last_status;
}

uint8_t oled_get_last_failed_command(void)
{
    return g_oled_last_failed_command;
}

uint8_t oled_get_last_failed_index(void)
{
    return g_oled_last_failed_index;
}
