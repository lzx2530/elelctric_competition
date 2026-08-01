#ifndef DRV_OLED_SSD1306_H
#define DRV_OLED_SSD1306_H

#include "common/status.h"
#include "common/types.h"

#define SSD1306_WIDTH       (128U)
#define SSD1306_HEIGHT      (64U)
#define SSD1306_BUF_SIZE    (SSD1306_WIDTH * SSD1306_HEIGHT / 8U)

typedef struct {
    uint8_t i2c_addr;
    uint8_t width;
    uint8_t height;
    uint8_t cursor_x;
    uint8_t cursor_y;
    bool dirty;
    bool async_flush_active;
    uint8_t async_page;
    uint8_t async_column;
    uint8_t async_state;
    uint8_t async_tx[17U];
    uint8_t buffer[SSD1306_BUF_SIZE];
} oled_handle_t;

/* 初始化后会发送 SSD1306 命令序列，并清屏一次 */
status_t oled_init(oled_handle_t *handle, uint8_t i2c_addr);
void oled_clear(oled_handle_t *handle);
void oled_set_cursor(oled_handle_t *handle, uint8_t x, uint8_t y);
void oled_write_char(oled_handle_t *handle, char ch);
void oled_write_str(oled_handle_t *handle, const char *str);
void oled_printf(oled_handle_t *handle, uint8_t x, uint8_t y, const char *fmt, ...);
/* 把本地 framebuffer 刷到屏上；只有 dirty 时才真正发数据 */
status_t oled_flush(oled_handle_t *handle);
status_t oled_flush_pages(oled_handle_t *handle, uint8_t first_page, uint8_t page_count);
status_t oled_flush_async_begin(oled_handle_t *handle);
status_t oled_flush_async_process(oled_handle_t *handle);
bool oled_flush_async_active(const oled_handle_t *handle);
status_t oled_get_last_status(void);
uint8_t oled_get_last_failed_command(void);
uint8_t oled_get_last_failed_index(void);

#endif
