#ifndef COMMON_RINGBUF_H
#define COMMON_RINGBUF_H

#include "common/types.h"

typedef struct {
    uint8_t *buffer;
    uint16_t capacity;
    volatile uint16_t head;
    volatile uint16_t tail;
} ringbuf_t;

/* 初始化外部提供存储区的环形缓冲区 */
void ringbuf_init(ringbuf_t *rb, uint8_t *storage, uint16_t capacity);
bool ringbuf_is_empty(const ringbuf_t *rb);
bool ringbuf_is_full(const ringbuf_t *rb);
uint16_t ringbuf_size(const ringbuf_t *rb);
/* 中断里可直接压单字节，满时返回 false */
bool ringbuf_push_byte(ringbuf_t *rb, uint8_t byte);
bool ringbuf_pop_byte(ringbuf_t *rb, uint8_t *byte);
void ringbuf_clear(ringbuf_t *rb);

#endif
