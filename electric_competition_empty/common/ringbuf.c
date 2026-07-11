#include "common/ringbuf.h"

static uint16_t ringbuf_next_index(const ringbuf_t *rb, uint16_t index)
{
    /* Keep wrap-around logic in one helper so push/pop stay branch-light. */
    index++;
    if (index >= rb->capacity) {
        index = 0U;
    }
    return index;
}

void ringbuf_init(ringbuf_t *rb, uint8_t *storage, uint16_t capacity)
{
    rb->buffer = storage;
    rb->capacity = capacity;
    rb->head = 0U;
    rb->tail = 0U;
}

bool ringbuf_is_empty(const ringbuf_t *rb)
{
    return (rb->head == rb->tail);
}

bool ringbuf_is_full(const ringbuf_t *rb)
{
    return (ringbuf_next_index(rb, rb->head) == rb->tail);
}

uint16_t ringbuf_size(const ringbuf_t *rb)
{
    if (rb->head >= rb->tail) {
        return (uint16_t) (rb->head - rb->tail);
    }
    return (uint16_t) (rb->capacity - rb->tail + rb->head);
}

bool ringbuf_push_byte(ringbuf_t *rb, uint8_t byte)
{
    uint16_t next = ringbuf_next_index(rb, rb->head);
    if (next == rb->tail) {
        /* Drop-on-full is acceptable here because upper-layer protocols resync on frame headers. */
        return false;
    }

    rb->buffer[rb->head] = byte;
    rb->head = next;
    return true;
}

bool ringbuf_pop_byte(ringbuf_t *rb, uint8_t *byte)
{
    if (ringbuf_is_empty(rb)) {
        return false;
    }

    *byte = rb->buffer[rb->tail];
    rb->tail = ringbuf_next_index(rb, rb->tail);
    return true;
}

void ringbuf_clear(ringbuf_t *rb)
{
    rb->head = 0U;
    rb->tail = 0U;
}
