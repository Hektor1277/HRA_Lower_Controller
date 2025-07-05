#ifndef _RINGBUF_H
#define _RINGBUF_H
#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t *buf;
    uint16_t cap, head, tail;
} RingBuf_t;

static inline void rb_init(RingBuf_t *rb, uint8_t *mem, uint16_t cap)
{
    rb->buf = mem; rb->cap = cap; rb->head = rb->tail = 0;
}
static inline uint16_t rb_size(RingBuf_t *rb)
{
    return (rb->head >= rb->tail) ? (rb->head - rb->tail)
                                  : (rb->cap - rb->tail + rb->head);
}
static inline void rb_push(RingBuf_t *rb, uint16_t n)   // 只移动 head 指针
{
    rb->head = (rb->head + n) % rb->cap;
}
static inline void rb_peek(RingBuf_t *rb, uint8_t *dst, uint16_t n)
{
    uint16_t first = (rb->cap - rb->tail < n) ? (rb->cap - rb->tail) : n;
    memcpy(dst,         rb->buf + rb->tail, first);
    memcpy(dst + first, rb->buf,           n - first);
}
static inline void rb_pop(RingBuf_t *rb, uint16_t n)
{
    rb->tail = (rb->tail + n) % rb->cap;
}
#endif
