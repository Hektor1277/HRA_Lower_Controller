#ifndef _RINGBUF_H
#define _RINGBUF_H
#include <stdint.h>
#include <string.h>

typedef struct
{
    uint8_t *buf;
    uint16_t cap;  /* 容量必须是 2^n                */
    uint16_t head; /* 写指针                         */
    uint16_t tail; /* 读指针                         */
} ringbuf_t;

/* ---------- 全局实例 ---------- */
extern ringbuf_t rb; /* 在  protocol.c  里真正定义     */

static inline void rb_init(ringbuf_t *rb, uint8_t *mem, uint16_t cap)
{
    rb->buf = mem;
    rb->cap = cap;
    rb->head = rb->tail = 0;
}
static inline void rb_reset(ringbuf_t *r)
{
    r->head = r->tail = 0;
}
static inline uint16_t rb_size(ringbuf_t *rb)
{
    return (rb->head >= rb->tail) ? (rb->head - rb->tail)
                                  : (rb->cap - rb->tail + rb->head);
}
static inline void rb_push(ringbuf_t *rb, uint16_t n) // 只移动 head 指针
{
    rb->head = (rb->head + n) % rb->cap;
}
static inline void rb_peek(ringbuf_t *rb, uint8_t *dst, uint16_t n)
{
    uint16_t first = (rb->cap - rb->tail < n) ? (rb->cap - rb->tail) : n;
    memcpy(dst, rb->buf + rb->tail, first);
    memcpy(dst + first, rb->buf, n - first);
}
static inline void rb_pop(ringbuf_t *rb, uint16_t n)
{
    rb->tail = (rb->tail + n) % rb->cap;
}
#endif
