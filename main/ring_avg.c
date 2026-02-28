#include "ring_avg.h"
#include <string.h>

void ring_avg_init(ring_avg_t *r)
{
    r->head = 0;
    r->count = 0;
    memset(r->buf, 0, sizeof r->buf);
}

void ring_avg_push(ring_avg_t *r, float value, bool *push)
{
    if (!push)
        return;
    r->buf[r->head] = value;
    r->head = (r->head + 1) % BUFFER_SIZE;
    if (r->count < BUFFER_SIZE)
        r->count++;
}

float ring_avg_get(const ring_avg_t *r)
{
    float sum = 0;
    for (int i = 0; i < r->count; ++i)
        sum += r->buf[i];
    return r->count ? sum / r->count : 0;
}