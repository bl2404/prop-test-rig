#include "ring_avg.h"
#include <string.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "RING-AVG";

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

void ring_avg_read_if_ready(const ring_avg_t *r, bool ready, char *label)
{
    if (ready)
    {
        float finalAvg = ring_avg_get(r);
        ESP_LOGI(TAG, "%s: %f", label, finalAvg);
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
