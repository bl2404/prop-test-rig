#define BUFFER_SIZE 100
typedef struct
{
    float buf[BUFFER_SIZE];
    int head;
    int count;
} ring_avg_t;

void ring_avg_init(ring_avg_t *r);
void ring_avg_push(ring_avg_t *r, float value, bool *push);
float ring_avg_get(const ring_avg_t *r);
void ring_avg_read_if_ready(const ring_avg_t *r, bool ready, char *label);