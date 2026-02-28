#include <hx711.h>

typedef struct
{
    hx711_t *tenso;
    double linear_coeff_a;
    double linear_coeff_b;
    char *label;
    float tare;
} tenso_t;

void tenso_init(tenso_t *tenso, gpio_num_t dout, gpio_num_t sck, double linear_coeff_a, double linear_coeff_b, char *label);
float get_tenso_data(tenso_t *tenso);