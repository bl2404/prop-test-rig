#include <hx711.h>
#include "tenso.h"
#include <esp_log.h>
#include <stdlib.h>

static char *TAG = "tenso";

void tenso_init(tenso_t *tenso, gpio_num_t dout, gpio_num_t sck, double linear_coeff_a, double linear_coeff_b, char *label)
{
    hx711_t *hxTensometer = malloc(sizeof(hx711_t));
    hxTensometer->dout = dout;
    hxTensometer->pd_sck = sck;
    hxTensometer->gain = HX711_GAIN_A_64;

    tenso->tenso = hxTensometer;
    tenso->linear_coeff_a = linear_coeff_a;
    tenso->linear_coeff_b = linear_coeff_b;
    tenso->label = label;

    ESP_ERROR_CHECK(hx711_init(hxTensometer));
    tenso->tare = get_tenso_data(tenso);
}

float get_tenso_data(tenso_t *tenso)
{
    int32_t rawData = 0;

    esp_err_t r = hx711_wait(tenso->tenso, 500);

    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "Device not found: %d (%s)\n", r, esp_err_to_name(r));
    }

    r = hx711_read_average(tenso->tenso, 5, &rawData);
    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not read data: %d (%s)\n", r, esp_err_to_name(r));
    }
    float weight = tenso->linear_coeff_a * (float)rawData + tenso->linear_coeff_b - tenso->tare;
    return weight;
}