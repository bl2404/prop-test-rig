#include <inttypes.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hx711.h>
#include <driver/gpio.h>
#include <sdkconfig.h>
#include <mpu6050.h>

#define LED_PIN 10
#define TSCK3 2
#define TDA3 3
#define TSCK2 4
#define TDA2 5

#define ADDR MPU6050_I2C_ADDRESS_LOW

static const char *TAG = "hx711-example";

int32_t get_tenso_data(hx711_t *tenso)
{
    int32_t data = 0;
    esp_err_t r = hx711_wait(tenso, 500);

    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "Device not found: %d (%s)\n", r, esp_err_to_name(r));
    }

    r = hx711_read_average(tenso, 5, &data);
    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not read data: %d (%s)\n", r, esp_err_to_name(r));
    }
    return data;
}

void test(void *pvParameters)
{
    hx711_t tenso3 =
        {
            .dout = TDA3,
            .pd_sck = TSCK3,
            .gain = HX711_GAIN_A_64};
    ESP_ERROR_CHECK(hx711_init(&tenso3));
    hx711_t tenso2 =
        {
            .dout = TDA2,
            .pd_sck = TSCK2,
            .gain = HX711_GAIN_A_64};
    ESP_ERROR_CHECK(hx711_init(&tenso2));

    while (1)
    {
        int32_t data_tenso_3 = get_tenso_data(&tenso3);
        int32_t data_tenso_2 = get_tenso_data(&tenso2);

        double weight_2 = 0.0007796891947777704 * (double)data_tenso_2 - 114.54579575873767;
        double weight_3 = 0.0007717730334784052 * (double)data_tenso_3 - 67.27574018734285;
        printf("raw3: %li weight3: %.2f raw2: %li weight2: %.2f\n", data_tenso_3, weight_3, data_tenso_2, weight_2);
        // ESP_LOGI(TAG, "Raw data tenso3: %" PRIi32 ", weight tenso2: %.2f", data_tenso_3, weight_2);
    }
}

void blink_led(void *pvParameters)
{
    gpio_output_enable(LED_PIN);
    bool state = 0;
    while (1)
    {
        gpio_set_level(LED_PIN, state);
        vTaskDelay(pdMS_TO_TICKS(500));
        state = !state;
    }
}

void mpu6050_test(void *pvParameters)
{
    mpu6050_dev_t dev = {0};

    gpio_set_pull_mode(GPIO_NUM_6, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_NUM_7, GPIO_PULLUP_ONLY);
    i2cdev_init();
    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, GPIO_NUM_6, GPIO_NUM_7));

    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(TAG, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));

    ESP_LOGI(TAG, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);

    while (1)
    {
        float temp;
        mpu6050_acceleration_t accel = {0};
        mpu6050_rotation_t rotation = {0};

        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

        ESP_LOGI(TAG, "**********************************************************************");
        ESP_LOGI(TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);
        ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);
        ESP_LOGI(TAG, "Temperature:  %.1f", temp);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main()
{
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    // xTaskCreate(blink_led, "blink_led", configMINIMAL_STACK_SIZE * 2, NULL, 4, NULL);
    // xTaskCreate(mpu6050_test, "mpu6050_test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}
