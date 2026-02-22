#include <inttypes.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hx711.h>
#include <driver/gpio.h>
#include <sdkconfig.h>
#include <mpu6050.h>
#include <esp_adc/adc_oneshot.h>
#include "driver/rmt_tx.h"
#include <dshot_esc_encoder.h>

#define LED_PIN 0 // no such a pin!
#define TSCK3 10
#define TDA3 9
#define TSCK2 4
#define TDA2 5

#define ADDR MPU6050_I2C_ADDRESS_LOW

#define ADC_PIN ADC_CHANNEL_2
#define ADC_UNIT ADC_UNIT_1
#define ADC_BITWIDTH ADC_BITWIDTH_DEFAULT // 12-bit resolution (0-4095)
#define ADC_ATTEN ADC_ATTEN_DB_12         // ~3.3V full-scale voltage

#define DSHOT_ESC_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution
#define DSHOT_ESC_GPIO_NUM 8

static const char *TAG = "prop-test-rig";

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

void adc_read(void *pvParameters)
{
    int adc_value;
    adc_oneshot_unit_handle_t adc_handle;

    // Initialize ADC Oneshot Mode Driver on the ADC Unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // Configure ADC channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_PIN, &config));

    // ADC Oneshot Analog Read loop
    while (1)
    {
        // Read ADC value with Oneshot
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_PIN, &adc_value));
        // Print ADC value
        float voltage = (float)adc_value / 4096.0 * 3.3;
        ESP_LOGI("Voltage", "%f", voltage);
        // Delay 1 second
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void throttle(void *pvParameters)
{
    int adc_value;
    adc_oneshot_unit_handle_t adc_handle;

    // Initialize ADC Oneshot Mode Driver on the ADC Unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // Configure ADC channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_3, &config));

    // ADC Oneshot Analog Read loop
    // while (1)
    // {
    //     // Read ADC value with Oneshot
    //     ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_3, &adc_value));
    //     // Print ADC value
    //     float voltage = (float)adc_value / 4096.0 * 100;
    //     ESP_LOGI("Throttle: ", "%f", voltage);
    //     // Delay 1 second
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t esc_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
        .gpio_num = DSHOT_ESC_GPIO_NUM,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &esc_chan));

    ESP_LOGI(TAG, "Install Dshot ESC encoder");
    rmt_encoder_handle_t dshot_encoder = NULL;
    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300 protocol
        .post_delay_us = 50, // extra delay between each frame
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(esc_chan));

    rmt_transmit_config_t tx_config = {
        .loop_count = -1, // infinite loop
    };
    dshot_esc_throttle_t throttle = {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    };

    ESP_LOGI(TAG, "Start ESC by sending zero throttle for a while...");
    ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "Increase throttle, no telemetry");
    for (uint16_t thro = 100; thro < 1000; thro += 10)
    {
        throttle.throttle = thro;
        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
        // the previous loop transfer is till undergoing, we need to stop it and restart,
        // so that the new throttle can be updated on the output
        ESP_ERROR_CHECK(rmt_disable(esc_chan));
        ESP_ERROR_CHECK(rmt_enable(esc_chan));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    // xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    // xTaskCreate(adc_read, "adc_read", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    //  xTaskCreate(blink_led, "blink_led", configMINIMAL_STACK_SIZE * 2, NULL, 4, NULL);
    // xTaskCreate(mpu6050_test, "mpu6050_test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
    xTaskCreate(throttle, "throttle", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}
