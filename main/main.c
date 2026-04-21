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
#include <math.h>
#include <dshot_esc_encoder.h>
#include "ring_avg.h"
#include "tenso.h"
#include <driver/i2c_master.h>
#include <as5600.h>
#include <esp_timer.h>
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

#define THRO_MAX 1500 // 2047
#define THRO_MIN 100
#define THRO_MAX_MS 5000

#define SINE_RANGE (THRO_MAX - THRO_MIN) * 0.5
#define SINE_MID THRO_MAX - SINE_RANGE / 2
#define ZERO_ANGLE -0.85 - M_PI / 2
#define INTERVAL_US 500

static const char *TAG = "prop-test-rig";
static bool PUSH = 0;
static bool READY_TO_READ = 0;
static float angle = 0;
static int throttleSin = 0;
static int maxDuration = 0;
static float lastAngle = 0;
volatile float omega_r = 0;
static int64_t oldTime = 0;

void tensometer_handler(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    tenso_t *tenso = (tenso_t *)pvParameters;
    ring_avg_t avg;
    ring_avg_init(&avg);

    while (1)
    {
        float weight = get_tenso_data(tenso);
        // ESP_LOGI(TAG, "weight: %f", weight);
        ring_avg_push(&avg, weight, &PUSH);
        ring_avg_read_if_ready(&avg, READY_TO_READ, tenso->label);
        vTaskDelay(pdMS_TO_TICKS(100));
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

void mpu6050_handler(void *pvParameters)
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

    ring_avg_t avg;
    ring_avg_init(&avg);

    while (1)
    {
        float temp;
        mpu6050_acceleration_t accel = {0};
        mpu6050_rotation_t rotation = {0};

        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

        float total_accel = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
        float vibr = fabsf(total_accel - 1.0f - 0.025385f); // with stationary offset.
        // ESP_LOGI(TAG, "total vibr: %f", vibr);
        ring_avg_push(&avg, vibr, &PUSH);
        ring_avg_read_if_ready(&avg, READY_TO_READ, "vibration [G]");

        // ESP_LOGI(TAG, "**********************************************************************");
        // ESP_LOGI(TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);
        // ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);
        // ESP_LOGI(TAG, "Temperature:  %.1f", temp);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void amperes_handler(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    int adc_value;
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t adc_cali_handle;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN, // Full 0–3.3V range
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc_handle, ADC_PIN, &config);

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    // ADC Oneshot Analog Read loop
    ring_avg_t avg;
    ring_avg_init(&avg);

    while (1)
    {
        int raw;
        float volts_to_binary_factor = 4096.0 / 3.3;
        float sensitivity = 0.04f;
        adc_oneshot_read(adc_handle, ADC_PIN, &raw);
        float volts = raw * 3.3 / 1024;
        float result = (volts - 1.65) / sensitivity;
        ring_avg_push(&avg, result, &PUSH);
        ring_avg_read_if_ready(&avg, READY_TO_READ, "current [A]");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/// @brief not used because it works more accurately with timer
void throttle(void *pvParameters)
{
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
        .post_delay_us = 20, // extra delay between each frame
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

    ESP_LOGI(TAG, "Increase throttle");
    for (uint16_t thro = THRO_MIN; thro < THRO_MAX; thro += 1)
    {
        // ESP_LOGI(TAG, "Throttle: %i", thro);
        throttle.throttle = thro;
        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
        // the previous loop transfer is till undergoing, we need to stop it and restart,
        // so that the new throttle can be updated on the output
        ESP_ERROR_CHECK(rmt_disable(esc_chan));
        ESP_ERROR_CHECK(rmt_enable(esc_chan));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Full throttle");
    PUSH = 1;
    vTaskDelay(pdMS_TO_TICKS(THRO_MAX_MS));
    PUSH = 0;
    READY_TO_READ = 1;
    ESP_LOGI(TAG, "Slowing down");
    for (uint16_t thro = THRO_MAX; thro > THRO_MIN; thro -= 1)
    {
        // ESP_LOGI(TAG, "Throttle: %i", thro);
        throttle.throttle = thro;
        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
        ESP_ERROR_CHECK(rmt_disable(esc_chan));
        ESP_ERROR_CHECK(rmt_enable(esc_chan));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Turning off");
    throttle.throttle = 0;
    ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
    ESP_ERROR_CHECK(rmt_disable(esc_chan));
    ESP_ERROR_CHECK(rmt_enable(esc_chan));
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void as5600_encoder(void *pvParameters)
{
    ESP_LOGI(TAG, "Init I2C (driver_ng)");

    // --- 1. Create I2C bus ---
    i2c_master_bus_handle_t bus_handle;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_6,
        .scl_io_num = GPIO_NUM_7,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    // --- 2. Add AS5600 device ---
    i2c_master_dev_handle_t dev_handle;

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDRESS,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    // --- 3. Read loop ---
    while (1)
    {
        uint8_t reg = 0x0E; // ANGLE register (high byte)
        uint8_t data[2];

        // Read 2 bytes (ANGLE register = 0x0E, 0x0F)
        esp_err_t err = i2c_master_transmit_receive(
            dev_handle,
            &reg, 1, // write register address
            data, 2, // read 2 bytes
            1000     // timeout ms
        );

        if (err == ESP_OK)
        {
            uint16_t raw = (data[0] << 8) | data[1];
            angle = (raw * 360.0f) / 4096.0f * M_PI / 180;
            float sin = sinf(angle);
            throttleSin = SINE_MID + sin * SINE_RANGE / 2;

            // ESP_LOGI(TAG, "debug: %.2f", SINE_RANGE);

            printf("Throttle: %i pts\n", throttleSin);
        }
        else
        {
            ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

typedef struct
{
    i2c_master_dev_handle_t *dev_handle;
    rmt_channel_handle_t *esc_chan;
    rmt_encoder_handle_t *dshot_encoder;
    rmt_transmit_config_t *tx_config;
    dshot_esc_throttle_t *throttle;
    esp_timer_handle_t *timer;
    ring_avg_t *ring_avg;
} timer_handle_t;

static void dshot_timer_cb(void *arg)
{
    int64_t start = esp_timer_get_time();
    timer_handle_t *timer_handle = (timer_handle_t *)arg;
    uint8_t reg = 0x0E; // ANGLE register (high byte)
    uint8_t data[2];

    // Read 2 bytes (ANGLE register = 0x0E, 0x0F)
    esp_err_t err = i2c_master_transmit_receive(
        *timer_handle->dev_handle,
        &reg, 1, // write register address
        data, 2, // read 2 bytes
        1000     // timeout ms
    );

    if (err == ESP_OK)
    {
        int64_t time = esp_timer_get_time();
        uint16_t raw = (data[0] << 8) | data[1];
        angle = (raw * 360.0f) / 4096.0f * M_PI / 180 + ZERO_ANGLE;
        if (angle < 0)
            angle += 2 * M_PI;
        if (angle < 0)
            return;
        // float speed = (angle - lastAngle) / ((time - oldTime) / 1e6);
        ring_avg_push(timer_handle->ring_avg, angle, &PUSH);
        // lastAngle = angle;
        // oldTime = time;
        float sin = sinf(angle);

        throttleSin = SINE_MID + sin * SINE_RANGE / 2;
        timer_handle->throttle->throttle = throttleSin;
        ESP_ERROR_CHECK(rmt_transmit(*timer_handle->esc_chan, *timer_handle->dshot_encoder, timer_handle->throttle, sizeof(*timer_handle->throttle), timer_handle->tx_config));
        ESP_ERROR_CHECK(rmt_disable(*timer_handle->esc_chan));
        ESP_ERROR_CHECK(rmt_enable(*timer_handle->esc_chan));
        // ESP_LOGI(TAG, "debug: %.2f", SINE_RANGE);

        // printf("Throttle: %i pts\n", throttleSin);
        //  printf("Angle: %.2f rad\n", angle);
    }
    else
    {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
    }
    int64_t end = esp_timer_get_time();

    int64_t duration = end - start; // microseconds

    // printf("Dur: %lli \n", duration);
    if (duration > maxDuration)
        maxDuration = duration;
}

void TerminateTimer(void *vParameters)
{
    vTaskDelay(pdMS_TO_TICKS(THRO_MAX_MS));
    PUSH = 0;
    READY_TO_READ = 1;
    ESP_LOGI(TAG, "Termination");
    timer_handle_t *timer_handle = (timer_handle_t *)vParameters;
    esp_timer_stop(*timer_handle->timer);
    esp_timer_delete(*timer_handle->timer);
    // print_all(timer_handle->ring_avg);

    for (uint16_t thro = THRO_MAX; thro > THRO_MIN; thro -= 1)
    {
        // ESP_LOGI(TAG, "Throttle: %i", thro);
        timer_handle->throttle->throttle = thro;
        ESP_ERROR_CHECK(rmt_transmit(*timer_handle->esc_chan, *timer_handle->dshot_encoder, timer_handle->throttle, sizeof(*timer_handle->throttle), timer_handle->tx_config));
        ESP_ERROR_CHECK(rmt_disable(*timer_handle->esc_chan));
        ESP_ERROR_CHECK(rmt_enable(*timer_handle->esc_chan));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    timer_handle->throttle->throttle = 0;
    ESP_ERROR_CHECK(rmt_transmit(*timer_handle->esc_chan, *timer_handle->dshot_encoder, timer_handle->throttle, sizeof(*timer_handle->throttle), timer_handle->tx_config));
    ESP_ERROR_CHECK(rmt_disable(*timer_handle->esc_chan));
    ESP_ERROR_CHECK(rmt_enable(*timer_handle->esc_chan));
    ESP_LOGI(TAG, "Turned off");
    vTaskDelete(NULL);
}

void InitTimer(void *vParameters)
{
    ESP_LOGI(TAG, "Init I2C (driver_ng)");

    // --- 1. Create I2C bus ---
    i2c_master_bus_handle_t bus_handle;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_6,
        .scl_io_num = GPIO_NUM_7,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    // --- 2. Add AS5600 device ---
    i2c_master_dev_handle_t dev_handle;

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDRESS,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

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
        .post_delay_us = 20, // extra delay between each frame
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
    ESP_LOGI(TAG, "Throttling up...");
    for (uint16_t thro = THRO_MIN; thro < 800; thro += 1)
    {
        throttle.throttle = thro;
        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
        ESP_ERROR_CHECK(rmt_disable(esc_chan));
        ESP_ERROR_CHECK(rmt_enable(esc_chan));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Throtte max");

    esp_timer_handle_t timer;
    ring_avg_t ring_avg;
    ring_avg_init(&ring_avg);

    timer_handle_t timer_handle = {
        .dev_handle = &dev_handle,
        .esc_chan = &esc_chan,
        .dshot_encoder = &dshot_encoder,
        .tx_config = &tx_config,
        .throttle = &throttle,
        .timer = &timer,
        .ring_avg = &ring_avg,
    };

    esp_timer_create_args_t args = {
        .callback = dshot_timer_cb,
        .arg = &timer_handle,
        .dispatch_method = ESP_TIMER_TASK, // safe default
        .name = "dshot_timer"};

    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, INTERVAL_US));

    xTaskCreate(TerminateTimer, "TerminationTimer", configMINIMAL_STACK_SIZE * 5, &timer_handle, 5, NULL);
    vTaskDelete(NULL);
}

void app_main()
{
    tenso_t *tenso2 = malloc(sizeof(tenso_t));
    tenso_init(tenso2, TDA2, TSCK2, 0.0007796891947777704, -114.54579575873767, "tenso 2 [g]");
    xTaskCreate(tensometer_handler, "tenso2", configMINIMAL_STACK_SIZE * 5, tenso2, 5, NULL);

    tenso_t *tenso3 = malloc(sizeof(tenso_t));
    tenso_init(tenso3, TDA3, TSCK3, 0.0007717730334784052, -67.27574018734285, "tenso 3 [g]");
    xTaskCreate(tensometer_handler, "tenso3", configMINIMAL_STACK_SIZE * 5, tenso3, 5, NULL);

    xTaskCreate(amperes_handler, "adc_read", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    ////  xTaskCreate(blink_led, "blink_led", configMINIMAL_STACK_SIZE * 2, NULL, 4, NULL);

    // xTaskCreate(mpu6050_handler, "mpu6050_test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
    // `xTaskCreate(throttle, "throttle", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    // xTaskCreate(as5600_encoder, "as5600_encoder", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    xTaskCreate(InitTimer, "InitTimer", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}
