#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO GPIO_NUM_8
#define ESC_GPIO GPIO_NUM_2
#define I2C_SDA_GPIO GPIO_NUM_0
#define I2C_SCL_GPIO GPIO_NUM_1
#define ESC_PWM_FREQUENCY_HZ 400
#define ESC_PWM_RESOLUTION LEDC_TIMER_14_BIT
#define ESC_PWM_PERIOD_US 2500U
#define ESC_MIN_PULSE_US 1060U
#define ESC_MID_PULSE_US 1460U
#define ESC_RUN_TIME_MS 10000U
#define ESC_RAMP_TIME_MS 1000U
#define ESC_RAMP_STEPS 100U
#define AS5600_I2C_ADDRESS 0x36
#define AS5600_RAW_ANGLE_REGISTER 0x0C

static uint32_t esc_pulse_to_duty(uint32_t pulse_us)
{
    const uint32_t duty_max = 1U << ESC_PWM_RESOLUTION;
    return (duty_max * pulse_us + ESC_PWM_PERIOD_US / 2U) / ESC_PWM_PERIOD_US;
}

static void ramp_esc_pulse(uint32_t start_pulse_us, uint32_t end_pulse_us)
{
    const int32_t pulse_difference = (int32_t)end_pulse_us - (int32_t)start_pulse_us;
    for (uint32_t step_index = 1; step_index <= ESC_RAMP_STEPS; step_index++) {
        const int32_t pulse_us = (int32_t)start_pulse_us +
                                 pulse_difference * (int32_t)step_index / (int32_t)ESC_RAMP_STEPS;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                                      esc_pulse_to_duty((uint32_t)pulse_us)));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
        vTaskDelay(pdMS_TO_TICKS(ESC_RAMP_TIME_MS / ESC_RAMP_STEPS));
    }
}

static void configure_esc_pwm(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = ESC_PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = ESC_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    const ledc_channel_config_t channel_config = {
        .gpio_num = ESC_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = esc_pulse_to_duty(ESC_MIN_PULSE_US),
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

static esp_err_t read_as5600_angle(i2c_master_dev_handle_t device, uint16_t *raw_angle)
{
    const uint8_t register_address = AS5600_RAW_ANGLE_REGISTER;
    uint8_t data[2];
    esp_err_t err = i2c_master_transmit_receive(device, &register_address, sizeof(register_address),
                                                data, sizeof(data), 1000);
    if (err != ESP_OK) {
        return err;
    }

    *raw_angle = ((uint16_t)(data[0] & 0x0F) << 8) | data[1];
    return ESP_OK;
}

void app_main(void)
{
    configure_esc_pwm();
    vTaskDelay(pdMS_TO_TICKS(2000));
    ramp_esc_pulse(ESC_MIN_PULSE_US, ESC_MID_PULSE_US);
    vTaskDelay(pdMS_TO_TICKS(ESC_RUN_TIME_MS));
    ramp_esc_pulse(ESC_MID_PULSE_US, ESC_MIN_PULSE_US);

    ESP_ERROR_CHECK(gpio_reset_pin(LED_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(LED_GPIO, 1));

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDRESS,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t sensor;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &device_config, &sensor));

    bool led_on = true;
    TickType_t last_led_toggle = xTaskGetTickCount();

    while (1) {
        uint16_t raw_angle;
        esp_err_t err = read_as5600_angle(sensor, &raw_angle);
        if (err == ESP_OK) {
            uint32_t angle_centi_degrees = (uint32_t)raw_angle * 36000U / 4096U;
            printf("AS5600 raw=%u angle=%lu.%02lu deg\n",
                   (unsigned)raw_angle,
                   (unsigned long)(angle_centi_degrees / 100U),
                   (unsigned long)(angle_centi_degrees % 100U));
        } else {
            printf("AS5600 read error: %s\n", esp_err_to_name(err));
        }

        TickType_t now = xTaskGetTickCount();
        if (now - last_led_toggle >= pdMS_TO_TICKS(2000)) {
            led_on = !led_on;
            ESP_ERROR_CHECK(gpio_set_level(LED_GPIO, led_on));
            last_led_toggle = now;
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
