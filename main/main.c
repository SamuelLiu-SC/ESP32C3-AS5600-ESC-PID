#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#define LED_GPIO GPIO_NUM_8
#define ESC_GPIO GPIO_NUM_2
#define I2C_SDA_GPIO GPIO_NUM_0
#define I2C_SCL_GPIO GPIO_NUM_1
#define ESC_PWM_FREQUENCY_HZ 400
#define ESC_PWM_RESOLUTION LEDC_TIMER_14_BIT
#define ESC_PWM_PERIOD_US 2500U
// #define ESC_MIN_PULSE_US 1060U
// #define ESC_MAX_PULSE_US 1860U
#define ESC_MIN_PULSE_US 1000U
#define ESC_MAX_PULSE_US 2000U
#define ESC_MAX_TARGET_RPM 30000
#define ESC_MAX_PULSE_STEP_US 50U
#define PID_KP_INITIAL 0.08f
#define PID_KI_INITIAL 0.40f
#define PID_KD_INITIAL 0.001f
#define PID_MAX_GAIN 1000.0f
#define PID_NVS_NAMESPACE "pid"
#define PID_NVS_KEY "gains"
#define PID_INTEGRAL_LIMIT 2000.0f
#define AS5600_I2C_ADDRESS 0x36
#define AS5600_RAW_ANGLE_REGISTER 0x0C
#define AS5600_SAMPLE_INTERVAL_MS 2U
#define AS5600_RPM_REPORT_INTERVAL_US 100000LL

static QueueHandle_t rpm_command_queue;
static QueueHandle_t pid_gains_queue;
static float pid_kp = PID_KP_INITIAL;
static float pid_ki = PID_KI_INITIAL;
static float pid_kd = PID_KD_INITIAL;

typedef struct {
    float kp;
    float ki;
    float kd;
} pid_gains_t;

typedef struct {
    bool has_previous_angle;
    uint16_t previous_angle;
    int64_t window_start_us;
    int64_t accumulated_delta_counts;
} rpm_meter_t;

static uint32_t esc_pulse_to_duty(uint32_t pulse_us)
{
    const uint32_t duty_max = 1U << ESC_PWM_RESOLUTION;
    return (duty_max * pulse_us + ESC_PWM_PERIOD_US / 2U) / ESC_PWM_PERIOD_US;
}

static int32_t as5600_angle_delta(uint16_t current_angle, uint16_t previous_angle)
{
    int32_t delta = (int32_t)current_angle - (int32_t)previous_angle;
    if (delta > 2048) {
        delta -= 4096;
    } else if (delta < -2048) {
        delta += 4096;
    }
    return delta;
}

static void set_esc_pulse(uint32_t pulse_us)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                                  esc_pulse_to_duty(pulse_us)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static uint32_t pid_update_pulse(int32_t target_rpm, float measured_rpm, float delta_seconds,
                                 float *integral, float *previous_error,
                                 bool *have_previous_error, uint32_t current_pulse_us)
{
    if (target_rpm == 0) {
        *integral = 0.0f;
        *previous_error = 0.0f;
        *have_previous_error = false;
        return ESC_MIN_PULSE_US;
    }

    const float error = (float)target_rpm - measured_rpm;
    const float derivative = *have_previous_error && delta_seconds > 0.0f
                                 ? (error - *previous_error) / delta_seconds
                                 : 0.0f;
    float candidate_integral = *integral + error * delta_seconds;
    if (candidate_integral > PID_INTEGRAL_LIMIT) {
        candidate_integral = PID_INTEGRAL_LIMIT;
    } else if (candidate_integral < -PID_INTEGRAL_LIMIT) {
        candidate_integral = -PID_INTEGRAL_LIMIT;
    }

    float pulse_request = ESC_MIN_PULSE_US + pid_kp * error +
                          pid_ki * candidate_integral + pid_kd * derivative;
    if ((pulse_request <= ESC_MAX_PULSE_US && pulse_request >= ESC_MIN_PULSE_US) ||
        (pulse_request > ESC_MAX_PULSE_US && error < 0.0f) ||
        (pulse_request < ESC_MIN_PULSE_US && error > 0.0f)) {
        *integral = candidate_integral;
    }

    pulse_request = ESC_MIN_PULSE_US + pid_kp * error +
                    pid_ki * *integral + pid_kd * derivative;
    if (pulse_request > ESC_MAX_PULSE_US) {
        pulse_request = ESC_MAX_PULSE_US;
    } else if (pulse_request < ESC_MIN_PULSE_US) {
        pulse_request = ESC_MIN_PULSE_US;
    }

    uint32_t target_pulse_us = (uint32_t)(pulse_request + 0.5f);
    if (target_pulse_us > current_pulse_us + ESC_MAX_PULSE_STEP_US) {
        target_pulse_us = current_pulse_us + ESC_MAX_PULSE_STEP_US;
    } else if (target_pulse_us + ESC_MAX_PULSE_STEP_US < current_pulse_us) {
        target_pulse_us = current_pulse_us - ESC_MAX_PULSE_STEP_US;
    }

    *previous_error = error;
    *have_previous_error = true;
    return target_pulse_us;
}

static esp_err_t save_pid_gains(const pid_gains_t *gains)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(PID_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, PID_NVS_KEY, gains, sizeof(*gains));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static void load_pid_gains(void)
{
    nvs_handle_t handle;
    if (nvs_open(PID_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return;
    }
    pid_gains_t gains;
    size_t size = sizeof(gains);
    const esp_err_t err = nvs_get_blob(handle, PID_NVS_KEY, &gains, &size);
    nvs_close(handle);
    if (err != ESP_OK || size != sizeof(gains) ||
        !isfinite(gains.kp) || !isfinite(gains.ki) || !isfinite(gains.kd) ||
        gains.kp < 0.0f || gains.kp > PID_MAX_GAIN ||
        gains.ki < 0.0f || gains.ki > PID_MAX_GAIN ||
        gains.kd < 0.0f || gains.kd > PID_MAX_GAIN) {
        return;
    }
    pid_kp = gains.kp;
    pid_ki = gains.ki;
    pid_kd = gains.kd;
    printf("Loaded saved PID gains: Kp=%.6g Ki=%.6g Kd=%.6g\n",
           (double)pid_kp, (double)pid_ki, (double)pid_kd);
}

static void rpm_command_task(void *parameter)
{
    QueueHandle_t command_queue = (QueueHandle_t)parameter;
    char command_buffer[96];
    printf("Commands: target RPM (0-%d; 0 stops), or PID <Kp> <Ki> <Kd> (0-%g each)\n",
           ESC_MAX_TARGET_RPM, (double)PID_MAX_GAIN);

    while (1) {
        if (fgets(command_buffer, sizeof(command_buffer), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        char *command_start = command_buffer;
        while (*command_start == ' ' || *command_start == '\t') {
            command_start++;
        }
        if (strncmp(command_start, "PID", 3) == 0 &&
            (command_start[3] == ' ' || command_start[3] == '\t')) {
            char *value_pointer = command_start + 3;
            pid_gains_t gains;
            float *gain_values[] = {&gains.kp, &gains.ki, &gains.kd};
            bool valid = true;
            for (size_t i = 0; i < sizeof(gain_values) / sizeof(gain_values[0]); i++) {
                while (*value_pointer == ' ' || *value_pointer == '\t') {
                    value_pointer++;
                }
                char *end_pointer;
                const float value = strtof(value_pointer, &end_pointer);
                if (end_pointer == value_pointer || !isfinite(value) ||
                    value < 0.0f || value > PID_MAX_GAIN) {
                    valid = false;
                    break;
                }
                *gain_values[i] = value;
                value_pointer = end_pointer;
            }
            while (*value_pointer == ' ' || *value_pointer == '\t' ||
                   *value_pointer == '\r' || *value_pointer == '\n') {
                value_pointer++;
            }
            if (!valid || *value_pointer != '\0') {
                printf("Invalid PID gains. Use: PID <Kp> <Ki> <Kd> (0-%g each)\n",
                       (double)PID_MAX_GAIN);
                continue;
            }
            xQueueOverwrite(pid_gains_queue, &gains);
            const esp_err_t save_err = save_pid_gains(&gains);
            printf("PID gains set: Kp=%.6g Ki=%.6g Kd=%.6g (%s)\n",
                   (double)gains.kp, (double)gains.ki, (double)gains.kd,
                   save_err == ESP_OK ? "saved" : esp_err_to_name(save_err));
            continue;
        }

        char *end_pointer;
        const long target_rpm = strtol(command_buffer, &end_pointer, 10);
        while (*end_pointer == ' ' || *end_pointer == '\t' ||
               *end_pointer == '\r' || *end_pointer == '\n') {
            end_pointer++;
        }
        if (end_pointer == command_buffer || *end_pointer != '\0' ||
            target_rpm < 0 || target_rpm > ESC_MAX_TARGET_RPM) {
            printf("Invalid target. Enter an integer RPM from 0 to %d.\n",
                   ESC_MAX_TARGET_RPM);
            continue;
        }

        const int32_t target = (int32_t)target_rpm;
        xQueueOverwrite(command_queue, &target);
        printf("RPM target set to %ld\n", target_rpm);
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

static esp_err_t sample_rpm(i2c_master_dev_handle_t device, rpm_meter_t *meter,
                            bool *measurement_ready, float *rpm, float *elapsed_seconds)
{
    uint16_t raw_angle;
    esp_err_t err = read_as5600_angle(device, &raw_angle);
    if (err != ESP_OK) {
        return err;
    }

    *measurement_ready = false;
    const int64_t sample_time_us = esp_timer_get_time();
    if (!meter->has_previous_angle) {
        meter->previous_angle = raw_angle;
        meter->window_start_us = sample_time_us;
        meter->has_previous_angle = true;
        return ESP_OK;
    }

    meter->accumulated_delta_counts += as5600_angle_delta(raw_angle, meter->previous_angle);
    meter->previous_angle = raw_angle;
    const int64_t elapsed_us = sample_time_us - meter->window_start_us;
    if (elapsed_us < AS5600_RPM_REPORT_INTERVAL_US) {
        return ESP_OK;
    }

    const int64_t speed_milli_rpm =
        meter->accumulated_delta_counts * 60000000000LL / (4096LL * elapsed_us);
    *rpm = (float)speed_milli_rpm / 1000.0f;
    *elapsed_seconds = (float)elapsed_us / 1000000.0f;
    meter->accumulated_delta_counts = 0;
    meter->window_start_us = sample_time_us;
    *measurement_ready = true;
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);
    load_pid_gains();

    configure_esc_pwm();

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
    set_esc_pulse(ESC_MIN_PULSE_US);
    rpm_command_queue = xQueueCreate(1, sizeof(int32_t));
    ESP_ERROR_CHECK(rpm_command_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    pid_gains_queue = xQueueCreate(1, sizeof(pid_gains_t));
    ESP_ERROR_CHECK(pid_gains_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(rpm_command_task, "rpm_command", 4096,
                                rpm_command_queue, 5, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_FAIL);

    bool led_on = true;
    TickType_t last_led_toggle = xTaskGetTickCount();
    bool have_previous_error = false;
    int32_t target_rpm = 0;
    int64_t last_error_time_us = 0;
    float pid_integral = 0.0f;
    float previous_error = 0.0f;
    uint32_t current_pulse_us = ESC_MIN_PULSE_US;
    rpm_meter_t rpm_meter = {0};
    uint32_t consecutive_read_errors = 0;

    while (1) {
        bool measurement_ready;
        float measured_rpm;
        float elapsed_seconds;
        esp_err_t err = sample_rpm(sensor, &rpm_meter, &measurement_ready,
                                   &measured_rpm, &elapsed_seconds);
        if (err == ESP_OK) {
            consecutive_read_errors = 0;
            if (measurement_ready) {
                if (measured_rpm < 0.0f) {
                    measured_rpm = -measured_rpm;
                }

                int32_t new_target_rpm;
                if (xQueueReceive(rpm_command_queue, &new_target_rpm, 0) == pdTRUE) {
                    target_rpm = new_target_rpm;
                }
                pid_gains_t new_pid_gains;
                if (xQueueReceive(pid_gains_queue, &new_pid_gains, 0) == pdTRUE) {
                    pid_kp = new_pid_gains.kp;
                    pid_ki = new_pid_gains.ki;
                    pid_kd = new_pid_gains.kd;
                    pid_integral = 0.0f;
                    have_previous_error = false;
                }
                current_pulse_us = pid_update_pulse(
                    target_rpm, measured_rpm, elapsed_seconds,
                    &pid_integral, &previous_error, &have_previous_error, current_pulse_us);
                set_esc_pulse(current_pulse_us);
                  printf("PID target=%ld RPM measured=%.1f RPM ESC=%lu us Kp=%.5f Ki=%.5f Kd=%.5f\n",
                       (long)target_rpm,
                       (double)measured_rpm,
                      (unsigned long)current_pulse_us,
                      (double)pid_kp,
                      (double)pid_ki,
                      (double)pid_kd);
            }
        } else {
            consecutive_read_errors++;
            if (consecutive_read_errors >= 5U && current_pulse_us != ESC_MIN_PULSE_US) {
                target_rpm = 0;
                pid_integral = 0.0f;
                current_pulse_us = ESC_MIN_PULSE_US;
                set_esc_pulse(current_pulse_us);
            }
            const int64_t error_time_us = esp_timer_get_time();
            if (error_time_us - last_error_time_us >= 1000000) {
                printf("AS5600 read error: %s\n", esp_err_to_name(err));
                last_error_time_us = error_time_us;
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (now - last_led_toggle >= pdMS_TO_TICKS(2000)) {
            led_on = !led_on;
            ESP_ERROR_CHECK(gpio_set_level(LED_GPIO, led_on));
            last_led_toggle = now;
        }

        vTaskDelay(pdMS_TO_TICKS(AS5600_SAMPLE_INTERVAL_MS));
    }
}
