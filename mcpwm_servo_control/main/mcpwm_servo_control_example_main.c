/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "servo_thread.h"
#include "serial_thread.h"

static const char *TAG = "example";

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_PULSE_GPIO             40        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

// Global thread contexts
static servo_thread_context_t g_servo_thread;
static serial_thread_context_t g_serial_thread;

void app_main(void)
{
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparators and generators from the operator");
    
    // CH0 comparator (GPIO 40, 500–2500 us)
    mcpwm_cmpr_handle_t comparator_ch0 = NULL;
    mcpwm_comparator_config_t comparator_config_ch0 = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config_ch0, &comparator_ch0));

    // CH1 comparator (GPIO 42, 800–1500 us)
    mcpwm_cmpr_handle_t comparator_ch1 = NULL;
    mcpwm_comparator_config_t comparator_config_ch1 = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config_ch1, &comparator_ch1));

    // CH0 generator
    mcpwm_gen_handle_t generator_ch0 = NULL;
    mcpwm_generator_config_t generator_config_ch0 = {
        .gen_gpio_num = SERVO_PULSE_GPIO, // GPIO 40
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config_ch0, &generator_ch0));

    // CH1 generator
    mcpwm_gen_handle_t generator_ch1 = NULL;
    mcpwm_generator_config_t generator_config_ch1 = {
        .gen_gpio_num = 42, // GPIO 42
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config_ch1, &generator_ch1));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    
    // CH0 actions
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator_ch0,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator_ch0,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       comparator_ch0,
                                       MCPWM_GEN_ACTION_LOW)));

    // CH1 actions
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator_ch1,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator_ch1,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       comparator_ch1,
                                       MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    // Initialize servo thread with comparator handles
    ESP_LOGI(TAG, "Initialize servo control thread");
    servo_thread_init(&g_servo_thread, comparator_ch0, comparator_ch1);

    // Start the servo control thread on Core 1
    ESP_LOGI(TAG, "Starting servo control thread");
    if (!servo_thread_start(&g_servo_thread)) {
        ESP_LOGE(TAG, "Failed to start servo control thread");
        return;
    }

    // Initialize serial input thread
    ESP_LOGI(TAG, "Initialize serial input thread");
    serial_thread_init(&g_serial_thread, &g_servo_thread);

    // Start serial input thread on Core 0
    ESP_LOGI(TAG, "Starting serial input thread");
    if (!serial_thread_start(&g_serial_thread)) {
        ESP_LOGE(TAG, "Failed to start serial input thread");
        return;
    }

    ESP_LOGI(TAG, "Initialization complete");
    ESP_LOGI(TAG, "- Servo control running on Core 1 at 50Hz");
    ESP_LOGI(TAG, "- Serial input running on Core 0 (non-blocking)");
    ESP_LOGI(TAG, "Enter pulsewidth values in format: 'CH0:1500 CH1:1000' or '1500 1000'");
    ESP_LOGI(TAG, "Valid ranges - CH0: 500-2500us, CH1: 800-1500us");
}
