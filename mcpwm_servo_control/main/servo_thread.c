/*
 * SPDX-FileCopyrightText: 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include "servo_thread.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ServoThread";

// FreeRTOS task wrapper
static void servo_thread_task(void *pvParameters)
{
    servo_thread_context_t *ctx = (servo_thread_context_t *)pvParameters;
    
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Invalid thread context");
        vTaskDelete(NULL);
        return;
    }

    ctx->last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(SERVO_THREAD_PERIOD_MS);

    ESP_LOGI(TAG, "Thread started on core %d, period: %d ms", xPortGetCoreID(), SERVO_THREAD_PERIOD_MS);

    while (ctx->is_running) {
        // Wait for next cycle with precise timing
        vTaskDelayUntil(&ctx->last_wake_time, period);

        // Execute servo control logic
        servo_thread_execute(ctx);

        ctx->cycle_count++;
    }

    ESP_LOGI(TAG, "Thread stopping after %lu cycles", (unsigned long)ctx->cycle_count);
    vTaskDelete(NULL);
}

void servo_thread_init(servo_thread_context_t *ctx,
                       mcpwm_cmpr_handle_t comparator_ch0,
                       mcpwm_cmpr_handle_t comparator_ch1)
{
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Cannot initialize NULL context");
        return;
    }

    memset(ctx, 0, sizeof(servo_thread_context_t));

    // Channel 0 configuration (GPIO 40, 500-2500 us)
    ctx->ch0.comparator = comparator_ch0;
    ctx->ch0.gpio = 40;
    ctx->ch0.min_pulsewidth = 500;
    ctx->ch0.max_pulsewidth = 2500;
    ctx->ch0.current_pulsewidth = (ctx->ch0.min_pulsewidth + ctx->ch0.max_pulsewidth) / 2;
    ctx->ch0.target_pulsewidth = ctx->ch0.current_pulsewidth;

    // Channel 1 configuration (GPIO 42, 800-1500 us)
    ctx->ch1.comparator = comparator_ch1;
    ctx->ch1.gpio = 42;
    ctx->ch1.min_pulsewidth = 500;
    ctx->ch1.max_pulsewidth = 1500;
    ctx->ch1.current_pulsewidth = (ctx->ch1.min_pulsewidth + ctx->ch1.max_pulsewidth) / 2;  // 1150us center
    ctx->ch1.target_pulsewidth = ctx->ch1.current_pulsewidth;

    ctx->is_running = false;
    ctx->cycle_count = 0;
    ctx->task_handle = NULL;

    ESP_LOGI(TAG, "Context initialized - CH0: GPIO %d (%d-%d us), CH1: GPIO %d (%d-%d us)",
             ctx->ch0.gpio, ctx->ch0.min_pulsewidth, ctx->ch0.max_pulsewidth,
             ctx->ch1.gpio, ctx->ch1.min_pulsewidth, ctx->ch1.max_pulsewidth);
}

bool servo_thread_start(servo_thread_context_t *ctx)
{
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Cannot start NULL context");
        return false;
    }

    if (ctx->is_running) {
        ESP_LOGW(TAG, "Thread already running");
        return false;
    }

    ctx->is_running = true;

    BaseType_t result = xTaskCreatePinnedToCore(
        servo_thread_task,
        SERVO_THREAD_NAME,
        SERVO_THREAD_STACK_SIZE,
        ctx,
        SERVO_THREAD_PRIORITY,
        &ctx->task_handle,
        SERVO_THREAD_CORE_ID
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create thread");
        ctx->is_running = false;
        return false;
    }

    ESP_LOGI(TAG, "Thread started successfully");
    return true;
}

void servo_thread_stop(servo_thread_context_t *ctx)
{
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Cannot stop NULL context");
        return;
    }

    if (!ctx->is_running) {
        ESP_LOGW(TAG, "Thread not running");
        return;
    }

    ESP_LOGI(TAG, "Stopping thread...");
    ctx->is_running = false;

    // Wait for task to terminate
    if (ctx->task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(50)); // Give task time to clean up
        ctx->task_handle = NULL;
    }
}

void servo_thread_execute(servo_thread_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    // Update CH0 to target pulsewidth instantly
    ctx->ch0.current_pulsewidth = ctx->ch0.target_pulsewidth;
    mcpwm_comparator_set_compare_value(ctx->ch0.comparator, (uint32_t)ctx->ch0.current_pulsewidth);

    // Update CH1 to target pulsewidth instantly
    ctx->ch1.current_pulsewidth = ctx->ch1.target_pulsewidth;
    mcpwm_comparator_set_compare_value(ctx->ch1.comparator, (uint32_t)ctx->ch1.current_pulsewidth);

    // Log every update at 50Hz
    ESP_LOGI(TAG, "CH0: %ld us, CH1: %ld us", (long)ctx->ch0.current_pulsewidth, (long)ctx->ch1.current_pulsewidth);
}

void servo_thread_set_target_pulsewidth(servo_thread_context_t *ctx, uint8_t channel, int32_t pulsewidth)
{
    if (ctx == NULL) {
        return;
    }

    if (channel == 0) {
        // Clamp to valid range for CH0
        if (pulsewidth < ctx->ch0.min_pulsewidth) {
            pulsewidth = ctx->ch0.min_pulsewidth;
        }
        if (pulsewidth > ctx->ch0.max_pulsewidth) {
            pulsewidth = ctx->ch0.max_pulsewidth;
        }
        ctx->ch0.target_pulsewidth = pulsewidth;
        ESP_LOGI(TAG, "CH0 target set: %ld us", (long)pulsewidth);
    } else if (channel == 1) {
        // Clamp to valid range for CH1
        if (pulsewidth < ctx->ch1.min_pulsewidth) {
            pulsewidth = ctx->ch1.min_pulsewidth;
        }
        if (pulsewidth > ctx->ch1.max_pulsewidth) {
            pulsewidth = ctx->ch1.max_pulsewidth;
        }
        ctx->ch1.target_pulsewidth = pulsewidth;
        ESP_LOGI(TAG, "CH1 target set: %ld us", (long)pulsewidth);
    }
}
