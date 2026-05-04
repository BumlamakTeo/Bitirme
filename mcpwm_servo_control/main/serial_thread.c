/*
 * SPDX-FileCopyrightText: 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include "serial_thread.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "SerialThread";

// UART configuration
#define UART_NUM            UART_NUM_0
#define UART_RX_BUF_SIZE    1024
#define UART_TX_BUF_SIZE    0

// Install UART driver for reading
static bool uart_driver_installed = false;

static void install_uart_driver(void)
{
    if (!uart_driver_installed) {
        // Install UART driver with RX buffer
        esp_err_t ret = uart_driver_install(UART_NUM, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 0, NULL, 0);
        if (ret == ESP_OK) {
            uart_driver_installed = true;
        }
    }
}

// FreeRTOS task wrapper
static void serial_thread_task(void *pvParameters)
{
    serial_thread_context_t *ctx = (serial_thread_context_t *)pvParameters;
    
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Invalid thread context");
        vTaskDelete(NULL);
        return;
    }

    while (ctx->is_running) {
        // Non-blocking serial read with timeout
        serial_thread_process(ctx);
        
        // Small delay to prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    vTaskDelete(NULL);
}

void serial_thread_init(serial_thread_context_t *ctx, servo_thread_context_t *servo_ctx)
{
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Cannot initialize NULL context");
        return;
    }

    memset(ctx, 0, sizeof(serial_thread_context_t));
    ctx->servo_ctx = servo_ctx;
    ctx->buffer_pos = 0;
    ctx->is_running = false;
    ctx->task_handle = NULL;

    // Install UART driver
    install_uart_driver();
}

bool serial_thread_start(serial_thread_context_t *ctx)
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

    // Create task pinned to Core 0
    BaseType_t result = xTaskCreatePinnedToCore(
        serial_thread_task,
        SERIAL_THREAD_NAME,
        SERIAL_THREAD_STACK_SIZE,
        ctx,
        SERIAL_THREAD_PRIORITY,
        &ctx->task_handle,
        SERIAL_THREAD_CORE_ID
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create thread");
        ctx->is_running = false;
        return false;
    }

    return true;
}

void serial_thread_stop(serial_thread_context_t *ctx)
{
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Cannot stop NULL context");
        return;
    }

    if (!ctx->is_running) {
        ESP_LOGW(TAG, "Thread not running");
        return;
    }

    ctx->is_running = false;

    // Wait for task to terminate
    if (ctx->task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(50));
        ctx->task_handle = NULL;
    }
}

void serial_thread_process(serial_thread_context_t *ctx)
{
    if (ctx == NULL || ctx->servo_ctx == NULL || !uart_driver_installed) {
        return;
    }

    // Read available bytes from UART (non-blocking)
    uint8_t data[128];
    int len = uart_read_bytes(UART_NUM, data, sizeof(data), pdMS_TO_TICKS(10));

    if (len <= 0) {
        return;
    }

    // Process each byte
    for (int i = 0; i < len; i++) {
        char c = (char)data[i];

        // Handle newline or carriage return as command terminator
        if (c == '\n' || c == '\r') {
            if (ctx->buffer_pos > 0) {
                ctx->input_buffer[ctx->buffer_pos] = '\0';
                
                // Parse the command: expected format "CH0:1500 CH1:1000" or "1500 1000"
                int ch0_pw = -1, ch1_pw = -1;
                
                // Try format: "CH0:1500 CH1:1000"
                if (sscanf(ctx->input_buffer, "CH0:%d CH1:%d", &ch0_pw, &ch1_pw) == 2) {
                    servo_thread_set_target_pulsewidth(ctx->servo_ctx, 0, ch0_pw);
                    servo_thread_set_target_pulsewidth(ctx->servo_ctx, 1, ch1_pw);
                    ESP_LOGI(TAG, "ACK");
                }
                // Try format: "1500 1000" (space-separated)
                else if (sscanf(ctx->input_buffer, "%d %d", &ch0_pw, &ch1_pw) == 2) {
                    servo_thread_set_target_pulsewidth(ctx->servo_ctx, 0, ch0_pw);
                    servo_thread_set_target_pulsewidth(ctx->servo_ctx, 1, ch1_pw);
                    ESP_LOGI(TAG, "ACK");
                }
                // Try single channel: "CH0:1500" or "CH1:1000"
                else if (sscanf(ctx->input_buffer, "CH0:%d", &ch0_pw) == 1) {
                    servo_thread_set_target_pulsewidth(ctx->servo_ctx, 0, ch0_pw);
                    ESP_LOGI(TAG, "ACK");
                }
                else if (sscanf(ctx->input_buffer, "CH1:%d", &ch1_pw) == 1) {
                    servo_thread_set_target_pulsewidth(ctx->servo_ctx, 1, ch1_pw);
                    ESP_LOGI(TAG, "ACK");
                }
                
                // Reset buffer
                ctx->buffer_pos = 0;
            } else {
                // Empty line (consecutive CR/LF), just ignore
                ctx->buffer_pos = 0;
            }
        }
        // Add character to buffer if not full
        else if (ctx->buffer_pos < SERIAL_INPUT_BUFFER_SIZE - 1) {
            ctx->input_buffer[ctx->buffer_pos++] = c;
        }
        else {
            // Buffer overflow, reset
            ctx->buffer_pos = 0;
        }
    }
}
