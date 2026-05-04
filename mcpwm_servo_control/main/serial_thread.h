/*
 * SPDX-FileCopyrightText: 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SERIAL_THREAD_H
#define SERIAL_THREAD_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo_thread.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Thread configuration
#define SERIAL_THREAD_STACK_SIZE    4096
#define SERIAL_THREAD_PRIORITY      3
#define SERIAL_THREAD_NAME          "SerialInputThread"
#define SERIAL_THREAD_CORE_ID       0  // Run on Core 0

// Input buffer size
#define SERIAL_INPUT_BUFFER_SIZE    256

// Thread context
typedef struct {
    TaskHandle_t task_handle;
    servo_thread_context_t *servo_ctx;
    char input_buffer[SERIAL_INPUT_BUFFER_SIZE];
    size_t buffer_pos;
    bool is_running;
} serial_thread_context_t;

/**
 * @brief Initialize serial thread context
 * 
 * @param ctx Pointer to thread context
 * @param servo_ctx Pointer to servo thread context for updating pulsewidths
 */
void serial_thread_init(serial_thread_context_t *ctx, servo_thread_context_t *servo_ctx);

/**
 * @brief Start serial input thread on Core 0
 * 
 * @param ctx Pointer to thread context
 * @return true if thread started successfully, false otherwise
 */
bool serial_thread_start(serial_thread_context_t *ctx);

/**
 * @brief Stop serial input thread
 * 
 * @param ctx Pointer to thread context
 */
void serial_thread_stop(serial_thread_context_t *ctx);

/**
 * @brief Process incoming serial data
 * 
 * @param ctx Pointer to thread context
 */
void serial_thread_process(serial_thread_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif // SERIAL_THREAD_H
