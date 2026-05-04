/*
 * SPDX-FileCopyrightText: 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SERVO_THREAD_H
#define SERVO_THREAD_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Thread configuration
#define SERVO_THREAD_STACK_SIZE     4096
#define SERVO_THREAD_PRIORITY       5
#define SERVO_THREAD_NAME           "ServoControlThread"
#define SERVO_THREAD_PERIOD_MS      20
#define SERVO_THREAD_CORE_ID        1  // Run on Core 1

// Servo channel configuration
typedef struct {
    mcpwm_cmpr_handle_t comparator;
    int32_t current_pulsewidth;
    int32_t target_pulsewidth;
    int32_t min_pulsewidth;
    int32_t max_pulsewidth;
    uint8_t gpio;
} servo_channel_t;

// Thread context
typedef struct {
    TaskHandle_t task_handle;
    servo_channel_t ch0;
    servo_channel_t ch1;
    TickType_t last_wake_time;
    uint32_t cycle_count;
    bool is_running;
} servo_thread_context_t;

/**
 * @brief Initialize servo thread context
 * 
 * @param ctx Pointer to thread context
 * @param comparator_ch0 MCPWM comparator handle for channel 0
 * @param comparator_ch1 MCPWM comparator handle for channel 1
 */
void servo_thread_init(servo_thread_context_t *ctx, 
                       mcpwm_cmpr_handle_t comparator_ch0,
                       mcpwm_cmpr_handle_t comparator_ch1);

/**
 * @brief Start servo control thread
 * 
 * @param ctx Pointer to thread context
 * @return true if thread started successfully, false otherwise
 */
bool servo_thread_start(servo_thread_context_t *ctx);

/**
 * @brief Stop servo control thread
 * 
 * @param ctx Pointer to thread context
 */
void servo_thread_stop(servo_thread_context_t *ctx);

/**
 * @brief Execute one cycle of servo control
 * 
 * @param ctx Pointer to thread context
 */
void servo_thread_execute(servo_thread_context_t *ctx);

/**
 * @brief Set target pulsewidth for a channel
 * 
 * @param ctx Pointer to thread context
 * @param channel Channel number (0 or 1)
 * @param pulsewidth Target pulsewidth in microseconds
 */
void servo_thread_set_target_pulsewidth(servo_thread_context_t *ctx, uint8_t channel, int32_t pulsewidth);

#ifdef __cplusplus
}
#endif

#endif // SERVO_THREAD_H
