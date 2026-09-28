#pragma once
#include <cstdint>
#include "freertos/FreeRTOS.h"
typedef int esp_err_t;
#define ESP_OK 0
typedef struct { uint32_t timeout_ms; uint32_t idle_core_mask; bool trigger_panic; } esp_task_wdt_config_t;
inline esp_err_t esp_task_wdt_init(const esp_task_wdt_config_t*) { return ESP_OK; }
inline esp_err_t esp_task_wdt_reconfigure(const esp_task_wdt_config_t*) { return ESP_OK; }
inline esp_err_t esp_task_wdt_add(TaskHandle_t) { return ESP_OK; }
inline esp_err_t esp_task_wdt_reset(void) { return ESP_OK; }
