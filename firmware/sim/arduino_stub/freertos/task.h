#pragma once
#include "FreeRTOS.h"
inline TickType_t xTaskGetTickCount() { return 0; }
inline BaseType_t xTaskDelayUntil(TickType_t* const, const TickType_t) { return pdTRUE; }
inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char* const, const uint32_t, void* const,
                                          UBaseType_t, TaskHandle_t* const, const BaseType_t) { return pdTRUE; }
