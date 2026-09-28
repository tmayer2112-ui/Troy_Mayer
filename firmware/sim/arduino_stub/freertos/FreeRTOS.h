#pragma once
#include <cstdint>
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef void* TaskHandle_t;
typedef void (*TaskFunction_t)(void*);
#define configTICK_RATE_HZ 1000
#define configMAX_PRIORITIES 25
#define pdFALSE 0
#define pdTRUE 1
