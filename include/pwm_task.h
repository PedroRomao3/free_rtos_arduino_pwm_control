#ifndef PWM_TASK_H
#define PWM_TASK_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Declare as extern - defined in main.cpp
extern TaskHandle_t pwmTaskHandle;

// Add mutex declarations
extern SemaphoreHandle_t controlOutputMutex;

void pwmTask(void *pvParameters);

#endif