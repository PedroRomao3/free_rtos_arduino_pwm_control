#ifndef PWM_TASK_H
#define PWM_TASK_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Declare as extern - defined in main.cpp
extern TaskHandle_t pwmTaskHandle;

void pwmTask(void *pvParameters);

#endif