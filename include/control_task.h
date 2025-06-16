#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Adafruit_AHRS.h>

extern TaskHandle_t controlTaskHandle;
extern int controlOutput;
extern float setpoint;
extern Adafruit_NXPSensorFusion filter; // Reference to the sensor fusion filter

// Add mutex declarations
extern SemaphoreHandle_t filterMutex;
extern SemaphoreHandle_t controlOutputMutex;

void controlTask(void *pvParameters);

#endif