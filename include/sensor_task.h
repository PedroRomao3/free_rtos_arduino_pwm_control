#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <Adafruit_LSM9DS0.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Adafruit_AHRS.h>

// Declare as extern - defined in main.cpp
extern Adafruit_LSM9DS0 lsm;
extern TaskHandle_t sensorTaskHandle;
extern Adafruit_NXPSensorFusion filter; // Reference to the sensor fusion filter

extern SemaphoreHandle_t filterMutex;
extern SemaphoreHandle_t controlOutputMutex;


void sensorTask(void *pvParameters);
void initSensor();

#endif