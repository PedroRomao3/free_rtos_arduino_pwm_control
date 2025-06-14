#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Adafruit_AHRS.h>


extern TaskHandle_t controlTaskHandle;
extern int controlOutput;
extern float setpoint;
extern float kp;
extern Adafruit_NXPSensorFusion filter; // Reference to the sensor fusion filter


void controlTask(void *pvParameters);
void initControlTask();

#endif