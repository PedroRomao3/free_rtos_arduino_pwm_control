#include "control_task.h"
#include "sensor_task.h"
#include "pwm_task.h"
#include <Arduino.h>
#include <Adafruit_AHRS.h>

// P Controller parameters
float setpoint = - 4.0;    // Target angle (degrees)
float kp = 1.0;          // Proportional gain - ADD THIS
int controlOutput = 0;   // Initialize to middle value

void controlTask(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for sensor initialization

    while (true) {
        float pitch = 0.0;
        
        // Protect filter access with mutex
        if (xSemaphoreTake(filterMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            pitch = filter.getPitch();
            xSemaphoreGive(filterMutex);
        } else {
            Serial.println("Control task: Failed to acquire filter mutex");
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Serial.print("Pitch: ");
        // Serial.println(pitch);

        float error = setpoint - pitch;

        // Protect controlOutput access with mutex
        if (xSemaphoreTake(controlOutputMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            // Simple increment/decrement control based on error sign
            if (error > 0) {
                controlOutput++;
                if (controlOutput > 95) {
                    controlOutput = 95;
                }
            } else if (error < 0) {
                controlOutput--;
                if (controlOutput < 5) {
                    controlOutput = 5;
                }
            }
            // If error == 0, do nothing (maintain current output)
            
            xSemaphoreGive(controlOutputMutex);
        } else {
            Serial.println("Control task: Failed to acquire controlOutput mutex");
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 10Hz control loop
    }
}