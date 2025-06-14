#include "control_task.h"
#include "sensor_task.h"
#include "pwm_task.h"
#include <Arduino.h>
#include <Adafruit_AHRS.h>

// P Controller parameters
float setpoint = 45.0;    // Target angle (degrees)
float kp = 2.0;          // Proportional gain (adjust as needed)
int controlOutput = 0.0; // Define the control output variable

void controlTask(void *pvParameters) {
    while (true) {
        // Wait for sensor data to be available
        float pitch = filter.getPitch(); // Function to get pitch from sensor task

        // Serial.print("Pitch: "); Serial.println(filter.getPitch());


        float error = setpoint - pitch;

        //print error


        if (error >= 0){

            if (controlOutput > 95) { // Limit control output to a maximum of 100
                controlOutput = 95; // Proportional control
            } else {
                controlOutput++; // Cap at maximum value
            }


        } else if (error < 0){

            if (controlOutput < 0) { // Limit control output to a minimum of -100
                controlOutput = 0; // Proportional control
            } else {
                controlOutput--; // Cap at minimum value
            }

        }


        

        // Serial.print("Pitch: "); Serial.print(pitch); Serial.print(", Control Output: "); Serial.println(controlOutput);

        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 ms
    }
}

void initControlTask() {
    xTaskCreate(controlTask, "ControlTask", 2048, NULL, 1, &controlTaskHandle);
}