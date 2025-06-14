#include "pwm_task.h"
#include <Arduino.h>
#include <Adafruit_LSM9DS0.h>

extern int controlOutput; // Control output from the control task

void pwmTask(void *pvParameters) {
    const int PWM_CHANNEL = 0;
    const int PWM_PIN = 5;
    const int PWM_MAX = 255;
    const int PWM_CENTER = PWM_MAX / 2; // 50% duty cycle

    // Configure the LEDC (PWM) channel

    while (true) {
        // Convert control output to PWM value
        int pwmValue = PWM_CENTER + (int)(controlOutput * (PWM_MAX - PWM_CENTER) / 100.0);
        
        // Constrain PWM value to valid range
        pwmValue = constrain(pwmValue, 0, PWM_MAX);
        
        // Apply PWM output
        ledcWrite(PWM_CHANNEL, pwmValue);

        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 ms (100 Hz)
    }
}

