#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_AHRS.h>
// #include <FreeRTOS.h>
#include "sensor_task.h"
#include "control_task.h"
#include "pwm_task.h"

// Initialize LSM9DS0 sensor (I2C, ID #1000)
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);
Adafruit_NXPSensorFusion filter; // Reference to the sensor fusion filter


// Task handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t pwmTaskHandle = NULL;

const int PWM_CHANNEL = 0;
const int PWM_PIN = 5;
const int PWM_MAX = 255;
const int PWM_CENTER = PWM_MAX / 2; // 50% duty cycle


void setup(void) {
  Serial.begin(9600);


  ledcSetup(PWM_CHANNEL, 5000, 8); // 5 kHz, 8-bit resolution
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  while (!Serial); // Wait for Serial

  // Initialize the sensor
  if (!lsm.begin()) {
    while (1);
  }

  filter.begin(100);

  // Create FreeRTOS tasks
  xTaskCreate(sensorTask, "SensorTask", 2048, NULL, 1, &sensorTaskHandle);
  xTaskCreate(controlTask, "ControlTask", 2048, NULL, 1, &controlTaskHandle);
  xTaskCreate(pwmTask, "PWMTask", 2048, NULL, 1, &pwmTaskHandle);

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();
}

void loop(void) {
  // Empty loop as FreeRTOS tasks handle the functionality
}