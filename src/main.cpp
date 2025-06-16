#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_AHRS.h>
#include "sensor_task.h"
#include "control_task.h"
#include "pwm_task.h"

// Initialize LSM9DS0 sensor (I2C, ID #1000)
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);
Adafruit_NXPSensorFusion filter;

SemaphoreHandle_t filterMutex = NULL;
SemaphoreHandle_t controlOutputMutex = NULL;

// Task handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t pwmTaskHandle = NULL;

void setup(void) {
  Serial.begin(9600);
  
  // Wait for Serial with timeout to prevent hanging
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 3000));
  
  Serial.println("Starting FreeRTOS Arduino PWM Control...");
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize the sensor
  if (!lsm.begin()) {
    Serial.println("Failed to initialize LSM9DS0 sensor!");
    while (1);
  }
  
  // Configure sensor
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  
  // Initialize filter
  filter.begin(100);

  // Create mutexes
  filterMutex = xSemaphoreCreateMutex();
  controlOutputMutex = xSemaphoreCreateMutex();
  
  if (filterMutex == NULL || controlOutputMutex == NULL) {
    Serial.println("Failed to create mutexes!");
    while (1);
  }

  Serial.println("Creating FreeRTOS tasks...");

  // Create FreeRTOS tasks with larger stack sizes
  BaseType_t result1 = xTaskCreate(sensorTask, "SensorTask", 8192, NULL, 1, &sensorTaskHandle);
  BaseType_t result2 = xTaskCreate(controlTask, "ControlTask", 8192, NULL, 1, &controlTaskHandle);
  BaseType_t result3 = xTaskCreate(pwmTask, "PWMTask", 8192, NULL, 1, &pwmTaskHandle);
  
  if (result1 != pdPASS || result2 != pdPASS || result3 != pdPASS) {
    Serial.println("Failed to create one or more tasks!");
    while (1);
  }

  Serial.println("All tasks created successfully");
  
  // The scheduler is already running in Arduino ESP32, no need to call vTaskStartScheduler()
}

void loop(void) {
}