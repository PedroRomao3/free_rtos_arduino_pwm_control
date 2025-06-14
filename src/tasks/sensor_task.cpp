#include <Arduino.h>
#include "sensor_task.h"




void sensorTask(void *pvParameters) {
    sensors_event_t accel, mag, gyro, temp;


    while (true) {
        lsm.getEvent(&accel, &mag, &gyro, &temp);

        // Send data to control task (e.g., using a queue or global variable)

        filter.update(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
                      accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                      mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 ms
        //print pitch 
        // Serial.print("Pitch: "); Serial.println(filter.getPitch());
    }
}

void initSensor() {
    if (!lsm.begin()) {
        while (1); // Handle sensor initialization failure
    }
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}