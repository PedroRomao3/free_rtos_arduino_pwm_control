#include "sensor_config.h"
#include <Adafruit_LSM9DS0.h>

extern Adafruit_LSM9DS0 lsm;

void configureSensor() {
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G); // ±2g
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS); // ±4 gauss
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS); // ±245 DPS
}

void calibrateGyro(float &gyroXOffset, float &gyroYOffset, float &gyroZOffset) {
    // Implement calibration logic here
    // This could involve averaging multiple readings over a period of time
}