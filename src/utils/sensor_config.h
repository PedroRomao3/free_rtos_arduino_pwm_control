#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <Adafruit_LSM9DS0.h>

void configureSensor(Adafruit_LSM9DS0 &lsm);
void calibrateGyro(Adafruit_LSM9DS0 &lsm, float &gyroXOffset, float &gyroYOffset, float &gyroZOffset);

#endif // SENSOR_CONFIG_H