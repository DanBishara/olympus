/*
* File: accelerometer.cpp
* Author: Daniel Bishara
* Date: October 13, 2025
* Description: define class methods for the imu class
*/

#include <zephyr/drivers/sensor.h>

#include "accelerometer.h"

static const struct device *imu = DEVICE_DT_GET(DT_NODELABEL(accelerometer));