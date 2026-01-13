/*
* File: accelerometer.cpp
* Author: Daniel Bishara
* Date: October 13, 2025
* Description: define class methods for the imu class
* Datasheet: https://www.st.com/resource/en/datasheet/lsm6dsox.pdf
*/

#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "LED.h"

LOG_MODULE_REGISTER( LED, LOG_LEVEL_INF );

#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define DEBUG_LED DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DEBUG_LED, gpios);