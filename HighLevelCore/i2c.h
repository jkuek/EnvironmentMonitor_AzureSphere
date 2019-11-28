#pragma once

#include <stdbool.h>
#include "epoll_timerfd_utilities.h"
#include "sensors.h"

#define LSM6DSO_ID         0x6C   // register value
#define LSM6DSO_ADDRESS	   0x6A	  // I2C Address

int initI2c(void);
void closeI2c(void);

bool GetNewAccelerometerData(struct AccelerometerData* ad);
bool GetNewGyroscopeData(struct GyroscopeData* ad);

bool GetNewTemperatureData(float* temperature);
bool GetLps22hhData(struct Lps22hhData* s);

// Export to use I2C in other file
extern int i2cFd;