#pragma once

struct AccelerometerData
{
	float x;
	float y;
	float z;
};

struct GyroscopeData
{
	float x;
	float y;
	float z;
};


struct Lsm6dsoData
{
	struct AccelerometerData accelerometer;
	struct GyroscopeData gyroscope;
	float temperature;
};

struct Lps22hhData
{
	float pressure_hPa;
	float temperature_degC;
};

extern struct Lsm6dsoData lsm6dso_sensors;
extern struct Lps22hhData lps22hh_sensors;
extern float altitude;