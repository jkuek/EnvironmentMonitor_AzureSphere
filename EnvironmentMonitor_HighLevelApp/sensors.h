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

struct Lps22hhData
{
	float pressure_hPa;
	float temperature_degC;
};
