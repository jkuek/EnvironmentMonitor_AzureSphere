#pragma once


#include "bme680.h"
#include "bsec_datatypes.h"


struct bme680_output
{
	/*! Contains new_data, gasm_valid & heat_stab */
	uint8_t status;

#ifndef BME680_FLOAT_POINT_COMPENSATION
	/*! Temperature in degree celsius x100 */
	int16_t temperature;
	/*! Pressure in Pascal */
	uint32_t pressure;
	/*! Humidity in % relative humidity x1000 */
	uint32_t humidity;
	/*! Gas resistance in Ohms */
	uint32_t gas_resistance;
#else
	/*! Temperature in degree celsius */
	float temperature;
	/*! Pressure in Pascal */
	float pressure;
	/*! Humidity in % relative humidity x1000 */
	float humidity;
	/*! Gas resistance in Ohms */
	float gas_resistance;

#endif
	int64_t timestamp_ns;
};

extern struct bme680_output bme680_raw_output;

extern bsec_bme_settings_t sensor_settings;

void InitBme680(void);
bool ReadBme680(struct bme680_output* b);

void HandleBme680(void);