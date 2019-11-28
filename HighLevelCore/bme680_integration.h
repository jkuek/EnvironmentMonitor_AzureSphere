#pragma once


#include "bme680.h"
#include "bsec_datatypes.h"


struct bme680_output
{
	/*! Contains new_data, gasm_valid & heat_stab */
	uint8_t status;

	/*! Temperature in degree celsius */
	float temperature;
	/*! Pressure in Pascal */
	float pressure;
	/*! Humidity in % relative humidity */
	float humidity;
	/*! Gas resistance in Ohms */
	float gas_resistance;

	int64_t timestamp_ns;
};

extern struct bme680_output bme680_raw_output;

extern bsec_bme_settings_t sensor_settings;

void InitBme680(void);
bool ReadBme680(struct bme680_output* b);

void HandleBme680(void);