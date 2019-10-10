#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>


#include "applibs_versions.h"

#include <applibs/log.h>
#include <applibs/i2c.h>

#include "i2c.h"
#include "BME680/bme680.h"
#include "BSEC/bsec_interface.h"
#include "BSEC/bsec_datatypes.h"
#include "BSEC/bsec_serialized_configurations_iaq.h"


//used by BME680 driver
static struct bme680_dev bme680_device;


static void user_delay_ms(uint32_t period)
{
	/*
	 * Return control or wait,
	 * for a period amount of milliseconds
	 */

	struct timespec ts;
	ts.tv_sec = 0;
	ts.tv_nsec = period * 1000;
	nanosleep(&ts, NULL);
}


static int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

	//NOTE: the dev_id parameter contains the I2C address 

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("I2C read reg %0x len %d\n", reg_addr, len);
#endif

#if 0
	I2CMaster_WriteThenRead(i2cFd, dev_id, &reg_addr, 1, reg_data, len);
#else
	// Set the register address to read
	int32_t retVal = I2CMaster_Write(i2cFd, dev_id, &reg_addr, 1);
	if (retVal < 0) {
		Log_Debug("ERROR: I2C write: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	uint16_t i = 0;

	/*
		there seems to be a bug where bad data is returned when reading more than 8 bytes at a time
		so read in blocks of 8 bytes for now
	*/

	for (i = 0; i < len; i += 8)
	{
		uint16_t readLength = len - i;
		if (readLength > 8)
		{
			readLength = 8;
		}

		retVal = I2CMaster_Read(i2cFd, dev_id, &reg_data[i], readLength);
		if (retVal < 0) {
			Log_Debug("ERROR: I2C read: errno=%d (%s)\n", errno, strerror(errno));
			return -1;
		}
	}

#endif

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("Read returned: ");
	for (int i = 0; i < len; i++) {
		Log_Debug("%0x: ", bufp[i]);
	}
	Log_Debug("\n\n");
#endif 	   

	return 0;


	return rslt;
}


static int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

	//NOTE: the dev_id parameter contains the I2C address 

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("I2C read reg %0x len %d\n", reg_addr, len);
	Log_Debug("buffer contents: ");
	for (int i = 0; i < len; i++) {

		Log_Debug("%0x: ", reg_data[i]);
	}
	Log_Debug("\n");
#endif 

	// Construct a new command buffer that contains the register to write to, then the data to write
	uint8_t cmdBuffer[len + 1];
	cmdBuffer[0] = reg_addr;
	for (int i = 0; i < len; i++) {
		cmdBuffer[i + 1] = reg_data[i];
	}

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("cmdBuffer contents: ");
	for (int i = 0; i < len + 1; i++) {

		Log_Debug("%0x: ", cmdBuffer[i]);
	}
	Log_Debug("\n");
#endif

	// Write the data to the device
	int32_t retVal = I2CMaster_Write(i2cFd, dev_id, cmdBuffer, (size_t)len + 1);
	if (retVal < 0) {
		Log_Debug("ERROR: I2C write: errno=%d (%s)\n", errno, strerror(errno));
		rslt = -1;
	}
#ifdef ENABLE_READ_WRITE_DEBUG
	else
	{
		Log_Debug("Wrote %d bytes to device.\n\n", retVal);
	}
#endif

	return rslt;
}




int64_t get_timestamp_ns()
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC_RAW, &t);	//get current time

	return (t.tv_sec * 1000000000) + (t.tv_nsec);
}

int64_t timestamp_ns;

void StartBsecMeasurement()
{
	bsec_bme_settings_t sensor_settings;
	int8_t rslt;

	/* record the current time */
	timestamp_ns = get_timestamp_ns();

	/* Get sensor settings */
	bsec_sensor_control(timestamp_ns, &sensor_settings);

	/* Configure BME680 sensor */
	if (sensor_settings.trigger_measurement)
	{
		//ready to trigger a measurement
		//Ref: BSEC integration guide 5.1.1

		uint8_t set_required_settings;

		/* Set the temperature, pressure and humidity settings */
		bme680_device.tph_sett.os_hum = BME680_OS_2X;
		bme680_device.tph_sett.os_pres = BME680_OS_4X;
		bme680_device.tph_sett.os_temp = BME680_OS_8X;
		bme680_device.tph_sett.filter = BME680_FILTER_SIZE_3;

		/* Set the remaining gas sensor settings and link the heating profile */
		bme680_device.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
		/* Create a ramp heat waveform in 3 steps */
		bme680_device.gas_sett.heatr_temp = 320; /* degree Celsius */
		bme680_device.gas_sett.heatr_dur = 150; /* milliseconds */

		/* Select the power mode */
		/* Must be set before writing the sensor configuration */
		bme680_device.power_mode = BME680_FORCED_MODE;

		/* Set the required sensor settings needed */
		set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL
			| BME680_GAS_SENSOR_SEL;

		/* Set the desired sensor configuration */
		rslt = bme680_set_sensor_settings(set_required_settings, &bme680_device);

		/* Set the power mode */
		rslt = bme680_set_sensor_mode(&bme680_device);

		/* Wait for measurement to finish (about 0.19 seconds for LP mode) */
		uint16_t meas_period;
		bme680_get_profile_dur(&meas_period, &bme680_device);
		/* Delay till the measurement is ready */
		user_delay_ms(meas_period); 

	}
}

static bool ReadDataFromBme680(struct bme680_field_data * data)
{

	return false;
}

static void ProcessBsecOutput(bsec_output_t* outputs, uint8_t* n_outputs)
{
	for (int i = 0; i < n_outputs; i++)
	{
		switch (outputs[i].sensor_id)
		{
			case BSEC_OUTPUT_IAQ:
//				Log_Debug("IAQ = %\n", outputs[);
				// Retrieve the IAQ results from outputs[i].signal
				// and do something with the data
				break;
			case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
				Log_Debug("Temperature [compensated] = %.2f\n", outputs[i].signal);
				break;
			case BSEC_OUTPUT_RAW_TEMPERATURE:
				Log_Debug("Temperature [raw] = %.2f\n", outputs[i].signal);
				break;
			default:
				Log_Debug("ERROR: Unhandled sensor type %u\n", outputs[i].sensor_id);
				break;

		}
	}

}

static void ProcessBsec(bsec_bme_settings_t * settings)
{
	struct bme680_field_data raw_data;

	uint8_t rslt = bme680_get_sensor_data(&raw_data, &bme680_device);

	if (rslt != BME680_OK)
	{
		return false;
	}

	if (!bme680_device.new_fields)
	{
		//no new data, but no error, so return true
		return true;
	}


	// Allocate input and output memory
	bsec_input_t inputs[4]; //up to 4 inputs
	uint8_t input_count = 0; //also used as index
	
	bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
	uint8_t  output_count = 0;

	/* find out which fields need to be processed */
	uint8_t process_data = settings->process_data;

	if (process_data & BSEC_PROCESS_TEMPERATURE)
	{
		inputs[input_count].sensor_id = BSEC_INPUT_TEMPERATURE;
		inputs[input_count].time_stamp = timestamp_ns;
#ifdef BME680_FLOAT_POINT_COMPENSATION
		inputs[input_count].signal = raw_data.temperature;
#else
		inputs[input_count].signal = raw_data.temperature / 100.0f;
#endif
		input_count++;
	}

	if (process_data & BSEC_PROCESS_HUMIDITY)
	{
		inputs[input_count].sensor_id = BSEC_INPUT_HUMIDITY;
		inputs[input_count].time_stamp = timestamp_ns;
#ifdef BME680_FLOAT_POINT_COMPENSATION
		inputs[input_count].signal = raw_data.humidity;
#else
		inputs[input_count].signal = raw_data.humidity / 1000.0f;
#endif
		input_count++;
	}

	if (process_data & BSEC_PROCESS_PRESSURE)
	{
		inputs[input_count].sensor_id = BSEC_INPUT_PRESSURE;
		inputs[input_count].time_stamp = timestamp_ns;
#ifdef BME680_FLOAT_POINT_COMPENSATION
		inputs[input_count].signal = raw_data.pressure;
#else
		inputs[input_count].signal = raw_data.pressure / 100.0f;
#endif
		input_count++;
	}

	if (process_data & BSEC_PROCESS_GAS)
	{
		inputs[input_count].sensor_id = BSEC_INPUT_GASRESISTOR;
		inputs[input_count].time_stamp = timestamp_ns;
		inputs[input_count].signal = raw_data.gas_resistance;
		input_count++;
	}


	bsec_library_return_t status;


	// Invoke main processing BSEC function
	status = bsec_do_steps(inputs, input_count, outputs, &output_count);

	// Iterate through the BSEC output data, if the call succeeded
	if (status == BSEC_OK)
	{
		ProcessBsecOutput(outputs, output_count);
	}
}

#if 0
void bsec_iot_loop(sleep_fct sleep, get_timestamp_us_fct get_timestamp_us, output_ready_fct output_ready,
	state_save_fct state_save, uint32_t save_intvl)
{
	/* Timestamp variables */
	int64_t time_stamp = 0;
	int64_t time_stamp_interval_ms = 0;

	/* Allocate enough memory for up to BSEC_MAX_PHYSICAL_SENSOR physical inputs*/
	bsec_input_t bsec_inputs[BSEC_MAX_PHYSICAL_SENSOR];

	/* Number of inputs to BSEC */
	uint8_t num_bsec_inputs = 0;

	/* BSEC sensor settings struct */
	bsec_bme_settings_t sensor_settings;

	/* Save state variables */
	uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE];
	uint8_t work_buffer[BSEC_MAX_STATE_BLOB_SIZE];
	uint32_t bsec_state_len = 0;
	uint32_t n_samples = 0;

	bsec_library_return_t bsec_status = BSEC_OK;

	while (1)
	{
		/* get the timestamp in nanoseconds before calling bsec_sensor_control() */
		time_stamp = get_timestamp_us() * 1000;

		/* Retrieve sensor settings to be used in this time instant by calling bsec_sensor_control */
		bsec_sensor_control(time_stamp, &sensor_settings);

		/* Trigger a measurement if necessary */
		bme680_bsec_trigger_measurement(&sensor_settings, sleep);

		/* Read data from last measurement */
		num_bsec_inputs = 0;
		bme680_bsec_read_data(time_stamp, bsec_inputs, &num_bsec_inputs, sensor_settings.process_data);

		/* Time to invoke BSEC to perform the actual processing */
		bme680_bsec_process_data(bsec_inputs, num_bsec_inputs, output_ready);

		/* Increment sample counter */
		n_samples++;

		/* Retrieve and store state if the passed save_intvl */
		if (n_samples >= save_intvl)
		{
			bsec_status = bsec_get_state(0, bsec_state, sizeof(bsec_state), work_buffer, sizeof(work_buffer), &bsec_state_len);
			if (bsec_status == BSEC_OK)
			{
				state_save(bsec_state, bsec_state_len);
			}
			n_samples = 0;
		}


		/* Compute how long we can sleep until we need to call bsec_sensor_control() next */
		/* Time_stamp is converted from microseconds to nanoseconds first and then the difference to milliseconds */
		time_stamp_interval_ms = (sensor_settings.next_call - get_timestamp_us() * 1000) / 1000000;
		if (time_stamp_interval_ms > 0)
		{
			sleep((uint32_t)time_stamp_interval_ms);
		}
	}
}

#endif

bool InitBME680(void)
{
	int8_t bme680_result; //used to check return values of BME680 driver calls
	bsec_library_return_t bsec_result; //used to check return values of BSEC library calls

	//working buffer needed for BSEC library
	uint8_t work_buffer[BSEC_MAX_PROPERTY_BLOB_SIZE];
	uint32_t n_work_buffer = BSEC_MAX_PROPERTY_BLOB_SIZE;


	// I2C configuration
	bme680_device.dev_id = BME680_I2C_ADDR_SECONDARY;
	bme680_device.intf = BME680_I2C_INTF;
	bme680_device.read = user_i2c_read;
	bme680_device.write = user_i2c_write;
	bme680_device.delay_ms = user_delay_ms;

	//TODO: not sure if we need to init amb_temp
	/* amb_temp can be set to 25 prior to configuring the gas sensor
	 * or by performing a few temperature readings without operating the gas sensor.
	 */
	//gas_sensor.amb_temp = 25;

	bme680_result = bme680_init(&bme680_device);

	if (bme680_result != BME680_OK)
	{
		Log_Debug("Error while initialising BME680 driver");
		return false;
	}
	
	bsec_result = bsec_init();
	if (bsec_result != BSEC_OK)
	{
		Log_Debug("Error while initialising BSEC library");
		return false;
	}


	/* 
		Configure the BSEC library
		The default configuration is "generic_18v_300s_4d"
		I will use "generic_33v_3s_4d", since we are running from 3.3V and I want data every 3s with a 4-day calibration interval
	*/
	bsec_result = bsec_set_configuration(bsec_config_iaq, sizeof(bsec_config_iaq), work_buffer, sizeof(work_buffer));
	if (bsec_result != BSEC_OK)
	{
		Log_Debug("bsec_set_configuration() error");
		return false;
	}

	// TODO: load saved state
	/*
	bsec_result = bsec_set_state(bsec_state, bsec_state_len, work_buffer, sizeof(work_buffer));
	if (bsec_result != BSEC_OK)
	{
		return false;
	}
	*/

	{
		// Set up which processed outputs we want from BSEC
		bsec_sensor_configuration_t requested_virtual_sensors[8];
		uint8_t n_requested_virtual_sensors = 8;

		requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
		requested_virtual_sensors[0].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
		requested_virtual_sensors[1].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
		requested_virtual_sensors[2].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
		requested_virtual_sensors[3].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_RAW_GAS;
		requested_virtual_sensors[4].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
		requested_virtual_sensors[5].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
		requested_virtual_sensors[6].sample_rate = BSEC_SAMPLE_RATE_LP;
		requested_virtual_sensors[7].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
		requested_virtual_sensors[7].sample_rate = BSEC_SAMPLE_RATE_LP;

		// Allocate a struct for the returned physical sensor settings
		bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
		uint8_t  n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

		// Call bsec_update_subscription() to enable/disable the requested virtual sensors
		bsec_result = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings, &n_required_sensor_settings);

		/* NOTE: according to the integration guide 4.1.2.10, 
			the required_sensor_settings are "purely informational" so no need to use them for anything
		*/

		if (bsec_result != BSEC_OK)
		{
			Log_Debug("bsec_update_subscription() error");
			return false;
		}
	}


	return true;
}


bool ReadBME680(struct Bme680Data* bd)
{
	int8_t rslt = BME680_OK;

	struct bme680_field_data data;
	rslt = bme680_get_sensor_data(&data, &bme680_device);

	if (rslt != BME680_OK)
	{
		return false;
	}


	if (bme680_device.new_fields)
	{

	}


	bd->temperature = data.temperature / 100.0f;
	bd->humidity = data.humidity / 1000.0f;
	bd->pressure = data.pressure / 100.0f;

	Log_Debug("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
		data.pressure / 100.0f, data.humidity / 1000.0f);

	/* Avoid using measurements from an unstable heating setup */
	if (data.status & BME680_GASM_VALID_MSK)
		Log_Debug(", G: %d ohms", data.gas_resistance);

	Log_Debug("\n");

	/* Trigger the next measurement if you would like to read data out continuously */
	if (bme680_device.power_mode == BME680_FORCED_MODE) {
		rslt = bme680_set_sensor_mode(&bme680_device);
	}

	return (rslt == BME680_OK);
}


#if 0

return_values_init bsec_iot_init(float sample_rate, float temperature_offset, bme680_com_fptr_t bus_write,
	bme680_com_fptr_t bus_read, sleep_fct sleep, state_load_fct state_load, config_load_fct config_load)
{
	return_values_init ret = { BME680_OK, BSEC_OK };
	bsec_library_return_t bsec_status = BSEC_OK;

	uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE] = { 0 };
	uint8_t bsec_config[BSEC_MAX_PROPERTY_BLOB_SIZE] = { 0 };
	uint8_t work_buffer[BSEC_MAX_PROPERTY_BLOB_SIZE] = { 0 };
	int bsec_state_len, bsec_config_len;

	/* Fixed I2C configuration */
	bme680_g.dev_id = BME680_I2C_ADDR_PRIMARY;
	bme680_g.intf = BME680_I2C_INTF;
	/* User configurable I2C configuration */
	bme680_g.write = bus_write;
	bme680_g.read = bus_read;
	bme680_g.delay_ms = sleep;

	/* Initialize BME680 API */
	ret.bme680_status = bme680_init(&bme680_g);
	if (ret.bme680_status != BME680_OK)
	{
		return ret;
	}

	/* Initialize BSEC library */
	ret.bsec_status = bsec_init();
	if (ret.bsec_status != BSEC_OK)
	{
		return ret;
	}

	/* Load library config, if available */
	bsec_config_len = config_load(bsec_config, sizeof(bsec_config));
	if (bsec_config_len != 0)
	{
		ret.bsec_status = bsec_set_configuration(bsec_config, bsec_config_len, work_buffer, sizeof(work_buffer));
		if (ret.bsec_status != BSEC_OK)
		{
			return ret;
		}
	}

	/* Load previous library state, if available */
	bsec_state_len = state_load(bsec_state, sizeof(bsec_state));
	if (bsec_state_len != 0)
	{
		ret.bsec_status = bsec_set_state(bsec_state, bsec_state_len, work_buffer, sizeof(work_buffer));
		if (ret.bsec_status != BSEC_OK)
		{
			return ret;
		}
	}

	/* Set temperature offset */
	bme680_temperature_offset_g = temperature_offset;

	/* Call to the function which sets the library with subscription information */
	ret.bsec_status = bme680_bsec_update_subscription(sample_rate);
	if (ret.bsec_status != BSEC_OK)
	{
		return ret;
	}

	return ret;
}

#endif 