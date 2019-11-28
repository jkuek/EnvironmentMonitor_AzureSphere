#include <errno.h>
#include <string.h>

#include <applibs/log.h>
#include <applibs/i2c.h>
#include "i2c.h"
#include "drivers/BME680/bme680.h"
#include "bsec_datatypes.h"
#include "bme680_integration.h"

//used by BME680 driver
static struct bme680_dev bme680_device;

struct bme680_output bme680_raw_output;

bsec_bme_settings_t sensor_settings;
int64_t timestamp_ns;

int64_t get_timestamp_ns()
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC_RAW, &t);	//get current time

	return ((int64_t)t.tv_sec * 1000000000) + (t.tv_nsec);
}


void user_delay_ms(uint32_t period)
{
	/*
	 * Return control or wait,
	 * for a period amount of milliseconds
	 */

	//convert period (ms) to sec/nanosec
	struct timespec ts;
	ts.tv_sec = (time_t)period / 1000;
	ts.tv_nsec = 1000000 * (period % 1000);
	nanosleep(&ts, NULL);
}


int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
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
		there is a bug where bad data is returned when reading more than 8 bytes at a time
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


int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
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


//bool ReadBme680(struct Bme680Data* bd)
//{
//	int8_t rslt = BME680_OK;
//	/* Get the total measurement duration so as to sleep or wait till the measurement is complete */
//	uint16_t meas_period;
//	bme680_get_profile_dur(&meas_period, &bme680_device);
//
//	struct bme680_field_data data;
//
//	user_delay_ms(meas_period); /* Delay till the measurement is ready */
//
//	rslt = bme680_get_sensor_data(&data, &bme680_device);
//
//	if (rslt != BME680_OK)
//	{
//		return false;
//	}
//
//	bd->temperature = data.temperature / 100.0f;
//	bd->humidity = data.humidity / 1000.0f;
//	bd->pressure = data.pressure / 100.0f;
//
//	Log_Debug("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
//		data.pressure / 100.0f, data.humidity / 1000.0f);
//
//	/* Avoid using measurements from an unstable heating setup */
//	if (data.status & BME680_GASM_VALID_MSK)
//		Log_Debug(", G: %d ohms", data.gas_resistance);
//
//	Log_Debug("\n");
//
//	/* Trigger the next measurement if you would like to read data out continuously */
//	if (bme680_device.power_mode == BME680_FORCED_MODE) {
//		rslt = bme680_set_sensor_mode(&bme680_device);
//	}
//
//	return (rslt == BME680_OK);
//}


static void bme680_bsec_trigger_measurement(bsec_bme_settings_t* sensor_settings)
{
	uint16_t meas_period;
	uint8_t set_required_settings;
	int8_t bme680_status = BME680_OK;

	/* Check if a forced-mode measurement should be triggered now */
	if (sensor_settings->trigger_measurement)
	{
		/* Set sensor configuration */

		bme680_device.tph_sett.os_hum = sensor_settings->humidity_oversampling;
		bme680_device.tph_sett.os_pres = sensor_settings->pressure_oversampling;
		bme680_device.tph_sett.os_temp = sensor_settings->temperature_oversampling;
		bme680_device.gas_sett.run_gas = sensor_settings->run_gas;
		bme680_device.gas_sett.heatr_temp = sensor_settings->heater_temperature; /* degree Celsius */
		bme680_device.gas_sett.heatr_dur = sensor_settings->heating_duration; /* milliseconds */

		/* Select the power mode */
		/* Must be set before writing the sensor configuration */
		bme680_device.power_mode = BME680_FORCED_MODE;
		/* Set the required sensor settings needed */
		set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_GAS_SENSOR_SEL;

		/* Set the desired sensor configuration */
		bme680_status = bme680_set_sensor_settings(set_required_settings, &bme680_device);

		/* record the time of measurement */
		timestamp_ns = get_timestamp_ns();

		/* Set power mode as forced mode and trigger forced mode measurement */
		bme680_status = bme680_set_sensor_mode(&bme680_device);

		/* Get the total measurement duration so as to sleep or wait till the measurement is complete */
		bme680_get_profile_dur(&meas_period, &bme680_device);

		/* Delay till the measurement is ready. Timestamp resolution in ms */
		user_delay_ms((uint32_t)meas_period);
	}

	/* Call the API to get current operation mode of the sensor */
	bme680_status = bme680_get_sensor_mode(&bme680_device);
	/* When the measurement is completed and data is ready for reading, the sensor must be in BME680_SLEEP_MODE.
	 * Read operation mode to check whether measurement is completely done and wait until the sensor is no more
	 * in BME680_FORCED_MODE. */
	while (bme680_device.power_mode == BME680_FORCED_MODE)
	{
		/* sleep for 5 ms */
		user_delay_ms(5);
		bme680_status = bme680_get_sensor_mode(&bme680_device);
	}
}

static bool bme680_bsec_read_data(struct bme680_output* b)
{
	static struct bme680_field_data data;

	int8_t result = bme680_get_sensor_data(&data, &bme680_device);

	if (result == BME680_OK)
	{
#ifndef BME680_FLOAT_POINT_COMPENSATION
		b->temperature = data.temperature / 100.0f;
		b->humidity = data.humidity / 1000.0f;
		b->pressure = data.pressure / 100.0f;
		b->gas_resistance = data.gas_resistance;
#else
		b->temperature = data.temperature;
		b->humidity = data.humidity;
		b->pressure = data.pressure;
		b->gas_resistance = data.gas_resistance;
#endif

		b->timestamp_ns = timestamp_ns;
		b->status = data.status;
	}
	else
	{
		Log_Debug("Error %d reading from bme680", result);
	}

	return (result == BME680_OK);
}




void InitBme680(void)
{
	bme680_device.dev_id = BME680_I2C_ADDR_SECONDARY;
	bme680_device.intf = BME680_I2C_INTF;
	bme680_device.read = user_i2c_read;
	bme680_device.write = user_i2c_write;
	bme680_device.delay_ms = user_delay_ms;
	/* amb_temp can be set to 25 prior to configuring the gas sensor
	 * or by performing a few temperature readings without operating the gas sensor.
	 */
	bme680_device.amb_temp = 25;

	int8_t result = bme680_init(&bme680_device);

	if (result != BME680_OK)
	{
		Log_Debug("Error while initialising BME680 driver");
		//return false;
	}

	sensor_settings.next_call = 0;

	/* Set the temperature, pressure and humidity settings */
	sensor_settings.humidity_oversampling = BME680_OS_2X;
	sensor_settings.pressure_oversampling = BME680_OS_4X;
	sensor_settings.temperature_oversampling = BME680_OS_8X;

	/* Set the remaining gas sensor settings and link the heating profile */
	sensor_settings.run_gas = BME680_ENABLE_GAS_MEAS;
	/* Create a ramp heat waveform in 3 steps */
	sensor_settings.heater_temperature = 320; /* degree Celsius */
	sensor_settings.heating_duration = 150; /* milliseconds */

	sensor_settings.trigger_measurement = 1; // trigger initial measurement
	bme680_bsec_trigger_measurement(&sensor_settings);
}

void HandleBme680(void)
{
	/* Trigger a measurement if necessary */
	bme680_bsec_trigger_measurement(&sensor_settings);

	/* We only have to read data if the previous call to bsec_sensor_control() actually asked for it */
	if (sensor_settings.process_data)
	{
		bme680_bsec_read_data(&bme680_raw_output);

		//data is now in bme680_raw_output
	}

}


bool ReadBme680(struct bme680_output * bd)
{
	bool result = false;

	/* Trigger a measurement if necessary */
	bme680_bsec_trigger_measurement(&sensor_settings);

	/* We only have to read data if the previous call to bsec_sensor_control() actually asked for it */
	//if (sensor_settings.process_data)
	{
		result = bme680_bsec_read_data(bd);
	}

	return result;
}