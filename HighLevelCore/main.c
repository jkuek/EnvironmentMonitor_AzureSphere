﻿/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

   /************************************************************************************************

   This project was originally based on the AvnetStarterKitReferenceDesign project but has been heavily modified.
   Sphere OS: 19.05
   This file contains the 'main' function. Program execution begins and ends there

   Authors:
   Peter Fenn (Avnet Engineering & Technology)
   Brian Willess (Avnet Engineering & Technology)

   Purpose:
   Using the Avnet Azure Sphere Starter Kit demonstrate the following features

   1. Read X,Y,Z accelerometer data from the onboard LSM6DSO device using the I2C Interface
   2. Read X,Y,Z Angular rate data from the onboard LSM6DSO device using the I2C Interface
   3. Read the barometric pressure from the onboard LPS22HH device using the I2C Interface
   4. Read the temperature from the onboard LPS22HH device using the I2C Interface
   5. Read the state of the A and B buttons
   6. Read BSSID address, Wi-Fi AP SSID, Wi-Fi Frequency
   *************************************************************************************************
      Connected application features: When connected to Azure IoT Hub or IoT Central
   *************************************************************************************************
   7. Send X,Y,Z accelerometer data to Azure
   8. Send barometric pressure data to Azure
   9. Send button state data to Azure
   10. Send BSSID address, Wi-Fi AP SSID, Wi-Fi Frequency data to Azure
   11. Send the application version string to Azure
   12. Control user RGB LEDs from the cloud using device twin properties
   13. Control optional Relay Click relays from the cloud using device twin properties
   14. Send Application version up as a device twin property
   TODO
   1. Add support for a OLED display
   2. Add supprt for on-board light sensor
   	 
   *************************************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h> 
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include "epoll_timerfd_utilities.h"
#include "i2c.h"
#include "deviceTwin.h"
#include "azure_iot_utilities.h"
#include "connection_strings.h"
#include "build_options.h"
#include "bme680_integration.h"

#include <applibs/log.h>
#include <applibs/i2c.h>
#include <applibs/gpio.h>
#include <applibs/wificonfig.h>
#include <azureiot/iothub_device_client_ll.h>


//// OLED
#include "oled.h"

//// ADC connection
#include <sys/time.h>
#include <sys/socket.h>
#include <applibs/application.h>


// By default, this sample is targeted at the MT3620 Reference Development Board (RDB).
// This can be changed using the project property "Target Hardware Definition Directory".
// This #include imports the sample_hardware abstraction from that hardware definition.
#include <hw/sample_hardware.h>

#include "bsec_datatypes.h"


struct
{
	int64_t timestamp_ns;
	uint32_t new_data; //will be non-zero if following data is new and valid, else 0

	/*! Temperature in degree celsius x100 */
	int16_t temperature;
	/*! Pressure in Pascal */
	uint32_t pressure;
	/*! Humidity in % relative humidity x1000 */
	uint32_t humidity;
	/*! Gas resistance in Ohms */
	uint32_t gas_resistance;

} outputPayload;

struct real_time_outputs
{
	uint32_t ambient_light;
	uint32_t ambient_noise;

	bsec_output_t iaq;
	bsec_output_t temperature;
	bsec_output_t humidity;
	bsec_output_t rawTemperature;
	bsec_output_t rawHumidity;
	bsec_output_t rawPressure;
	bsec_output_t rawGas;
	bsec_output_t co2Equivalent;

	bsec_bme_settings_t sensor_settings;
} inputPayload;



// Provide local access to variables in other files
extern twin_t twinArray[];
extern int twinArraySize;
extern IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle;

// Support functions.
static void TerminationHandler(int signalNumber);
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);

// File descriptors - initialized to invalid value
int epollFd = -1;
static int buttonPollTimerFd = -1;
static int buttonAGpioFd = -1;
static int buttonBGpioFd = -1;

int userLedRedFd = -1;
int userLedGreenFd = -1;
int userLedBlueFd = -1;
int appLedFd = -1;
int wifiLedFd = -1;
int clickSocket1Relay1Fd = -1;
int clickSocket1Relay2Fd = -1;

//// ADC connection
static const char rtAppComponentId[] = "54e89e6e-13fb-4dca-a067-279e8f060cf7";
static int sockFd = -1;
static void SendMessageToRTCore(void);
static void TimerEventHandler(EventData *eventData);
static void SocketEventHandler(EventData *eventData);
static int timerFd = -1;
uint8_t RTCore_status;

// event handler data structures. Only the event handler field needs to be populated.
static EventData timerEventData = { .eventHandler = &TimerEventHandler };
static EventData socketEventData = { .eventHandler = &SocketEventHandler };
//// end ADC connection


static int readSensorTimerFd = -1;

// Button state variables, initialize them to button not-pressed (High)
static GPIO_Value_Type buttonAState = GPIO_Value_High;
static GPIO_Value_Type buttonBState = GPIO_Value_High;

static uint32_t ambient_noise;

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
	bool versionStringSent = false;
#endif

// Define the Json string format for the accelerator button press data
static const char cstrButtonTelemetryJson[] = "{\"%s\":\"%d\"}";

// Termination state
volatile sig_atomic_t terminationRequired = false;


float altitude;
struct Lsm6dsoData lsm6dso_sensors;
struct Lps22hhData lps22hh_sensors;
//struct Bme680Data bme680_sensors;


/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    terminationRequired = true;
}

void SendTelemetryDataToAzure(void) {
	struct
	{
		double temperature;
		double pressure;
		double humidity;
		double iaq;
		double ambient_light;
	} telemetryData
		= { 1,2,3,4,5 };


	JSON_Value* root_value = json_value_init_object();
	JSON_Object* root_object = json_value_get_object(root_value);
	char* serialized_string = NULL;
	//json_object_set_string(root_object, "name", "John Smith");
	json_object_set_number(root_object, "temperature", telemetryData.temperature);
	json_object_set_number(root_object, "pressure", telemetryData.pressure);
	json_object_set_number(root_object, "humidity", telemetryData.humidity);
	//json_object_set_number(root_object, "iaq", 2);
	if (RTCore_status == 0)
	{
		//we have comms with the M4 app, so these values should be good

		json_object_set_number(root_object, "ambient_light", telemetryData.ambient_light);
	}
	serialized_string = json_serialize_to_string(root_value);

	Log_Debug("Sending JSON string: %s", serialized_string);

	AzureIoT_SendMessage(serialized_string);

	json_free_serialized_string(serialized_string);
	json_value_free(root_value);
}

/// <summary>
///     Handle button timer event: if the button is pressed, report the event to the IoT Hub.
/// </summary>
static void ButtonTimerEventHandler(EventData *eventData)
{

	bool sendTelemetryButtonA = false;
	bool sendTelemetryButtonB = false;

	if (ConsumeTimerFdEvent(buttonPollTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	// Check for button A press
	GPIO_Value_Type newButtonAState;
	int result = GPIO_GetValue(buttonAGpioFd, &newButtonAState);
	if (result != 0) {
		Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	// If the A button has just been pressed, send a telemetry message
	// The button has GPIO_Value_Low when pressed and GPIO_Value_High when released
	if (newButtonAState != buttonAState) {
		if (newButtonAState == GPIO_Value_Low) {
			Log_Debug("Button A pressed!\n");
			sendTelemetryButtonA = true;
			SendTelemetryDataToAzure();
		}
		else {
			Log_Debug("Button A released!\n");
		}
		
		// Update the static variable to use next time we enter this routine
		buttonAState = newButtonAState;
	}

	// Check for button B press
	GPIO_Value_Type newButtonBState;
	result = GPIO_GetValue(buttonBGpioFd, &newButtonBState);
	if (result != 0) {
		Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	// If the B button has just been pressed/released, send a telemetry message
	// The button has GPIO_Value_Low when pressed and GPIO_Value_High when released
	if (newButtonBState != buttonBState) {
		if (newButtonBState == GPIO_Value_Low) {
			// Send Telemetry here
			Log_Debug("Button B pressed!\n");
			sendTelemetryButtonB = true;

			//// OLED

			oled_state++;

			if (oled_state > OLED_NUM_SCREEN)
			{
				oled_state = 0;
			}
		}
		else {
			Log_Debug("Button B released!\n");

		}

		// Update the static variable to use next time we enter this routine
		buttonBState = newButtonBState;


	}
	
	// If either button was pressed, then enter the code to send the telemetry message
	if (sendTelemetryButtonA || sendTelemetryButtonB) {

		char *pjsonBuffer = (char *)malloc(JSON_BUFFER_SIZE);
		if (pjsonBuffer == NULL) {
			Log_Debug("ERROR: not enough memory to send telemetry");
		}

		if (sendTelemetryButtonA) {
			// construct the telemetry message  for Button A
			snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrButtonTelemetryJson, "buttonA", newButtonAState);
			Log_Debug("\n[Info] Sending telemetry %s\n", pjsonBuffer);
			AzureIoT_SendMessage(pjsonBuffer);
		}

		if (sendTelemetryButtonB) {
			// construct the telemetry message for Button B
			snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrButtonTelemetryJson, "buttonB", newButtonBState);
			Log_Debug("\n[Info] Sending telemetry %s\n", pjsonBuffer);
			AzureIoT_SendMessage(pjsonBuffer);
		}

		free(pjsonBuffer);
	}

}

// event handler data structures. Only the event handler field needs to be populated.
static EventData buttonEventData = { .eventHandler = &ButtonTimerEventHandler };

//// ADC connection

/// <summary>
///     Handle socket event by reading incoming data from real-time capable application.
/// </summary>
static void SocketEventHandler(EventData *eventData)
{
	// Read response from real-time capable application.
	char rxBuf[1024];

	int bytesReceived = recv(sockFd, rxBuf, sizeof(rxBuf), 0);

	if (bytesReceived == -1) {
		Log_Debug("ERROR: Unable to receive message: %d (%s)\n", errno, strerror(errno));
		terminationRequired = true;
	}

	if (bytesReceived >= sizeof(struct real_time_outputs))
	{
		struct real_time_outputs * payload = (struct real_time_outputs*)rxBuf;
		// get voltage (2.5*adc_reading/4096)
		// divide by 3650 (3.65 kohm) to get current (A)
		// multiply by 1000000 to get uA
		// divide by 0.1428 to get Lux (based on fluorescent light Fig. 1 datasheet)
		// divide by 0.5 to get Lux (based on incandescent light Fig. 1 datasheet)
		// We can simplify the factors, but for demonstration purpose it's OK
		light_sensor = ((float)payload->ambient_light * 2.5 / 4095) * 1000000 / (3650 * 0.1428);

		ambient_noise = payload->ambient_noise;
		Log_Debug("Audio = %u\n", payload->ambient_noise);
		Log_Debug("BSEC: T = %f, H = %f, P = %f, IAQ = %f\n", payload->temperature.signal, payload->humidity.signal, payload->rawPressure.signal, payload->iaq.signal);
	}


	Log_Debug("Received %d bytes.\n", bytesReceived);
}

/// <summary>
///     Handle send timer event by writing data to the real-time capable application.
/// </summary>
static void TimerEventHandler(EventData *eventData)
{
	if (ConsumeTimerFdEvent(timerFd) != 0) {
		terminationRequired = true;
		return;
	}


	struct bme680_output bme680_data;
	
	ReadBme680(&bme680_data);
	if (bme680_data.status & BME680_NEW_DATA_MSK)
	{
		//send data to BSEC (on RT app) for processing
		outputPayload.timestamp_ns = bme680_data.timestamp_ns;
		outputPayload.temperature = bme680_data.temperature;
		outputPayload.humidity = bme680_data.humidity;
		outputPayload.pressure = bme680_data.pressure;
		outputPayload.gas_resistance = bme680_data.gas_resistance;

		//print raw values for debug
		Log_Debug("BME680: T = %d, H = %d, P = %d, G = %d\n",
			bme680_data.temperature,
			bme680_data.humidity,
			bme680_data.pressure,
			bme680_data.gas_resistance);

		SendMessageToRTCore();
	}

}

/// <summary>
///     Helper function for TimerEventHandler sends message to real-time capable application.
/// </summary>
static void SendMessageToRTCore(void)
{
	//static char txMessage[256];

	//Log_Debug("Sending: %s\n", txMessage);

	//int bytesSent = send(sockFd, txMessage, strlen(txMessage), 0);
	int bytesSent = send(sockFd, (void *)&outputPayload, sizeof(outputPayload), 0);
	if (bytesSent == -1)
	{
		Log_Debug("ERROR: Unable to send message: %d (%s)\n", errno, strerror(errno));
		terminationRequired = true;
		return;
	}
}

//// end ADC connection



float CalculateDewPoint(float temperature, float relative_humidity)
{
	/*  Dew point calculation [Ref: https://www.omnicalculator.com/physics/dew-point#howto]

	The dew point is calculated according to the following formula:

		Ts = (b * α(T,RH)) / (a - α(T,RH))
		s
	where:

		Ts is the dew point;
		T is the temperature;
		RH is the relative humidity of the air;
		a and b are coefficients. For Sonntag90 constant set, a=17.62 and b=243.12°C;
		α(T,RH) = ln(RH/100) +a*T/(b+T).
	*/
	const float a = 17.62;
	const float b = 243.12;

	float alpha = log(relative_humidity / 100.0f) + (a * temperature) / (b + temperature);

	float dew_point = (b * alpha) / (a - alpha);

	return dew_point;
}


/// <summary>
///     Update sensor data
/// </summary>
void ReadSensorTimerEventHandler(EventData* eventData)
{
	// Consume the event to stop it from recurring immediately

	if (ConsumeTimerFdEvent(readSensorTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	/* check each of the sensors in turn
		for each sensor that has new data available, add the key/value to the json object
		finally, send the new data up to the cloud
	*/

	JSON_Value* root_value = json_value_init_object();
	JSON_Object* root_object = json_value_get_object(root_value);
	char* serialized_string = NULL;

	//Read the LSM6DSO sensors
	if (GetNewAccelerometerData(&lsm6dso_sensors.accelerometer))
	{
		Log_Debug("\nLSM6DSO: Acceleration [mg]  : %.4lf, %.4lf, %.4lf\n",
			lsm6dso_sensors.accelerometer.x, lsm6dso_sensors.accelerometer.y, lsm6dso_sensors.accelerometer.z);

		json_object_set_number(root_object, "aX", lsm6dso_sensors.accelerometer.x);
		json_object_set_number(root_object, "aY", lsm6dso_sensors.accelerometer.y);
		json_object_set_number(root_object, "aZ", lsm6dso_sensors.accelerometer.z);
	}

	if (GetNewGyroscopeData(&lsm6dso_sensors.gyroscope))
	{
		Log_Debug("LSM6DSO: Angular rate [dps] : %4.2f, %4.2f, %4.2f\n",
			lsm6dso_sensors.gyroscope.x, lsm6dso_sensors.gyroscope.y, lsm6dso_sensors.gyroscope.z);

		json_object_set_number(root_object, "gX", lsm6dso_sensors.gyroscope.x);
		json_object_set_number(root_object, "gY", lsm6dso_sensors.gyroscope.y);
		json_object_set_number(root_object, "gZ", lsm6dso_sensors.gyroscope.z);
	}

	if (GetNewTemperatureData(&lsm6dso_sensors.temperature))
	{
		Log_Debug("LSM6DSO: Temperature1 [degC]: %.2f\n", lsm6dso_sensors.temperature);
		//This values is unused, instead use temperature from the BME680
	}

	// Read the LPS22HH sensors
	if (GetLps22hhData(&lps22hh_sensors))
	{
		Log_Debug("LPS22HH: Pressure     [hPa] : %.2f\n", lps22hh_sensors.pressure_hPa);
		Log_Debug("LPS22HH: Temperature2 [degC]: %.2f\n", lps22hh_sensors.temperature_degC);
		//These values are unused, instead use temperature and pressure from the BME680
	}

	sensor_data.acceleration_mg[0] = lsm6dso_sensors.accelerometer.x;
	sensor_data.acceleration_mg[1] = lsm6dso_sensors.accelerometer.y;
	sensor_data.acceleration_mg[2] = lsm6dso_sensors.accelerometer.z;
	sensor_data.angular_rate_dps[0] = lsm6dso_sensors.gyroscope.x;
	sensor_data.angular_rate_dps[1] = lsm6dso_sensors.gyroscope.y;
	sensor_data.angular_rate_dps[2] = lsm6dso_sensors.gyroscope.z;
	sensor_data.lsm6dsoTemperature_degC = lsm6dso_sensors.temperature;
	sensor_data.lps22hhpressure_hPa = lps22hh_sensors.pressure_hPa;
	sensor_data.lps22hhTemperature_degC = lps22hh_sensors.temperature_degC;

	/*
	The ALTITUDE value calculated is actually "Pressure Altitude". This lacks correction for temperature (and humidity)
	"pressure altitude" calculator located at: https://www.weather.gov/epz/wxcalc_pressurealtitude
	"pressure altitude" formula is defined at: https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
	 altitude in feet = 145366.45 * (1 - (hPa / 1013.25) ^ 0.190284) feet
	 altitude in meters = 145366.45 * 0.3048 * (1 - (hPa / 1013.25) ^ 0.190284) meters
	*/
	// weather.com formula
	//altitude = 44307.69396 * (1 - powf((atm / 1013.25), 0.190284));  // pressure altitude in meters
	// Bosch's formula
	altitude = 44330 * (1 - powf((lps22hh_sensors.pressure_hPa / 1013.25), 1 / 5.255));  // pressure altitude in meters

	if (RTCore_status == 0)
	{
		//intercore comms is good, so these values should be good
		json_object_set_number(root_object, "ambient_light", light_sensor);
		json_object_set_number(root_object, "ambient_noise", ambient_noise);

		Log_Debug("ALSPT19: Ambient Light[Lux] : %.2f\n", light_sensor);

	}

	//// OLED
	//update_oled();


	//if (ReadBME680(&bme680_sensors))
	//{
	//	json_object_set_number(root_object, "temperature", bme680_sensors.temperature);
	//	json_object_set_number(root_object, "humidity", bme680_sensors.humidity);
	//	json_object_set_number(root_object, "pressure", bme680_sensors.pressure);
	//	//TODO: json_object_set_number(root_object, "iaq", 2);

	//	float dew_point = CalculateDewPoint(bme680_sensors.temperature, bme680_sensors.humidity);

	//	json_object_set_number(root_object, "dew_point", dew_point);
	//}

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
	{

		//// Allocate memory for a telemetry message to Azure
		//char *pjsonBuffer = (char *)malloc(JSON_BUFFER_SIZE);
		//if (pjsonBuffer == NULL) {
		//	Log_Debug("ERROR: not enough memory to send telemetry");
		//}
		//
		//snprintf(pjsonBuffer, JSON_BUFFER_SIZE, "{\"gX\":\"%.2lf\", \"gY\":\"%.2lf\", \"gZ\":\"%.2lf\", \"aX\": \"%.2f\", \"aY\": \"%.2f\", \"aZ\": \"%.2f\", \"pressure\": \"%.2f\", \"light_intensity\": \"%.2f\", \"altitude\": \"%.2f\", \"temp\": \"%.2f\",  \"rssi\": \"%d\"}",
		//	angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2], acceleration_mg[0], acceleration_mg[1], acceleration_mg[2], pressure_hPa, light_sensor, altitude, lsm6dsoTemperature_degC, network_data.rssi);

		//Log_Debug("\n[Info] Sending telemetry: %s\n", pjsonBuffer);
		//
		//AzureIoT_SendMessage(pjsonBuffer);
		//free(pjsonBuffer);

		//create json string from object
		serialized_string = json_serialize_to_string(root_value);

		Log_Debug("Sending JSON string: %s", serialized_string);

		AzureIoT_SendMessage(serialized_string);
	}

#endif 

	//clean up
	json_free_serialized_string(serialized_string);
	json_value_free(root_value);

}



/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    epollFd = CreateEpollFd();
    if (epollFd < 0) {
        return -1;
    }

	//// ADC connection
	 	
	// Open connection to real-time capable application.
	sockFd = Application_Socket(rtAppComponentId);
	if (sockFd == -1) 
	{
		Log_Debug("ERROR: Unable to create socket: %d (%s)\n", errno, strerror(errno));
		Log_Debug("Real Time Core disabled or Component Id is not correct.\n");
		Log_Debug("The program will continue without showing light sensor data.\n");
		// Communication with RT core error
		RTCore_status = 1;
		//return -1;
	}
	else
	{
		// Communication with RT core success
		RTCore_status = 0;
		// Set timeout, to handle case where real-time capable application does not respond.
		static const struct timeval recvTimeout = { .tv_sec = 5,.tv_usec = 0 };
		int result = setsockopt(sockFd, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(recvTimeout));
		if (result == -1)
		{
			Log_Debug("ERROR: Unable to set socket timeout: %d (%s)\n", errno, strerror(errno));
			return -1;
		}

		// Register handler for incoming messages from real-time capable application.
		if (RegisterEventHandlerToEpoll(epollFd, sockFd, &socketEventData, EPOLLIN) != 0)
		{
			return -1;
		}

		// Register one second timer to send a message to the real-time core.
		static const struct timespec sendPeriod = { .tv_sec = 1,.tv_nsec = 0 };
		timerFd = CreateTimerFdAndAddToEpoll(epollFd, &sendPeriod, &timerEventData, EPOLLIN);
		if (timerFd < 0)
		{
			return -1;
		}
		RegisterEventHandlerToEpoll(epollFd, timerFd, &timerEventData, EPOLLIN);
	}

	//// end ADC Connection

	
	if (initI2c() == -1) {
		return -1;
	}

	InitBme680();

	//set up a periodic timer to read the sensors

	// Define the period in the build_options.h file
	struct timespec accelReadPeriod = { .tv_sec = SENSOR_READ_INTERVAL_SECONDS,.tv_nsec = 0 };
	// event handler data structures. Only the event handler field needs to be populated.
	static EventData readSensorEventData = { .eventHandler = &ReadSensorTimerEventHandler };
	readSensorTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &accelReadPeriod, &readSensorEventData, EPOLLIN);
	if (readSensorTimerFd < 0) {
		return -1;
	}


	// Traverse the twin Array and for each GPIO item in the list open the file descriptor
	for (int i = 0; i < twinArraySize; i++) {

		// Verify that this entry is a GPIO entry
		if (twinArray[i].twinGPIO != NO_GPIO_ASSOCIATED_WITH_TWIN) {

			*twinArray[i].twinFd = -1;

			// For each item in the data structure, initialize the file descriptor and open the GPIO for output.  Initilize each GPIO to its specific inactive state.
			*twinArray[i].twinFd = (int)GPIO_OpenAsOutput(twinArray[i].twinGPIO, GPIO_OutputMode_PushPull, twinArray[i].active_high ? GPIO_Value_Low : GPIO_Value_High);

			if (*twinArray[i].twinFd < 0) {
				Log_Debug("ERROR: Could not open LED %d: %s (%d).\n", twinArray[i].twinGPIO, strerror(errno), errno);
				return -1;
			}
		}
	}

	// Open button A GPIO as input
	Log_Debug("Opening Starter Kit Button A as input.\n");
	buttonAGpioFd = GPIO_OpenAsInput(AVNET_MT3620_SK_USER_BUTTON_A);
	if (buttonAGpioFd < 0) {
		Log_Debug("ERROR: Could not open button A GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	// Open button B GPIO as input
	Log_Debug("Opening Starter Kit Button B as input.\n");
	buttonBGpioFd = GPIO_OpenAsInput(AVNET_MT3620_SK_USER_BUTTON_B);
	if (buttonBGpioFd < 0) {
		Log_Debug("ERROR: Could not open button B GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	// Set up a timer to poll the buttons
	
	struct timespec buttonPressCheckPeriod = { 0, 1000000 };
	buttonPollTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &buttonPressCheckPeriod, &buttonEventData, EPOLLIN);
	if (buttonPollTimerFd < 0) {
		return -1;
	}
	
	// Tell the system about the callback function that gets called when we receive a device twin update message from Azure
	AzureIoT_SetDeviceTwinUpdateCallback(&deviceTwinChangedHandler);

    return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    Log_Debug("Closing file descriptors.\n");
    
	closeI2c();
	CloseFdAndPrintError(readSensorTimerFd, "readSensorTimer");
    CloseFdAndPrintError(epollFd, "Epoll");
	CloseFdAndPrintError(buttonPollTimerFd, "buttonPoll");
	CloseFdAndPrintError(buttonAGpioFd, "buttonA");
	CloseFdAndPrintError(buttonBGpioFd, "buttonB");

	// Traverse the twin Array and for each GPIO item in the list the close the file descriptor
	for (int i = 0; i < twinArraySize; i++) {

		// Verify that this entry has an open file descriptor
		if (twinArray[i].twinGPIO != NO_GPIO_ASSOCIATED_WITH_TWIN) {

			CloseFdAndPrintError(*twinArray[i].twinFd, twinArray[i].twinKey);
		}
	}
}

/// <summary>
///     Main entry point for this application.
/// </summary>
int main(int argc, char *argv[])
{
	// Variable to help us send the version string up only once
	bool networkConfigSent = false;
	char ssid[128];
	uint32_t frequency;
	char bssid[20];
	
	// Clear the ssid array
	memset(ssid, 0, 128);

	Log_Debug("Version String: %s\n", argv[1]);
	Log_Debug("Environment Monitor - Azure Sphere starting.\n");
	
    if (InitPeripheralsAndHandlers() != 0) {
        terminationRequired = true;
    }

    // Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
    while (!terminationRequired) {
        if (WaitForEventAndCallHandler(epollFd) != 0) {
            terminationRequired = true;
        }

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
		// Setup the IoT Hub client.
		// Notes:
		// - it is safe to call this function even if the client has already been set up, as in
		//   this case it would have no effect;
		// - a failure to setup the client is a fatal error.
		if (!AzureIoT_SetupClient()) {
			Log_Debug("ERROR: Failed to set up IoT Hub client\n");
			break;
		}
#endif 

		WifiConfig_ConnectedNetwork network;
		int result = WifiConfig_GetCurrentNetwork(&network);

		if (result < 0) 
		{
			// Log_Debug("INFO: Not currently connected to a WiFi network.\n");
			//// OLED
			strncpy(network_data.SSID, "Not Connected", 20);

			network_data.frequency_MHz = 0;

			network_data.rssi = 0;
		}
		else 
		{

			frequency = network.frequencyMHz;
			snprintf(bssid, JSON_BUFFER_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x",
				network.bssid[0], network.bssid[1], network.bssid[2], 
				network.bssid[3], network.bssid[4], network.bssid[5]);

			if ((strncmp(ssid, (char*)&network.ssid, network.ssidLength)!=0) || !networkConfigSent) {
				
				memset(ssid, 0, 128);
				strncpy(ssid, network.ssid, network.ssidLength);
				Log_Debug("SSID: %s\n", ssid);
				Log_Debug("Frequency: %dMHz\n", frequency);
				Log_Debug("bssid: %s\n", bssid);
				networkConfigSent = true;

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
				// Note that we send up this data to Azure if it changes, but the IoT Central Properties elements only 
				// show the data that was current when the device first connected to Azure.
				checkAndUpdateDeviceTwin("wifi_ssid", &ssid, TYPE_STRING, false);
				checkAndUpdateDeviceTwin("wifi_frequency", &frequency, TYPE_INT, false);
				checkAndUpdateDeviceTwin("wifi_bssid", &bssid, TYPE_STRING, false);
#endif 
			}

			//// OLED

			memset(network_data.SSID, 0, WIFICONFIG_SSID_MAX_LENGTH);
			if (network.ssidLength <= SSID_MAX_LEGTH)
			{
				strncpy(network_data.SSID, network.ssid, network.ssidLength);
			}
			else
			{
				strncpy(network_data.SSID, network.ssid, SSID_MAX_LEGTH);
			}

			network_data.frequency_MHz = network.frequencyMHz;

			network_data.rssi = network.signalRssi;
		}	   		 	  	  	   	
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
		if (iothubClientHandle != NULL && !versionStringSent) {

			checkAndUpdateDeviceTwin("versionString", argv[1], TYPE_STRING, false);
			versionStringSent = true;
		}

		// AzureIoT_DoPeriodicTasks() needs to be called frequently in order to keep active
		// the flow of data with the Azure IoT Hub
		AzureIoT_DoPeriodicTasks();
#endif
    }

    ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting.\n");
    return 0;
}
