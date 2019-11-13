#  Copyright (c) Microsoft Corporation. All rights reserved.
#  Licensed under the MIT License.

cmake_minimum_required (VERSION 3.8)
project (EnvironmentMonitor_AzureSphere_HighLevelCore C)


add_subdirectory(drivers/BME680)

# Create executable
add_executable(${PROJECT_NAME} main.c epoll_timerfd_utilities.c parson.c azure_iot_utilities.c device_twin.c i2c.c lps22hh_reg.c lsm6dso_reg.c oled.c sd1306.c)
target_include_directories(${PROJECT_NAME} PUBLIC ${AZURE_SPHERE_API_SET_DIR}/usr/include/azureiot ${PROJECT_SOURCE_DIR}/Drivers ${PROJECT_SOURCE_DIR}/Drivers/BME680)
target_compile_definitions(${PROJECT_NAME} PUBLIC AZURE_IOT_HUB_CONFIGURED)
target_link_libraries(${PROJECT_NAME} m azureiot applibs pthread gcc_s c)
target_link_libraries(${PROJECT_NAME} bme680)



# Add MakeImage post-build command
include (${AZURE_SPHERE_MAKE_IMAGE_FILE})

