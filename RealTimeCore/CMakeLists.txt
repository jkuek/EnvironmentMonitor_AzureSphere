#  Copyright (c) Microsoft Corporation. All rights reserved.
#  Licensed under the MIT License.

cmake_minimum_required (VERSION 3.8)
project (EnvironmentMonitor_AzureSphere_RealTimeCore C)

# Create executable
add_executable (${PROJECT_NAME} main.c mt3620-intercore.c mt3620-timer-poll.c mt3620-uart-poll.c mt3620-adc.c bsec_serialized_configurations_iaq.c)
target_link_libraries (${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/libalgobsec.a -lm)
set_target_properties (${PROJECT_NAME} PROPERTIES LINK_DEPENDS ${CMAKE_SOURCE_DIR}/linker.ld)

# Add MakeImage post-build command
include ("${AZURE_SPHERE_MAKE_IMAGE_FILE}")
