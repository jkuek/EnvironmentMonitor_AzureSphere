# BSEC code goes here
The Bosch Sensortec Environmental Cluster (BSEC) library takes in the sensor values from the BME680 driver to create compensated readings.

The output sensors from this library include:
* indoor air quality (IAQ) based on the relative change in total volatile organic compounds (TVOC)
* temperature and humidity values compensated for self-heating
* pressure


Unfortunately the BSEC library libalgobsec.a is under a licence, so I can't include it here. You can download the package for free from [here](https://www.bosch-sensortec.com/bst/products/all_products/BSEC).

In order to get this project compiling, you'll need to copy the .a file from the 'algo/normal_version/bin/gcc/Cortex_A7' folder. I used BSEC version 1.4.7.4 