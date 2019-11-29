This application tends to the analogue sensors, reading values for ambient light and ambient noise.

It also processes the raw BME680 values (which are read in the high level app), sending them to the BSEC library and then dispatching the BSEC outputs to the high level app.

This app requires the BSEC library from Bosch, which can be downloaded from https://www.bosch-sensortec.com/bst/products/all_products/BSEC
At the time of development, the available version is BSEC_1.4.7.4_Generic_Release.
Once you have downloaded the .zip file from the Bosch website, you need to copy the file libalgosec.a from \BSEC_1.4.7.4_Generic_Release\algo\normal_version\bin\gcc\Cortex_M4 into the RealTimeCore folder.