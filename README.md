![SparkFun Capacitive Soil Moisture Sensor (CY8CMBR3102) Arduino Library](docs/images/Qwiic_Soil_Moisture-ArduinoBanner.png "SparkFun Capacitive Soil Moisture Sensor (CY8CMBR3102) Arduino Library]")

# SparkFun Capacitive Soil Moisture Sensor - CY8CMBR3 Arduino Library

![License](https://img.shields.io/github/license/sparkfun/sparkfun_cy8cmbr3_arduino_library)
![Release](https://img.shields.io/github/v/release/sparkfun/SparkFun_CY8CMBR3_Arduino_Library)
![Release Date](https://img.shields.io/github/release-date/sparkfun/SparkFun_CY8CMBR3_Arduino_Library)
![Documentation - build](https://img.shields.io/github/actions/workflow/status/sparkfun/SparkFun_CY8CMBR3_Arduino_Library/build-deploy-ghpages.yml?label=doc%20build)
![Compile - Test](https://img.shields.io/github/actions/workflow/status/sparkfun/SparkFun_CY8CMBR3_Arduino_Library/compile-sketch.yml?label=compile%20test)
![GitHub issues](https://img.shields.io/github/issues/sparkfun/SparkFun_CY8CMBR3_Arduino_Library)

This library provides full access to the functions of the CY8CMBR3102 through an I2C connection using the SparkFun Qwiic connectors and cables. This allows for reading the raw capacitance measured by the SparkFun Capacitive Soil Moisture Sensor. Capacitance increases with soil moisture, so this can be used to measure how moist soil is. 

### Supported Products
This library is intended for use with the following SparkFun Product - available at [www.sparkfun.com](https://www.sparkfun.com). 

| Product | Description|
|--|--|
TODO: update with actual description and link.
|[SparkFun Capacitive Soil Moisture Sensor - CY8CMBR3102 (Qwiic)](https://www.sparkfun.com/products/30480) | SparkFun Qwiic Capacitive Soil Moisture Sensor enables users to sense soil moisture via a capacitive plate in the blade and the embedded CY8CMBR3102 capacitive touch sensor. It features a convenient ruler to allow precise soil moisture measurements. Unlike resistive soil moisture sensors, it does not have exposed metal leads which can suffer from rapid corrosion. If you were so inclined, you could even use this sensor as a touch or proximity sensor since touching the measuruing plate leads to a large increase in measured capacitance. 

## Documentation
TODO: Update links
|Reference | Description |
|---|---|
|[Library Documentation](https://docs.sparkfun.com/SparkFun_CY8CMBR3_Arduino_Library/classsf_dev_c_y8_c_m_b_r3.html)| The full documentation and API for this Arduino library|
|[SparkFun Capacitive Soil Moisture Sensor - CY8CMBR3102 (Qwiic)](https://github.com/sparkfun/SparkFun_Spectral_Sensor_Breakout_AS7343_Qwiic/)| Hardware GitHub Repository|
|[Hook Up Guide - Capacitive Soil Moisture Sensor - CY8CMBR3102 (Qwiic)](https://docs.sparkfun.com/SparkFun_Spectral_Sensor_Breakout_AS7343_Qwiic/) | Hardware Overview and Quick Start for the SparkFun  Capacitive Soil Moisture Sensor - CY8CMBR3102 (Qwiic)|
|[CY8CMBR3102 Datasheet](https://www.infineon.com/assets/row/public/documents/30/49/infineon-cy8cmbr3002-cy8cmbr3102-cy8cmbr3106s-cy8cmbr3108-cy8cmbr3110-cy8cmbr3116-datasheet-en.pdf) | Datasheet for the CY8CMBR3102 IC|
[CY8CMBR3102 Technical Reference Manual](https://www.infineon.com/assets/row/public/documents/30/57/infineon-cy8cmbr3xxx-capsense-express-controllers-registers-trm-additionaltechnicalinformation-en.pdf) | Hardware Overview and Quick Start for the SparkFun  Capacitive Soil Moisture Sensor - CY8CMBR3102 (Qwiic)|
|[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)| Basic information on how to install an Arduino library|

## Examples

The following examples are provided with the library

| Example | Description |
|---|---|
|[Basic Readings](examples/Example_01_BasicUsage/Example_01_BasicUsage.ino)| Take basic readings from the sensor (capacitance in pF) and print them to the terminal.|
|[Advanced](examples/Example_02_Advance/Example_02_Advanced.ino)| Adjust parameters and read multiple forms of counts from the sensor included base counts, diff counts, and raw counts.|
|[LED](examples/Example_03_LED/Example_03_LED.ino)| Demonstrates how to toggle the LED and use it as an indicator for when soil moisture goes below a threshold |
|[I2C Address Setting](examples/Example_04_I2CAddress/Example_04_I2CAddress.ino)| Shows how to set up a threshold and trigger an interrupt when the light reading crosses that threshold.|
|[Multiple Sensors](examples/Example_05_MultipleSensors/Example_05_MultipleSensors.ino)| Demonstrates how to setup and use multiple sensors at once. It requires previously setting the I2C address of two sensors with Example 4 |
|[Set Range](examples/Example_06_SetRange/Example_06_SetRange.ino)| Shows how to set the sensor's raw count values to an appropriate range.


## License Information

This product is ***open source***!

This product is licensed using the [MIT Open Source License](https://opensource.org/license/mit).

[def]: examples