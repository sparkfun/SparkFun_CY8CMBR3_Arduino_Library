/*
  Using the CY8CMBR3 Sensor.

  This example shows how to perform advanced setup the CY8CMBR3 sensor and
  use it for various capacitance readings from a moisture sensor.

  SparkFun Electronics
  Date: 2025/12
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> CY8CMBR3
  QWIIC --> QWIIC

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/TODO
*/
#include <SparkFun_CY8CMBR3.h>

SfeCY8CMBR3ArdI2C mySensor;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("CY8CMBR3 Example 2 - Advanced Readings");
    
    // Start the underlying Arduino I2C bus
    Wire.begin();

    // Initialize CY8CMBR3 sensor
    mySensor.begin();

    // Initialize as a moisture sensor (with default settings)
    if (!mySensor.defaultMoistureSensorInit())
    {
        Serial.println("Sensor failed to initialize. Please check your wiring!");
        Serial.println("Halting...");
        while (1); // Enter infinite loop if we reach this failure
    }

    // Set sensitivity
    // Other options are:
    // CS_SENSITIVITY_250_COUNTS_PER_PF
    // CS_SENSITIVITY_167_COUNTS_PER_PF
    // CS_SENSITIVITY_125_COUNTS_PER_PF
    mySensor.setSensitivity(CS_SENSITIVITY_500_COUNTS_PER_PF); 

    // Set refresh interval
    // Options range from REFRESH_INTERVAL_20MS to REFRESH_INTERVAL_500MS
    // in 20ms increments. See sfe_cy8cmbr3_refresh_interval_t enum in sfDevCY8CMBR3.h for all options.
    mySensor.setRefreshInterval(REFRESH_INTERVAL_100MS);
}

void loop()
{   
    // Depending on your application, you may want to tune sensitity and refresh interval
    // and you may find any of the following different ways of reading the capacitance counts
    // most useful (balancing accuracy, range, and resolution).

    // Read capacitance in pF
    uint8_t capacitancePF = mySensor.readCapacitancePF();
    if (capacitancePF == 0)
    {
        Serial.println("Failed to read capacitance.");
    }
    else
    {
        Serial.print("Capacitance: ");
        Serial.print(capacitancePF);
        Serial.println(" pF");
    }
    
    // Read raw count data
    uint8_t rawCounts = mySensor.readRawCounts();
    if (rawCounts == 0)
    {
        Serial.println("Failed to read raw counts.");
    }
    else
    {
        Serial.print("Raw Counts: ");
        Serial.println(rawCounts);
    }

    // Since most capaictive sensors are used with touch sensors to detect when 
    // a button or pad is being touched, they often use a baseline and differential 
    // count method to improve sensitivity and noise immunity.

    // The baseline count is the "no touch" reading and the differential count is the
    // difference between the current reading and the baseline.
    // For moisture sensing however, we probably care more about absolute capacitance
    // this can be read from raw counts or pF directly as shown above.
    
    // However, we can also use baseline to get the general absolute capacitance level
    // and differential counts to see small changes in capacitance. This would 
    // allow for higher resolution readings within a smaller range while still 
    // being able to detect larger changes by looking at the baseline.

    // Read base count data
    uint8_t baseCounts = mySensor.readBaseCounts();
    if (baseCounts == 0)
    {
        Serial.println("Failed to read base counts.");
    }
    else
    {
        Serial.print("Base Counts: ");
        Serial.println(baseCounts);
    }

    // Read differential count data
    uint8_t diffCounts = mySensor.readDiffCounts();
    if (diffCounts == 0)
    {
        Serial.println("Failed to read differential counts.");
    }
    else
    {
        Serial.print("Differential Counts: ");
        Serial.println(diffCounts);
    }
}