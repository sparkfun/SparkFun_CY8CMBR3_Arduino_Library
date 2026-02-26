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
  https://www.sparkfun.com/products/30480
*/
#include <SparkFun_CY8CMBR3.h>

SfeCY8CMBR3ArdI2C mySensor;

void setup()
{
    // Start serial
    Serial.begin(115200);

    delay(1000);

    Serial.println("CY8CMBR3 Example 2 - Advanced Readings");
    
    // Start the underlying Arduino I2C bus
    Wire.begin();

    // Initialize CY8CMBR3 sensor
    if (!mySensor.begin())
    {
        Serial.println("Sensor failed to begin. Please check your wiring!");
        Serial.println("Halting...");
        while (1); // Enter infinite loop if we reach this failure
    }

    // Set sensitivity
    // Other options are:
    // CS_SENSITIVITY_250_COUNTS_PER_PF
    // CS_SENSITIVITY_167_COUNTS_PER_PF
    // CS_SENSITIVITY_125_COUNTS_PER_PF
    if (!mySensor.setSensitivity(CS_SENSITIVITY_500_COUNTS_PER_PF)){
      Serial.println("Failed to set sensitivity...");
      while(1){}
    }

    // Set refresh interval
    // Options range from REFRESH_INTERVAL_20MS to REFRESH_INTERVAL_500MS
    // in 20ms increments. See sfe_cy8cmbr3_refresh_interval_t enum in sfDevCY8CMBR3.h for all options.
    if (!mySensor.setRefreshInterval(REFRESH_INTERVAL_20MS)){
      Serial.println("Failed to set refresh interval...");
      while(1){}
    }

    if (!mySensor.saveConfig()){
        Serial.println("Failed to save configuration. Please check your wiring!");
        Serial.println("Halting...");
        while (1); // Enter infinite loop if we reach this failure
    }

    if (!mySensor.reset()){
        Serial.println("Failed to reset device after saving configuration. Please check your wiring!");
        Serial.println("Halting...");
        while (1); // Enter infinite loop if we reach this failure
    }

}

void loop()
{   
    // Read debug capacitance in pF
    uint8_t capacitancePF = mySensor.readCapacitancePF();

    Serial.print("Capacitance(PF): ");
    Serial.println(capacitancePF);

    // Read raw count data 
    // The range of this is set up automatically by the "SmartSense" algorithm in the CY8CMBR3 
    // based on capacitance it senses at start up. So, for best results, ensure the sensor is in
    // the wettest condition you expect when you power on so it can set the range accordingly.
    uint16_t rawCounts = mySensor.readRawCount();
    Serial.print("Raw Counts: ");
    Serial.println(rawCounts);

    // Since most capacitive sensors are used with touch sensors to detect when 
    // a button or pad is being touched, they often use a baseline and differential 
    // count method to improve sensitivity and noise immunity.

    // The baseline count is the "no touch" reading and the differential count is the
    // difference between the current reading and the baseline. If the differential 
    // count exceeds a certain threshold, the baseline is adjusted 

    // For moisture sensing however, we probably care more about absolute capacitance
    // this can be read from raw counts or pF directly as shown above.
    
    // However, we can also use baseline to get the general absolute capacitance level
    // and differential counts to see small changes in capacitance. This would 
    // allow for higher resolution readings within a smaller range while still 
    // being able to detect larger changes by looking at the baseline.
    // See the "SetRange" example for a more useful way to use raw counts.

    // Read base count data
    uint16_t baseCounts = mySensor.readBaselineCount();

    Serial.print("Base Counts: ");
    Serial.println(baseCounts);

    // Read differential count data
    uint16_t diffCounts = mySensor.readDifferenceCount();

    Serial.print("Differential Counts: ");
    Serial.println(diffCounts);

    // Read debug differential count data
    diffCounts = mySensor.readDebugDifferenceCount();

    Serial.print("Debug Differential Counts: ");
    Serial.println(diffCounts);

    Serial.println();
    Serial.println("----------------------------------");
    Serial.println();

    delay(1000);
}