/*
  Using the CY8CMBR3 Sensor.

  This example shows how to set the range of the CY8CMBR3102 moisture sensor 
  to use the higher-resolution (but less objective/universal) rawCounts value 
  to measure the capacitance on the sensor.

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

    delay(1000);

    Serial.println("CY8CMBR3 Example 6 - Set Raw Count Range");
    
    // Start the underlying Arduino I2C bus
    Wire.begin();

    // Initialize CY8CMBR3 sensor
    if (!mySensor.begin())
    {
        Serial.println("Sensor failed to begin. Please check your wiring!");
        Serial.println("Halting...");
        while (1); // Enter infinite loop if we reach this failure
    }

    // Initialize as a moisture sensor (with default settings)
    if (!mySensor.defaultMoistureSensorInit())
    {
        Serial.println("Sensor failed to initialize. Please check your wiring!");
        Serial.println("Halting...");
        while (1); // Enter infinite loop if we reach this failure
    }

    mySensor.ledOff();
}

void loop()
{   
    // Capacitance PF is great, easy to use, and standard/objective accross all soil types, densities, and conditions.
    // However, it has limited resolution (1 count / pF) and range (0-255pF). Depending on your application, you may want to use
    // raw counts instead for higher resolution and range.

    // The sensor will automatically configure the range of its "Raw Count" Readings based on 
    // the capacitance that it senses during power up/initialization. This allows it to maximize
    // resolution for the expected capacitance range. However, if the capacitance changes significantly
    // (for example, a moisture sensor going from dry to very wet), the raw count readings may saturate
    // and get stuck at a maximum value. You can run reset() again to set the raw count range if needed.
    
    // Since we want to explore the entire range of readings up to the wettest soil possible,
    // we should prompt the user to ensure the sensor is in the wettest possible state
    // and then run reset() again to set the raw count range.
    Serial.println("To set the raw count range, ensure the moisture sensor in your target soil type with your target density and at the most watered state possible.");
    Serial.println("Then press any key in the serial monitor to set the sensor's raw count range...");
    Serial.println("This setting will NOT last between power cycles...");

    // If user has sent us anything, reset the sensor to set the raw count range...
    if (Serial.available()){
      Serial.println("User input received!");
      Serial.read(); // clear the input buffer
      Serial.println("Setting sensor's raw count range...");

      if (!mySensor.reset()){
          Serial.println("Failed to reset device for setting raw count range. Please check your wiring!");
          Serial.println("Halting...");
          while (1); // Enter infinite loop if we reach this failure
      }
      else{
        Serial.println("Raw count range set successfully!");
        mySensor.ledOff();
      }
    }
    
    // Read raw count data (if it is getting capped at a maximum value, then consider recalibrating...)
    uint16_t rawCounts = mySensor.readRawCount();
    Serial.print("Raw Counts: ");
    Serial.println(rawCounts);

    // Read debug capacitance in pF
    // 
    uint8_t capacitancePF = mySensor.readCapacitancePF();
    {
        Serial.print("Capacitance: ");
        Serial.print(capacitancePF);
        Serial.println(" pF");
        // Serial.print("Hex: 0x");
        // Serial.println(capacitancePF, HEX);
    }
    
    Serial.println();
    Serial.println("----------------------------------");
    Serial.println();

    delay(1000);
}