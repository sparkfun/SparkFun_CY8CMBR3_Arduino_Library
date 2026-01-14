/*
  Using the CY8CMBR3 Sensor.

  This example shows how to setup the CY8CMBR3 sensor and change 
  its I2C address.

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
uint8_t address = kCY8CMBR3DefaultAddr; // Change this to your new address if you have modified it
uint8_t newAddress = 0x50; // Example new address to set (addresses from 0x08 to 0x77 are valid)

void setup()
{
    delay(1000);
    // Start serial
    Serial.begin(115200);
    Serial.println("CY8CMBR3 Example 4 - I2C Address"); 
    
    // Start the underlying Arduino I2C bus
    Wire.begin();

    // Initialize CY8CMBR3 sensor, note how we pass in the old address
    mySensor.begin(address);

    // Change the I2C address
    if (mySensor.setI2CAddress(newAddress))
    {
        Serial.print("I2C address changed successfully to 0x");
        Serial.println(newAddress, HEX);
    }
    else
    {
        Serial.println("Failed to change I2C address.");
        Serial.println("Make sure the old address is correct and the new address is valid (0x08 to 0x77).");
    }

    delay(1000);

    // Initialize as a moisture sensor (with default settings)
    if (!mySensor.defaultMoistureSensorInit())
    {
        Serial.println("Sensor failed to initialize. Please check your wiring!");
        Serial.println("Halting...");
        while (1); // Enter infinite loop if we reach this failure
    }
}

void loop()
{   
    // Read capacitance in pF
    uint8_t capacitance = mySensor.readCapacitancePF();
    
    // Wet soil has a higher dialectric constant as it can hold more charge
    // thus it will yield a higher capacitance reading (in pF) than dry soil.
    // For more on capacitance and soil moisture sensing, see:
    // https://metergroup.com/measurement-insights/soil-moisture-sensors-how-they-work-why-some-are-not-research-grade/
    if (capacitance == 0)
    {
        Serial.println("Failed to read capacitance.");
    }
    else
    {
        Serial.print("Capacitance: ");
        Serial.print(capacitance);
        Serial.println(" pF");
    }
    delay(1000);
}