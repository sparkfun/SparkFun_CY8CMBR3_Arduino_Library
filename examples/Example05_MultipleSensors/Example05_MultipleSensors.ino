/*
  Using the CY8CMBR3 Sensor.

  This example shows how to setup 4 CY8CMBR3 sensors on the same I2C bus
  with different I2C addresses and read capacitance data from each.

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

// We'll use four sensors on the same I2C bus with different addresses
SfeCY8CMBR3ArdI2C mySensors[4];
const uint8_t sensorAddresses[4] = {0x10, 0x20, 0x30, 0x40}; // Example I2C addresses for the sensors

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("CY8CMBR3 Example 5 - Multiple Sensors"); 
    
    // Start the underlying Arduino I2C bus
    Wire.begin();

    for (int i = 0 ; i < 4; i++)
    {
        // Initialize each CY8CMBR3 sensor
        mySensors[i].begin(sensorAddresses[i]);

        // Initialize each as a moisture sensor (with default settings)
        // Note that we need to call setI2CAddress for each before interfacing with them
        // each time since they share the same bus
        if (!mySensors[i].setI2CAddress(sensorAddresses[i]))
        {
            Serial.print("Failed to set I2C address for sensor ");
            Serial.print(i);
            Serial.println(". Halting...");
            while (1); // Enter infinite loop if we reach this failure
        }
        if (!mySensors[i].defaultMoistureSensorInit())
        {
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.println(" failed to initialize. Please check your wiring!");
            Serial.println("Halting...");
            while (1); // Enter infinite loop if we reach this failure
        }
    }
}

void loop()
{   
    // Read capacitance in pF from each sensor 
    // Note we need to set the I2C address for each before reading
    for (int i = 0 ; i < 4; i++){
        if (!mySensors[i].setI2CAddress(sensorAddresses[i]))
        {
            Serial.print("Failed to set I2C address for sensor ");
            Serial.print(i);
            Serial.println(" before reading capacitance.");
        }
    }
    for (int i = 0 ; i < 4; i++)
    {
        uint8_t capacitance = mySensors[i].readCapacitancePF();
        if (capacitance == 0)
        {
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.println(" failed to read capacitance.");
        }
        else
        {
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.print(" Capacitance: ");
            Serial.print(capacitance);
            Serial.println(" pF");
        }
    }
    delay(1000);
}