/*
  Using the CY8CMBR3 Sensor.

  This example shows how to setup the CY8CMBR3 sensor and use its LED.

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
// Capacitance threshold for LED control. If the capacitance is below this value, turn on the LED.
uint8_t threshold = 50; 

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("CY8CMBR3 Example 3 - LED"); 
    
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
}

void loop()
{   
    // Read capacitance in pF
    uint8_t capacitance = mySensor.readCapacitancePF();

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

    if (capacitance < threshold)
    {
        mySensor.ledOn();
        Serial.println("LED ON - Soil moisture is below threshold.");
    }
    else 
    {
        mySensor.ledOff();
        Serial.println("LED OFF - Soil moisture is above threshold.");
    }

    delay(1000);
}