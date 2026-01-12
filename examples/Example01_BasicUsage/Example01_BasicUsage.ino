/*
  Using the CY8CMBR3 Sensor.

  This example shows how to setup the CY8CMBR3 sensor with default settings and
  use it for basic capacitance readings from a moisture sensor.

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
    Serial.println("CY8CMBR3 Example 1 - Basic Readings");
    
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
        while (1){
          Serial.println("Halted after sensor failure...");
          delay(2000);
        } // Enter infinite loop if we reach this failure
    }

    // TODO: maybe put these two into the same command and maybe also combine into the init function...
    // Save the config
    // if (!mySensor.saveConfig()) {
    //   Serial.println("Saving Config Failed!");
    //   while(1){}
    // }

    // if (!mySensor.reset()){
    //   Serial.println("Reset Failed!");
    //   while(1){}
    // }
}

void loop()
{   
    // Initialize as a moisture sensor (with default settings)
    if (!mySensor.defaultMoistureSensorInit())
    {
        Serial.println("Sensor failed to initialize. Please check your wiring!");
        Serial.println("Halting...");
        while (1){
          Serial.println("Halted after sensor failure...");
          delay(2000);
        } // Enter infinite loop if we reach this failure
    }
    // Read capacitance in pF
    // TODO: assess whether this value, or base + diff counts is more useful (balance of range vs resolution)
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
    // if (!mySensor.reset()) {
    //   Serial.println("Reset Failed!");
    // }
    
    delay(5000);
}