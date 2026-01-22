/*
  Using the CY8CMBR3 Sensor.

  This example shows how to perform calibration of the CY8CMBR3102 moisture sensor 
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

    Serial.println("CY8CMBR3 Example 7 - Calibration Advanced");
    
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
    Serial.println("Press any key to bring up calibration menu...");

    // TODO: Maybe spit out finger threshold, noise threshold, negative noise threshold, hysteresis and LBR here
    // and then user can save them and manually set them later and override them.
    // Could even make a menu with:
       // 1 - Recalibrate
       // 2 - Save current calibration to non-volatile memory
       // 3 - Display current calibration values
    
    // If user has sent us anything, recalibrate...
    if (Serial.available()){

        while (Serial.available()){
            Serial.read(); // Clear the input buffer
        }

        bool calibrationRunning = true;
        while (calibrationRunning){
            Serial.println();
            Serial.println("=== CY8CMBR3 Calibration Menu ===");
      
            Serial.read(); // Clear the input buffer
            Serial.println("Press the corresponding key for the desired action:");
            Serial.println("Calibration Options:");
            Serial.println("1 - Recalibrate sensor");
            Serial.println("2 - Save current calibration to non-volatile memory");
            Serial.println("3 - Display current calibration values");
            Serial.println("4 - Manually set calibration values");
            Serial.println("5 - Set calibration based on count value");
            Serial.println("6 - Exit");
            while (!Serial.available()){
                // Wait for user input
            }
            char userInput = Serial.read();
            while (Serial.available()){
                Serial.read(); // Clear the input buffer
            }

            uint8_t baseThreshold = 0;
            uint8_t hysteresis = 0;
            uint8_t lbr = 0;
            uint8_t nnt = 0;
            uint8_t nt = 0;
            bool success = true;
            char recalInput = 0;
            uint8_t countValue = 0;

            switch (userInput){
                case '1':
                    Serial.println("Place the sensor in the wettest desired environment and press \"r\" to recalibrate.");
                    Serial.println("Press any other key to cancel recalibration.");
                    while (!Serial.available()){
                        // Wait for user input
                    }
                    recalInput = Serial.read();
                    while (Serial.available()){
                        Serial.read(); // Clear the input buffer
                    }
                    if (recalInput == 'r'){
                        Serial.println("Recalibrating sensor...");

                        // Turn on automatic calibration features so the reset sets them based on current environment
                        success = true;
                        success &= mySensor.setAutoThresholdEnable(true);
                        success &= mySensor.setHysteresisOverride(false);
                        success &= mySensor.setLowBaselineResetOverride(false);
                        success &= mySensor.setNegativeNoiseThresholdOverride(false);
                        success &= mySensor.setNoiseThresholdOverride(false);
                        success &= mySensor.saveConfig();
                        if (!success){
                            Serial.println("Failed to set automatic calibration features!");
                        }

                        // Reset the sensor to recalibrate based on current environment
                        if (mySensor.reset()){
                            Serial.println("Recalibration successful!");
                        } else {
                            Serial.println("Recalibration failed!");
                        }
                    } else {
                        Serial.println("Recalibration cancelled.");
                    }
                    break;
                case '2':
                    Serial.println("Saving current calibration to non-volatile memory...");

                    // Disable automatic calibration features so the saved values are used on next boot
                    success = true;
                    success &= mySensor.setAutoThresholdEnable(true);
                    success &= mySensor.setHysteresisOverride(true);
                    success &= mySensor.setLowBaselineResetOverride(true);
                    success &= mySensor.setNegativeNoiseThresholdOverride(true);
                    success &= mySensor.setNoiseThresholdOverride(true);
                    if (!success){
                        Serial.println("Failed to set calibration overrides!");
                    }

                    if (mySensor.saveConfig()){
                        Serial.println("Calibration saved successfully!");
                    } else {
                        Serial.println("Failed to save calibration!");
                    }
                    // if (mySensor.setAutoResetEnable()){
                    //     Serial.println("Re-init successfully!");
                    // } else {
                    //     Serial.println("Failed to Re-init!");
                    // }
                    if (mySensor.reset()){
                        Serial.println("Reset successful!");
                    } else {
                        Serial.println("Reset failed!");
                    }

                    break;
                case '3':
                    Serial.println();
                    Serial.println("Displaying current calibration values...");

                    baseThreshold = mySensor.getBaseThreshold();
                    hysteresis = mySensor.getHysteresis();
                    lbr = mySensor.getLowBaselineReset();
                    nnt = mySensor.getNegativeNoiseThreshold();
                    nt = mySensor.getNoiseThreshold();

                    Serial.print("Base Threshold: ");
                    Serial.println(baseThreshold);
                    Serial.print("Hysteresis: ");
                    Serial.println(hysteresis);
                    Serial.print("Low Baseline Reset: ");
                    Serial.println(lbr);
                    Serial.print("Negative Noise Threshold: ");
                    Serial.println(nnt);
                    Serial.print("Noise Threshold: ");
                    Serial.println(nt);
                    mySensor.printOverrides();
                    break;
                case '4':
                    Serial.println("Manually setting calibration values...");
                    Serial.println("Enter new calibration values:");
                    Serial.print("Base Threshold (0-255): ");
                    while (!Serial.available()){}
                    baseThreshold = Serial.parseInt();
                    while (Serial.available()){ Serial.read(); } // Clear the input buffer
                    Serial.println("Setting Base Threshold to " + String(baseThreshold));
                    if (!mySensor.setBaseThreshold(baseThreshold)){
                        Serial.println("Failed to set Base Threshold!");
                        break;
                    }
                    Serial.print("Hysteresis (0-31): ");
                    while (!Serial.available()){}
                    hysteresis = Serial.parseInt();
                    while (Serial.available()){ Serial.read(); } // Clear the input buffer
                    Serial.println("Setting Hysteresis to " + String(hysteresis));
                    if (!mySensor.setHysteresis(hysteresis)){
                        Serial.println("Failed to set Hysteresis!");
                        break;
                    }
                    Serial.print("Low Baseline Reset (0-127): ");
                    while (!Serial.available()){}
                    lbr = Serial.parseInt();
                    while (Serial.available()){ Serial.read(); } // Clear the input buffer
                    Serial.println("Setting Low Baseline Reset to " + String(lbr));
                    if (!mySensor.setLowBaselineReset(lbr)){
                        Serial.println("Failed to set Low Baseline Reset!");
                        break;
                    }
                    Serial.print("Negative Noise Threshold (0-127): ");
                    while (!Serial.available()){}
                    nnt = Serial.parseInt();
                    while (Serial.available()){ Serial.read(); } // Clear the input buffer
                    Serial.println("Setting Negative Noise Threshold to " + String(nnt));
                    if (!mySensor.setNegativeNoiseThreshold(nnt)){
                        Serial.println("Failed to set Negative Noise Threshold!");
                        break;
                    }
                    Serial.print("Noise Threshold (0-127): ");
                    while (!Serial.available()){}
                    nt = Serial.parseInt();
                    while (Serial.available()){ Serial.read(); } // Clear the input buffer
                    Serial.println("Setting Noise Threshold to " + String(nt));
                    if (!mySensor.setNoiseThreshold(nt)){
                        Serial.println("Failed to set Noise Threshold!");
                        break;
                    }
                    Serial.println("Calibration values set successfully!");
                    Serial.println();
                    Serial.println("New calibration values:");
                    Serial.print("Base Threshold: ");
                    Serial.println(baseThreshold);
                    Serial.print("Hysteresis: ");
                    Serial.println(hysteresis);
                    Serial.print("Low Baseline Reset: ");
                    Serial.println(lbr);
                    Serial.print("Negative Noise Threshold: ");
                    Serial.println(nnt);
                    Serial.print("Noise Threshold: ");
                    Serial.println(nt);
                    Serial.println();
                    Serial.println("Calibration complete.");
                    Serial.println();
                    Serial.println("----------------------------------");
                    Serial.println();
                    break;
                case '5':
                    Serial.println("Setting calibration based on count value...");
                    Serial.print("Enter count value (0-255): ");
                    while (!Serial.available()){}
                    countValue = Serial.parseInt();
                    while (Serial.available()){ Serial.read(); } // Clear the input buffer
                    Serial.println("Setting calibration based on count value of " + String(countValue));
                    if (mySensor.setCalibrationByCount(countValue)){
                        Serial.println("Calibration set successfully based on count value!");
                    } else {
                        Serial.println("Failed to set calibration based on count value!");
                    }
                    break;
                case '6':
                    calibrationRunning = false;
                    Serial.println("Exiting calibration menu.");
                    break;
                default:
                    Serial.println("Invalid input. Please try again.");
                    break;
            }
        }
    }
    // Read raw count data (if it is getting capped at a maximum value, then consider recalibrating...)
    uint16_t rawCounts = mySensor.readRawCount();
    Serial.print("Raw Counts: ");
    Serial.println(rawCounts);

    // Read debug capacitance in pF
    // 
    // uint8_t capacitancePF = mySensor.readCapacitancePF();
    // {
    //     Serial.print("Capacitance: ");
    //     Serial.print(capacitancePF);
    //     Serial.println(" pF");
    //     // Serial.print("Hex: 0x");
    //     // Serial.println(capacitancePF, HEX);
    // }

    Serial.println();
    Serial.println("----------------------------------");
    Serial.println();

    delay(1000);
}