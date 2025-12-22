/**
 * @file SparkFun_CY8CMBR3.h
 * @brief Arduino-specific implementation for the SparkFun CY8CMBR3 Sensor.
 *
 * @details
 * This file provides the Arduino-specific implementation of the CY8CMBR3 driver 
 * class. The SfeCY8CMBR3ArdI2C` class inherits from the base driver class and 
 * implements the I2C communication interface using Arduino's Wire library.
 *
 * Key features:
 * - Arduino I2C initialization
 * - Connection verification
 * - Toolkit integration
 *
 * @section Class SfeCY8CMBR3ArdI2C Class
 * - begin(): Initializes I2C communication
 * - isConnected(): Verifies sensor connection
 *
 * @section Dependencies
 * - Arduino.h
 * - SparkFun_Toolkit.h
 * - sfCY8CMBR3.h
 *
 * @author SparkFun Electronics
 * @date 2025
 * @copyright Copyright 2025, SparkFun Electronics Inc. All rights reserved.
 *
 * @section License License
 * SPDX-License-Identifier: MIT
 *
 * @section Product_Links Product Links
 * - Qwiic 1x1: https://www.sparkfun.com/products/TODO
 *
 * @see https://github.com/sparkfun/SparkFun_CY8CMBR3_Arduino_Library
 */

 #pragma once

 // clang-format off
 #include <SparkFun_Toolkit.h>
 #include "sfTk/sfDevCY8CMBR3.h"
 #include <Arduino.h>
 // clang-format on
 
/**
 * @class SfeCY8CMBR3ArdI2C
 * @brief Arduino I2C implementation for the CY8CMBR3 sensor.
 *
 * @details
 * This class provides Arduino-specific I2C communication implementation for the CY8CMBR3 sensor.
 * It inherits from the base driver class and implements the I2C interface using Arduino's Wire library.
 * The class manages device addressing and connection verification.
 *
 * Example usage:
 * @code
 * SfeCY8CMBR3ArdI2C sensor;
 * if (sensor.begin()) {
 *     // Sensor initialized successfully
 * }
 * @endcode
 *
 * @note This class uses the Arduino Wire library for I2C communication
 *
 * @see sfDevCY8CMBR3
 * @see TwoWire
 *
 */
class SfeCY8CMBR3ArdI2C : public sfDevCY8CMBR3
{
  public:
    SfeCY8CMBR3ArdI2C()
    {
    }

    /**
     * @brief Initializes the CY8CMBR3 sensor with I2C communication.
     *
     * @details
     * This method performs the following initialization steps:
     * 1. Initializes I2C communication with device address and Wire port
     * 2. Enables repeat start support
     * 3. Sets up communication bus
     * 4. Verifies device connection
     * 5. Calls base class initialization
     *
     * @param address I2C address of the device (default: kDefaultCY8CMBR3Addr)
     * @param wirePort TwoWire instance to use for I2C communication (default: Wire)
     *
     * @return true If initialization successful
     * @return false If any initialization step fails
     *
     * Example:
     * @code
     * SfeCY8CMBR3ArdI2C sensor;
     * if (!sensor.begin()) {
     *     Serial.println("Sensor initialization failed!");
     *     while (1); // halt
     * }
     * @endcode
     */
    bool begin(const uint8_t &address = kCY8CMBR3Addr, TwoWire &wirePort = Wire)
    {
        if (_theI2CBus.init(wirePort, address) != ksfTkErrOk)
            return false;

        setCommunicationBus(&_theI2CBus);

        // Perform the necessary checks to set up the device

        if (!isConnected())
            return false;
        return true;
    }

    /**
     * @brief Checks if the CY8CMBR3 sensor is connected and responding.
     *
     * @details
     * This method performs two checks:
     * 1. Attempts to ping the device at the current I2C address
     * 2. Verifies the device ID matches the expected CY8CMBR3 ID
     *
     * @return true If device responds to ping and returns correct device ID
     * @return false If communication fails or device ID is incorrect
     *
     * Example:
     * @code
     * SfeCY8CMBR3ArdI2C sensor;
     * if (!sensor.isConnected()) {
     *     Serial.println("Device not found or incorrect ID!");
     *     return;
     * }
     * @endcode
     */
    bool isConnected(void)
    {
        if (_theI2CBus.ping() != ksfTkErrOk)
            return false;

        // For now, we only support the CY8CMBR3102 device ussed in our soil moisture sensor
        // if more devices are supported in the future, this check may need to be updated
        // Check the device ID
        
        return (kDefaultCY8CMBR3102DeviceID == getDeviceID()) && (kDefaultCY8CMBR3102FamilyID == getFamilyID());

    }

    /**
     * @brief Gets the currently configured I2C address of the CY8CMBR3 sensor.
     *
     * @details
     * Returns the I2C address currently being used to communicate with the sensor.
     *
     * @return uint8_t The current I2C address
     *
     * Example:
     * @code
     * SfeCY8CMBR3ArdI2C sensor;
     * uint8_t address = sensor.getDeviceAddress();
     * Serial.print("Current I2C address: 0x");
     * Serial.println(address, HEX);
     * @endcode
     */
    uint8_t getDeviceAddress(void)
    {   
        return _theI2CBus.address();
    }

  private:
    /**
     * @brief Arduino I2C bus interface instance for the CY8CMBR3 sensor.
     *
     * @details
     * This member handles the low-level I2C communication between the Arduino and the CY8CMBR3 sensor.
     *
     * The bus interface is configured during begin() and used by all communication methods.
     *
     * @see sfTkArdI2C
     * @see begin()
     */
    sfTkArdI2C _theI2CBus;
};