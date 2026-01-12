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

 const uint8_t kMaxConnectAttempts = 10; ///< Maximum number of connection attempts

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
    bool begin(const uint8_t &address = kCY8CMBR3DefaultAddr, TwoWire &wirePort = Wire)
    {
        Serial.println("In SfeCY8CMBR3ArdI2C::begin()...");

        uint8_t connectAttempts = 0;
        
        // Retry calling init on the I2C bus up to kMaxConnectAttempts times
        // since the spec says the device may nack until it's ready
        // If successful break out of loop, otherwise return false after max attempts

        while (connectAttempts < kMaxConnectAttempts){
            Serial.print("Connection attempt ");
            Serial.println(connectAttempts + 1);
            if (_theI2CBus.init(wirePort, address) == ksfTkErrOk){
                Serial.println("I2C bus initialized successfully");
                break; // Success
            }
            connectAttempts++;
        }

        if (connectAttempts == kMaxConnectAttempts){
            Serial.println("Failed to initialize I2C bus after max attempts");
            return false; // Failed after max attempts
        }

        Serial.println("I2C bus initialized");
        setCommunicationBus(&_theI2CBus);

        // Perform the necessary checks to set up the device

        Serial.println("Checking device connection...");
        if (!isConnected()){
            Serial.println("Device connection check failed");
            return false;
        }

        Serial.println("Device connected successfully");
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
        // Since the device may nack until it's ready, we must perform this ping 
        // repeatedly as well

        uint8_t connectAttempts = 0;
        while (connectAttempts < kMaxConnectAttempts){
            Serial.print("Ping attempt ");
            Serial.println(connectAttempts + 1);
            if (_theI2CBus.ping() == ksfTkErrOk){
                Serial.println("Ping successful");
                break; // Success
            }
            connectAttempts++;
        }
        if (connectAttempts == kMaxConnectAttempts){
            Serial.println("Failed to ping device after max attempts");
            return false; // Failed after max attempts
        }

        // For now, we only support the CY8CMBR3102 device ussed in our soil moisture sensor
        // if more devices are supported in the future, this check may need to be updated
        // Check the device ID and family ID
        return (kDefaultCY8CMBR3102DeviceID == getDeviceID()) && (kDefaultCY8CMBR3102FamilyID == getFamilyID());

    }

    /**
     * @brief Sets the I2C address of the CY8CMBR3 sensor.
     *
     * @details
     * This method updates the I2C address used for communication with the sensor.
     *
     * @param i2cAddress The new I2C address to set
     * @return true If address set successfully
     * @return false If setting the address fails
     *
     * Example:
     * @code
     * SfeCY8CMBR3ArdI2C sensor;
     * if (!sensor.setI2CAddress(0x2A)) {
     *     Serial.println("Failed to set I2C address!");
     * }
     * @endcode
     */
    bool setI2CAddress(uint8_t i2cAddress)
    {
        // First set the address on the device itself 
        if (!_setI2CAddress(i2cAddress))
            return false;
        
        // Now set the address on the bus interface so we can communicate at the new address
        _theI2CBus.setAddress(i2cAddress);
        
        return true;
    }

    /**
     * @brief Gets the currently configured I2C address of the CY8CMBR3 sensor.
     *
     * @details
     * Returns the I2C addresses currently being used to communicate with the sensor.
     *
     * @param busAddress Reference to store the read I2C bus address (currently set in the bus interface)
     * @param deviceAddress Reference to store the read device I2C address (from reading sensor register)
     * 
     * @return bool true If address retrieved successfully, false if it fails
     *
     * Example:
     * @code
     * SfeCY8CMBR3ArdI2C sensor;
     * uint8_t busAddress, deviceAddress;
     * if (sensor.getDeviceAddress(busAddress, deviceAddress)) {
     *    Serial.print("Bus Address: 0x");
     *    Serial.println(busAddress, HEX);
     *    Serial.print("Device Address: 0x");
     *    Serial.println(deviceAddress, HEX);
     *    if (busAddress != deviceAddress) {
     *       // If this is the case, call setI2CAddress to fix it
     *       Serial.println("Warning: Bus and Device addresses do not match!");
     *    } 
     * }
     * else {
     *    Serial.println("Failed to get I2C addresses!");
     * }
     * @endcode
     */
    bool getI2CAddress(uint8_t &busAddress, uint8_t &deviceAddress)
    {
        deviceAddress = 0;
        if (!_theI2CBus.address())
            return false;

        busAddress = _theI2CBus.address();
        if (!_readI2CAddress(deviceAddress))
            return false;

        return true;
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