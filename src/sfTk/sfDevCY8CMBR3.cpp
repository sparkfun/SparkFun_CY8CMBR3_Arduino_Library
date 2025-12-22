/**
 * @file sfDevCY8CMBR3.cpp
 * @brief Implementation file for the SparkFun CY8CMBR3 Sensor device driver.
 *
 * @details
 * This file implements the sfDevCY8CMBR3 class methods for configuring and reading data from
 * the CY8CMBR3 sensor. The driver provides a comms-agnostic interface using the SparkFun Toolkit.
 * However, note that the only supported communication method for this device is I2C.
 *
 * Key features:
 * - Device initialization and configuration
 * - Sensor readings
 * - Gain settings
 * - Power management
 *
 * @section Implementation Implementation Details
 * - Register read/write operations
 *
 * @author SparkFun Electronics
 * @date 2025
 * @copyright Copyright (c) 2024-2025, SparkFun Electronics Inc. All rights reserved.
 *
 * @section License License
 * SPDX-License-Identifier: MIT
 *
 * @see https://github.com/sparkfun/SparkFun_AS7343_Arduino_Library
 */
#include "sfDevCY8CMBR3.h"

// consts

/*
class sfDevCY8CMBR3
{
  public:
    sfDevCY8CMBR3() : _last_data_pF{0}, _theBus{nullptr}
    {
    }

    /// @brief This method is called to initialize the CY8CMBR3 device through the
    /// specified bus.
    /// @param theBus Pointer to the bus object.
    /// @return True if successful, false if it fails.
    bool begin(sfTkIBus *theBus = nullptr);

    /// @brief Requests the family ID from the sensor.
    /// @return The family ID of the sensor.
    uint8_t getFamilyID(void);

    /// @brief Requests the device ID from the sensor.
    /// @return The device ID of the sensor.
    uint8_t getDeviceID(void);

    /// @brief Sets the communication bus to the specified bus.
    /// @param theBus Bus to set as the communication device.
    void setCommunicationBus(sfTkIBus *theBus);

    /// @brief Set the sensor Id 
    /// @param sensorId The sensor Id to set.
    /// @details This will set the sensor Id (and by extension the debug sensor Id) for debug operations.
    /// @return True if successful, false if it fails.
    bool setSensorId(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Set sensitivity for the specified sensor Id
    /// @details This method sets the sensitivity for the specified sensor by writing to the SENSITIVITY0 register.
    /// @param sensorId The sensor Id to set the sensitivity for.
    /// @param sensitivity The sensitivity value to set (0-3).
    /// @return True if successful, false if it fails.
    bool setSensitivity(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0, sfe_cy8cmbr3_sensitivity_t sensitivity);

    /// @brief Enable or disable sensor by sensor Id
    /// @details This method enables the specified sensor by setting the appropriate bits in the SENSOR_EN register.
    /// @param sensorId The sensor Id to enable or disable.
    /// @param enable True to enable the sensor, false to disable.
    /// @return True if successful, false if it fails.
    bool enable(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0, bool enable = true);

    /// @brief Reads the capacitance value in pF from the sensor.
    /// @details This method reads the capacitance in pF from the DEBUG_CP register.
    /// @param sensorId The sensor Id to read the capacitance from.
    /// @return The capacitance value in pF.
    uint8_t readCapacitancePF(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Reads the difference count value from the sensor.
    /// @details This method reads the difference count from the DEBUG_DIFFERENCE_COUNTx register.
    /// @param sensorId The sensor Id to read the difference count from.
    uint16_t readDifferenceCount(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Reads the baseline count value from the sensor.
    /// @details This method reads the baseline count from the DEBUG_BASELINE_COUNT resister.
    /// @param sensorId The sensor Id to read the baseline count from.
    uint16_t readBaselineCount(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Turn on or off the LED.
    /// @details This method turns on or off the LED by setting or clearing the
    /// LED_ACT bit in the LED register (ksfAS7343RegLed).
    /// @param ledOn True to turn on the LED, false to turn off.
    /// @return True if successful, false if it fails.
    bool ledOn(bool ledOn = true);

    /// @brief Turn off the LED.
    /// @details This method turns off the LED by calling the ledOn method
    /// with false.
    /// @return True if successful, false if it fails.
    bool ledOff(void);

  private:
    sfe_cy8cmbr3_reg_diff_cnt_t _last_data_pF; // Last read data from the sensor.

    sfTkIBus *_theBus; // Pointer to bus device.
};
*/

bool sfDevCY8CMBR3::begin(sfTkIBus *theBus)
{
    // Nullptr check.
    if (!_theBus && !theBus)
        return false;

    // Set the internal bus pointer, overriding current bus if it exists.
    if (theBus != nullptr)
        setCommunicationBus(theBus);

    return true; // Return true to indicate success
}

uint16_t sfDevCY8CMBR3::getDeviceID(void)
{
    sfe_cy8cmbr3_reg_device_id_t devID; // Create a variable to hold the device ID.

    if (!_theBus)
        return 0; // Return 0 to indicate error.
    
    // Read the device ID register. If it errors, then return 0.
    if (ksfTkErrOk != _theBus->readRegister(ksfCY8CMBR3RegDeviceId, devID.word))
        return 0; // Return 0 to indicate error.
    
    // TODO: Depending on endianness, may need to byte-swap here.

    return devID.word; // Return the device ID.
}

uint8_t sfDevCY8CMBR3::getFamilyID(void)
{
    uint8_t familyID = 0; // Create a variable to hold the family ID.

    if (!_theBus)
        return 0; // Return 0 to indicate error.
    
    // Read the family ID register. If it errors, then return 0.
    if (ksfTkErrOk != _theBus->readRegister(ksfCY8CMBR3RegFamilyId, familyID))
        return 0; // Return 0 to indicate error.

    return familyID; // Return the family ID.
}

void sfDevCY8CMBR3::setCommunicationBus(sfTkIBus *theBus)
{
    _theBus = theBus;
}

bool sfDevCY8CMBR3::_setI2CAddress(uint8_t i2cAddress){
    // Ensure valid inputs
    if ( (!_theBus) || (i2cAddress < kCY8CMBRMinAddr) || (i2cAddress > kCY8CMBRMaxAddr) )
        return false;

    sfe_cy8cmbr3_reg_i2c_addr_t regValue = {0}; // Create a register structure to hold the current register value
    regValue.I2C_ADDRESS = i2cAddress & 0x7F; // Set the 7-bit I2C address

    // Set the I2C address in the device while using the old address to communicate
    if (ksfTkErrOk != _theBus->writeRegister(ksfCY8CMBR3RegI2cAddr, i2cAddress))
        return false;

    _i2cAddress = i2cAddress; // Update the internal I2C address

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::setSensorId(sfe_cy8cmbr3_sensor_id_t sensorId)
{
    if (_currentSensorId == sensorId)
        return true; // Already set, return true.

    // Ensure valid inputs
    if ( (!_theBus) ||  (sensorId < SID_0) || (sensorId > SID_15) )
        return false;

    if (ksfTkErrOk != _theBus->writeRegister(ksfCY8CMBR3RegDebugSensorId, (uint8_t)sensorId))
        return false;

    _currentSensorId = sensorId;
}

bool sfDevCY8CMBR3::setSensitivity(sfe_cy8cmbr3_sensor_id_t sensorId, sfe_cy8cmbr3_sensitivity_t sensitivity)
{
    // Ensure valid inputs
    if (    (!_theBus) 
            ||  (sensorId < SID_0) || (sensorId > SID_15) 
            ||  (sensitivity < CS_SENSITIVITY_500_COUNTS_PER_PF) || (sensitivity > CS_SENSITIVITY_125_COUNTS_PER_PF) 
        )
        return false;
    
    // Calculate the register address for the specified sensorId
    // Registers are split into 4 sensitivity registers, each controlling 4 sensors via 2 bits each
    uint8_t regAddress = ksfCY8CMBR3RegSensitivity0 + (sensorId / 4);
    
    sfe_cy8cmbr3_reg_sensitivity0_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (ksfTkErrOk != _theBus->readRegister(regAddress, regValue.byte))
        return false;

    // Update the appropriate bits for the specified sensorId
    switch (sensorId % 4)
    {
        case 0:
            regValue.CS0_SENSITIVITY = static_cast<uint8_t>(sensitivity);
            break;
        case 1:
            regValue.CS1_SENSITIVITY = static_cast<uint8_t>(sensitivity);
            break;
        case 2:
            regValue.CS2_SENSITIVITY = static_cast<uint8_t>(sensitivity);
            break;
        case 3:
            regValue.CS3_SENSITIVITY = static_cast<uint8_t>(sensitivity);
            break;
        default:
            return false; // Should never reach here
    }

    // Write the updated register value back to the device
    if (ksfTkErrOk != _theBus->writeRegister(regAddress, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::enable(sfe_cy8cmbr3_sensor_id_t sensorId, bool enable)
{
    // Ensure valid inputs
    if (    (!_theBus) 
            ||  (sensorId < SID_0) || (sensorId > SID_15) 
        )
        return false;
    
    sfe_cy8cmbr3_reg_sensor_en_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (ksfTkErrOk != _theBus->readRegister(ksfCY8CMBR3RegSensorEn, regValue.word))
        return false;

    // Update the appropriate bit for the specified sensorId
    if (enable)
        regValue.word |= (1 << sensorId); // Set the bit to enable
    else
        regValue.word &= ~(1 << sensorId); // Clear the bit to disable
    
    // Write the updated register value back to the device
    if (ksfTkErrOk != _theBus->writeRegister(ksfCY8CMBR3RegSensorEn, regValue.word))
        return false;
    
    return true; // Return true to indicate success
}

uint8_t sfDevCY8CMBR3::readCapacitancePF(sfe_cy8cmbr3_sensor_id_t sensorId)
{
    // Ensure valid inputs
    if (    (!_theBus) 
            ||  (sensorId < SID_0) || (sensorId > SID_15) 
        )
        return 0; // Return 0 to indicate error.
    
    // Set the debug sensor Id to the specified sensorId
    if (!setSensorId(sensorId))
        return 0; // Return 0 to indicate error.

    uint8_t capacitancePF = 0; // Variable to hold the capacitance value in pF

    // Read the DEBUG_CP register to get the capacitance in pF
    if (ksfTkErrOk != _theBus->readRegister(ksfCY8CMBR3RegDebugCp, capacitancePF))
        return 0; // Return 0 to indicate error.

    return capacitancePF; // Return the capacitance value in pF
}

uint16_t sfDevCY8CMBR3::readDifferenceCount(sfe_cy8cmbr3_sensor_id_t sensorId)
{
    // Ensure valid inputs
    if (    (!_theBus) 
            ||  (sensorId < SID_0) || (sensorId > SID_15) 
        )
        return 0; // Return 0 to indicate error.
    
    // Set the debug sensor Id to the specified sensorId
    if (!setSensorId(sensorId))
        return 0; // Return 0 to indicate error.

    sfe_cy8cmbr3_reg_debug_diff_cnt_t diffCount = {0}; // Variable to hold the difference count

    // Read the DEBUG_DIFFERENCE_COUNTx register to get the difference count
    if (ksfTkErrOk != _theBus->readRegister(ksfCY8CMBR3RegDebugDiffCnt0, diffCount.word))
        return 0; // Return 0 to indicate error.
    
    // TODO: Depending on endianness, may need to byte-swap here.

    return diffCount.word; // Return the difference count
}

uint16_t sfDevCY8CMBR3::readBaselineCount(sfe_cy8cmbr3_sensor_id_t sensorId)
{
    // Ensure valid inputs
    if (    (!_theBus) 
            ||  (sensorId < SID_0) || (sensorId > SID_15) 
        )
        return 0; // Return 0 to indicate error.
    
    // Set the debug sensor Id to the specified sensorId
    if (!setSensorId(sensorId))
        return 0; // Return 0 to indicate error.

    sfe_cy8cmbr3_reg_debug_baseline_t baselineCount = {0}; // Variable to hold the baseline count

    // Read the DEBUG_BASELINE_COUNT register to get the baseline count
    if (ksfTkErrOk != _theBus->readRegister(ksfCY8CMBR3RegDebugBaseline0, baselineCount.word))
        return 0; // Return 0 to indicate error.
    
    // TODO: Depending on endianness, may need to byte-swap here.

    return baselineCount.word; // Return the baseline count
}

uint16_t sfDevCY8CMBR3::readRawCount(sfe_cy8cmbr3_sensor_id_t sensorId){
    // Ensure valid inputs
    if (    (!_theBus) 
            ||  (sensorId < SID_0) || (sensorId > SID_15) 
        )
        return 0; // Return 0 to indicate error.
    
    // Set the debug sensor Id to the specified sensorId
    if (!setSensorId(sensorId))
        return 0; // Return 0 to indicate error.

    sfe_cy8cmbr3_reg_debug_raw_cnt_t rawCount = {0}; // Variable to hold the raw count

    // Read the DEBUG_RAW_COUNTx register to get the raw count
    if (ksfTkErrOk != _theBus->readRegister(ksfCY8CMBR3RegDebugRawCnt0, rawCount.word))
        return 0; // Return 0 to indicate error.
}

bool sfDevCY8CMBR3::ledOn(bool ledOn, sfe_cy8cmbr3_gpo_t gpo)
{
    if (!_theBus)
        return false;

    sfe_cy8cmbr3_reg_gpo_output_state_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (ksfTkErrOk != _theBus->readRegister(ksfCY8CMBR3RegGpoOutputState, regValue.byte))
        return false;

    // Update the bit corresponding to our gpo param
    if (ledOn)
        regValue.byte |= (1 << gpo); // Set the bit to turn on the LED
    else
        regValue.byte &= ~(1 << gpo); // Clear the bit to turn off the LED


    // Write the updated register value back to the device
    if (ksfTkErrOk != _theBus->writeRegister(ksfCY8CMBR3RegGpoOutputState, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::ledOff(sfe_cy8cmbr3_gpo_t gpo = GPO_0)
{
    return ledOn(false, gpo);
}

