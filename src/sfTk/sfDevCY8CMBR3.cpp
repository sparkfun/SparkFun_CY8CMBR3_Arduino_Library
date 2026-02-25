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
 * @see https://github.com/sparkfun/SparkFun_CY8CMBR3_Arduino_Library
 */
#include "sfDevCY8CMBR3.h"

#define DEBUG_SERIAL_PRINTS (0)
#if DEBUG_SERIAL_PRINTS
#include "Arduino.h" 
#endif

bool sfDevCY8CMBR3::begin(sfTkIBus *theBus)
{
    // Nullptr check.
    if (!_theBus && !theBus)
        return false;

    // Set the internal bus pointer, overriding current bus if it exists.
    if (theBus != nullptr)
        setCommunicationBus(theBus);
    
    _theBus->setByteOrder(sfTkByteOrder::LittleEndian); // Set byte order to little-endian for this device.

    return true; // Return true to indicate success
}

uint16_t sfDevCY8CMBR3::getDeviceID(void)
{
    sfe_cy8cmbr3_reg_device_id_t devID; // Create a variable to hold the device ID.

    if (!_theBus)
        return 0x00; // Return 0x00 to indicate error.
    
    // Read the device ID register. If it errors, then return 0x00.
    if (!_readWithRetry(ksfCY8CMBR3RegDeviceId, devID.word))
        return 0x00; // Return 0x00 to indicate error.

    return devID.word; // Return the device ID.
}

uint8_t sfDevCY8CMBR3::getFamilyID(void)
{
    uint8_t familyID = 0; // Create a variable to hold the family ID.

    if (!_theBus)
        return 0; // Return 0 to indicate error.
    
    // Read the family ID register. If it errors, then return 0.
    if (!_readWithRetry(ksfCY8CMBR3RegFamilyId, familyID))
        return 0; // Return 0 to indicate error.

    return familyID; // Return the family ID.
}

void sfDevCY8CMBR3::setCommunicationBus(sfTkIBus *theBus)
{
    _theBus = theBus;
}

bool sfDevCY8CMBR3::_setI2CAddress(uint8_t i2cAddress){
    // Ensure valid inputs
    if ( (!_theBus) || (i2cAddress < kCY8CMBR3MinAddr) || (i2cAddress > kCY8CMBR3MaxAddr) )
        return false;

    // Set the I2C address in the device while using the old address to communicate
    if (!_writeWithRetry(ksfCY8CMBR3RegI2cAddr, i2cAddress)){
        #if DEBUG_SERIAL_PRINTS
        Serial.print("Failed to write I2C address to I2C_ADDR register: 0x");
        Serial.println(i2cAddress, HEX);
        #endif

        return false;
    }
        

    // The new address will only take effect once the configuration is saved and the device is reset
    if (!saveConfig()){
        #if DEBUG_SERIAL_PRINTS
        Serial.println("Failed to save configuration after setting I2C address.");
        #endif
        return false;
    }
        
    // Note the "false" parameter passed to reset() to indicate we do not want to wait for completion
    // (after the reset the new I2C address will take effect so we cannot wait for completion)
    if (!reset(false)){
        #if DEBUG_SERIAL_PRINTS
        Serial.println("Failed to reset device after setting I2C address.");
        #endif
        return false;
    }

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::_readI2CAddress(uint8_t &i2cAddress){
    // Ensure valid inputs
    if ( !_theBus )
        return false;

    sfe_cy8cmbr3_reg_i2c_addr_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the I2C address from the device
    if (!_readWithRetry(ksfCY8CMBR3RegI2cAddr, regValue.byte))
        return false;

    i2cAddress = regValue.I2C_ADDRESS; // Get the 7-bit I2C address

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::isCtrlCommandComplete(void){
    uint8_t ctrlCmd = 0; // Variable to hold the CTRL_CMD register value

    if ( !_theBus )
        return false;

    // Read the CTRL_CMD register
    if (!_readWithRetry(ksfCY8CMBR3RegCtrlCmd, ctrlCmd))
        return false;

    // If CTRL_CMD is 0, then the last command is complete
    return (ctrlCmd == CTRL_CMD_NO_OP);
}

bool sfDevCY8CMBR3::sendCtrlCommand(sfe_cy8cmbr3_ctrl_cmd_t command, bool waitForCompletion){
    // Ensure valid inputs
    if ( !_theBus )
        return false;
    
    // Write the command to the CTRL_CMD register
    if (!_writeWithRetry(ksfCY8CMBR3RegCtrlCmd, (uint8_t)command)){
        #if DEBUG_SERIAL_PRINTS
        Serial.print("Failed to write control command to CTRL_CMD register: 0x");
        Serial.println((uint8_t)command, HEX);
        #endif
        return false;
    }

    if (!waitForCompletion)
        return true;

    // Wait for the command to complete
    while (!isCtrlCommandComplete()){
        // Optionally, add a timeout here to avoid infinite loops
    }

    // Read the CTRL_CMD_ERR register to check for errors
    uint8_t ctrlCmdErr = 0; // Variable to hold the CTRL_CMD_ERR
    if (!_readWithRetry(ksfCY8CMBR3RegCtrlCmdErr, ctrlCmdErr))
        return false;

    if (ctrlCmdErr != CTRL_CMD_ERR_NO_ERROR){
        #if DEBUG_SERIAL_PRINTS
        switch (ctrlCmdErr){
            case CTRL_CMD_ERR_SAVE_FAILED:
                Serial.println("Control Command Error: SAVE_FAILED");
                break;
            case CTRL_CMD_ERR_CRC_FAILED:
                Serial.println("Control Command Error: CRC_FAILED");
                break;
            case CTRL_CMD_ERR_INVALID_CMD:
                Serial.println("Control Command Error: INVALID_CMD");
                break;
            default:
                Serial.println("Control Command Error: Unknown error code");
                break;
        }
        #endif
        return false;
    }

    #if DEBUG_SERIAL_PRINTS
    Serial.print("Control Command 0x");
    Serial.print((uint8_t)command, HEX);
    Serial.println(" executed successfully!");
    #endif

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::saveConfig(void){
    // Calculate the CRC (using the CALC_CRC register) for the current configuration and load it in the CONFIG_CRC register
    if (!sendCtrlCommand(CTRL_CMD_CALC_CRC))
        return false;

    // Read the calculated CRC from the CALC_CRC register and write it to the CONFIG_CRC register
    sfe_cy8cmbr3_reg_config_crc_t calcCrc = {0}; // Variable to hold the calculated CRC

    if (!_readWithRetry(ksfCY8CMBR3RegCalcCrc, calcCrc.word))
        return false;
    
    if (!_writeWithRetry(ksfCY8CMBR3RegConfigCrc, calcCrc.word))
        return false;

    // Send the SAVE_CONFIG command to save the current configuration to non-volatile memory
    if (!sendCtrlCommand(CTRL_CMD_SAVE_CONFIG))
        return false;

    return true; // Return true to indicate success
}


bool sfDevCY8CMBR3::readWithSyncCounter(uint8_t reg, uint8_t &data, uint8_t retries){
    if (retries == 0 || !_theBus)
        return false; 

    if (reg < ksfCY8CMBR3RegSyncCounter0 || reg > ksfCY8CMBR3RegSyncCounter2)
        return false; // Invalid reg for sync check

    // Any regs between SYNC0 and SYNC1 are valid only if SYNC0 and SYNC1 are equal
    if ( (reg >= ksfCY8CMBR3RegSyncCounter0) && (reg <= ksfCY8CMBR3RegSyncCounter1) ){
        uint8_t sync0 = 0;
        uint8_t sync1 = 0;

        
        for (uint8_t attempt = 0; attempt < retries; attempt++){
            if (!_readWithRetry(ksfCY8CMBR3RegSyncCounter0, sync0))
                return false;
            if (!_readWithRetry(reg, data))
                return false;
            if (!_readWithRetry(ksfCY8CMBR3RegSyncCounter1, sync1))
                return false;

            if (sync0 == sync1)
                return true; // Sync successful
        }
    }
    // Any regs BETWEEN SYNC1 and SYNC2 are valid only if SYNC1 and SYNC2 are equal
    else if ( (reg > ksfCY8CMBR3RegSyncCounter1) && (reg <= ksfCY8CMBR3RegSyncCounter2) ){
        uint8_t sync1 = 0;
        uint8_t sync2 = 0;

        for (uint8_t attempt = 0; attempt < retries; attempt++){
            if (!_readWithRetry(ksfCY8CMBR3RegSyncCounter1, sync1))
                return false;
            if (!_readWithRetry(reg, data))
                return false;
            if (!_readWithRetry(ksfCY8CMBR3RegSyncCounter2, sync2))
                return false;

            if (sync1 == sync2)
                return true; // Sync successful
        }
    }
    return false;
}

// uint16_t version of readWithSyncCounter
bool sfDevCY8CMBR3::readWithSyncCounter(uint8_t reg, uint16_t &data, uint8_t retries){
    if (retries == 0 || !_theBus)
        return false; 

    if (reg < ksfCY8CMBR3RegSyncCounter0 || reg > ksfCY8CMBR3RegSyncCounter2)
        return false; // Invalid reg for sync check

    // Any regs between SYNC0 and SYNC1 are valid only if SYNC0 and SYNC1 are equal
    if ( (reg >= ksfCY8CMBR3RegSyncCounter0) && (reg <= ksfCY8CMBR3RegSyncCounter1) ){
        uint8_t sync0 = 0;
        uint8_t sync1 = 0;

        
        for (uint8_t attempt = 0; attempt < retries; attempt++){
            if (!_readWithRetry(ksfCY8CMBR3RegSyncCounter0, sync0))
                return false;
            if (!_readWithRetry(reg, data))
                return false;
            if (!_readWithRetry(ksfCY8CMBR3RegSyncCounter1, sync1))
                return false;

            if (sync0 == sync1)
                return true; // Sync successful
        }
    }
    // Any regs BETWEEN SYNC1 and SYNC2 are valid only if SYNC1 and SYNC2 are equal
    else if ( (reg > ksfCY8CMBR3RegSyncCounter1) && (reg <= ksfCY8CMBR3RegSyncCounter2) ){
        uint8_t sync1 = 0;
        uint8_t sync2 = 0;

        for (uint8_t attempt = 0; attempt < retries; attempt++){
            if (!_readWithRetry(ksfCY8CMBR3RegSyncCounter1, sync1))
                return false;
            if (!_readWithRetry(reg, data))
                return false;
            if (!_readWithRetry(ksfCY8CMBR3RegSyncCounter2, sync2))
                return false;

            if (sync1 == sync2)
                return true; // Sync successful
        }
    }
    return false;
}

bool sfDevCY8CMBR3::reset(bool waitForCompletion){
    // Send the SW_RESET command to perform a software reset
    if (!sendCtrlCommand(CTRL_CMD_SW_RESET, waitForCompletion))
        return false;

    ledOff(); // Ensure LED is off after reset

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::defaultMoistureSensorInit(void)
{
    // Enable CS0 as capacitive sensor 
    if (!enable(SID_0, true)){
        #if DEBUG_SERIAL_PRINTS
            Serial.println("Failed to enable sensor SID_0");
        #endif
        return false;
    }

    // Disable CS1 as capacitive sensor (it will be the GPO)
    if(!enable(SID_1, false)){
        #if DEBUG_SERIAL_PRINTS
            Serial.println("Failed to disable sensor SID_1");
        #endif
        return false;
    }

    // By default we'll use the highest sensitivity
    if (!setSensitivity(CS_SENSITIVITY_500_COUNTS_PER_PF, SID_0)){
        #if DEBUG_SERIAL_PRINTS
        Serial.println("Failed to set sensitivity for sensor SID_0");
        #endif
        return false;
    }
       
    // Set up refresh interval to 100ms
    if (!setRefreshInterval(REFRESH_INTERVAL_100MS)){
        #if DEBUG_SERIAL_PRINTS
        Serial.println("Failed to set refresh interval");
        #endif
        return false;
    }

    if (!setSPO0Config(SPO_GPO)){
        #if DEBUG_SERIAL_PRINTS
        Serial.println("Failed to set SPO0 configuration");
        #endif
        return false;
    }

    if (!setGPOToggleEnable(false, GPO_0)){
        #if DEBUG_SERIAL_PRINTS
        Serial.println("Failed to enable GPO toggle");
        #endif
        return false;
    }

    // Set GPO0 to be controlled by host, DC output, hi-z, active low
    if (!setGPOConfig(true, false, true, true)){
        #if DEBUG_SERIAL_PRINTS
        Serial.println("Failed to set GPO configuration");
        #endif
        return false;
    }

    if (!setAutoResetEnable()){
        #if DEBUG_SERIAL_PRINTS
        Serial.println("Failed to enable auto reset");
        #endif
        return false;
    }

    // Save the configuration to non-volatile memory and reset
    if (!saveConfig()){
        #if DEBUG_SERIAL_PRINTS
        Serial.println("Failed to save configuration");
        #endif
        return false;
    }

    if (!reset()){
        #if DEBUG_SERIAL_PRINTS
        Serial.println("Failed to reset device after saving configuration");
        #endif
        return false;
    }
    
    // Ensure GPO0 is not driving the LED
    if (!ledOff(GPO_0)){
        #if DEBUG_SERIAL_PRINTS
        Serial.println("Failed to turn off LED");
        #endif
        return false;
    }

    // Set sensor Id to SID_0
    if (!setSensorId(SID_0)){
        #if DEBUG_SERIAL_PRINTS
        Serial.println("Failed to set sensor Id to SID_0");
        #endif
        return false;
    }

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::setSensorId(sfe_cy8cmbr3_sensor_id_t sensorId)
{
    // if (_currentSensorId == sensorId)
    //     return true; // Already set, return true.

    // Ensure valid inputs
    if ( (!_theBus) ||  (sensorId < SID_0) || (sensorId > SID_15) )
        return false;

    if (!_writeWithRetry(ksfCY8CMBR3RegSensorId, (uint8_t)sensorId))
        return false;

    _currentSensorId = sensorId;

    sfe_cy8cmbr3_sensor_id_t debugSensorId = getDebugSensorId();
    while (debugSensorId != sensorId){
        // Wait until the sensor ID is updated
        debugSensorId = getDebugSensorId();
    }

    return true; // Return true to indicate success
}

sfe_cy8cmbr3_sensor_id_t sfDevCY8CMBR3::getDebugSensorId(void){
    uint8_t sensorId = 0; // Create a variable to hold the sensor ID.

    if (!_theBus)
        return SID_INVALID; // Return invalid to indicate error.
    
    // Read the DEBUG_SENSOR_ID register. If it errors, then return invalid.
    if (!readWithSyncCounter(ksfCY8CMBR3RegDebugSensorId, sensorId))
        return SID_INVALID; // Return invalid to indicate error.

    if (sensorId > SID_15)
        return SID_INVALID; // Return invalid to indicate error.

    return static_cast<sfe_cy8cmbr3_sensor_id_t>(sensorId); // Return the sensor ID.
}

bool sfDevCY8CMBR3::setSensitivity(sfe_cy8cmbr3_sensitivity_t sensitivity, sfe_cy8cmbr3_sensor_id_t sensorId)
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
    if (!_readWithRetry(regAddress, regValue.byte))
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
    if (!_writeWithRetry(regAddress, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

sfe_cy8cmbr3_sensitivity_t sfDevCY8CMBR3::getSensitivity(sfe_cy8cmbr3_sensor_id_t sensorId)
{
    // Ensure valid inputs
    if (    (!_theBus) 
            ||  (sensorId < SID_0) || (sensorId > SID_15) 
        )
        return CS_SENSITIVITY_500_COUNTS_PER_PF; // Return default sensitivity to indicate error.
    
    // Calculate the register address for the specified sensorId
    // Registers are split into 4 sensitivity registers, each controlling 4 sensors via 2 bits each
    uint8_t regAddress = ksfCY8CMBR3RegSensitivity0 + (sensorId / 4);
    
    sfe_cy8cmbr3_reg_sensitivity0_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value
    if (!_readWithRetry(regAddress, regValue.byte))
        return CS_SENSITIVITY_500_COUNTS_PER_PF; // Return default sensitivity to indicate error.

    // Extract the appropriate bits for the specified sensorId
    switch (sensorId % 4)
    {
        case 0:
            return static_cast<sfe_cy8cmbr3_sensitivity_t>(regValue.CS0_SENSITIVITY);
        case 1:
            return static_cast<sfe_cy8cmbr3_sensitivity_t>(regValue.CS1_SENSITIVITY);
        case 2:
            return static_cast<sfe_cy8cmbr3_sensitivity_t>(regValue.CS2_SENSITIVITY);
        case 3:
            return static_cast<sfe_cy8cmbr3_sensitivity_t>(regValue.CS3_SENSITIVITY);
        default:
            return CS_SENSITIVITY_500_COUNTS_PER_PF; // Should never reach here, return default sensitivity to indicate error.
    }
}

bool sfDevCY8CMBR3::setProxEnable(bool enable, sfe_cy8cmbr3_sensor_id_t sensorId)
{
    // Ensure valid inputs
    if (    (!_theBus) 
            ||  (sensorId < SID_0) || (sensorId > SID_15) 
        )
        return false;
    
    sfe_cy8cmbr3_reg_prox_en_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegProxEn, regValue.byte))
        return false;

    uint8_t val = enable ? 1 : 0;
    if (sensorId == SID_0)
        regValue.PS0 = val;
    else if (sensorId == SID_1)
        regValue.PS1 = val;
    else
        return false; // Only SID_0 and SID_1 are valid for proximity sensing.
    
    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegProxEn, regValue.byte))
        return false;
    
    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::setAutoThresholdEnable(bool enable)
{
    // Ensure valid inputs
    if ( !_theBus )
        return false;
    
    sfe_cy8cmbr3_reg_device_cfg2_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegDeviceCfg2, regValue.byte))
        return false;

    regValue.ATH_EN = enable ? 1 : 0; // Set or clear the ATH_EN bit

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegDeviceCfg2, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::setAutoResetEnable(bool enable, sfe_cy8cmbr3_auto_reset_timeout_t timeout){
    // Ensure valid inputs
    if ( !_theBus )
        return false;
    
    sfe_cy8cmbr3_reg_device_cfg2_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegDeviceCfg2, regValue.byte))
        return false;

    regValue.PROXIMITY_ARST = timeout;
    regValue.BUTTON_SLD_ARST = timeout; // Set the BUTTON_SLD_ARST bit to enable auto-reset

    if (!enable){
        regValue.PROXIMITY_ARST = AUTO_RESET_TIMEOUT_DISABLED;
        regValue.BUTTON_SLD_ARST = AUTO_RESET_TIMEOUT_DISABLED; // Clear the BUTTON_SLD_ARST bit to disable auto-reset
    }

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegDeviceCfg2, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::setSystemDiagnosticsEnable(bool enable){
    // Ensure valid inputs
    if ( !_theBus )
        return false;
    
    sfe_cy8cmbr3_reg_device_cfg1_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegDeviceCfg1, regValue.byte))
        return false;

    regValue.SYSD_EN = enable ? 1 : 0; // Set or clear the SYSD_EN bit

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegDeviceCfg1, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::setBaseThreshold(uint8_t threshold, sfe_cy8cmbr3_sensor_id_t sensorId){
    // Ensure valid inputs
    if (    (!_theBus) 
            ||  (sensorId < SID_0) || (sensorId > SID_15) 
        )
        return false;
    
    // Calculate the register address for the specified sensorId
    uint8_t regAddress = ksfCY8CMBR3RegBaseThreshold0 + sensorId;

    // Write the threshold value to the appropriate register
    if (!_writeWithRetry(regAddress, threshold))
        return false;

    return true; // Return true to indicate success
}

uint8_t sfDevCY8CMBR3::getBaseThreshold(sfe_cy8cmbr3_sensor_id_t sensorId){
    // Ensure valid inputs
    if (    (!_theBus) 
            ||  (sensorId < SID_0) || (sensorId > SID_15) 
        )
        return 0;
    
    // Calculate the register address for the specified sensorId
    uint8_t regAddress = ksfCY8CMBR3RegBaseThreshold0 + sensorId;

    uint8_t threshold = 0;
    if (!_readWithRetry(regAddress, threshold))
        return 0;

    return threshold;
}

bool sfDevCY8CMBR3::setHysteresisOverride(bool override){
    // Ensure valid inputs
    if ( !_theBus )
        return false;
    
    sfe_cy8cmbr3_reg_button_hys_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegButtonHys, regValue.byte))
        return false;

    regValue.OVERRIDE = override ? 1 : 0; // Set or clear the OVERRIDE bit

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegButtonHys, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::setHysteresis(uint8_t hysteresis){
    // Ensure valid inputs
    if ( !_theBus || (hysteresis > 31) ){
        return false;
    }

    sfe_cy8cmbr3_reg_button_hys_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegButtonHys, regValue.byte))
        return false;

    // To update the hysteresis, the override must be set. You can turn off later using setHysteresisOverride(false)
    regValue.OVERRIDE = 1;
    regValue.BUTTON_HYSTERESIS = hysteresis; // Set the HYS bits to the specified hysteresis value

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegButtonHys, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

uint8_t sfDevCY8CMBR3::getHysteresis(void){
    if ( !_theBus )
        return 0;

    sfe_cy8cmbr3_reg_button_hys_t regValue = {0};
    if (!_readWithRetry(ksfCY8CMBR3RegButtonHys, regValue.byte))
        return 0;
    
    return regValue.BUTTON_HYSTERESIS;
}

bool sfDevCY8CMBR3::setLowBaselineResetOverride(bool override){
    // Ensure valid inputs
    if ( !_theBus )
        return false;
    
    sfe_cy8cmbr3_reg_button_lbr_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegButtonLbr, regValue.byte))
        return false;
    
    regValue.OVERRIDE = override ? 1 : 0; // Set or clear the OVERRIDE bit

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegButtonLbr, regValue.byte))
        return false;
    
    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::setLowBaselineReset(uint8_t baseline)
{
    // Ensure valid inputs
    if ( !_theBus || (baseline > 127) )
        return false;

    sfe_cy8cmbr3_reg_button_lbr_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegButtonLbr, regValue.byte))
        return false;

    // To update the low baseline reset, the override must be set. You can turn off later using setLowBaselineResetOverride(false)
    regValue.OVERRIDE = 1;
    regValue.LOW_BASELINE_RESET_THRESHOLD = baseline; // Set the LOW_BASELINE_RESET bits to the specified baseline value

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegButtonLbr, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

uint8_t sfDevCY8CMBR3::getLowBaselineReset(void){
    if ( !_theBus )
        return 0;

    sfe_cy8cmbr3_reg_button_lbr_t regValue = {0};
    if (!_readWithRetry(ksfCY8CMBR3RegButtonLbr, regValue.byte))
        return 0;
    
    return regValue.LOW_BASELINE_RESET_THRESHOLD;
}

bool sfDevCY8CMBR3::setNegativeNoiseThresholdOverride(bool override){
    if ( !_theBus )
        return false;

    sfe_cy8cmbr3_reg_button_nnt_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegButtonNnt, regValue.byte))
        return false;

    regValue.OVERRIDE = override ? 1 : 0; // Set or clear the OVERRIDE bit

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegButtonNnt, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::setNegativeNoiseThreshold(uint8_t threshold)
{
    // Ensure valid inputs
    if ( !_theBus || (threshold > 127) )
        return false;

    sfe_cy8cmbr3_reg_button_nnt_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegButtonNnt, regValue.byte))
        return false;

    regValue.OVERRIDE = 1; // To update the negative noise threshold, the override must be set. You can turn off later using setNegativeNoiseThresholdOverride(false)
    regValue.NEGATIVE_NOISE_THRESHOLD = threshold; // Set the NEGATIVE_NOISE_THRESHOLD bits to the specified threshold value

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegButtonNnt, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

uint8_t sfDevCY8CMBR3::getNegativeNoiseThreshold(void){
    if ( !_theBus )
        return 0;

    sfe_cy8cmbr3_reg_button_nnt_t regValue = {0};
    if (!_readWithRetry(ksfCY8CMBR3RegButtonNnt, regValue.byte))
        return 0;
    
    return regValue.NEGATIVE_NOISE_THRESHOLD;
}

bool sfDevCY8CMBR3::setNoiseThresholdOverride(bool override)
{
    if ( !_theBus )
        return false;

    sfe_cy8cmbr3_reg_button_nnt_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegButtonNnt, regValue.byte))
        return false;

    regValue.OVERRIDE = override ? 1 : 0; // Set or clear the OVERRIDE bit

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegButtonNnt, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::setNoiseThreshold(uint8_t threshold)
{
    // Ensure valid inputs
    if ( !_theBus || (threshold > 127) )
        return false;

    sfe_cy8cmbr3_reg_button_nnt_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegButtonNnt, regValue.byte))
        return false;

    regValue.OVERRIDE = 1; // To update the negative noise threshold, the override must be set. You can turn off later using setNegativeNoiseThresholdOverride(false)
    regValue.NEGATIVE_NOISE_THRESHOLD = threshold; // Set the NEGATIVE_NOISE_THRESHOLD bits to the specified threshold value

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegButtonNnt, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

uint8_t sfDevCY8CMBR3::getNoiseThreshold(void){
    if ( !_theBus )
        return 0;

    sfe_cy8cmbr3_reg_button_nnt_t regValue = {0};
    if (!_readWithRetry(ksfCY8CMBR3RegButtonNnt, regValue.byte))
        return 0;
    
    return regValue.NEGATIVE_NOISE_THRESHOLD;
}

bool sfDevCY8CMBR3::setRefreshInterval(sfe_cy8cmbr3_refresh_interval_t interval)
{
    // Ensure valid inputs
    if ( (!_theBus) ||  (interval < REFRESH_INTERVAL_20MS) || (interval > REFRESH_INTERVAL_500MS) )
        return false;
    
    sfe_cy8cmbr3_reg_refresh_ctrl_t regValue = {0}; // Create a register structure to hold the current register value

    regValue.REFRESH_INTERVAL = static_cast<uint8_t>(interval); // Set the refresh interval

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegRefreshCtrl, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::setSPO0Config(sfe_cy8cmbr3_spo_config_t config)
{
    // Ensure valid inputs
    if ( !_theBus )
        return false;
    
    sfe_cy8cmbr3_reg_spo_cfg_t regValue = {0};
    if (!_readWithRetry(ksfCY8CMBR3RegSpoCfg, regValue.byte))
        return false;

    regValue.SPO0 = static_cast<uint8_t>(config);

    // Write the SPO0 configuration to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegSpoCfg, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::setGPOConfig(bool controlByHost, bool pwmOutput, bool strongDrive, bool activeHigh)
{
    // Ensure valid inputs
    if ( !_theBus )
        return false;
    
    sfe_cy8cmbr3_reg_gpo_cfg_t regValue = {0}; // Create a register structure to hold the current register value

    regValue.GPO_CTL = controlByHost ? 1 : 0;
    regValue.GPO_PWM = pwmOutput ? 1 : 0;
    regValue.DRIVE_MODE = strongDrive ? 1 : 0;
    regValue.ACTIVE_STATE = activeHigh ? 1 : 0;

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegGpoCfg, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

sfe_cy8cmbr3_reg_gpo_cfg_t sfDevCY8CMBR3::getGPOConfig(void){
    sfe_cy8cmbr3_reg_gpo_cfg_t regValue = {0}; // Create a register structure to hold the current register value

    if (!_theBus)
        return regValue; // Return default to indicate error.
    
    // Read the GPO_CFG register. If it errors, then return default.
    if (!_readWithRetry(ksfCY8CMBR3RegGpoCfg, regValue.byte))
        return regValue; // Return default to indicate error.

    return regValue; // Return the GPO configuration.
}

bool sfDevCY8CMBR3::setGPOToggleEnable(bool enable, sfe_cy8cmbr3_gpo_t gpo){
    // Ensure valid inputs
    if (!_theBus)
        return false;

    sfe_cy8cmbr3_reg_toggle_en_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    if (!_readWithRetry(ksfCY8CMBR3RegToggleEn, regValue.word))
        return false;

    uint8_t val = enable ? 1 : 0;
    if (gpo == GPO_0)
        regValue.GPO0 = val;
    else if (gpo == GPO_1)
        regValue.GPO1 = val;
    else
        return false; // Only GPO_0 and GPO_1 are valid.

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegToggleEn, regValue.word))
        return false;

    return true; // Return true to indicate success
}

uint8_t sfDevCY8CMBR3::getGPOOutputState(void){
    uint8_t gpoOutputState = 0; // Create a variable to hold the GPO output state.

    if (!_theBus)
        return 0; // Return 0 to indicate error.
    
    // Read the GPO_OUTPUT_STATE register. If it errors, then return 0.
    if (!_readWithRetry(ksfCY8CMBR3RegGpoOutputState, gpoOutputState))
        return 0; // Return 0 to indicate error.

    return gpoOutputState; // Return the GPO output state.
}

uint8_t sfDevCY8CMBR3::getGPOData(void){
    uint8_t gpoData = 0; // Create a variable to hold the GPO data.

    if (!_theBus)
        return 0; // Return 0 to indicate error.
    
    // Read the GPO_DATA register. If it errors, then return 0.
    if (!readWithSyncCounter(ksfCY8CMBR3RegGpoData, gpoData))
        return 0; // Return 0 to indicate error.

    return gpoData; // Return the GPO data.
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
    if (!_readWithRetry(ksfCY8CMBR3RegSensorEn, regValue.word))
        return false;

    // Update the appropriate bit for the specified sensorId
    if (enable)
        regValue.word |= (1 << sensorId); // Set the bit to enable
    else
        regValue.word &= ~(1 << sensorId); // Clear the bit to disable
    
    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegSensorEn, regValue.word))
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
    
    
    // From datasheet 1.5.123 DEBUG_CP register (measurement is updated whenever there is a change in value of SENSOR_ID register)
    // So, we'll first set the sensor ID to something else and then back to the desired sensorId to force an update.
    if (!setSensorId(sensorId == SID_0 ? SID_1 : SID_0))
        return 0; // Return 0 to indicate error.

    // Set the debug sensor Id to the specified sensorId
    if (!setSensorId(sensorId))
        return 0; // Return 0 to indicate error.

    uint8_t capacitancePF = 0; // Variable to hold the capacitance value in pF

    // Read the DEBUG_CP register to get the capacitance in pF
    if (!readWithSyncCounter(ksfCY8CMBR3RegDebugCp, capacitancePF))
        return 0; // Return 0 to indicate error.
    
    _last_data_pF = capacitancePF; // Store the last read data
    
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

    sfe_cy8cmbr3_reg_diff_cnt_t diffCount = {0}; // Variable to hold the difference count
    if (!readWithSyncCounter(ksfCY8CMBR3RegDiffCnt0, diffCount.word))
        return 0; // Return 0 to indicate error.

    return diffCount.word; // Return the difference count
}

uint16_t sfDevCY8CMBR3::readDebugDifferenceCount(sfe_cy8cmbr3_sensor_id_t sensorId)
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
    if (!readWithSyncCounter(ksfCY8CMBR3RegDebugDiffCnt0, diffCount.word))
        return 0; // Return 0 to indicate error.

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
    if (!readWithSyncCounter(ksfCY8CMBR3RegDebugBaseline0, baselineCount.word))
        return 0; // Return 0 to indicate error.

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
    if (!readWithSyncCounter(ksfCY8CMBR3RegDebugRawCnt0, rawCount.word))
        return 0; // Return 0 to indicate error.
    
    return rawCount.word; // Return the raw count
}


bool sfDevCY8CMBR3::ledOn(bool ledOn, sfe_cy8cmbr3_gpo_t gpo)
{
    if (!_theBus)
        return false;

    sfe_cy8cmbr3_reg_gpo_output_state_t regValue = {0}; // Create a register structure to hold the current register value

    // Read the current register value to retain other bits
    
    if (!_readWithRetry(ksfCY8CMBR3RegGpoOutputState, regValue.byte))
        return false;

    // Update the bit corresponding to our gpo param (active low logic for LED)
    if (ledOn)
        regValue.byte &= ~(1 << gpo); // Clear the bit to turn on the LED (active low)
    else
        regValue.byte |= (1 << gpo); // Set the bit to turn off the LED (active low)

    // Write the updated register value back to the device
    if (!_writeWithRetry(ksfCY8CMBR3RegGpoOutputState, regValue.byte))
        return false;

    return true; // Return true to indicate success
}

bool sfDevCY8CMBR3::ledOff(sfe_cy8cmbr3_gpo_t gpo)
{
    return ledOn(false, gpo);
}

// The CY8CMBR3 spec actually states that it can ack or nack as many times as it wants to until it's ready
// so we need to replace the writeRegister and readRegister calls with versions that can handle that behavior.
static const uint8_t maxRetries = 5;

// uint8_t write with retry
bool sfDevCY8CMBR3::_writeWithRetry(uint8_t regAddress, uint8_t data)
{
    uint8_t attempt = 0;

    while (attempt < maxRetries)
    {
        #if DEBUG_SERIAL_PRINTS
        Serial.print("Attempt ");
        Serial.println(attempt + 1);
        Serial.print("Writing to reg 0x");
        Serial.print(regAddress, HEX);
        Serial.print(": 0x");
        Serial.println(data, HEX);
        #endif

        if (ksfTkErrOk == _theBus->writeRegister(regAddress, data)){
            #if DEBUG_SERIAL_PRINTS
            Serial.println("Write successful");
            #endif
            return true; // Success
        }
        attempt++;
    }

    #if DEBUG_SERIAL_PRINTS
    Serial.println("Write failed after max retries");
    #endif
    return false; // Failed after retries
}

// uint16_t write with retry
bool sfDevCY8CMBR3::_writeWithRetry(uint8_t regAddress, uint16_t data)
{
    uint8_t attempt = 0;

    while (attempt < maxRetries)
    {
        #if DEBUG_SERIAL_PRINTS
        Serial.print("Attempt ");
        Serial.println(attempt + 1);
        Serial.print("Writing to reg 0x");
        Serial.print(regAddress, HEX);
        Serial.print(": 0x");
        Serial.println(data, HEX);
        #endif
        if (ksfTkErrOk == _theBus->writeRegister(regAddress, data)){
            #if DEBUG_SERIAL_PRINTS
            Serial.println("Write successful");
            #endif
            return true; // Success
        }
        attempt++;
    }

    #if DEBUG_SERIAL_PRINTS
    Serial.println("Write failed after max retries");
    #endif
    return false; // Failed after retries
}

// uint8_t read with retry
bool sfDevCY8CMBR3::_readWithRetry(uint8_t regAddress, uint8_t &data)
{
    uint8_t attempt = 0;

    while (attempt < maxRetries)
    {
        #if DEBUG_SERIAL_PRINTS
        Serial.print("Attempt ");
        Serial.println(attempt + 1);
        Serial.print("Reading from reg 0x");
        Serial.println(regAddress, HEX);
        #endif
        if (ksfTkErrOk == _theBus->readRegister(regAddress, data)){
            #if DEBUG_SERIAL_PRINTS
            Serial.println("Read successful");
            Serial.print("Data: 0x");
            Serial.println(data, HEX);
            #endif
            return true; // Success
        }
        attempt++;
    }

    #if DEBUG_SERIAL_PRINTS
    Serial.println("Read failed after max retries");
    #endif
    return false; // Failed after retries
}

// uint16_t read with retry
bool sfDevCY8CMBR3::_readWithRetry(uint8_t regAddress, uint16_t &data)
{
    uint8_t attempt = 0;

    while (attempt < maxRetries)
    {
        #if DEBUG_SERIAL_PRINTS
        Serial.print("Attempt ");
        Serial.println(attempt + 1);
        Serial.print("Reading from reg 0x");
        Serial.println(regAddress, HEX);
        #endif
        if (ksfTkErrOk == _theBus->readRegister(regAddress, data)){
            #if DEBUG_SERIAL_PRINTS
            Serial.println("Read successful");
            Serial.print("Data: 0x");
            Serial.println(data, HEX);
            #endif
            return true; // Success
        }
        attempt++;
    }

    #if DEBUG_SERIAL_PRINTS
    Serial.println("Read failed after max retries");
    #endif
    return false; // Failed after retries
}

// ------------------ DEFAULT CONFIGURATION -------------------------
// Create a default configuration structure that contains the registers and values to write to them to get a default config:
// This will be an array where each row contains a register address, the number of bytes to write to it, and the corresponding byte(s) to write to that register for it's default state.
// So it will take the form:
// {register address, number of bytes to write, byte 0, byte 1 (if applicable)}
static const uint8_t kDefaultCY8CMBR3Config[][4] = {
    {ksfCY8CMBR3RegSensorEn, 0x02, 0x03, 0x00},
    {ksfCY8CMBR3RegFssEn, 0x02, 0x00, 0x00},
    {ksfCY8CMBR3RegToggleEn, 0x02, 0x00, 0x00},
    {ksfCY8CMBR3RegSensitivity0, 0x01, 0x00, 0x00},
    {ksfCY8CMBR3RegBaseThreshold0, 0x01, 0x80, 0x00},
    {ksfCY8CMBR3RegBaseThreshold1, 0x01, 0x80, 0x00},
    {ksfCY8CMBR3RegSensorDebounce, 0x01, 0x03, 0x00},
    {ksfCY8CMBR3RegButtonHys, 0x01, 0x0C, 0x00},
    {ksfCY8CMBR3RegButtonLbr, 0x01, 0x32, 0x00},
    {ksfCY8CMBR3RegButtonNnt, 0x01, 0x33, 0x00},
    {ksfCY8CMBR3RegButtonNt, 0x01, 0x33, 0x00},
    {ksfCY8CMBR3RegProxEn, 0x01, 0x00, 0x00},
    {ksfCY8CMBR3RegProxCfg, 0x01, 0x80, 0x00},
    {ksfCY8CMBR3RegProxCfg2, 0x01, 0x05, 0x00},
    {ksfCY8CMBR3RegProxTouchTh0, 0x02, 0x00, 0x02}, // 0x0200 = 512
    {ksfCY8CMBR3RegProxTouchTh1, 0x02, 0x00, 0x02}, // 0x0200 = 512
    {ksfCY8CMBR3RegProxResolution0, 0x01, 0x00, 0x00},
    {ksfCY8CMBR3RegProxResolution1, 0x01, 0x00, 0x00},
    {ksfCY8CMBR3RegProxHys, 0x01, 0x05, 0x00},
    {ksfCY8CMBR3RegProxLbr, 0x01, 0x32, 0x00},
    {ksfCY8CMBR3RegProxNnt, 0x01, 0x14, 0x00},
    {ksfCY8CMBR3RegProxNt, 0x01, 0x14, 0x00},
    {ksfCY8CMBR3RegProxPositiveTh0, 0x01, 0x1e, 0x00},
    {ksfCY8CMBR3RegProxPositiveTh1, 0x01, 0x1e, 0x00},
    {ksfCY8CMBR3RegProxNegativeTh0, 0x01, 0x1e, 0x00},
    {ksfCY8CMBR3RegProxNegativeTh1, 0x01, 0x1e, 0x00},
    {ksfCY8CMBR3RegLedOnTime, 0x01, 0x00, 0x00},
    {ksfCY8CMBR3RegGpoCfg, 0x01, 0x00, 0x00},
    {ksfCY8CMBR3RegPwmDutyCycleCfg0, 0x01, 0x0F, 0x00},
    {ksfCY8CMBR3RegSpoCfg, 0x01, 0x01, 0x00}, 
    {ksfCY8CMBR3RegDeviceCfg0, 0x01, 0x03, 0x00},
    {ksfCY8CMBR3RegDeviceCfg1, 0x01, 0x01, 0x00},
    {ksfCY8CMBR3RegDeviceCfg2, 0x01, 0x08, 0x00},
    {ksfCY8CMBR3RegDeviceCfg3, 0x01, 0x00, 0x00},
    {ksfCY8CMBR3RegRefreshCtrl, 0x01, 0x06, 0x00},
    {ksfCY8CMBR3RegStateTimeout, 0x01, 0x0A, 0x00},
    {ksfCY8CMBR3RegScratchpad0, 0x01, 0x00, 0x00},
    {ksfCY8CMBR3RegScratchpad1, 0x01, 0x00, 0x00}
};

bool sfDevCY8CMBR3::saveDefaultConfig(void)
{
    if (!_theBus)
        return false;

    for (size_t i = 0; i < sizeof(kDefaultCY8CMBR3Config) / sizeof(kDefaultCY8CMBR3Config[0]); i++)
    {
        uint8_t reg = kDefaultCY8CMBR3Config[i][0];
        uint8_t numBytes = kDefaultCY8CMBR3Config[i][1];
        uint8_t data0 = kDefaultCY8CMBR3Config[i][2];
        uint8_t data1 = kDefaultCY8CMBR3Config[i][3];

        if (numBytes == 1) {
            if (!_writeWithRetry(reg, data0))
                return false;
        }
        else if (numBytes == 2) {
            uint16_t data = (data1 << 8) | data0;
            if (!_writeWithRetry(reg, data))
                return false;
        }
        else {
            // Unsupported number of bytes
            return false;
        }
    }

    if (!reset())
        return false;
    
    return true;
}