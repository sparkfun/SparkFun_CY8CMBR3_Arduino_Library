/**
 * @file sfDevCY8CMBR.h
 * @brief Header file for the SparkFun Capacitive Soil Moisture Sensor - CY8CMBR3
 *
 * This file contains the class definitions, constants, and enums for interacting with the CY8CMBR3 sensor.
 *
 * @details
 * SfeCY8CMBRDriver is a comms-agnostic driver for the CY8CMBR3 Capacitive Soil Moisture sensor that uses the SparkFun Toolkit.
 * The SfeCY8CMBRArdI2C class defines the Arduino specific behavior for initializing and interacting with devices.
 *
 *
 * @author SparkFun Electronics
 * @date 2025
 * @copyright Copyright (c) 2025, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 *
 * @section Registers Configuration Register Descriptions
 * - Configuration Registers
 * - Measurement Registers
 *
 * @section Classes Classes
 * - sfDevCY8CMBR: Main class for interfacing with the CY8CMBR3 sensor.
 *
 * @section Repository Repository
 * https://github.com/sparkfun/SparkFun_CY8CMBR_Arduino_Library
 *
 * @section Product_Links Product Links
 * - Qwiic 1x1: https://www.sparkfun.com/products/TODO
 *
 */

#pragma once

#include <stdint.h>

// include the sparkfun toolkit headers
#include <sfTk/sfToolkit.h>

// Bus interfaces
#include <sfTk/sfTkII2C.h>
///////////////////////////////////////////////////////////////////////////////
// I2C Addressing
///////////////////////////////////////////////////////////////////////////////
const uint8_t kCY8CMBR3DefaultAddr = 0x37; // I2C address for the CY8CMBR3 device.
const uint8_t kCY8CMBR3MinAddr = 0x08;     // Minimum I2C address for the CY8CMBR3 device.
const uint8_t kCY8CMBR3MaxAddr = 0x77;     // Maximum I2C address for the CY8CMBR3 device.

// These are the default expeced ID values for the 102 device (CY8CMBR3102)
const uint8_t kDefaultCY8CMBR3102FamilyID = 0x9A; // When polling the FAMILY_ID register, this should be returned on boot.
const uint16_t kDefaultCY8CMBR3102DeviceID = 0xA01; // When polling the DEVICE_ID register, this should be returned on boot.

///////////////////////////////////////////////////////////////////////////////
// Register Map
///////////////////////////////////////////////////////////////////////////////
const uint8_t ksfCY8CMBR3RegSensorEn = 0x00;
const uint8_t ksfCY8CMBR3RegFssEn = 0x02;
const uint8_t ksfCY8CMBR3RegToggleEn = 0x04;
const uint8_t ksfCY8CMBR3RegLedOnEn = 0x06;
const uint8_t ksfCY8CMBR3RegSensitivity0 = 0x08;
const uint8_t ksfCY8CMBR3RegSensitivity1 = 0x09;
const uint8_t ksfCY8CMBR3RegSensitivity2 = 0x0A;
const uint8_t ksfCY8CMBR3RegSensitivity3 = 0x0B;
const uint8_t ksfCY8CMBR3RegBaseThreshold0 = 0x0C;
const uint8_t ksfCY8CMBR3RegBaseThreshold1 = 0x0D;
const uint8_t ksfCY8CMBR3RegFingerThreshold2 = 0x0E;
const uint8_t ksfCY8CMBR3RegFingerThreshold3 = 0x0F;
const uint8_t ksfCY8CMBR3RegFingerThreshold4 = 0x10;
const uint8_t ksfCY8CMBR3RegFingerThreshold5 = 0x11;
const uint8_t ksfCY8CMBR3RegFingerThreshold6 = 0x12;
const uint8_t ksfCY8CMBR3RegFingerThreshold7 = 0x13;
const uint8_t ksfCY8CMBR3RegFingerThreshold8 = 0x14;
const uint8_t ksfCY8CMBR3RegFingerThreshold9 = 0x15;
const uint8_t ksfCY8CMBR3RegFingerThreshold10 = 0x16;
const uint8_t ksfCY8CMBR3RegFingerThreshold11 = 0x17;
const uint8_t ksfCY8CMBR3RegFingerThreshold12 = 0x18;
const uint8_t ksfCY8CMBR3RegFingerThreshold13 = 0x19;
const uint8_t ksfCY8CMBR3RegFingerThreshold14 = 0x1A;
const uint8_t ksfCY8CMBR3RegFingerThreshold15 = 0x1B;
const uint8_t ksfCY8CMBR3RegSensorDebounce = 0x1C;
const uint8_t ksfCY8CMBR3RegButtonHys = 0x1D;
const uint8_t ksfCY8CMBR3RegButtonLbr = 0x1F;
const uint8_t ksfCY8CMBR3RegButtonNnt = 0x20;
const uint8_t ksfCY8CMBR3RegButtonNt = 0x21;
const uint8_t ksfCY8CMBR3RegProxEn = 0x26;
const uint8_t ksfCY8CMBR3RegProxCfg = 0x27;
const uint8_t ksfCY8CMBR3RegProxCfg2 = 0x28;
const uint8_t ksfCY8CMBR3RegProxTouchTh0 = 0x2A;
const uint8_t ksfCY8CMBR3RegProxTouchTh1 = 0x2C;
const uint8_t ksfCY8CMBR3RegProxResolution0 = 0x2E;
const uint8_t ksfCY8CMBR3RegProxResolution1 = 0x2F;
const uint8_t ksfCY8CMBR3RegProxHys = 0x30;
const uint8_t ksfCY8CMBR3RegProxLbr = 0x32;
const uint8_t ksfCY8CMBR3RegProxNnt = 0x33;
const uint8_t ksfCY8CMBR3RegProxNt = 0x34;
const uint8_t ksfCY8CMBR3RegProxPositiveTh0 = 0x35;
const uint8_t ksfCY8CMBR3RegProxPositiveTh1 = 0x36;
const uint8_t ksfCY8CMBR3RegProxNegativeTh0 = 0x39;
const uint8_t ksfCY8CMBR3RegProxNegativeTh1 = 0x3A;
const uint8_t ksfCY8CMBR3RegLedOnTime = 0x3D;
const uint8_t ksfCY8CMBR3RegBuzzerCfg = 0x3E;
const uint8_t ksfCY8CMBR3RegBuzzerOnTime = 0x3F;
const uint8_t ksfCY8CMBR3RegGpoCfg = 0x40;
const uint8_t ksfCY8CMBR3RegPwmDutyCycleCfg0 = 0x41;
const uint8_t ksfCY8CMBR3RegPwmDutyCycleCfg1 = 0x42;
const uint8_t ksfCY8CMBR3RegPwmDutyCycleCfg2 = 0x43;
const uint8_t ksfCY8CMBR3RegPwmDutyCycleCfg3 = 0x44;
const uint8_t ksfCY8CMBR3RegPwmDutyCycleCfg4 = 0x45;
const uint8_t ksfCY8CMBR3RegPwmDutyCycleCfg5 = 0x46;
const uint8_t ksfCY8CMBR3RegPwmDutyCycleCfg6 = 0x47;
const uint8_t ksfCY8CMBR3RegPwmDutyCycleCfg7 = 0x48;
const uint8_t ksfCY8CMBR3RegSpoCfg = 0x4C;
const uint8_t ksfCY8CMBR3RegDeviceCfg0 = 0x4D;
const uint8_t ksfCY8CMBR3RegDeviceCfg1 = 0x4E;
const uint8_t ksfCY8CMBR3RegDeviceCfg2 = 0x4F;
const uint8_t ksfCY8CMBR3RegDeviceCfg3 = 0x50;
const uint8_t ksfCY8CMBR3RegI2cAddr = 0x51;
const uint8_t ksfCY8CMBR3RegRefreshCtrl = 0x52;
const uint8_t ksfCY8CMBR3RegStateTimeout = 0x55;
const uint8_t ksfCY8CMBR3RegSliderCfg = 0x5D;
const uint8_t ksfCY8CMBR3RegSlider1Cfg = 0x61;
const uint8_t ksfCY8CMBR3RegSlider1Resolution = 0x62;
const uint8_t ksfCY8CMBR3RegSlider1Threshold = 0x63;
const uint8_t ksfCY8CMBR3RegSlider2Cfg = 0x67;
const uint8_t ksfCY8CMBR3RegSlider2Resolution = 0x68;
const uint8_t ksfCY8CMBR3RegSlider2Threshold = 0x69;
const uint8_t ksfCY8CMBR3RegSliderLbr = 0x71;
const uint8_t ksfCY8CMBR3RegSliderNnt = 0x72;
const uint8_t ksfCY8CMBR3RegSliderNt = 0x73;
const uint8_t ksfCY8CMBR3RegScratchpad0 = 0x7A;
const uint8_t ksfCY8CMBR3RegScratchpad1 = 0x7B;
const uint8_t ksfCY8CMBR3RegConfigCrc = 0x7E;
const uint8_t ksfCY8CMBR3RegGpoOutputState = 0x80;
const uint8_t ksfCY8CMBR3RegSensorId = 0x82;
const uint8_t ksfCY8CMBR3RegCtrlCmd = 0x86;
const uint8_t ksfCY8CMBR3RegCtrlCmdStatus = 0x88;
const uint8_t ksfCY8CMBR3RegCtrlCmdErr = 0x89;
const uint8_t ksfCY8CMBR3RegSystemStatus = 0x8A;
const uint8_t ksfCY8CMBR3RegPrevCtrlCmdCode = 0x8C;
const uint8_t ksfCY8CMBR3RegFamilyId = 0x8F;
const uint8_t ksfCY8CMBR3RegDeviceId = 0x90;
const uint8_t ksfCY8CMBR3RegDeviceRev = 0x92;
const uint8_t ksfCY8CMBR3RegCalcCrc = 0x94;
const uint8_t ksfCY8CMBR3RegTotalWorkingSns = 0x97;
const uint8_t ksfCY8CMBR3RegSnsCpHigh = 0x98;
const uint8_t ksfCY8CMBR3RegSnsVddShort = 0x9A;
const uint8_t ksfCY8CMBR3RegSnsGndShort = 0x9C;
const uint8_t ksfCY8CMBR3RegSnsSnsShort = 0x9E;
const uint8_t ksfCY8CMBR3RegCmodShieldTest = 0xA0;
const uint8_t ksfCY8CMBR3RegButtonStat = 0xAA;
const uint8_t ksfCY8CMBR3RegLatchedButtonStat = 0xAC;
const uint8_t ksfCY8CMBR3RegProxStat = 0xAE;
const uint8_t ksfCY8CMBR3RegLatchedProxStat = 0xAF;
const uint8_t ksfCY8CMBR3RegSlider1Position = 0xB0;
const uint8_t ksfCY8CMBR3RegLiftoffSlider1Position = 0xB1;
const uint8_t ksfCY8CMBR3RegSlider2Position = 0xB2;
const uint8_t ksfCY8CMBR3RegLiftoffSlider2Position = 0xB3;
const uint8_t ksfCY8CMBR3RegSyncCounter0 = 0xB9;
const uint8_t ksfCY8CMBR3RegDiffCnt0 = 0xBA;
const uint8_t ksfCY8CMBR3RegDiffCnt1 = 0xBC;
const uint8_t ksfCY8CMBR3RegDiffCnt2 = 0xBE;
const uint8_t ksfCY8CMBR3RegDiffCnt3 = 0xC0;
const uint8_t ksfCY8CMBR3RegDiffCnt4 = 0xC2;
const uint8_t ksfCY8CMBR3RegDiffCnt5 = 0xC4;
const uint8_t ksfCY8CMBR3RegDiffCnt6 = 0xC6;
const uint8_t ksfCY8CMBR3RegDiffCnt7 = 0xC8;
const uint8_t ksfCY8CMBR3RegDiffCnt8 = 0xCA;
const uint8_t ksfCY8CMBR3RegDiffCnt9 = 0xCC;
const uint8_t ksfCY8CMBR3RegDiffCnt10 = 0xCE;
const uint8_t ksfCY8CMBR3RegDiffCnt11 = 0xD0;
const uint8_t ksfCY8CMBR3RegDiffCnt12 = 0xD2;
const uint8_t ksfCY8CMBR3RegDiffCnt13 = 0xD4;
const uint8_t ksfCY8CMBR3RegDiffCnt14 = 0xD6;
const uint8_t ksfCY8CMBR3RegDiffCnt15 = 0xD8;
const uint8_t ksfCY8CMBR3RegGpoData = 0xDA;
const uint8_t ksfCY8CMBR3RegSyncCounter1 = 0xDB;
const uint8_t ksfCY8CMBR3RegDebugSensorId = 0xDC;
const uint8_t ksfCY8CMBR3RegDebugCp = 0xDD;
const uint8_t ksfCY8CMBR3RegDebugDiffCnt0 = 0xDE;
const uint8_t ksfCY8CMBR3RegDebugBaseline0 = 0xE0;
const uint8_t ksfCY8CMBR3RegDebugRawCnt0 = 0xE2;
const uint8_t ksfCY8CMBR3RegDebugAvgRawCnt0 = 0xE4;
const uint8_t ksfCY8CMBR3RegSyncCounter2 = 0xE7;

///////////////////////////////////////////////////////////////////////////////
// Register Bit Definitions
//
// Feel free to submit a PR to add additional bitfield definitions and functions 
// as needed. For now, this library focuses on the registers necessary for basic
// operation with the CY8CMBR3 for soil moisture sensing.
///////////////////////////////////////////////////////////////////////////////

// SENSOR_EN: Capacitive sensor enable/disable configuration.
typedef union {
    struct {
        uint8_t CS0: 1;
        uint8_t CS1: 1;
        uint8_t CS2: 1;
        uint8_t CS3: 1;
        uint8_t CS4: 1;
        uint8_t CS5: 1;
        uint8_t CS6: 1;
        uint8_t CS7: 1;
        uint8_t CS8: 1;
        uint8_t CS9: 1;
        uint8_t CS10: 1;
        uint8_t CS11: 1;
        uint8_t CS12: 1;
        uint8_t CS13: 1;
        uint8_t CS14: 1;
        uint8_t CS15: 1;
    };
    uint16_t word;
} sfe_cy8cmbr3_reg_sensor_en_t;

// TOGGLE_EN: GPO toggle enable/disable.
typedef union {
    struct {
        uint8_t GPO0: 1;
        uint8_t GPO1: 1;
        uint8_t GPO2: 1;
        uint8_t GPO3: 1;
        uint8_t GPO4: 1;
        uint8_t GPO5: 1;
        uint8_t GPO6: 1;
        uint8_t GPO7: 1;
        uint8_t reserved: 8;
    };
    uint16_t word;
} sfe_cy8cmbr3_reg_toggle_en_t;

// LED_ON_EN: GPO extended LED ON duration enable/disable.
typedef union {
    struct {
        uint8_t GPO0: 1;
        uint8_t GPO1: 1;
        uint8_t GPO2: 1;
        uint8_t GPO3: 1;
        uint8_t GPO4: 1;
        uint8_t GPO5: 1;
        uint8_t GPO6: 1;
        uint8_t GPO7: 1;
        uint8_t reserved: 8;
    };
    uint16_t word;
} sfe_cy8cmbr3_reg_led_on_en_t;

// SENSITIVITY0: Sensitivities (units: counts/pF) for button sensors 0 - 3
typedef union {
    struct {
        uint8_t CS0_SENSITIVITY: 2;
        uint8_t CS1_SENSITIVITY: 2;
        uint8_t CS2_SENSITIVITY: 2;
        uint8_t CS3_SENSITIVITY: 2;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_sensitivity0_t;

// TODO: Should we even make bitfield definitions for regs that are simply a value setting that spans the whole register?
// BASE_THRESHOLDx: Base threshold for button sensors 0 - 1
typedef union {
    struct {
        uint8_t CS_BASE_THRESHOLD: 8;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_base_threshold_t;

// FINGER_THRESHOLDx: Finger threshold for button sensors 2 - 15
typedef union {
    struct {
        uint8_t CS_FINGER_THRESHOLD: 8;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_finger_threshold_t;

// BUTTON_HYS: Button hysteresis
typedef union {
    struct {
        uint8_t BUTTON_HYSTERESIS: 5;
        uint8_t RESERVED: 2;
        uint8_t OVERRIDE: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_button_hys_t;

// BUTTON_LBR: Button low baseline reset
typedef union {
    struct {
        uint8_t LOW_BASELINE_RESET_THRESHOLD: 7;
        uint8_t OVERRIDE: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_button_lbr_t;

// BUTTON_NNT: Button negative noise threshold
typedef union {
    struct {
        uint8_t NEGATIVE_NOISE_THRESHOLD: 7;
        uint8_t OVERRIDE: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_button_nnt_t;

// BUTTON_NT: Button noise threshold
typedef union {
    struct {
        uint8_t NOISE_THRESHOLD: 7;
        uint8_t OVERRIDE: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_button_nt_t;

// PROX_EN: Proximity sensor enable/disable configuration.
typedef union {
    struct {
        uint8_t PS0: 1;
        uint8_t PS1: 1;
        uint8_t reserved: 6;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_prox_en_t;

// ALP_FILTER_EN
// RESERVED
// PS1_WAKE_ON_APPROACH
// PS0_WAKE_ON_APPROACH

// PROX_CFG: Proximity sensor configuration
typedef union {
    struct {
        uint8_t PS0_WAKE_ON_APPROACH: 1;
        uint8_t PS1_WAKE_ON_APPROACH: 1;
        uint8_t RESERVED: 5;
        uint8_t ALP_FILTER_EN: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_prox_cfg_t;

// PROX_CFG2: Proximity sensor configuration 2
typedef union {
    struct {
        uint8_t ALP_FILTER_K: 3;
        uint8_t RESERVED: 5;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_prox_cfg2_t;

// PROX_RESOLUTIONx: Proximity sensor resolution for PS0 - PS1
typedef union {
    struct {
        uint8_t PROX_RESOLUTION: 3;
        uint8_t RESERVED: 5;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_prox_resolution_t;

// PROX_HYS: Proximity sensor hysteresis
typedef union {
    struct {
        uint8_t PROX_HYSTERESIS: 7;
        uint8_t RESERVED: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_prox_hys_t;

// PROX_LBR: Proximity sensor low baseline reset
typedef union {
    struct {
        uint8_t LOW_BASELINE_RESET_THRESHOLD: 7;
        uint8_t OVERRIDE: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_prox_lbr_t;

// PROX_NNT: Proximity sensor negative noise threshold
typedef union {
    struct {
        uint8_t NEGATIVE_NOISE_THRESHOLD: 7;
        uint8_t OVERRIDE: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_prox_nnt_t;

// PROX_NT: Proximity sensor noise threshold
typedef union {
    struct {
        uint8_t NOISE_THRESHOLD: 7;
        uint8_t OVERRIDE: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_prox_nt_t;

// LED_ON_TIME: LED on time configuration
typedef union {
    struct {
        uint8_t LED_ON_TIME: 7;
        uint8_t RESERVED: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_led_on_time_t;

// GPO_CFG: GPO configuration
typedef union {
    struct {
        uint8_t GPO_CTL: 1; // 0: GPO controlled by CS status, 1: GPO controlled by host via GPO_OUTPUT_STATE register
        uint8_t GPO_PWM: 1; // 0: GPO output DC, 1: GPO output PWM
        uint8_t DRIVE_MODE: 1; // 0: high = Hi-Z, low = strong drive; 1: high = strong drive, low = strong drive
        uint8_t ACTIVE_STATE: 1; // 0: active low, 1: active high
        uint8_t RESERVED: 4;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_gpo_cfg_t;

// PWM_DUTYCYCLE_CFGx: PWM duty cycle configuration for GPO0 - GPO7
typedef union {
    struct {
        uint8_t HIGH_DUTY_CYCLE: 4;
        uint8_t LOW_DUTY_CYCLE: 4;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_pwm_dutycycle_cfg_t;

// SPO_CFG: Secial purpose output configuration
typedef union {
    struct {
        uint8_t SPO0: 3; // 0: disabled, 1: cap sens, 2: shield elec, 3: buzzer, 4: host int, 5: GPO
        uint8_t RESERVED0: 1;
        uint8_t SPO1: 3;
        uint8_t RESERVED1: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_spo_cfg_t;

// DEVICE_CFG0: Button sensing filter enable/disable
typedef union {
    struct {
        uint8_t MED_EN: 1;
        uint8_t IIR_EN: 1;
        uint8_t RESERVED: 6;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_device_cfg0_t;

// DEVICE_CFG1: System diagnostics enable/disable
typedef union {
    struct {
        uint8_t SYSD_EN: 1;
        uint8_t RESERVED: 7;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_device_cfg1_t;

// DEVICE_CFG2: Global sensing and processing configuration
// PROXIMITY_ARST BUTTON_SLD_ARST ATH_EN EMC_EN GUARD_EN SHIELD_EN
typedef union {
    struct {
        uint8_t SHIELD_EN: 1;
        uint8_t GUARD_EN: 1;
        uint8_t EMC_EN: 1;
        uint8_t ATH_EN: 1;
        uint8_t BUTTON_SLD_ARST: 2; // 0: auto-reset disabled, 1: auto-reset enabled 5sec timeout
        uint8_t PROXIMITY_ARST: 2;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_device_cfg2_t;

// DEVICE_CFG3: Device Power Configuration
typedef union {
    struct {
        uint8_t LOW_POWER_MODE: 1; // 0: 1.8-5.5V internally regulated, 1: 1.8V externally regulated
        uint8_t RESERVED: 7;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_device_cfg3_t;

// I2C_ADDR: I2C Address configuration
typedef union {
    struct {
        uint8_t I2C_ADDRESS: 7; // 7-bit I2C address
        uint8_t RESERVED: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_i2c_addr_t;

// REFRESH_CTRL: Refresh control configuration
typedef union {
    struct {
        uint8_t REFRESH_INTERVAL: 6; // Units of 20 ms
        uint8_t RESERVED: 2;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_refresh_ctrl_t;

// STATE_TIMEOUT: Timeout (units: seconds) of no touch activity in Active mode to trigger mode transition
typedef union {
    struct {
        uint8_t TIMEOUT: 6;
        uint8_t RESERVED: 2;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_state_timeout_t;

// CONFIG_CRC: Configuration CRC value
typedef union {
    struct {
        uint8_t CRC_LSB: 8;
        uint8_t CRC_MSB: 8;
    };
    uint16_t word;
} sfe_cy8cmbr3_reg_config_crc_t;

// GPO_OUTPUT_STATE: GPO output state control
typedef union {
    struct {
        uint8_t GPO0: 1;
        uint8_t GPO1: 1;
        uint8_t GPO2: 1;
        uint8_t GPO3: 1;
        uint8_t GPO4: 1;
        uint8_t GPO5: 1;
        uint8_t GPO6: 1;
        uint8_t GPO7: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_gpo_output_state_t;

// DEVICE_ID: Device ID register
typedef union {
    struct {
        uint8_t DEVICE_ID_LSB: 8;
        uint8_t DEVICE_ID_MSB: 8;
    };
    uint16_t word;
} sfe_cy8cmbr3_reg_device_id_t;

// DEVICE_REV: Device revision register
typedef union {
    struct {
        uint8_t REVISION: 8;
        uint8_t reserved: 8;
    };
    uint16_t word;
} sfe_cy8cmbr3_reg_device_rev_t;

// TOTAL_WORKING_SNS: System diagnostics results summary
typedef union {
    struct {
        uint8_t SENSOR_COUNT: 5;
        uint8_t RESERVED: 2;
        uint8_t SYSD_ERR: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_total_working_sns_t;

// BUTTON_STAT: Button sensor status
typedef union {
    struct {
        uint8_t CS0: 1;
        uint8_t CS1: 1;
        uint8_t CS2: 1;
        uint8_t CS3: 1;
        uint8_t CS4: 1;
        uint8_t CS5: 1;
        uint8_t CS6: 1;
        uint8_t CS7: 1;
        uint8_t CS8: 1;
        uint8_t CS9: 1;
        uint8_t CS10: 1;
        uint8_t CS11: 1;
        uint8_t CS12: 1;
        uint8_t CS13: 1;
        uint8_t CS14: 1;
        uint8_t CS15: 1;
    };
    uint16_t word;
} sfe_cy8cmbr3_reg_button_stat_t;

// LATCHED_BUTTON_STAT: Latched button sensor status
typedef union {
    struct {
        uint8_t CS0: 1;
        uint8_t CS1: 1;
        uint8_t CS2: 1;
        uint8_t CS3: 1;
        uint8_t CS4: 1;
        uint8_t CS5: 1;
        uint8_t CS6: 1;
        uint8_t CS7: 1;
        uint8_t CS8: 1;
        uint8_t CS9: 1;
        uint8_t CS10: 1;
        uint8_t CS11: 1;
        uint8_t CS12: 1;
        uint8_t CS13: 1;
        uint8_t CS14: 1;
        uint8_t CS15: 1;
    };
    uint16_t word;
} sfe_cy8cmbr3_reg_latched_button_stat_t;

// PROX_STAT: Proximity sensor status
typedef union {
    struct {
        uint8_t PS0: 1;
        uint8_t PS1: 1;
        uint8_t RESERVED: 6;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_prox_stat_t;

// LATCHED_PROX_STAT: Latched proximity sensor status
typedef union {
    struct {
        uint8_t PS0: 1;
        uint8_t PS1: 1;
        uint8_t RESERVED: 6;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_latched_prox_stat_t;

// SYNC_COUNTER0: Synchronization counter 0
typedef union {
    struct {
        uint8_t COUNTER: 4;
        uint8_t RESERVED: 4;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_sync_counter0_t;

// DIFFERENCE_COUNT_SENSOR0: Difference count for sensor 0
typedef union {
    struct {
        uint8_t DIFF_COUNT_LSB: 8;
        uint8_t DIFF_COUNT_MSB: 8;
    };
    uint16_t word;
} sfe_cy8cmbr3_reg_diff_cnt_t;

// GPO_DATA: Current GPO state values
typedef union {
    struct {
        uint8_t GPO0: 1;
        uint8_t GPO1: 1;
        uint8_t GPO2: 1;
        uint8_t GPO3: 1;
        uint8_t GPO4: 1;
        uint8_t GPO5: 1;
        uint8_t GPO6: 1;
        uint8_t GPO7: 1;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_gpo_data_t;

// SYNC_COUNTER1: Synchronization counter 1
typedef union {
    struct {
        uint8_t COUNTER: 4;
        uint8_t RESERVED: 4;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_sync_counter1_t;

// DEBUG_CP: Debug capacitor measurement
typedef uint8_t sfe_cy8cmbr3_reg_debug_cp_t;

// DEBUG_DIFFERENCE_COUNT0: Debug difference count for sensor 0
typedef union {
    struct {
        uint8_t DIFF_COUNT_LSB: 8;
        uint8_t DIFF_COUNT_MSB: 8;
    };
    uint16_t word;
} sfe_cy8cmbr3_reg_debug_diff_cnt_t;

// DEBUG_BASELINE0: Debug baseline count for sensor 0
typedef union {
    struct {
        uint8_t BASELINE_LSB: 8;
        uint8_t BASELINE_MSB: 8;
    };
    uint16_t word;
} sfe_cy8cmbr3_reg_debug_baseline_t;

// DEBUG_RAW_COUNT0: Debug raw count for sensor 0
typedef union {
    struct {
        uint8_t RAW_COUNT_LSB: 8;
        uint8_t RAW_COUNT_MSB: 8;
    };
    uint16_t word;
} sfe_cy8cmbr3_reg_debug_raw_cnt_t;

// SYNC_COUNTER2: Synchronization counter 2
typedef union {
    struct {
        uint8_t COUNTER: 4;
        uint8_t RESERVED: 4;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_sync_counter2_t;

// SYSTEM_STATUS: System configuration status indicators
typedef union {
    struct {
        uint8_t F_DEFAULT: 1; // Indicates if default configuration is loaded
        uint8_t RESERVED: 7;
    };
    uint8_t byte;
} sfe_cy8cmbr3_reg_system_status_t;

///////////////////////////////////////////////////////////////////////////////
// Enum Definitions
//
// Feel free to submit a PR to add additional enum definitions and functions 
// as needed. For now, this library focuses on the registers necessary for basic
// operation with the CY8CMBR3 for soil moisture sensing.
///////////////////////////////////////////////////////////////////////////////
typedef enum {
    SID_0 = 0x00,
    SID_1 = 0x01,
    SID_2 = 0x02,
    SID_3 = 0x03,
    SID_4 = 0x04,
    SID_5 = 0x05,
    SID_6 = 0x06,
    SID_7 = 0x07,
    SID_8 = 0x08,
    SID_9 = 0x09,
    SID_10 = 0x0A,
    SID_11 = 0x0B,
    SID_12 = 0x0C,
    SID_13 = 0x0D,
    SID_14 = 0x0E,
    SID_15 = 0x0F,
    SID_INVALID = 0xFF
} sfe_cy8cmbr3_sensor_id_t;

typedef enum {
    CS_SENSITIVITY_500_COUNTS_PER_PF = 0, // 50 counts/0.1pF
    CS_SENSITIVITY_250_COUNTS_PER_PF = 1, // 50 counts/0.2pF
    CS_SENSITIVITY_167_COUNTS_PER_PF = 2, // 50 counts/0.3pF
    CS_SENSITIVITY_125_COUNTS_PER_PF = 3 // 50 counts/0.4pF
} sfe_cy8cmbr3_sensitivity_t;

typedef enum {
    // note: for the CY8CMBR3102 which is used in our soil moisture sensor, GPO_0 is the only valid GPO
    // and is tied to the LED
    GPO_0 = 0, 
    GPO_1 = 1,
    GPO_2 = 2,
    GPO_3 = 3,
    GPO_4 = 4,
    GPO_5 = 5,
    GPO_6 = 6,
    GPO_7 = 7
} sfe_cy8cmbr3_gpo_t; 

typedef enum {
    REFRESH_INTERVAL_20MS = 1,
    REFRESH_INTERVAL_40MS = 2,
    REFRESH_INTERVAL_60MS = 3,
    REFRESH_INTERVAL_80MS = 4,
    REFRESH_INTERVAL_100MS = 5,
    REFRESH_INTERVAL_120MS = 6,
    REFRESH_INTERVAL_140MS = 7,
    REFRESH_INTERVAL_160MS = 8,
    REFRESH_INTERVAL_180MS = 9,
    REFRESH_INTERVAL_200MS = 10,
    REFRESH_INTERVAL_220MS = 11,
    REFRESH_INTERVAL_240MS = 12,
    REFRESH_INTERVAL_260MS = 13,
    REFRESH_INTERVAL_280MS = 14,
    REFRESH_INTERVAL_300MS = 15,
    REFRESH_INTERVAL_320MS = 16,
    REFRESH_INTERVAL_340MS = 17,
    REFRESH_INTERVAL_360MS = 18,
    REFRESH_INTERVAL_380MS = 19,
    REFRESH_INTERVAL_400MS = 20,
    REFRESH_INTERVAL_420MS = 21,
    REFRESH_INTERVAL_440MS = 22,
    REFRESH_INTERVAL_460MS = 23,
    REFRESH_INTERVAL_480MS = 24,
    REFRESH_INTERVAL_500MS = 25
} sfe_cy8cmbr3_refresh_interval_t;

// Host commands for the CTRL_CMD register
typedef enum {
    CTRL_CMD_NO_OP = 0,
    CTRL_CMD_SAVE_CONFIG = 2, //Checks CONFIG_CRC vs. a calculated CRC; if they match, saves current config to non-volatile memory
    CTRL_CMD_CALC_CRC = 3, //Calculates CRC over config registers and stores result in CALC_CRC register
    CTRL_CMD_DEEP_SLEEP = 7, //Puts device into Deep Sleep mode
    CTRL_CMD_RESET_LATCH = 8, //Resets all latched button and proximity status bits
    CTRL_CMD_ALP_RESET_PS0 = 9, //Resets ALP filter for proximity sensor 0
    CTRL_CMD_ALP_RESET_PS1 = 10, //Resets ALP filter for proximity sensor 1
    CTRL_CMD_SW_RESET = 255 //Performs a software reset of the entire device
} sfe_cy8cmbr3_ctrl_cmd_t;

// Host command errors
typedef enum {
    CTRL_CMD_ERR_NO_ERROR = 0,
    CTRL_CMD_ERR_SAVE_FAILED = 253,
    CTRL_CMD_ERR_CRC_FAILED = 254,
    CTRL_CMD_ERR_INVALID_CMD = 255
} sfe_cy8cmbr3_ctrl_cmd_err_t;

typedef enum {
    SPO_DISABLED = 0,
    SPO_CAP_SENSE = 1,
    SPO_SHIELD_ELEC = 2,
    SPO_BUZZER = 3,
    SPO_HOST_INT = 4,
    SPO_GPO = 5
} sfe_cy8cmbr3_spo_config_t;

typedef enum {
    AUTO_RESET_TIMEOUT_DISABLED = 0,
    AUTO_RESET_TIMEOUT_5_SECONDS = 1,
    AUTO_RESET_TIMEOUT_20_SECONDS = 2,
} sfe_cy8cmbr3_auto_reset_timeout_t;


///////////////////////////////////////////////////////////////////////////////

class sfDevCY8CMBR3
{
  public:
    sfDevCY8CMBR3() : _last_data_pF{0}, _currentSensorId{SID_INVALID}, _theBus{nullptr}
    {
    }

    /// @brief This method is called to set the communication bus directly and initialize the sensor.
    /// @param theBus Pointer to the bus object.
    /// @return True if successful, false if it fails.
    bool begin(sfTkIBus *theBus = nullptr);

    /// @brief Requests the family ID from the sensor.
    /// @return The family ID of the sensor.
    uint8_t getFamilyID(void);

    /// @brief Requests the device ID from the sensor.
    /// @return The device ID of the sensor or 0 on error.
    uint16_t getDeviceID(void);

    /// @brief Sets the communication bus to the specified bus.
    /// @param theBus Bus to set as the communication device.
    void setCommunicationBus(sfTkIBus *theBus);

    /// @brief Checks if the last sent control command is complete.
    /// @return True if the command is complete, false if it is still in progress.
    bool isCtrlCommandComplete(void);

    /// @brief Sends a control command to the sensor.
    /// @param command The control command to send.
    /// @param waitForCompletion If true, the function will wait until the command is complete before returning.
    /// @return True if successful, false if it fails.
    bool sendCtrlCommand(sfe_cy8cmbr3_ctrl_cmd_t command, bool waitForCompletion = true);

    /// @brief Save the current configuration to non-volatile memory.
    /// @details This method saves the current configuration to non-volatile memory by sending the SAVE_CONFIG command.
    ///          the device will require a reset or power cycle to load the saved configuration and leave CONFIG state.
    /// @return True if successful, false if it fails.
    bool saveConfig(void);

    /// @brief Perform a software reset of the sensor.
    /// @details This method performs a software reset of the sensor by sending the SW_RESET command.
    /// @param waitForCompletion If true, the function will wait until the reset is complete before returning.
    /// @return True if successful, false if it fails.
    bool reset(bool waitForCompletion = true);

    /// @brief Reads target register while waiting for the sync counter on either end of the target register to be equal indicating data is valid.
    /// @param register The data register we are reading from
    /// @param data Reference to store the read data.
    /// @param retries The number of times to retry checking the sync counters before giving up.
    /// @return True if successful, false if it fails.
    bool readWithSyncCounter(uint8_t reg, uint8_t &data, uint8_t retries = 10);
    bool readWithSyncCounter(uint8_t reg, uint16_t &data, uint8_t retries = 10);

    /// @brief Set the sensor Id 
    /// @param sensorId The sensor Id to set.
    /// @details This will set the sensor Id (and by extension the debug sensor Id) for debug operations.
    /// @return True if successful, false if it fails.
    bool setSensorId(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Get the current debug sensor Id
    /// @details This method gets the current debug sensor Id by reading from the DEBUG_SENSOR_ID register.
    /// @return The current debug sensor Id.
    sfe_cy8cmbr3_sensor_id_t getDebugSensorId(void);

    /// @brief Set sensitivity for the specified sensor Id
    /// @details This method sets the sensitivity for the specified sensor by writing to the SENSITIVITY0 register.
    /// @param sensorId The sensor Id to set the sensitivity for.
    /// @param sensitivity The sensitivity value to set (0-3).
    /// @return True if successful, false if it fails.
    bool setSensitivity(sfe_cy8cmbr3_sensitivity_t sensitivity = CS_SENSITIVITY_500_COUNTS_PER_PF, sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Get sensitivity for the specified sensor Id
    /// @details This method gets the sensitivity for the specified sensor by reading from the SENSITIVITY0 register.
    /// @param sensorId The sensor Id to get the sensitivity for.
    sfe_cy8cmbr3_sensitivity_t getSensitivity(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Enable or disable the auto-threshold feature.
    /// @details This method enables or disables the auto-threshold feature by setting the ATH_EN bit in the DEVICE_CFG2 register.
    /// @param enable True to enable auto-threshold, false to disable.
    /// @return True if successful, false if it fails.
    bool setAutoThresholdEnable(bool enable = true);

    /// @brief Enable or disable the auto-reset feature.
    /// @details This method enables or disables the auto-reset feature by setting the BUTTON_SLD_ARST bits in the DEVICE_CFG2 register.
    /// @param enable True to enable auto-reset, false to disable.
    /// @return True if successful, false if it fails.
    bool setAutoResetEnable(bool enable = true, sfe_cy8cmbr3_auto_reset_timeout_t timeout = AUTO_RESET_TIMEOUT_5_SECONDS);

    /// @brief Enable or disable system diagnostics.
    /// @details This method enables or disables system diagnostics by setting the SYSD_EN bit in the DEVICE_CFG1 register.
    /// @param enable True to enable system diagnostics, false to disable.
    /// @return True if successful, false if it fails.
    bool setSystemDiagnosticsEnable(bool enable = true);

    /// @brief Set the refresh interval for the sensor.
    /// @details This method sets the refresh interval by writing to the REFRESH_CTRL register.
    /// @param interval The refresh interval to set (default is 100ms).
    /// @return True if successful, false if it fails.
    bool setRefreshInterval(sfe_cy8cmbr3_refresh_interval_t interval = REFRESH_INTERVAL_100MS);

    /// @brief Set the SPO0 configuration
    /// @details This method sets the SPO0 configuration by writing to the SPO_CFG register.
    /// @param config The configuration for SPO0 (default is GPO).
    bool setSPO0Config(sfe_cy8cmbr3_spo_config_t config = SPO_GPO);

    /// @brief  Enable or disable proximity sensing for the specified sensor Id
    /// @param enable True to enable proximity sensing, false to disable.
    /// @param sensorId The sensor Id to enable or disable proximity sensing for (Only SID_0 and SID_1 are valid).
    /// @return True if successful, false if it fails.
    bool setProxEnable(bool enable = false, sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Set the base threshold for the specified sensor Id
    /// @details This method sets the base threshold for the specified sensor by writing to the BASE_THRESHOLDx register.
    /// @param threshold The base threshold value to set (0-255).
    /// @param sensorId The sensor Id to set the base threshold for.
    /// @return True if successful, false if it fails.
    bool setBaseThreshold(uint8_t threshold = 128, sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Get the base threshold for the specified sensor Id
    /// @details This method gets the base threshold for the specified sensor by reading from the BASE_THRESHOLDx register.
    /// @param sensorId The sensor Id to get the base threshold for.
    /// @return The base threshold value for the specified sensor (0-255).
    uint8_t getBaseThreshold(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);
    
    /// @brief Set the hysteresis override. This allows manual control of the hysteresis value.
    /// @param override True to enable hysteresis override, false to disable.
    /// @return True if successful, false if it fails.
    bool setHysteresisOverride(bool override = false);

    /// @brief Set the hysteresis value.
    /// @details This method sets the hysteresis value by writing to the BUTTON_HYS register.
    /// @param hysteresis The hysteresis value to set (0-31).
    /// @return True if successful, false if it fails.
    bool setHysteresis(uint8_t hysteresis = 0);

    /// @brief Get the hysteresis value.
    /// @details This method gets the hysteresis value by reading from the BUTTON_HYS register.
    /// @return The current hysteresis value (0-31)
    uint8_t getHysteresis(void);

    /// @brief Set the low baseline reset override. This allows manual control of the low baseline reset.
    /// @param override True to enable low baseline reset override, false to disable.
    /// @return True if successful, false if it fails.
    bool setLowBaselineResetOverride(bool override = false);

    /// @brief Set the low baseline reset value.
    /// @details This method sets the low baseline reset value by writing to the BUTTON_LBR register.
    /// @param baseline The low baseline reset value to set (0-127).
    /// @return True if successful, false if it fails.
    bool setLowBaselineReset(uint8_t baseline = 0);

    /// @brief Get the low baseline reset value.
    /// @details This method gets the low baseline reset value by reading from the BUTTON_LBR register.
    /// @return The current low baseline reset value (0-127).
    uint8_t getLowBaselineReset(void);

    /// @brief Set the negative noise threshold override. This allows manual control of the negative noise threshold.
    /// @param override True to enable negative noise threshold override, false to disable.
    /// @return True if successful, false if it fails.
    bool setNegativeNoiseThresholdOverride(bool override = false);
    
    /// @brief Set the negative noise threshold value.
    /// @details This method sets the negative noise threshold value by writing to the BUTTON_NNT register.
    /// @param threshold The negative noise threshold value to set (0-127).
    /// @return True if successful, false if it fails.
    bool setNegativeNoiseThreshold(uint8_t threshold = 0);

    /// @brief Get the negative noise threshold value.
    /// @details This method gets the negative noise threshold value by reading from the BUTTON_NNT register.
    /// @return The current negative noise threshold value (0-127).
    uint8_t getNegativeNoiseThreshold(void);

    /// @brief Set the noise threshold override. This allows manual control of the noise threshold.
    /// @param override True to enable noise threshold override, false to disable.
    /// @return True if successful, false if it fails.
    bool setNoiseThresholdOverride(bool override = false);

    /// @brief Set the noise threshold value.
    /// @details This method sets the noise threshold value by writing to the BUTTON_NT register.
    /// @param threshold The noise threshold value to set (0-127).
    /// @return True if successful, false if it fails.
    bool setNoiseThreshold(uint8_t threshold = 0);

    /// @brief Get the noise threshold value.
    /// @details This method gets the noise threshold value by reading from the BUTTON_NT register.
    /// @return The current noise threshold value (0-127).
    uint8_t getNoiseThreshold(void);

    /// @brief Set all the calibration parameters based on a count value.
    /// @param count The count value to use for calibration (default is 128).
    /// @return True if successful, false if it fails.
    bool setCalibrationByCount(uint8_t count = 128);

    void printOverrides(void);

    /// @brief Check if the current configuration matches the default configuration.
    /// @return True if the current configuration matches the default configuration, false otherwise.
    bool checkDefaultConfiguration(void);

    /// @brief Load the default configuration and save it to non-volatile memory.
    /// @return True if successful, false if it fails.
    bool saveDefaultConfig(void);

    /// @brief Set the GPO configuration.
    /// @details This method sets the GPO configuration by writing to the GPO_CFG register.
    /// @param controlByHost True to control GPO by host, false to control by sensor.
    /// @param pwmOutput True to set GPO as PWM output, false for DC output
    /// @param strongDrive True for strong drive mode, false for high-Z mode.
    /// @param activeHigh True for active high, false for active low. (note we use active low since our LED is tied to GPO0 on it's low side)
    bool setGPOConfig(bool controlByHost = true, bool pwmOutput = false, bool strongDrive = true, bool activeHigh = false);

    /// @brief Get the GPO configuration.
    /// @details This method gets the GPO configuration by reading from the GPO_CFG register.
    /// @param None
    /// @return The GPO configuration.
    sfe_cy8cmbr3_reg_gpo_cfg_t getGPOConfig(void);

    /// @brief Enable or disable GPO toggle.
    /// @param enable True to enable GPO toggle, false to disable.
    /// @param gpo The GPO to set the toggle enable for.
    /// @return True if successful, false if it fails.
    bool setGPOToggleEnable(bool enable = false, sfe_cy8cmbr3_gpo_t gpo = GPO_0);

    /// @brief Get the current GPO output state.
    /// @details This method gets the current GPO output state by reading from the GPO_OUTPUT_STATE register.
    /// @param None
    /// @return The current GPO output state.
    uint8_t getGPOOutputState(void);

    /// @brief Get the current GPO data.
    /// @details This method gets the current GPO data by reading from the GPO_DATA register.
    /// @param None
    /// @return The current GPO data.
    uint8_t getGPOData(void);

    /// @brief Enable or disable sensor by sensor Id
    /// @details This method enables the specified sensor by setting the appropriate bits in the SENSOR_EN register.
    /// @param sensorId The sensor Id to enable or disable.
    /// @param enable True to enable the sensor, false to disable.
    /// @return True if successful, false if it fails.
    bool enable(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0, bool enable = true);

    /// @brief Reads the capacitance value in pF from the sensor.
    /// @details This method reads the capacitance in pF from the DEBUG_CP register.
    /// @param sensorId The sensor Id to read the capacitance from.
    /// @return The capacitance value in pF (or 0 on error).
    uint8_t readCapacitancePF(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Reads the difference count value from the sensor.
    /// @details This method reads the difference count from the DEBUG_DIFFERENCE_COUNTx register.
    /// @param sensorId The sensor Id to read the difference count from.
    /// @return The difference count value (or 0 on error).
    uint16_t readDebugDifferenceCount(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Reads the difference count value from the sensor.
    /// @details This method reads the difference count from the DIFFERENCE_COUNTx register.
    /// @param sensorId The sensor Id to read the difference count from.
    /// @return The difference count value (or 0 on error).
    uint16_t readDifferenceCount(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Reads the baseline count value from the sensor.
    /// @details This method reads the baseline count from the DEBUG_BASELINE_COUNT resister.
    /// @param sensorId The sensor Id to read the baseline count from.
    /// @return The baseline count value (or 0 on error).
    uint16_t readBaselineCount(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Reads the raw count value from the sensor.
    /// @details This method reads the raw count from the DEBUG_RAW_COUNT0 register.
    /// @param sensorId The sensor Id to read the raw count from.
    /// @return The raw count value (or 0 on error).
    uint16_t readRawCount(sfe_cy8cmbr3_sensor_id_t sensorId = SID_0);

    /// @brief Turn on or off the LED.
    /// @details This method turns on or off the LED by setting or clearing the GPO output state.
    /// @param ledOn True to turn on the LED, false to turn off.
    /// @return True if successful, false if it fails.
    bool ledOn(bool ledOn = true, sfe_cy8cmbr3_gpo_t gpo = GPO_0);

    /// @brief Turn off the LED.
    /// @details This method turns off the LED by calling the ledOn method
    /// with false.
    /// @return True if successful, false if it fails.
    bool ledOff(sfe_cy8cmbr3_gpo_t gpo = GPO_0);

    /// @brief Initialize the moisture sensor with default settings.
    /// @details This method initializes the moisture sensor with default settings
    /// @return True if successful, false if it fails.
    bool defaultMoistureSensorInit(void);

  protected:
    /// @brief Set the I2C address of the sensor.
    /// @details This method is responsible for sending the command to change the I2C slave address of the sensor.
    ///          It won't change the address of the communication bus, that is the caller's responsibility.
    /// @param i2cAddress The I2C address to set (7-bit).
    /// @return True if successful, false if it fails.
    bool _setI2CAddress(uint8_t i2cAddress);

    /// @brief Read the I2C address of the sensor.
    /// @details This method reads the current I2C address from the sensor.
    /// @param i2cAddress Reference to store the read I2C address (7-bit).
    /// @return True if successful, false if it fails.
    bool _readI2CAddress(uint8_t &i2cAddress);

  private:
    bool _writeWithRetry(uint8_t regAddress, uint8_t data);
    bool _writeWithRetry(uint8_t regAddress, uint16_t data);
    bool _readWithRetry(uint8_t regAddress, uint8_t &data);
    bool _readWithRetry(uint8_t regAddress, uint16_t &data);

    sfe_cy8cmbr3_reg_debug_cp_t _last_data_pF; // Last read data from the sensor.
    sfe_cy8cmbr3_sensor_id_t _currentSensorId; // Current sensor Id for debug operations.

    sfTkIBus *_theBus; // Pointer to bus device.
};
