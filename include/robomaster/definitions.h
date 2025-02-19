/*
 * Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
 *
 * This project contains contributions from multiple authors.
 * The original code is licensed under the MIT License by Fraunhofer IML.
 * All modifications and additional code are licensed under the MIT License by Vinzenz Weist.
 */

#pragma once

namespace robomaster {
    enum BlasterMode: uint8_t {
        BLASTER_MODE_GEL = 0x00,
        BLASTER_MODE_IR = 0x01
    };

    enum ChassisMode: uint8_t {
        CHASSIS_MODE_ENABLE = 0x01,
        CHASSIS_MODE_DISABLE = 0x00
    };

    enum GimbalMode: uint8_t {
        GIMBAL_MODE_FREE = 0x00,
        GIMBAL_MODE_FOLLOW = 0x02
    };

    enum GimbalHibernate: uint16_t {
        GIMBAL_STATE_SUSPEND = 0x2ab5,
        GIMBAL_STATE_RESUME = 0x7ef2
    };

    enum LEDMode: uint8_t {
        LED_MODE_STATIC = 0x71,
        LED_MODE_BREATHE = 0x72,
        LED_MODE_FLASH = 0x73
    };

    enum LEDMask: uint8_t {
        LED_MASK_ALL = 0x3f,
        LED_MASK_BOTTOM_ALL = 0x0f,
        LED_MASK_BOTTOM_BACK = 0x01,
        LED_MASK_BOTTOM_FRONT = 0x02,
        LED_MASK_BOTTOM_LEFT = 0x04,
        LED_MASK_BOTTOM_RIGHT = 0x08,
        LED_MASK_TOP_LEFT = 0x10,
        LED_MASK_TOP_RIGHT = 0x20,
        LED_MASK_TOP_ALL = 0x30
    };

    enum DeviceID: uint16_t {
        DEVICE_ID_INTELLI_CONTROLLER = 0x201,
        DEVICE_ID_MOTION_CONTROLLER = 0x202,
        DEVICE_ID_GIMBAL = 0x203,
        DEVICE_ID_HIT_DETECTOR_1 = 0x211,
        DEVICE_ID_HIT_DETECTOR_2 = 0x212,
        DEVICE_ID_HIT_DETECTOR_3 = 0x213,
        DEVICE_ID_HIT_DETECTOR_4 = 0x214
    };
} // namespace robomaster