// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#ifndef ROBOMASTER_DEFINITIONS_H_
#define ROBOMASTER_DEFINITIONS_H_

namespace robomaster {
    enum BlasterMode {
        INFRARED,
        GELBEADS
    };

    enum LEDMode: uint8_t {
        STATIC = 0x71,
        BREATHE = 0x72,
        FLASH = 0x73
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

    enum LEDMask: uint16_t {
        LED_MASK_ALL = 0x3f,
        LED_MASK_BOTTOM_ALL = 0xf,
        LED_MASK_BOTTOM_BACK = 0x1,
        LED_MASK_BOTTOM_FRONT = 0x2,
        LED_MASK_BOTTOM_LEFT = 0x4,
        LED_MASK_BOTTOM_RIGHT = 0x8,
        LED_MASK_TOP_LEFT = 0x10,
        LED_MASK_TOP_RIGHT = 0x20,
        LED_MASK_TOP_ALL = 0x30
    };
} // namespace robomaster

#endif // ROBOMASTER_DEFINITIONS_H_