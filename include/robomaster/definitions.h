/*
 * MIT License
 *
 * Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

namespace robomaster {
    /**
     * @brief Enum contains the BlasterMode's
     */
    enum BlasterMode: uint8_t {
        BLASTER_MODE_GEL = 0x00,
        BLASTER_MODE_IR = 0x01
    };

    /**
     * @brief Enum contains the ChassisMode's
     */
    enum ChassisMode: uint8_t {
        CHASSIS_MODE_ENABLE = 0x01,
        CHASSIS_MODE_DISABLE = 0x00
    };

    /**
     * @brief Enum contains the GimbalMode's
     */
    enum GimbalMode: uint8_t {
        GIMBAL_MODE_FREE = 0x00,
        GIMBAL_MODE_FOLLOW = 0x02
    };

    /**
     * @brief Enum contains the GimbalHibernate's
     */
    enum GimbalHibernate: uint16_t {
        GIMBAL_HIBERNATE_SUSPEND = 0x2ab5,
        GIMBAL_HIBERNATE_RESUME = 0x7ef2
    };

    /**
     * @brief Enum contains the LEDMode's
     */
    enum LEDMode: uint8_t {
        LED_MODE_STATIC = 0x71,
        LED_MODE_BREATHE = 0x72,
        LED_MODE_FLASH = 0x73
    };

    /**
     * @brief Enum contains the LEDMask's
     */
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

    /**
     * @brief Enum contains the DeviceID's
     */
    enum DeviceID: uint16_t {
        DEVICE_ID_INTELLI_CONTROLLER = 0x201,
        DEVICE_ID_MOTION_CONTROLLER = 0x202,
        DEVICE_ID_GIMBAL = 0x203,
        DEVICE_ID_HIT_DETECTOR_1 = 0x211,
        DEVICE_ID_HIT_DETECTOR_2 = 0x212,
        DEVICE_ID_HIT_DETECTOR_3 = 0x213,
        DEVICE_ID_HIT_DETECTOR_4 = 0x214
    };

    /**
     * @brief Enum contains the Device Type's
     */
    enum DeviceType: uint16_t {
        DEVICE_TYPE_CHASSIS = 0xc3c9,
        DEVICE_TYPE_GIMBAL = 0x04c9,
        DEVICE_TYPE_BLASTER = 0x17c9,
        DEVICE_TYPE_LED = 0x18c9,
    };

    /**
     * @brief Sequence numbers.
     */
    enum SequenceID: uint8_t {
        SEQUENCE_ID_ZERO = 0x00,
        SEQUENCE_ID_ONE = 0x01,
        SEQUENCE_ID_TWO = 0x02,
        SEQUENCE_ID_THREE = 0x03,
        SEQUENCE_ID_FOUR = 0x04,
    };
} // namespace robomaster