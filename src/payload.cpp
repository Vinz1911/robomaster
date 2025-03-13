/*
 * MIT License
 *
 * Copyright (c) 2025 Vinzenz Weist
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

#include "robomaster/payload.h"

namespace robomaster {
    const uint8_t Payload::DEVICE_SEQUENCE_ZERO = 0x00;
    const uint8_t Payload::DEVICE_SEQUENCE_ONE = 0x01;
    const uint8_t Payload::DEVICE_SEQUENCE_TWO = 0x02;
    const uint8_t Payload::DEVICE_SEQUENCE_THREE = 0x03;
    const uint8_t Payload::DEVICE_SEQUENCE_FOUR = 0x04;
    
    const uint16_t Payload::DEVICE_TYPE_CHASSIS = 0xc3c9;
    const uint16_t Payload::DEVICE_TYPE_GIMBAL = 0x04c9;
    const uint16_t Payload::DEVICE_TYPE_BLASTER = 0x17c9;
    const uint16_t Payload::DEVICE_TYPE_LED = 0x18c9;

    const uint16_t Payload::DEVICE_ID_INTELLI_CONTROLLER = 0x201;
    const uint16_t Payload::DEVICE_ID_MOTION_CONTROLLER = 0x202;
    const uint16_t Payload::DEVICE_ID_GIMBAL = 0x203;
    const uint16_t Payload::DEVICE_ID_HIT_DETECTOR_1 = 0x211;
    const uint16_t Payload::DEVICE_ID_HIT_DETECTOR_2 = 0x212;
    const uint16_t Payload::DEVICE_ID_HIT_DETECTOR_3 = 0x213;
    const uint16_t Payload::DEVICE_ID_HIT_DETECTOR_4 = 0x214;

    const std::vector<uint8_t> Payload::BOOT_CHASSIS_PRIMARY = { 0x40, 0x48, 0x04, 0x00, 0x09, 0x00 };
    const std::vector<uint8_t> Payload::BOOT_CHASSIS_SECONDARY = { 0x40, 0x48, 0x01, 0x09, 0x00, 0x00, 0x00, 0x03 };
    const std::vector<uint8_t> Payload::BOOT_CHASSIS_SUB = { 0x40, 0x48, 0x03, 0x09, 0x01, 0x03, 0x00, 0x07, 0xa7, 0x02, 0x29, 0x88, 0x03, 0x00, 0x02, 0x00, 0x66, 0x3e, 0x3e, 0x4c, 0x03, 0x00, 0x02, 0x00, 0xfb, 0xdc, 0xf5, 0xd7, 0x03, 0x00, 0x02, 0x00, 0x09, 0xa3, 0x26, 0xe2, 0x03, 0x00, 0x02, 0x00, 0xf4, 0x1d, 0x1c, 0xdc, 0x03, 0x00, 0x02, 0x00, 0x42, 0xee, 0x13, 0x1d, 0x03, 0x00, 0x02, 0x00, 0xb3, 0xf7, 0xe6, 0x47, 0x03, 0x00, 0x02, 0x00, 0x32, 0x00 };
    const std::vector<uint8_t> Payload::BOOT_GIMBAL_SUB = { 0x40, 0x04, 0x1e, 0x05, 0xff };
    const std::vector<uint8_t> Payload::BOOT_LED_RST = { 0x00, 0x3f, 0x32, 0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    const std::vector<uint8_t> Payload::CHASSIS_MODE = { 0x40, 0x3f, 0x19, 0x00 };
    const std::vector<uint8_t> Payload::CHASSIS_RPM = { 0x40, 0x3f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const std::vector<uint8_t> Payload::CHASSIS_VELOCITY = { 0x00, 0x3f, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const std::vector<uint8_t> Payload::CHASSIS_POSITION = { 0x00, 0x3f, 0x25, 0x02, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00 };

    const std::vector<uint8_t> Payload::GIMBAL_MODE = { 0x40, 0x04, 0x4c, 0x00 };
    const std::vector<uint8_t> Payload::GIMBAL_HIBERNATE = { 0x20, 0x04, 0x0d, 0x00, 0x00 };
    const std::vector<uint8_t> Payload::GIMBAL_DEGREE = { 0x00, 0x04, 0x69, 0x08, 0x05, 0x00, 0x00, 0x00, 0x00 };
    const std::vector<uint8_t> Payload::GIMBAL_VELOCITY = { 0x00, 0x04, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcd };
    const std::vector<uint8_t> Payload::GIMBAL_POSITION = { 0x00, 0x3f, 0xb0, 0x03, 0x08, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const std::vector<uint8_t> Payload::GIMBAL_RECENTER = { 0x00, 0x3f, 0xb2, 0x01, 0x08, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    const std::vector<uint8_t> Payload::BLASTER_MODE_GEL = { 0x00, 0x3f, 0x51, 0x00 };
    const std::vector<uint8_t> Payload::BLASTER_MODE_LED = { 0x00, 0x3f, 0x55, 0x73, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00 };
    const std::vector<uint8_t> Payload::LED_MODE = { 0x00, 0x3f, 0x32, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const std::vector<uint8_t> Payload::HEARTBEAT = { 0x00, 0x3f, 0x60, 0x00, 0x04, 0x20, 0x00, 0x01, 0x00, 0x40, 0x00, 0x02, 0x10, 0x00, 0x03, 0x00, 0x00 };
} // namespace robomaster
