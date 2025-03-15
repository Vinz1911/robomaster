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

#pragma once
#include <vector>
#include <cstdint>

namespace robomaster {
    class Payload {
        /**
         * @brief The Device Sequence's.
         */
        static const uint8_t DEVICE_SEQ_ZERO;
        static const uint8_t DEVICE_SEQ_ONE;
        static const uint8_t DEVICE_SEQ_TWO;
        static const uint8_t DEVICE_SEQ_THREE;
        static const uint8_t DEVICE_SEQ_FOUR;

        /**
         * @brief The Device Type's.
         */
        static const uint16_t DEVICE_TYPE_CHASSIS;
        static const uint16_t DEVICE_TYPE_GIMBAL;
        static const uint16_t DEVICE_TYPE_BLASTER;
        static const uint16_t DEVICE_TYPE_LED;

        /**
         * @brief The Device ID's.
         */
        static const uint16_t DEVICE_ID_INTELLI_CONTROLLER;
        static const uint16_t DEVICE_ID_MOTION_CONTROLLER;
        static const uint16_t DEVICE_ID_GIMBAL;
        static const uint16_t DEVICE_ID_HIT_DETECTOR_1;
        static const uint16_t DEVICE_ID_HIT_DETECTOR_2;
        static const uint16_t DEVICE_ID_HIT_DETECTOR_3;
        static const uint16_t DEVICE_ID_HIT_DETECTOR_4;

        /**
         * @brief The Message Type's.
         */
        static const uint16_t DEVICE_RC_TYPE_MOTION_CONTROLLER;
        static const uint16_t DEVICE_RC_TYPE_GIMBAL;
        static const uint16_t DEVICE_RC_TYPE_HIT_DETECTOR_1;
        static const uint16_t DEVICE_RC_TYPE_HIT_DETECTOR_2;
        static const uint16_t DEVICE_RC_TYPE_HIT_DETECTOR_3;
        static const uint16_t DEVICE_RC_TYPE_HIT_DETECTOR_4;

        /**
         * @brief The boot payload data.
         * [1, 2, 3] -> Chassis, [4] -> Gimbal, [5] -> LED's
         */
        static const std::vector<uint8_t> BOOT_CHASSIS_SPECIAL;
        static const std::vector<uint8_t> BOOT_CHASSIS_CONFIRM;
        static const std::vector<uint8_t> BOOT_CHASSIS_INFO;
        static const std::vector<uint8_t> BOOT_GIMBAL_INFO;
        static const std::vector<uint8_t> BOOT_LED_RESET;

        /**
         * @brief The chassis payload data.
         */
        static const std::vector<uint8_t> CHASSIS_MODE;
        static const std::vector<uint8_t> CHASSIS_RPM;
        static const std::vector<uint8_t> CHASSIS_VELOCITY;
        static const std::vector<uint8_t> CHASSIS_POSITION;

        /**
         * @brief The gimbal payload data.
         */
        static const std::vector<uint8_t> GIMBAL_MODE;
        static const std::vector<uint8_t> GIMBAL_HIBERNATE;
        static const std::vector<uint8_t> GIMBAL_DEGREE;
        static const std::vector<uint8_t> GIMBAL_VELOCITY;
        static const std::vector<uint8_t> GIMBAL_POSITION;
        static const std::vector<uint8_t> GIMBAL_RECENTER;

        /**
         * @brief The blaster payload data.
         */
        static const std::vector<uint8_t> BLASTER_MODE_GEL;
        static const std::vector<uint8_t> BLASTER_MODE_LED;

        /**
         * @brief The led payload data.
         */
        static const std::vector<uint8_t> LED_MODE;

        /**
         * @brief The heartbeat payload data.
         */
        static const std::vector<uint8_t> HEART_BEAT;

        /**
         * @brief The message payload data.
         */
        static const std::vector<uint8_t> MESSAGE_MOTION_CONTROLLER;
        static const std::vector<uint8_t> MESSAGE_GIMBAL;
        static const std::vector<uint8_t> MESSAGE_HIT_DETECTOR_1;
        static const std::vector<uint8_t> MESSAGE_HIT_DETECTOR_2;
        static const std::vector<uint8_t> MESSAGE_HIT_DETECTOR_3;
        static const std::vector<uint8_t> MESSAGE_HIT_DETECTOR_4;

    public:
        /**
         * @brief Constructor of the Payload class.
         */
        Payload(/* args */);

        /**
         * @brief Destructor of the Payload class.
         */
        ~Payload() = default;

        /**
         * @brief Friend class RoboMaster.
         */
        friend class RoboMaster;

        /**
         * @brief Friend class RoboMaster.
         */
        friend class Handler;
    };
} // namespace robomaster