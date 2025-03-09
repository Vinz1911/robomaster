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
#include <vector>
#include <cstdint>

namespace robomaster {
    class Payload {
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
         * @brief The boot payload data.
         */
        static const std::vector<uint8_t> BOOT_1_CHASSIS;
        static const std::vector<uint8_t> BOOT_2_CHASSIS;
        static const std::vector<uint8_t> BOOT_3_CHASSIS;
        static const std::vector<uint8_t> BOOT_4_GIMBAL;
        static const std::vector<uint8_t> BOOT_5_LED;

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
        static const std::vector<uint8_t> HEARTBEAT;
    };
} // namespace robomaster