/*
 * Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
 *
 * This project contains contributions from multiple authors.
 * The original code is licensed under the MIT License by Fraunhofer IML.
 * All modifications and additional code are licensed under the MIT License by Vinzenz Weist.
 */

#include <cstdint>
#include <string>

#include "robomaster/utils.h"
#include "robomaster/message.h"
#include "robomaster/definitions.h"
#include "gtest/gtest.h"

namespace robomaster {
    const static auto MSG_ENABLE = Message(DEVICE_ID_INTELLI_CONTROLLER, 0xc309, 0, { 0x40, 0x3f, 0x19, 0x01 });
    const static auto MSG_DISABLE = Message(DEVICE_ID_INTELLI_CONTROLLER, 0xc309, 0, { 0x40, 0x3f, 0x19, 0x00 });

    TEST(UtilTest, Little) {
        constexpr uint8_t lsb = 0xAD;
        constexpr uint8_t msb = 0xDE;
        ASSERT_EQ(little_endian_to_uint16(lsb, msb), 0xDEAD);
    }

    TEST(UtilTest, calculate_crc16) {
        std::vector<uint8_t> vector_enable = MSG_ENABLE.to_vector();
        std::vector<uint8_t> vector_disable = MSG_DISABLE.to_vector();

        ASSERT_EQ(calculate_crc16(vector_enable.data(), vector_enable.size() -2), calculate_crc16(vector_enable.data(), vector_enable.size() - 2));
        ASSERT_NE(calculate_crc16(vector_enable.data(), vector_enable.size() -2), calculate_crc16(vector_disable.data(), vector_disable.size() - 2));

        const uint16_t crc16 = calculate_crc16(vector_enable.data(), vector_enable.size() -2);
        vector_enable[10]++;

        ASSERT_NE(calculate_crc16(vector_enable.data(), vector_enable.size() - 2), crc16);
    }

    TEST(UtilTest, calculate_crc8) {
        std::vector<uint8_t> vector_enable = MSG_ENABLE.to_vector();
        std::vector<uint8_t> vector_disable = MSG_DISABLE.to_vector();

        ASSERT_EQ(calculate_crc8(vector_enable.data(), vector_enable.size() -2), calculate_crc8(vector_enable.data(), vector_enable.size() - 2));
        ASSERT_NE(calculate_crc8(vector_enable.data(), vector_enable.size() -2), calculate_crc8(vector_disable.data(), vector_disable.size() - 2));

        const uint8_t crc8 = calculate_crc8(vector_enable.data(), vector_enable.size() -2);
        vector_enable[10]++;

        ASSERT_NE(calculate_crc8(vector_enable.data(), vector_enable.size() - 2), crc8);
    }

    TEST(UtilTest, clip) {
        ASSERT_FLOAT_EQ(clip<float>(-10.0f, -1.0f, 1.0f), -1.0f);
        ASSERT_FLOAT_EQ(clip<float>( -1.0f, -1.0f, 1.0f), -1.0f);
        ASSERT_FLOAT_EQ(clip<float>(  0.0f, -1.0f, 1.0f),  0.0f);
        ASSERT_FLOAT_EQ(clip<float>(  1.0f, -1.0f, 1.0f),  1.0f);
        ASSERT_FLOAT_EQ(clip<float>( 10.0f, -1.0f, 1.0f),  1.0f);

        ASSERT_FLOAT_EQ(clip<int>(-101, -100, 100), -100);
        ASSERT_FLOAT_EQ(clip<int>(-100, -100, 100), -100);
        ASSERT_FLOAT_EQ(clip<int>(   0, -100, 100),    0);
        ASSERT_FLOAT_EQ(clip<int>( 100, -100, 100),  100);
        ASSERT_FLOAT_EQ(clip<int>( 101, -100, 100),  100);
    }
} // namespace robomaster
