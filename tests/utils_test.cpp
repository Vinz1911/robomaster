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

#include <cstdint>
#include <cstddef>

#include "robomaster/utils.h"
#include "robomaster/message.h"
#include "gtest/gtest.h"

namespace robomaster {
    const static auto MSG_ENABLE = Message(0x202, 0xc309, 0, { 0x40, 0x3f, 0x19, 0x01 });
    const static auto MSG_DISABLE = Message(0x202, 0xc309, 0, { 0x40, 0x3f, 0x19, 0x00 });

    TEST(UtilTest, Little) {
        constexpr uint8_t lsb = 0xAD;
        constexpr uint8_t msb = 0xDE;
        ASSERT_EQ(get_little_endian(lsb, msb), 0xDEAD);
    }

    TEST(UtilTest, calculate_crc16) {
        std::vector<uint8_t> vector_enable = MSG_ENABLE.vector();
        const std::vector<uint8_t> vector_disable = MSG_DISABLE.vector();

        ASSERT_EQ(get_crc16(vector_enable.data(), vector_enable.size() -2), get_crc16(vector_enable.data(), vector_enable.size() - 2));
        ASSERT_NE(get_crc16(vector_enable.data(), vector_enable.size() -2), get_crc16(vector_disable.data(), vector_disable.size() - 2));

        const uint16_t crc16 = get_crc16(vector_enable.data(), vector_enable.size() -2);
        vector_enable[10]++;

        ASSERT_NE(get_crc16(vector_enable.data(), vector_enable.size() - 2), crc16);
    }

    TEST(UtilTest, calculate_crc8) {
        std::vector<uint8_t> vector_enable = MSG_ENABLE.vector();
        const std::vector<uint8_t> vector_disable = MSG_DISABLE.vector();

        ASSERT_EQ(get_crc8(vector_enable.data(), vector_enable.size() -2), get_crc8(vector_enable.data(), vector_enable.size() - 2));
        ASSERT_NE(get_crc8(vector_enable.data(), vector_enable.size() -2), get_crc8(vector_disable.data(), vector_disable.size() - 2));

        const uint8_t crc8 = get_crc8(vector_enable.data(), vector_enable.size() -2);
        vector_enable[10]++;

        ASSERT_NE(get_crc8(vector_enable.data(), vector_enable.size() - 2), crc8);
    }
} // namespace robomaster
