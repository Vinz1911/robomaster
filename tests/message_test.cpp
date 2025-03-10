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

#include "robomaster/data.h"
#include "gtest/gtest.h"

namespace robomaster {
    TEST(MessageTest, ValueUint8) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>{1,2});

        ASSERT_EQ(msg.get_uint8(0), 1);
        ASSERT_EQ(msg.get_uint8(1), 2);

        msg.set_uint8(0, 10);
        msg.set_uint8(1, 11);

        ASSERT_EQ(msg.get_payload()[0], 10);
        ASSERT_EQ(msg.get_payload()[1], 11);
    }

    TEST(MessageTest, ValueUint16) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>{0xDE, 0xAD, 0xBE, 0xEF});

        ASSERT_EQ(msg.get_uint16(0), 0xADDE);
        ASSERT_EQ(msg.get_uint16(2), 0xEFBE);

        msg.set_uint16(0, 0xDEAD);
        msg.set_uint16(2, 0xBEEF);

        ASSERT_EQ(msg.get_payload()[0], 0xAD);
        ASSERT_EQ(msg.get_payload()[1], 0xDE);
        ASSERT_EQ(msg.get_payload()[2], 0xEF);
        ASSERT_EQ(msg.get_payload()[3], 0xBE);
    }

    TEST(MessageTest, ValueUint32) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>{0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xCA, 0xFB, 0xAD});

        ASSERT_EQ(msg.get_uint32(0), 0xEFBEADDE);
        ASSERT_EQ(msg.get_uint32(4), 0xADFBCADE);

        msg.set_uint32(0, 0xDECAFBAD);
        msg.set_uint32(4, 0xDEADBEEF);

        ASSERT_EQ(msg.get_payload()[0], 0xAD);
        ASSERT_EQ(msg.get_payload()[1], 0xFB);
        ASSERT_EQ(msg.get_payload()[2], 0xCA);
        ASSERT_EQ(msg.get_payload()[3], 0xDE);
        ASSERT_EQ(msg.get_payload()[4], 0xEF);
        ASSERT_EQ(msg.get_payload()[5], 0xBE);
        ASSERT_EQ(msg.get_payload()[6], 0xAD);
        ASSERT_EQ(msg.get_payload()[7], 0xDE);
    }


    TEST(MessageTest, ValueInt8) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>{255,1});

        ASSERT_EQ(msg.get_int8(0), -1);
        ASSERT_EQ(msg.get_int8(1), 1);

        msg.set_int8(0, -10);
        msg.set_int8(1,  10);

        ASSERT_EQ(msg.get_payload()[0], 246);
        ASSERT_EQ(msg.get_payload()[1], 10);
    }

    TEST(MessageTest, ValueInt16) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>{0xDE, 0xAD, 0xBE, 0xEF});

        ASSERT_EQ(msg.get_int16(0), -21026);
        ASSERT_EQ(msg.get_int16(2), -4162);

        msg.set_int16(0, 0xDEAD);
        msg.set_int16(2, 0xBEEF);

        ASSERT_EQ(msg.get_payload()[0], 0xAD);
        ASSERT_EQ(msg.get_payload()[1], 0xDE);
        ASSERT_EQ(msg.get_payload()[2], 0xEF);
        ASSERT_EQ(msg.get_payload()[3], 0xBE);
    }

    TEST(MessageTest, ValueInt32) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>{0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xCA, 0xFB, 0xAD});

        ASSERT_EQ(msg.get_int32(0), 0xEFBEADDE);
        ASSERT_EQ(msg.get_int32(4), 0xADFBCADE);

        msg.set_int32(0, 0xDECAFBAD);
        msg.set_int32(4, 0xDEADBEEF);

        ASSERT_EQ(msg.get_payload()[0], 0xAD);
        ASSERT_EQ(msg.get_payload()[1], 0xFB);
        ASSERT_EQ(msg.get_payload()[2], 0xCA);
        ASSERT_EQ(msg.get_payload()[3], 0xDE);
        ASSERT_EQ(msg.get_payload()[4], 0xEF);
        ASSERT_EQ(msg.get_payload()[5], 0xBE);
        ASSERT_EQ(msg.get_payload()[6], 0xAD);
        ASSERT_EQ(msg.get_payload()[7], 0xDE);
    }

    TEST(MessageTest, ValueFloat32) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>{0x00, 0x00, 0x80, 0xBF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F});

        ASSERT_FLOAT_EQ(msg.get_float(0), -1.0f);
        ASSERT_FLOAT_EQ(msg.get_float(4),  0.0);
        ASSERT_FLOAT_EQ(msg.get_float(8),  1.0f);

        msg.set_float(0, 1337.0f); // 0x44a72000
        msg.set_float(4, 3.14f);   // 0x4048f5c3
        msg.set_float(8, 0.0f);    // 0x00000000

        ASSERT_EQ(msg.get_payload()[0], 0x00);
        ASSERT_EQ(msg.get_payload()[1], 0x20);
        ASSERT_EQ(msg.get_payload()[2], 0xA7);
        ASSERT_EQ(msg.get_payload()[3], 0x44);
        ASSERT_EQ(msg.get_payload()[4], 0xC3);
        ASSERT_EQ(msg.get_payload()[5], 0xF5);
        ASSERT_EQ(msg.get_payload()[6], 0x48);
        ASSERT_EQ(msg.get_payload()[7], 0x40);
        ASSERT_EQ(msg.get_payload()[8], 0x00);
        ASSERT_EQ(msg.get_payload()[9], 0x00);
        ASSERT_EQ(msg.get_payload()[10], 0x00);
        ASSERT_EQ(msg.get_payload()[11], 0x00);
    }

    TEST(MessageTest, Creation) {
        auto msg = Message(0, 1337, 1, std::vector<uint8_t>{0xDE, 0xAD, 0xBE, 0xEF});

        ASSERT_EQ(msg.get_device_id(), 0);
        ASSERT_EQ(msg.get_type(), 1337);
        ASSERT_EQ(msg.get_sequence(), 1);
        ASSERT_EQ(msg.get_length(), 14);
        ASSERT_TRUE(msg.is_valid());
        ASSERT_EQ(msg.get_payload()[0], 0xDE);
        ASSERT_EQ(msg.get_payload()[1], 0xAD);
        ASSERT_EQ(msg.get_payload()[2], 0xBE);
        ASSERT_EQ(msg.get_payload()[3], 0xEF);

        msg = Message(1337, std::vector<uint8_t>{0xDE, 0xAD, 0xBE, 0xEF});

        ASSERT_EQ(msg.get_device_id(), 1337);
        ASSERT_EQ(msg.get_type(), 0);
        ASSERT_EQ(msg.get_sequence(), 0);
        ASSERT_EQ(msg.get_length(), 10);
        ASSERT_FALSE(msg.is_valid());
        ASSERT_EQ(msg.get_payload().size(), 0);
    }
} // namespace robomaster
