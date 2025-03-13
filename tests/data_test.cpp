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
    TEST(StateDataTest, StateGimbal) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(12, 0));

        msg.set_int16(0, 1000);
        msg.set_int16(2, 2000);

        auto [pitch, yaw] = decode_data_gimbal(0, msg);

        ASSERT_EQ(pitch, 1000);
        ASSERT_EQ(yaw, 2000);
    }

    TEST(StateDataTest, StateESC) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(36, 0));

        msg.set_int16(0, 0);
        msg.set_int16(2, 1);
        msg.set_int16(4, 2);
        msg.set_int16(6, 3);

        msg.set_int16(8, 10);
        msg.set_int16(10, 11);
        msg.set_int16(12, 12);
        msg.set_int16(14, 13);

        msg.set_uint32(16, 20);
        msg.set_uint32(20, 21);
        msg.set_uint32(24, 22);
        msg.set_uint32(28, 23);

        msg.set_uint8(32, 30);
        msg.set_uint8(33, 31);
        msg.set_uint8(34, 32);
        msg.set_uint8(35, 33);

        auto [speed, angle, time_stamp, state] = decode_data_esc(0, msg);

        ASSERT_EQ(speed[0], 0);
        ASSERT_EQ(speed[1], 1);
        ASSERT_EQ(speed[2], 2);
        ASSERT_EQ(speed[3], 3);

        ASSERT_EQ(angle[0], 10);
        ASSERT_EQ(angle[1], 11);
        ASSERT_EQ(angle[2], 12);
        ASSERT_EQ(angle[3], 13);

        ASSERT_EQ(time_stamp[0], 20);
        ASSERT_EQ(time_stamp[1], 21);
        ASSERT_EQ(time_stamp[2], 22);
        ASSERT_EQ(time_stamp[3], 23);

        ASSERT_EQ(state[0], 30);
        ASSERT_EQ(state[1], 31);
        ASSERT_EQ(state[2], 32);
        ASSERT_EQ(state[3], 33);
    }

    TEST(StateDataTest, StateIMU) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(24, 0));

        msg.set_float(0, 0.0f);
        msg.set_float(4, 1.0f);
        msg.set_float(8, 2.0f);

        msg.set_float(12, 10.0f);
        msg.set_float(16, 11.0f);
        msg.set_float(20, 12.0f);

        auto [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z] = decode_data_imu(0, msg);

        ASSERT_FLOAT_EQ(acc_x, 0.0f);
        ASSERT_FLOAT_EQ(acc_y, 1.0f);
        ASSERT_FLOAT_EQ(acc_z, 2.0f);

        ASSERT_FLOAT_EQ(gyro_x, 10.0f);
        ASSERT_FLOAT_EQ(gyro_y, 11.0f);
        ASSERT_FLOAT_EQ(gyro_z, 12.0f);
    }

    TEST(StateDataTest, StateAttitude) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(12, 0));

        msg.set_float(0, 0.0f);
        msg.set_float(4, 1.0f);
        msg.set_float(8, 2.0f);

        auto [roll, pitch, yaw] = decode_data_attitude(0, msg);

        ASSERT_FLOAT_EQ(yaw, 0.0f);
        ASSERT_FLOAT_EQ(pitch, 1.0f);
        ASSERT_FLOAT_EQ(roll, 2.0f);
    }

    TEST(StateDataTest, StateBattery) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(10, 0));

        msg.set_int16(0, 0);
        msg.set_int16(2, 1);
        msg.set_int32(4, 2);
        msg.set_uint8(8, 3);
        msg.set_uint8(9, 4);

        auto [adc, temperature, current, percent, recv] = decode_data_battery(0, msg);

        ASSERT_EQ(adc, 0);
        ASSERT_EQ(temperature, 1);
        ASSERT_EQ(current, 2);
        ASSERT_EQ(percent, 3);
        ASSERT_EQ(recv, 4);
    }

    TEST(StateDataTest, StatePosition) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(12, 0));

        msg.set_float(0, 0.0f);
        msg.set_float(4, 1.0f);
        msg.set_float(8, 2.0f);

        auto [pos_x, pos_y, pos_z] = decode_data_position(0, msg);

        ASSERT_FLOAT_EQ(pos_x, 0.0f);
        ASSERT_FLOAT_EQ(pos_y, 1.0f);
        ASSERT_FLOAT_EQ(pos_z, 2.0f);
    }

    TEST(StateDataTest, StateVelocity) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(24, 0));

        msg.set_float(0, 0.0f);
        msg.set_float(4, 1.0f);
        msg.set_float(8, 2.0f);

        msg.set_float(12, 10.0f);
        msg.set_float(16, 11.0f);
        msg.set_float(20, 12.0f);

        auto [vg_x, vg_y, vg_z, vb_x, vb_y, vb_z] = decode_data_velocity(0, msg);

        ASSERT_FLOAT_EQ(vg_x, 0.0f);
        ASSERT_FLOAT_EQ(vg_y, 1.0f);
        ASSERT_FLOAT_EQ(vg_z, 2.0f);

        ASSERT_FLOAT_EQ(vb_x, 10.0f);
        ASSERT_FLOAT_EQ(vb_y, 11.0f);
        ASSERT_FLOAT_EQ(vb_z, 12.0f);
    }
} // namespace robomaster
