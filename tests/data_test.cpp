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
    TEST(StateDataTest, StateGimbalAttitude) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(12, 0));

        msg.set_value_int16(0, 1000);
        msg.set_value_int16(2, 2000);

        StateGimbalAttitude attitude = decode_data_gimbal(0, msg);

        ASSERT_EQ(attitude.pitch, 1000);
        ASSERT_EQ(attitude.yaw, 2000);
    }

    TEST(StateDataTest, StateChassisESC) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(36, 0));

        msg.set_value_int16(0, 0);
        msg.set_value_int16(2, 1);
        msg.set_value_int16(4, 2);
        msg.set_value_int16(6, 3);

        msg.set_value_int16(8, 10);
        msg.set_value_int16(10, 11);
        msg.set_value_int16(12, 12);
        msg.set_value_int16(14, 13);

        msg.set_value_uint32(16, 20);
        msg.set_value_uint32(20, 21);
        msg.set_value_uint32(24, 22);
        msg.set_value_uint32(28, 23);

        msg.set_value_uint8(32, 30);
        msg.set_value_uint8(33, 31);
        msg.set_value_uint8(34, 32);
        msg.set_value_uint8(35, 33);

        StateChassisESC esc = decode_data_esc(0, msg);

        ASSERT_EQ(esc.speed[0], 0);
        ASSERT_EQ(esc.speed[1], 1);
        ASSERT_EQ(esc.speed[2], 2);
        ASSERT_EQ(esc.speed[3], 3);

        ASSERT_EQ(esc.angle[0], 10);
        ASSERT_EQ(esc.angle[1], 11);
        ASSERT_EQ(esc.angle[2], 12);
        ASSERT_EQ(esc.angle[3], 13);

        ASSERT_EQ(esc.time_stamp[0], 20);
        ASSERT_EQ(esc.time_stamp[1], 21);
        ASSERT_EQ(esc.time_stamp[2], 22);
        ASSERT_EQ(esc.time_stamp[3], 23);

        ASSERT_EQ(esc.state[0], 30);
        ASSERT_EQ(esc.state[1], 31);
        ASSERT_EQ(esc.state[2], 32);
        ASSERT_EQ(esc.state[3], 33);
    }

    TEST(StateDataTest, StateChassisIMU) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(24, 0));

        msg.set_value_float(0, 0.0f);
        msg.set_value_float(4, 1.0f);
        msg.set_value_float(8, 2.0f);

        msg.set_value_float(12, 10.0f);
        msg.set_value_float(16, 11.0f);
        msg.set_value_float(20, 12.0f);

        StateChassisIMU imu = decode_data_imu(0, msg);

        ASSERT_FLOAT_EQ(imu.acc_x, 0.0f);
        ASSERT_FLOAT_EQ(imu.acc_y, 1.0f);
        ASSERT_FLOAT_EQ(imu.acc_z, 2.0f);

        ASSERT_FLOAT_EQ(imu.gyro_x, 10.0f);
        ASSERT_FLOAT_EQ(imu.gyro_y, 11.0f);
        ASSERT_FLOAT_EQ(imu.gyro_z, 12.0f);
    }

    TEST(StateDataTest, StateChassisAttitude) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(12, 0));

        msg.set_value_float(0, 0.0f);
        msg.set_value_float(4, 1.0f);
        msg.set_value_float(8, 2.0f);

        StateChassisAttitude attitude = decode_data_attitude(0, msg);

        ASSERT_FLOAT_EQ(attitude.yaw,  0.0f);
        ASSERT_FLOAT_EQ(attitude.pitch, 1.0f);
        ASSERT_FLOAT_EQ(attitude.roll,   2.0f);
    }

    TEST(StateDataTest, StateChassisBattery) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(10, 0));

        msg.set_value_int16(0, 0);
        msg.set_value_int16(2, 1);
        msg.set_value_int32(4, 2);
        msg.set_value_uint8(8, 3);
        msg.set_value_uint8(9, 4);

        StateChassisBattery battery = decode_data_battery(0, msg);

        ASSERT_EQ(battery.adc,   0);
        ASSERT_EQ(battery.temperature, 1);
        ASSERT_EQ(battery.current,     2);
        ASSERT_EQ(battery.percent,     3);
        ASSERT_EQ(battery.recv,        4);
    }

    TEST(StateDataTest, StateChassisPosition) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(12, 0));

        msg.set_value_float(0, 0.0f);
        msg.set_value_float(4, 1.0f);
        msg.set_value_float(8, 2.0f);

        StateChassisPosition position = decode_data_position(0, msg);

        ASSERT_FLOAT_EQ(position.x, 0.0f);
        ASSERT_FLOAT_EQ(position.y, 1.0f);
        ASSERT_FLOAT_EQ(position.z, 2.0f);
    }

    TEST(StateDataTest, StateChassisVelocity) {
        auto msg = Message(0, 0, 0, std::vector<uint8_t>(24, 0));

        msg.set_value_float(0, 0.0f);
        msg.set_value_float(4, 1.0f);
        msg.set_value_float(8, 2.0f);

        msg.set_value_float(12, 10.0f);
        msg.set_value_float(16, 11.0f);
        msg.set_value_float(20, 12.0f);

        StateChassisVelocity velocity = decode_data_velocity(0, msg);

        ASSERT_FLOAT_EQ(velocity.vg_x, 0.0f);
        ASSERT_FLOAT_EQ(velocity.vg_y, 1.0f);
        ASSERT_FLOAT_EQ(velocity.vg_z, 2.0f);

        ASSERT_FLOAT_EQ(velocity.vb_x, 10.0f);
        ASSERT_FLOAT_EQ(velocity.vb_y, 11.0f);
        ASSERT_FLOAT_EQ(velocity.vb_z, 12.0f);
    }
} // namespace robomaster
