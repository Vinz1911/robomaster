// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#include "robomaster/data.h"
#include "gtest/gtest.h"

namespace robomaster {
    TEST(DataTest, DecodeDataEsc) {
        Message msg = Message(0, 0, 0, std::vector<uint8_t>(36, 0));

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

        DataEsc esc = decode_data_esc(0, msg);

        ASSERT_TRUE(esc.has_data);

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

    TEST(DataTest, DecodeDataImu) {
        Message msg = Message(0, 0, 0, std::vector<uint8_t>(24, 0));

        msg.set_value_float(0, 0.0f);
        msg.set_value_float(4, 1.0f);
        msg.set_value_float(8, 2.0f);

        msg.set_value_float(12, 10.0f);
        msg.set_value_float(16, 11.0f);
        msg.set_value_float(20, 12.0f);

        DataImu imu = decode_data_imu(0, msg);

        ASSERT_TRUE(imu.has_data);

        ASSERT_FLOAT_EQ(imu.acc_x, 0.0f);
        ASSERT_FLOAT_EQ(imu.acc_y, 1.0f);
        ASSERT_FLOAT_EQ(imu.acc_z, 2.0f);

        ASSERT_FLOAT_EQ(imu.gyro_x, 10.0f);
        ASSERT_FLOAT_EQ(imu.gyro_y, 11.0f);
        ASSERT_FLOAT_EQ(imu.gyro_z, 12.0f);
    }

    TEST(DataTest, DecodeDataAttitude) {
        Message msg = Message(0, 0, 0, std::vector<uint8_t>(12, 0));

        msg.set_value_float(0, 0.0f);
        msg.set_value_float(4, 1.0f);
        msg.set_value_float(8, 2.0f);

        DataAttitude attitude = decode_data_attitude(0, msg);

        ASSERT_TRUE(attitude.has_data);

        ASSERT_FLOAT_EQ(attitude.yaw,  0.0f);
        ASSERT_FLOAT_EQ(attitude.pitch, 1.0f);
        ASSERT_FLOAT_EQ(attitude.roll,   2.0f);
    }

    TEST(DataTest, DecodeDataBattery) {
        Message msg = Message(0, 0, 0, std::vector<uint8_t>(10, 0));

        msg.set_value_int16(0, 0);
        msg.set_value_int16(2, 1);
        msg.set_value_int32(4, 2);
        msg.set_value_uint8(8, 3);
        msg.set_value_uint8(9, 4);

        DataBattery battery = decode_data_battery(0, msg);

        ASSERT_TRUE(battery.has_data);

        ASSERT_EQ(battery.adc_value,   0);
        ASSERT_EQ(battery.temperature, 1);
        ASSERT_EQ(battery.current,     2);
        ASSERT_EQ(battery.percent,     3);
        ASSERT_EQ(battery.recv,        4);
    }

    TEST(DataTest, DecodeDataPosition) {
        Message msg = Message(0, 0, 0, std::vector<uint8_t>(12, 0));

        msg.set_value_float(0, 0.0f);
        msg.set_value_float(4, 1.0f);
        msg.set_value_float(8, 2.0f);

        DataPosition position = decode_data_position(0, msg);

        ASSERT_TRUE(position.has_data);

        ASSERT_FLOAT_EQ(position.x, 0.0f);
        ASSERT_FLOAT_EQ(position.y, 1.0f);
        ASSERT_FLOAT_EQ(position.z, 2.0f);
    }

    TEST(DataTest, DecodeDataVelocity) {
        Message msg = Message(0, 0, 0, std::vector<uint8_t>(24, 0));

        msg.set_value_float(0, 0.0f);
        msg.set_value_float(4, 1.0f);
        msg.set_value_float(8, 2.0f);

        msg.set_value_float(12, 10.0f);
        msg.set_value_float(16, 11.0f);
        msg.set_value_float(20, 12.0f);

        DataVelocity velocity = decode_data_velocity(0, msg);

        ASSERT_TRUE(velocity.has_data);

        ASSERT_FLOAT_EQ(velocity.vgx, 0.0f);
        ASSERT_FLOAT_EQ(velocity.vgy, 1.0f);
        ASSERT_FLOAT_EQ(velocity.vgz, 2.0f);

        ASSERT_FLOAT_EQ(velocity.vbx, 10.0f);
        ASSERT_FLOAT_EQ(velocity.vby, 11.0f);
        ASSERT_FLOAT_EQ(velocity.vbz, 12.0f);
    }
} // namespace robomaster
