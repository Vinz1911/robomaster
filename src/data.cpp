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

namespace robomaster {
    StateGimbal decode_data_gimbal(const size_t index, const Message& msg) {
        StateGimbal data; if (index + 4 > msg.get_payload().size()) { return data; }
        data.pitch = msg.get_value_int16(index);
        data.yaw = msg.get_value_int16(index + 2);
        return data;
    }

    StateESC decode_data_esc(const size_t index, const Message& msg) {
        StateESC data; if (index + 36 > msg.get_payload().size()) { return data; }
        data.speed[0] = msg.get_value_int16(index);
        data.speed[1] = msg.get_value_int16(index + 2);
        data.speed[2] = msg.get_value_int16(index + 4);
        data.speed[3] = msg.get_value_int16(index + 6);

        data.angle[0] = msg.get_value_int16(index + 8);
        data.angle[1] = msg.get_value_int16(index + 10);
        data.angle[2] = msg.get_value_int16(index + 12);
        data.angle[3] = msg.get_value_int16(index + 14);

        data.time_stamp[0] = msg.get_value_uint32(index + 16);
        data.time_stamp[1] = msg.get_value_uint32(index + 20);
        data.time_stamp[2] = msg.get_value_uint32(index + 24);
        data.time_stamp[3] = msg.get_value_uint32(index + 28);

        data.state[0] = msg.get_value_uint8(index + 32);
        data.state[1] = msg.get_value_uint8(index + 33);
        data.state[2] = msg.get_value_uint8(index + 34);
        data.state[3] = msg.get_value_uint8(index + 35);
        return data;
    }

    StateIMU decode_data_imu(const size_t index, const Message& msg) {
        StateIMU data; if (index + 24 > msg.get_payload().size()) { return data; }
        data.acc_x = msg.get_value_float(index);
        data.acc_y = msg.get_value_float(index + 4);
        data.acc_z = msg.get_value_float(index + 8);

        data.gyro_x = msg.get_value_float(index + 12);
        data.gyro_y = msg.get_value_float(index + 16);
        data.gyro_z = msg.get_value_float(index + 20);
        return data;
    }

    StateAttitude decode_data_attitude(const size_t index, const Message& msg) {
        StateAttitude data; if (index + 12 > msg.get_payload().size()) { return data; }
        data.yaw = msg.get_value_float(index);
        data.pitch = msg.get_value_float(index + 4);
        data.roll = msg.get_value_float(index + 8);
        return data;
    }

    StateBattery decode_data_battery(const size_t index, const Message& msg) {
        StateBattery data; if (index + 10 > msg.get_payload().size()) { return data; }
        data.adc = msg.get_value_uint16(index);
        data.temperature = msg.get_value_uint16(index + 2);
        data.current = msg.get_value_int32(index + 4);
        data.percent = msg.get_value_uint8(index + 8);
        data.recv = msg.get_value_uint8(index + 9);
        return data;
    }

    StateVelocity decode_data_velocity(const size_t index, const Message& msg) {
        StateVelocity data; if (index + 24 > msg.get_payload().size()) { return data; }
        data.vg_x = msg.get_value_float(index);
        data.vg_y = msg.get_value_float(index + 4);
        data.vg_z = msg.get_value_float(index + 8);

        data.vb_x = msg.get_value_float(index + 12);
        data.vb_y = msg.get_value_float(index + 16);
        data.vb_z = msg.get_value_float(index + 20);
        return data;
    }

    StatePosition decode_data_position(const size_t index, const Message& msg) {
        StatePosition data; if (index + 12 > msg.get_payload().size()) { return data; }
        data.x = msg.get_value_float(index);
        data.y = msg.get_value_float(index + 4);
        data.z = msg.get_value_float(index + 8);
        return data;
    }
} // namespace robomaster