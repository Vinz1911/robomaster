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
    StateGimbal decode_data_gimbal(const size_t index, const Message& message) {
        StateGimbal data; if (index + 4 > message.get_payload().size()) { return data; }
        data.pitch = message.get_int16(index);
        data.yaw = message.get_int16(index + 2);
        return data;
    }

    StateDetector decode_data_detector(const size_t index, const Message& message) {
        StateDetector data; if (index + 4 > message.get_payload().size()) { return data; }
        data.intensity = message.get_uint16(index);
        data.hit_time = std::chrono::high_resolution_clock::now();
        return data;
    }

    StateESC decode_data_esc(const size_t index, const Message& message) {
        StateESC data; if (index + 36 > message.get_payload().size()) { return data; }
        data.speed[0] = message.get_int16(index);
        data.speed[1] = message.get_int16(index + 2);
        data.speed[2] = message.get_int16(index + 4);
        data.speed[3] = message.get_int16(index + 6);

        data.angle[0] = message.get_int16(index + 8);
        data.angle[1] = message.get_int16(index + 10);
        data.angle[2] = message.get_int16(index + 12);
        data.angle[3] = message.get_int16(index + 14);

        data.time_stamp[0] = message.get_uint32(index + 16);
        data.time_stamp[1] = message.get_uint32(index + 20);
        data.time_stamp[2] = message.get_uint32(index + 24);
        data.time_stamp[3] = message.get_uint32(index + 28);

        data.state[0] = message.get_uint8(index + 32);
        data.state[1] = message.get_uint8(index + 33);
        data.state[2] = message.get_uint8(index + 34);
        data.state[3] = message.get_uint8(index + 35);
        return data;
    }

    StateIMU decode_data_imu(const size_t index, const Message& message) {
        StateIMU data; if (index + 24 > message.get_payload().size()) { return data; }
        data.acc_x = message.get_float(index);
        data.acc_y = message.get_float(index + 4);
        data.acc_z = message.get_float(index + 8);

        data.gyro_x = message.get_float(index + 12);
        data.gyro_y = message.get_float(index + 16);
        data.gyro_z = message.get_float(index + 20);
        return data;
    }

    StateAttitude decode_data_attitude(const size_t index, const Message& message) {
        StateAttitude data; if (index + 12 > message.get_payload().size()) { return data; }
        data.yaw = message.get_float(index);
        data.pitch = message.get_float(index + 4);
        data.roll = message.get_float(index + 8);
        return data;
    }

    StateBattery decode_data_battery(const size_t index, const Message& message) {
        StateBattery data; if (index + 10 > message.get_payload().size()) { return data; }
        data.adc = message.get_uint16(index);
        data.temperature = message.get_uint16(index + 2);
        data.current = message.get_int32(index + 4);
        data.percent = message.get_uint8(index + 8);
        data.recv = message.get_uint8(index + 9);
        return data;
    }

    StateVelocity decode_data_velocity(const size_t index, const Message& message) {
        StateVelocity data; if (index + 24 > message.get_payload().size()) { return data; }
        data.vg_x = message.get_float(index);
        data.vg_y = message.get_float(index + 4);
        data.vg_z = message.get_float(index + 8);

        data.vb_x = message.get_float(index + 12);
        data.vb_y = message.get_float(index + 16);
        data.vb_z = message.get_float(index + 20);
        return data;
    }

    StatePosition decode_data_position(const size_t index, const Message& message) {
        StatePosition data; if (index + 12 > message.get_payload().size()) { return data; }
        data.pos_x = message.get_float(index);
        data.pos_y = message.get_float(index + 4);
        data.pos_z = message.get_float(index + 8);
        return data;
    }
} // namespace robomaster