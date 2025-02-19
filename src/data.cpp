/*
 * Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
 *
 * This project contains contributions from multiple authors.
 * The original code is licensed under the MIT License by Fraunhofer IML.
 * All modifications and additional code are licensed under the MIT License by Vinzenz Weist.
 */

#include "robomaster/data.h"

namespace robomaster {
    StateGimbalAttitude decode_data_gimbal(const size_t index, const Message& msg) {
        StateGimbalAttitude data; if (index + 4 > msg.get_payload().size()) { return data; }
        data.pitch = msg.get_value_int16(index);
        data.yaw = msg.get_value_int16(index + 2);
        return data;
    }

    StateChassisESC decode_data_esc(const size_t index, const Message& msg) {
        StateChassisESC data; if (index + 36 > msg.get_payload().size()) { return data; }
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

    StateChassisIMU decode_data_imu(const size_t index, const Message& msg) {
        StateChassisIMU data; if (index + 24 > msg.get_payload().size()) { return data; }
        data.acc_x = msg.get_value_float(index);
        data.acc_y = msg.get_value_float(index + 4);
        data.acc_z = msg.get_value_float(index + 8);

        data.gyro_x = msg.get_value_float(index + 12);
        data.gyro_y = msg.get_value_float(index + 16);
        data.gyro_z = msg.get_value_float(index + 20);
        return data;
    }

    StateChassisAttitude decode_data_attitude(const size_t index, const Message& msg) {
        StateChassisAttitude data; if (index + 12 > msg.get_payload().size()) { return data; }
        data.yaw = msg.get_value_float(index);
        data.pitch = msg.get_value_float(index + 4);
        data.roll = msg.get_value_float(index + 8);
        return data;
    }

    StateChassisBattery decode_data_battery(const size_t index, const Message& msg) {
        StateChassisBattery data; if (index + 10 > msg.get_payload().size()) { return data; }
        data.adc = msg.get_value_uint16(index);
        data.temperature = msg.get_value_uint16(index + 2);
        data.current = msg.get_value_int32(index + 4);
        data.percent = msg.get_value_uint8(index + 8);
        data.recv = msg.get_value_uint8(index + 9);
        return data;
    }

    StateChassisVelocity decode_data_velocity(const size_t index, const Message& msg) {
        StateChassisVelocity data; if (index + 24 > msg.get_payload().size()) { return data; }
        data.vg_x = msg.get_value_float(index);
        data.vg_y = msg.get_value_float(index + 4);
        data.vg_z = msg.get_value_float(index + 8);

        data.vb_x = msg.get_value_float(index + 12);
        data.vb_y = msg.get_value_float(index + 16);
        data.vb_z = msg.get_value_float(index + 20);
        return data;
    }

    StateChassisPosition decode_data_position(const size_t index, const Message& msg) {
        StateChassisPosition data; if (index + 12 > msg.get_payload().size()) { return data; }
        data.x = msg.get_value_float(index);
        data.y = msg.get_value_float(index + 4);
        data.z = msg.get_value_float(index + 8);
        return data;
    }
} // namespace robomaster