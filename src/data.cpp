// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#include <iostream>

#include "robomaster/data.h"

namespace robomaster {
    DataPosition decode_data_position(const size_t index, const Message &msg) {
        DataPosition data;
        if (index + 12 <= msg.get_payload().size()) {
            data.x = msg.get_value_float(index);
            data.y = msg.get_value_float(index + 4);
            data.z = msg.get_value_float(index + 8);
            data.has_data = true;
        } return data;
    }

    DataEsc decode_data_esc(const size_t index, const Message &msg) {
        DataEsc data;
        if (index + 36 <= msg.get_payload().size()) {
            data.speed[0] = msg.get_value_int16(index);
            data.speed[1] = msg.get_value_int16(index + 2);
            data.speed[2] = msg.get_value_int16(index + 4);
            data.speed[3] = msg.get_value_int16(index + 6);
            data.angle[0] = msg.get_value_int16(index + 8);
            data.angle[1] = msg.get_value_int16 (index + 10);
            data.angle[2] = msg.get_value_int16 (index + 12);
            data.angle[3] = msg.get_value_int16(index + 14);
            data.time_stamp[0] = msg.get_value_uint32(index + 16);
            data.time_stamp[1] = msg.get_value_uint32(index + 20);
            data.time_stamp[2] = msg.get_value_uint32(index + 24);
            data.time_stamp[3] = msg.get_value_uint32(index + 28);
            data.state[0] = msg.get_value_uint8(index + 32);
            data.state[1] = msg.get_value_uint8(index + 33);
            data.state[2] = msg.get_value_uint8(index + 34);
            data.state[3] = msg.get_value_uint8(index + 35);
            data.has_data = true;
        } return data;
    }

    DataImu decode_data_imu(const size_t index, const Message &msg) {
        DataImu data;
        if (index + 24 <= msg.get_payload().size()) {
            data.acc_x = msg.get_value_float(index);
            data.acc_y = msg.get_value_float(index + 4);
            data.acc_z = msg.get_value_float(index + 8);
            data.gyro_x = msg.get_value_float(index + 12);
            data.gyro_y = msg.get_value_float(index + 16);
            data.gyro_z = msg.get_value_float(index + 20);
            data.has_data = true;
        } return data;
    }

    DataAttitude decode_data_attitude(const size_t index, const Message &msg) {
        DataAttitude data;
        if (index + 12 <= msg.get_payload().size()) {
            data.yaw = msg.get_value_float(index);
            data.pitch = msg.get_value_float(index + 4);
            data.roll = msg.get_value_float(index + 8);
            data.has_data = true;
        } return data;
    }

    DataBattery decode_data_battery(const size_t index, const Message &msg) {
        DataBattery data;
        if (index + 10 <= msg.get_payload().size()) {
            data.adc_value = msg.get_value_uint16(index);;
            data.temperature = msg.get_value_uint16(index + 2);
            data.current = msg.get_value_int32(index + 4);
            data.percent = msg.get_value_uint8(index + 8);
            data.recv = msg.get_value_uint8(index + 9);
            data.has_data = true;
        } return data;
    }

    DataVelocity decode_data_velocity(const size_t index, const Message &msg) {
        DataVelocity data;
        if (index + 24 <= msg.get_payload().size()) {
            data.vgx = msg.get_value_float(index);
            data.vgy = msg.get_value_float(index + 4);
            data.vgz = msg.get_value_float(index + 8);
            data.vbx = msg.get_value_float(index + 12);
            data.vby = msg.get_value_float(index + 16);
            data.vbz = msg.get_value_float(index + 20);
            data.has_data = true;
        } return data;
    }

    std::ostream& operator<<(std::ostream& os, const DataEsc &data) {
        os << "{";
        if (data.has_data) {
            os << "\"speed\": [" << data.speed[0] << ", " << data.speed[1] << ", " << data.speed[2] << ", " << data.speed[3] << "]"
               << ", \"angle\": [" << data.angle[0] << ", " << data.angle[1] << ", " << data.angle[2] << ", " << data.angle[3] << "]"
               << ", \"time_stamp\": [" << data.time_stamp[0] << ", " << data.time_stamp[1] << ", " << data.time_stamp[2] << ", " << data.time_stamp[3] << "]"
               << ", \"state\": [" << static_cast<uint16_t>(data.state[0]) << ", " << static_cast<uint16_t>(data.state[1]) << ", "
               << static_cast<uint16_t>(data.state[2]) << ", " << static_cast<uint16_t>(data.state[3]) << "]";
        }
        os << "}"; return os;
    }

    std::ostream& operator<<(std::ostream& os, const DataImu &data) {
        os << "{";
        if (data.has_data) {
            os << "\"accel\": [" << data.acc_x << ", " << data.acc_y << ", " << data.acc_z << "]"
               << ", \"gyro\": [" << data.gyro_x << ", " << data.gyro_y << ", " << data.gyro_z << "]";
        }
        os << "}"; return os;
    }

    std::ostream& operator<<(std::ostream& os, const DataAttitude &data) {
        os << "{";
        if (data.has_data) {
            os << "\"roll\": " << data.roll << ", \"pitch\": " << data.pitch << ", \"yaw\": " << data.yaw;
        }
        os << "}"; return os;
    }

    std::ostream& operator<<(std::ostream& os, const DataBattery &data) {
        os << "{";
        if (data.has_data) {
            os << "\"adc\": " << data.adc_value << ", \"temp\": " << data.temperature
               << ", \"current\": " << data.current << ", \"percent\": " << static_cast<uint16_t>(data.percent);
        }
        os << "}"; return os;
    }

    std::ostream& operator<<(std::ostream& os, const DataVelocity &data) {
        os << "{";
        if (data.has_data) {
            os << "\"global\": [" << data.vgx << ", " << data.vgy << ", " << data.vgz << "]"
               << ", \"body\": [" << data.vbx << ", " << data.vby << ", " << data.vbz << "]";
        }
        os << "}"; return os;
    }

    std::ostream& operator<<(std::ostream& os, const DataPosition &data) {
        os << "{";
        if (data.has_data) {
            os << "\"x\": " << data.x << ", \"y\": " << data.y << ", \"z\": " << data.z;
        }
        os << "}"; return os;
    }

    std::ostream& operator<<(std::ostream& os, const DataRoboMasterState &data) {
        os << "{\"robomaster\":{"
           << "\"battery\":" << data.battery << ","
           << "\"esc\":" << data.esc << ","
           << "\"imu\":" << data.imu << ","
           << "\"velocity\":" << data.velocity << ","
           << "\"position\":" << data.position << ","
           << "\"attitude\":" << data.attitude
           << "}}"; return os;
    }
} // namespace robomaster