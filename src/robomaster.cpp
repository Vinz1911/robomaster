/*
 * Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
 *
 * This project contains contributions from multiple authors.
 * The original code is licensed under the MIT License by Fraunhofer IML.
 * All modifications and additional code are licensed under the MIT License by Vinzenz Weist.
 */

#include "robomaster/robomaster.h"
#include "robomaster/definitions.h"
#include "robomaster/utils.h"

namespace robomaster {
    static constexpr auto MEMORY_ORDER = std::memory_order::relaxed;

    RoboMaster::RoboMaster():message_counter_() { }

    void RoboMaster::boot_sequence() {
        this->handler_.push_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, 0x00, { 0x40, 0x48, 0x04, 0x00, 0x09, 0x00 }));
        this->handler_.push_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, 0x01, { 0x40, 0x48, 0x01, 0x09, 0x00, 0x00, 0x00, 0x03 }));
        this->handler_.push_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, 0x02, { 0x40, 0x48, 0x03, 0x09, 0x01, 0x03, 0x00, 0x07, 0xa7, 0x02, 0x29, 0x88, 0x03, 0x00, 0x02, 0x00, 0x66, 0x3e, 0x3e, 0x4c, 0x03, 0x00, 0x02, 0x00, 0xfb, 0xdc, 0xf5, 0xd7, 0x03, 0x00, 0x02, 0x00, 0x09, 0xa3, 0x26, 0xe2, 0x03, 0x00, 0x02, 0x00, 0xf4, 0x1d, 0x1c, 0xdc, 0x03, 0x00, 0x02, 0x00, 0x42, 0xee, 0x13, 0x1d, 0x03, 0x00, 0x02, 0x00, 0xb3, 0xf7, 0xe6, 0x47, 0x03, 0x00, 0x02, 0x00, 0x32, 0x00 }));
        this->handler_.push_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0x18c9, 0x03, { 0x00, 0x3f, 0x32, 0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }));
        this->handler_.push_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0x04c9, 0x04, { 0x40, 0x04, 0x1e, 0x05, 0xff }));
    }

    bool RoboMaster::init(const std::string& interface) {
        if (!this->handler_.init(interface)) { return false;}
        this->handler_.set_callback([this](const Message& msg) {
            if (msg.get_device_id() == DEVICE_ID_MOTION_CONTROLLER) { this->motion_state_.store(decode_motion_state(msg), MEMORY_ORDER); }
            if (msg.get_device_id() == DEVICE_ID_GIMBAL) { this->gimbal_state_.store(decode_gimbal_state(msg), MEMORY_ORDER); }
        });
        this->boot_sequence(); return true;
    }

    bool RoboMaster::is_running() const {
        return this->handler_.is_running();
    }

    RoboMasterMotionState RoboMaster::get_motion_state() const {
        return this->motion_state_.load(MEMORY_ORDER);
    }

    RoboMasterGimbalState RoboMaster::get_gimbal_state() const {
        return this->gimbal_state_.load(MEMORY_ORDER);
    }

    void RoboMaster::set_chassis_mode(const ChassisMode mode) {
        auto msg = Message(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, 0x00, { 0x40, 0x3f, 0x19, 0x00 });
        msg.set_value_uint8(3, mode);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_chassis_rpm(const int16_t front_right, const int16_t front_left, const int16_t rear_left, const int16_t rear_right) {
        const auto w1 = clip<int16_t>(front_right, -1000, 1000), w2 = clip<int16_t>(front_left, -1000, 1000), w3 = clip<int16_t>(rear_left, -1000, 1000), w4 = clip<int16_t>(rear_right, -1000, 1000);
        auto msg = Message(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->message_counter_++, { 0x40, 0x3f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_int16(3, w1);
        msg.set_value_int16(5, static_cast<int16_t>(-w2));
        msg.set_value_int16(7, static_cast<int16_t>(-w3));
        msg.set_value_int16(9, w4);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_chassis_velocity(const float linear_x, const float linear_y, const float angular_z) {
        const auto linear_x_ = clip<float>(linear_x, -3.5f, 3.5f), linear_y_ = clip<float>(linear_y, -3.5f, 3.5f), angular_z_ = clip<float>(angular_z, -600.0f, 600.0f);
        auto msg = Message(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->message_counter_++, { 0x00, 0x3f, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_float(3, linear_x_);
        msg.set_value_float(7, linear_y_);
        msg.set_value_float(11, angular_z_);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_chassis_position(const int16_t linear_x, const int16_t linear_y, const int16_t angular_z) {
        const auto linear_x_ = clip<int16_t>(linear_x, -500, 500), linear_y_ = clip<int16_t>(linear_y, -500, 500), angular_z_ = clip<int16_t>(angular_z, -18000, 18000);
        auto msg = Message(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->message_counter_++, { 0x00, 0x3f, 0x25, 0x02, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00 });
        msg.set_value_int16(7, linear_x_);
        msg.set_value_int16(9, linear_y_);
        msg.set_value_int16(11, angular_z_);
        msg.set_value_int16(14, 0x12c);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_gimbal_mode(const GimbalMode mode) {
        auto msg = Message(DEVICE_ID_INTELLI_CONTROLLER, 0x04c9, 0x00, { 0x40, 0x04, 0x4c, 0x00 });
        msg.set_value_uint8(3, mode);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_gimbal_state(const GimbalState state) {
        auto msg = Message(DEVICE_ID_INTELLI_CONTROLLER, 0x04c9, 0x00, { 0x20, 0x04, 0x0d, 0x00, 0x00 });
        msg.set_value_uint16(3, state);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_gimbal_degree(const int16_t pitch, const int16_t yaw) {
        const auto pitch_ = clip<int16_t>(pitch, -1000, 1000), yaw_ = clip<int16_t>(yaw, -1000, 1000);
        auto msg = Message(DEVICE_ID_INTELLI_CONTROLLER, 0x04c9, this->message_counter_++, { 0x00, 0x04, 0x69, 0x08, 0x05, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_int16(5, pitch_);
        msg.set_value_int16(7, yaw_);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_gimbal_velocity(const int16_t pitch, const int16_t yaw) {
        const auto pitch_ = clip<int16_t>(pitch, -1000, 1000), yaw_ = clip<int16_t>(yaw, -1000, 1000);
        auto msg = Message(DEVICE_ID_INTELLI_CONTROLLER, 0x04c9, this->message_counter_++, { 0x00, 0x04, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcd });
        msg.set_value_int16(3, yaw_);
        msg.set_value_int16(7, pitch_);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_gimbal_position(const int16_t pitch, const int16_t yaw, const uint16_t pitch_acceleration, const uint16_t yaw_acceleration) {
        const auto pitch_ = clip<int16_t>(pitch, -500, 500), yaw_ = clip<int16_t>(yaw, -2500, 2500);
        const auto pitch_acceleration_ = clip<uint16_t>(pitch_acceleration, 10, 500), yaw_acceleration_ = clip<uint16_t>(yaw_acceleration, 10, 500);
        auto msg = Message(DEVICE_ID_INTELLI_CONTROLLER, 0x04c9, this->message_counter_++, { 0x00, 0x3f, 0xb0, 0x03, 0x08, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_int16(6, yaw_);
        msg.set_value_int16(10, pitch_);
        msg.set_value_uint16(14, yaw_acceleration_);
        msg.set_value_uint16(18, pitch_acceleration_);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_gimbal_recenter(const int16_t pitch, const int16_t yaw) {
        const auto pitch_ = clip<int16_t>(pitch, 10, 500), yaw_ = clip<int16_t>(yaw, 10, 500);
        auto msg = Message(DEVICE_ID_INTELLI_CONTROLLER, 0x04c9, this->message_counter_++, { 0x00, 0x3f, 0xb2, 0x01, 0x08, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_int16(6, yaw_);
        msg.set_value_int16(10, pitch_);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_blaster(const BlasterMode mode, const uint8_t count) {
        const auto count_ = clip<uint8_t>(count, 1, 8); auto msg = std::vector<Message>();
        msg.push_back(Message(DEVICE_ID_INTELLI_CONTROLLER, 0x17c9, this->message_counter_++, { 0x00, 0x3f, 0x51, 0x00 }));
        msg.push_back(Message(DEVICE_ID_INTELLI_CONTROLLER, 0x17c9, this->message_counter_++, { 0x00, 0x3f, 0x55, 0x73, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00 }));
        msg[0].set_value_uint8(3, static_cast<uint8_t>((mode << 4 & 0xf0) + (count_ & 0x0f)));
        msg[1].set_value_uint16(8, static_cast<uint16_t>(count_ * 100));
        msg[1].set_value_uint16(10, static_cast<uint16_t>(count_ * 100));
        for (const auto& msg_ : msg) { this->handler_.push_message(msg_); }
    }

    void RoboMaster::set_led(const LEDMode mode, const LEDMask mask, const uint8_t red, const uint8_t green, const uint8_t blue, const uint16_t up_time, const uint16_t down_time) {
        const auto up_time_ = clip<uint16_t>(up_time, 0, 60000), down_time_ = clip<uint16_t>(down_time, 0, 60000);
        auto msg = Message(DEVICE_ID_INTELLI_CONTROLLER, 0x18c9, this->message_counter_++, { 0x00, 0x3f, 0x32, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_uint8(3, mode);
        msg.set_value_uint8(6, red);
        msg.set_value_uint8(7, green);
        msg.set_value_uint8(8, blue);
        msg.set_value_uint16(10, mode == LED_MODE_STATIC ? 0x00 : up_time_);
        msg.set_value_uint16(12, mode == LED_MODE_STATIC ? 0x00 : down_time_);
        msg.set_value_uint16(14, mask);
        this->handler_.push_message(msg);
    }

    RoboMasterMotionState RoboMaster::decode_motion_state(const Message& msg) {
        auto data = RoboMasterMotionState();
        data.velocity = decode_data_velocity(27, msg);
        data.battery = decode_data_battery(51, msg);
        data.esc = decode_data_esc(61, msg);
        data.imu = decode_data_imu(97, msg);
        data.attitude = decode_data_attitude(121, msg);
        data.position = decode_data_position(133, msg);
        return data;
    }

    RoboMasterGimbalState RoboMaster::decode_gimbal_state(const Message& msg) {
        auto data = RoboMasterGimbalState();
        data.gimbal = decode_data_gimbal(5, msg);
        return data;
    }
} // namespace robomaster