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

#include "robomaster/robomaster.h"
#include "robomaster/definitions.h"
#include "robomaster/payload.h"

namespace robomaster {
    static constexpr auto STD_MEMORY_ORDER = std::memory_order::relaxed;

    RoboMaster::RoboMaster(): sequence_{} { }

    void RoboMaster::boot_sequence() {
        this->handler_.push_message(Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, Payload::DEVICE_SEQUENCE_ZERO, Payload::BOOT_CHASSIS_PRIMARY));
        this->handler_.push_message(Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, Payload::DEVICE_SEQUENCE_ONE, Payload::BOOT_CHASSIS_SECONDARY));
        this->handler_.push_message(Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, Payload::DEVICE_SEQUENCE_TWO, Payload::BOOT_CHASSIS_SUB));
        this->handler_.push_message(Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, Payload::DEVICE_SEQUENCE_THREE, Payload::BOOT_GIMBAL_SUB));
        this->handler_.push_message(Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_LED, Payload::DEVICE_SEQUENCE_FOUR, Payload::BOOT_LED_RST));
    }

    bool RoboMaster::init(const std::string& interface) {
        if (!this->handler_.init(interface)) { return false;}
        this->handler_.set_callback([this](const Message& msg) { this->state_.store(decode_state(msg), STD_MEMORY_ORDER); });
        this->boot_sequence(); return true;
    }

    bool RoboMaster::is_running() const {
        return this->handler_.is_running();
    }

    RoboMasterState RoboMaster::get_state() const {
        return this->state_.load(STD_MEMORY_ORDER);
    }

    void RoboMaster::set_chassis_mode(const ChassisMode mode) {
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, Payload::DEVICE_SEQUENCE_ZERO, Payload::CHASSIS_MODE);
        message.set_uint8(3, mode);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_chassis_rpm(const int16_t front_right, const int16_t front_left, const int16_t rear_left, const int16_t rear_right) {
        const auto front_right_ = std::clamp(front_right, int16_t { -1000 }, int16_t { 1000 }), front_left_ = std::clamp(front_left, int16_t { -1000 }, int16_t { 1000 });
        const auto rear_left_ = std::clamp(rear_left, int16_t { -1000 }, int16_t { 1000 }), rear_right_ = std::clamp(rear_right, int16_t { -1000 }, int16_t { 1000 });
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER,Payload::DEVICE_TYPE_CHASSIS, this->sequence_++, Payload::CHASSIS_RPM);
        message.set_int16(3, front_right_);
        message.set_int16(5, static_cast<int16_t>(-front_left_));
        message.set_int16(7, static_cast<int16_t>(-rear_left_));
        message.set_int16(9, rear_right_);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_chassis_velocity(const float linear_x, const float linear_y, const float angular_z) {
        const auto linear_x_ = std::clamp(linear_x, -3.5f, 3.5f), linear_y_ = std::clamp(linear_y, -3.5f, 3.5f), angular_z_ = std::clamp(angular_z, -600.0f, 600.0f);
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, this->sequence_++, Payload::CHASSIS_VELOCITY);
        message.set_float(3, linear_x_);
        message.set_float(7, linear_y_);
        message.set_float(11, angular_z_);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_chassis_position(const int16_t linear_x, const int16_t linear_y, const int16_t angular_z) {
        const auto linear_x_ = std::clamp(linear_x, int16_t { -500 }, int16_t { 500 });
        const auto linear_y_ = std::clamp(linear_y, int16_t { -500 }, int16_t { 500 });
        const auto angular_z_ = std::clamp(angular_z, int16_t { -18000 }, int16_t { 18000 });
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, this->sequence_++, Payload::CHASSIS_POSITION);
        message.set_int16(7, linear_x_);
        message.set_int16(9, linear_y_);
        message.set_int16(11, angular_z_);
        message.set_int16(14, 0x12c);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_gimbal_mode(const GimbalMode mode) {
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, Payload::DEVICE_SEQUENCE_ZERO, Payload::GIMBAL_MODE);
        message.set_uint8(3, mode);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_gimbal_hibernate(const GimbalHibernate hibernate) {
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, Payload::DEVICE_SEQUENCE_ZERO, Payload::GIMBAL_HIBERNATE);
        message.set_uint16(3, hibernate);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_gimbal_degree(const int16_t pitch, const int16_t yaw) {
        const auto pitch_ = std::clamp(pitch, int16_t { -1000 }, int16_t { 1000 }), yaw_ = std::clamp(yaw, int16_t { -1000 }, int16_t { 1000 });
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, this->sequence_++, Payload::GIMBAL_DEGREE);
        message.set_int16(5, pitch_);
        message.set_int16(7, yaw_);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_gimbal_velocity(const int16_t pitch, const int16_t yaw) {
        const auto pitch_ = std::clamp(pitch, int16_t { -1000 }, int16_t { 1000 }), yaw_ = std::clamp(yaw, int16_t { -1000 }, int16_t { 1000 });
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, this->sequence_++, Payload::GIMBAL_VELOCITY);
        message.set_int16(3, yaw_);
        message.set_int16(7, pitch_);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_gimbal_position(const int16_t pitch, const int16_t yaw, const uint16_t pitch_acceleration, const uint16_t yaw_acceleration) {
        const auto pitch_ = std::clamp(pitch, int16_t { -500 }, int16_t { 500 }), yaw_ = std::clamp(yaw, int16_t { -2500 }, int16_t { 2500 });
        const auto pitch_acceleration_ = std::clamp(pitch_acceleration, uint16_t { 10 }, uint16_t { 500 }), yaw_acceleration_ = std::clamp(yaw_acceleration, uint16_t { 10 }, uint16_t { 500 });
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, this->sequence_++, Payload::GIMBAL_POSITION);
        message.set_int16(6, yaw_);
        message.set_int16(10, pitch_);
        message.set_uint16(14, yaw_acceleration_);
        message.set_uint16(18, pitch_acceleration_);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_gimbal_recenter(const int16_t pitch, const int16_t yaw) {
        const auto pitch_ = std::clamp(pitch, int16_t { 10 }, int16_t { 500 }), yaw_ = std::clamp(yaw, int16_t { 10 }, int16_t { 500 });
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, this->sequence_++, Payload::GIMBAL_RECENTER);
        message.set_int16(6, yaw_);
        message.set_int16(10, pitch_);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_blaster_mode(const BlasterMode mode, const uint8_t count) {
        const auto count_ = std::clamp(count, uint8_t { 1 }, uint8_t { 8 }); auto message = std::vector<Message>();
        message.emplace_back(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_BLASTER, this->sequence_++, Payload::BLASTER_MODE_GEL);
        message.emplace_back(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_BLASTER, this->sequence_++, Payload::BLASTER_MODE_LED);
        message[0].set_uint8(3, static_cast<uint8_t>((mode << 4 & 0xf0) + (count_ & 0x0f)));
        message[1].set_uint16(8, static_cast<uint16_t>(count_ * 100));
        message[1].set_uint16(10, static_cast<uint16_t>(count_ * 100));
        for (const auto& msg_ : message) { this->handler_.push_message(msg_); }
    }

    void RoboMaster::set_led_mode(const LEDMode mode, const LEDMask mask, const uint8_t red, const uint8_t green, const uint8_t blue, const uint16_t up_time, const uint16_t down_time) {
        const auto up_time_ = std::clamp(up_time, uint16_t { 0 }, uint16_t { 60000 }), down_time_ = std::clamp(down_time, uint16_t { 0 }, uint16_t { 60000 });
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_LED, this->sequence_++, Payload::LED_MODE);
        message.set_uint8(3, mode);
        message.set_uint8(6, red);
        message.set_uint8(7, green);
        message.set_uint8(8, blue);
        message.set_uint16(10, mode == LED_MODE_STATIC ? 0x00 : up_time_);
        message.set_uint16(12, mode == LED_MODE_STATIC ? 0x00 : down_time_);
        message.set_uint16(14, mask);
        this->handler_.push_message(message);
    }

    RoboMasterState RoboMaster::decode_state(const Message& message) {
        static auto data = RoboMasterState{};
        if (message.get_device_id() == Payload::DEVICE_ID_MOTION_CONTROLLER) {
            data.velocity = decode_data_velocity(27, message); data.battery = decode_data_battery(51, message); data.esc = decode_data_esc(61, message);
            data.imu = decode_data_imu(97, message); data.attitude = decode_data_attitude(121, message); data.position = decode_data_position(133, message);
        }
        if (message.get_device_id() == Payload::DEVICE_ID_GIMBAL) { data.gimbal = decode_data_gimbal(5, message); }
        if (message.get_device_id() == Payload::DEVICE_ID_HIT_DETECTOR_1) { data.detector[0] = decode_data_detector(4, message); }
        if (message.get_device_id() == Payload::DEVICE_ID_HIT_DETECTOR_2) { data.detector[1] = decode_data_detector(4, message); }
        if (message.get_device_id() == Payload::DEVICE_ID_HIT_DETECTOR_3) { data.detector[2] = decode_data_detector(4, message); }
        if (message.get_device_id() == Payload::DEVICE_ID_HIT_DETECTOR_4) { data.detector[3] = decode_data_detector(4, message); }

        data.is_active = true; return data;
    }
} // namespace robomaster