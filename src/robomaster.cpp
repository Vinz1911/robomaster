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
        this->handler_.push_message(Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, Payload::DEVICE_SEQ_ZERO, Payload::BOOT_CHASSIS_SPECIAL));
        this->handler_.push_message(Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, Payload::DEVICE_SEQ_ONE, Payload::BOOT_CHASSIS_CONFIRM));
        this->handler_.push_message(Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, Payload::DEVICE_SEQ_TWO, Payload::BOOT_CHASSIS_INFO));
        this->handler_.push_message(Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, Payload::DEVICE_SEQ_THREE, Payload::BOOT_GIMBAL_INFO));
        this->handler_.push_message(Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_LED, Payload::DEVICE_SEQ_FOUR, Payload::BOOT_LED_RESET));
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
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, Payload::DEVICE_SEQ_ZERO, Payload::CHASSIS_MODE);
        message.set_uint8(3, mode);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_chassis_rpm(const int16_t front_right, const int16_t front_left, const int16_t rear_left, const int16_t rear_right) {
        constexpr int16_t rpm_min = -1000, rpm_max = 1000;
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER,Payload::DEVICE_TYPE_CHASSIS, this->sequence_++, Payload::CHASSIS_RPM);
        message.set_int16(3, std::clamp(front_right, rpm_min, rpm_max));
        message.set_int16(5, std::clamp(static_cast<int16_t>(-front_left), rpm_min, rpm_max));
        message.set_int16(7, std::clamp(static_cast<int16_t>(-rear_left), rpm_min, rpm_max));
        message.set_int16(9, std::clamp(rear_right, rpm_min, rpm_max));
        this->handler_.push_message(message);
    }

    void RoboMaster::set_chassis_velocity(const float linear_x, const float linear_y, const float angular_z) {
        constexpr float linear_min = -3.5f, linear_max = 3.5f, angular_min = -600.0f, angular_max = 600.0f;
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, this->sequence_++, Payload::CHASSIS_VELOCITY);
        message.set_float(3, std::clamp(linear_x, linear_min, linear_max));
        message.set_float(7, std::clamp(linear_y, linear_min, linear_max));
        message.set_float(11, std::clamp(angular_z, angular_min, angular_max));
        this->handler_.push_message(message);
    }

    void RoboMaster::set_chassis_position(const int16_t linear_x, const int16_t linear_y, const int16_t angular_z) {
        constexpr int16_t linear_min = -500, linear_max = 500, angular_min = -18000, angular_max = 18000;
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, this->sequence_++, Payload::CHASSIS_POSITION);
        message.set_int16(7, std::clamp(linear_x, linear_min, linear_max));
        message.set_int16(9, std::clamp(linear_y, linear_min, linear_max));
        message.set_int16(11, std::clamp(angular_z, angular_min, angular_max));
        message.set_int16(14, 0x12c);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_gimbal_mode(const GimbalMode mode) {
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, Payload::DEVICE_SEQ_ZERO, Payload::GIMBAL_MODE);
        message.set_uint8(3, mode);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_gimbal_hibernate(const GimbalHibernate hibernate) {
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, Payload::DEVICE_SEQ_ZERO, Payload::GIMBAL_HIBERNATE);
        message.set_uint16(3, hibernate);
        this->handler_.push_message(message);
    }

    void RoboMaster::set_gimbal_motion(const int16_t pitch, const int16_t yaw) {
        constexpr int16_t pitch_yaw_min = -1000, pitch_yaw_max = 1000;
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, this->sequence_++, Payload::GIMBAL_DEGREE);
        message.set_int16(5, std::clamp(pitch, pitch_yaw_min, pitch_yaw_max));
        message.set_int16(7, std::clamp(yaw, pitch_yaw_min, pitch_yaw_max));
        this->handler_.push_message(message);
    }

    void RoboMaster::set_gimbal_velocity(const int16_t pitch, const int16_t yaw) {
        constexpr int16_t pitch_yaw_min = -1000, pitch_yaw_max = 1000;
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, this->sequence_++, Payload::GIMBAL_VELOCITY);
        message.set_int16(3, std::clamp(yaw, pitch_yaw_min, pitch_yaw_max));
        message.set_int16(7, std::clamp(pitch, pitch_yaw_min, pitch_yaw_max));
        this->handler_.push_message(message);
    }

    void RoboMaster::set_gimbal_position(const int16_t pitch, const int16_t yaw, const uint16_t pitch_acceleration, const uint16_t yaw_acceleration) {
        constexpr int16_t yaw_min = -2500, yaw_max = 2500, pitch_min = -500, pitch_max = 500; constexpr uint16_t acceleration_min = 10, acceleration_max = 500;
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, this->sequence_++, Payload::GIMBAL_POSITION);
        message.set_int16(6, std::clamp(yaw, yaw_min, yaw_max));
        message.set_int16(10, std::clamp(pitch, pitch_min, pitch_max));
        message.set_uint16(14, std::clamp(yaw_acceleration, acceleration_min, acceleration_max));
        message.set_uint16(18, std::clamp(pitch_acceleration, acceleration_min, acceleration_max));
        this->handler_.push_message(message);
    }

    void RoboMaster::set_gimbal_recenter(const int16_t pitch, const int16_t yaw) {
        constexpr int16_t pitch_yaw_min = 10, pitch_yaw_max = 500;
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_GIMBAL, this->sequence_++, Payload::GIMBAL_RECENTER);
        message.set_int16(6, std::clamp(yaw, pitch_yaw_min, pitch_yaw_max));
        message.set_int16(10, std::clamp(pitch, pitch_yaw_min, pitch_yaw_max));
        this->handler_.push_message(message);
    }

    void RoboMaster::set_blaster_mode(const BlasterMode mode, const uint8_t count) {
        constexpr uint8_t count_min = 1, count_max = 8; auto message = std::vector<Message>();
        message.emplace_back(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_BLASTER, this->sequence_++, Payload::BLASTER_MODE_GEL);
        message.emplace_back(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_BLASTER, this->sequence_++, Payload::BLASTER_MODE_LED);
        message[0].set_uint8(3, static_cast<uint8_t>((mode << 4 & 0xf0) + (std::clamp(count, count_min, count_max) & 0x0f)));
        message[1].set_uint16(8, static_cast<uint16_t>(std::clamp(count, count_min, count_max) * 100));
        message[1].set_uint16(10, static_cast<uint16_t>(std::clamp(count, count_min, count_max) * 100));
        for (const auto& msg_ : message) { this->handler_.push_message(msg_); }
    }

    void RoboMaster::set_led_mode(const LEDMode mode, const LEDMask mask, const uint8_t red, const uint8_t green, const uint8_t blue, const uint16_t up_time, const uint16_t down_time) {
        constexpr uint16_t time_min = 0, time_max = 60000;
        auto message = Message(Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_LED, this->sequence_++, Payload::LED_MODE);
        message.set_uint8(3, mode);
        message.set_uint8(6, red);
        message.set_uint8(7, green);
        message.set_uint8(8, blue);
        message.set_uint16(10, mode == LED_MODE_STATIC ? 0x0 : std::clamp(up_time, time_min, time_max));
        message.set_uint16(12, mode == LED_MODE_STATIC ? 0x0 : std::clamp(down_time, time_min, time_max));
        message.set_uint16(14, mask);
        this->handler_.push_message(message);
    }

    RoboMasterState RoboMaster::decode_state(const Message& message) {
        static auto data = RoboMasterState{};
        if (message.get_device_id() == Payload::DEVICE_ID_GIMBAL) { data.gimbal = decode_gimbal(5, message); }
        if (message.get_device_id() == Payload::DEVICE_ID_HIT_DETECTOR_1) { data.detector[0] = decode_detector(4, message); }
        if (message.get_device_id() == Payload::DEVICE_ID_HIT_DETECTOR_2) { data.detector[1] = decode_detector(4, message); }
        if (message.get_device_id() == Payload::DEVICE_ID_HIT_DETECTOR_3) { data.detector[2] = decode_detector(4, message); }
        if (message.get_device_id() == Payload::DEVICE_ID_HIT_DETECTOR_4) { data.detector[3] = decode_detector(4, message); }
        if (message.get_device_id() == Payload::DEVICE_ID_MOTION_CONTROLLER) {
            data.velocity = decode_velocity(27, message); data.battery = decode_battery(51, message); data.esc = decode_esc(61, message);
            data.imu = decode_imu(97, message); data.attitude = decode_attitude(121, message); data.position = decode_position(133, message);
        }
        data.is_active = true; return data;
    }
} // namespace robomaster