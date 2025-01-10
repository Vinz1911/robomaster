// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#include <utility>
#include <optional>

#include "robomaster/robomaster.h"
#include "robomaster/definitions.h"
#include "robomaster/utils.h"

namespace robomaster {
    RoboMaster::RoboMaster():counter_drive_(), counter_led_(), counter_gimbal_(), counter_blaster_() {
        this->handler_.bind_callback([this]<typename T0>(T0 && PH1) { this->state_.store(decode_state(std::forward<T0>(PH1)), std::memory_order::relaxed); });
    }

    RoboMaster::~RoboMaster() = default;

    void RoboMaster::boot_sequence() {
        this->handler_.push_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0x0309, 0, { 0x40, 0x48, 0x04, 0x00, 0x09, 0x00 }));
        this->handler_.push_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0x0309, 1, { 0x40, 0x48, 0x01, 0x09, 0x00, 0x00, 0x00, 0x03 }));
        this->handler_.push_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0x0309, 2, { 0x40, 0x48, 0x03, 0x09, 0x01, 0x03, 0x00, 0x07, 0xa7, 0x02, 0x29, 0x88, 0x03, 0x00, 0x02, 0x00, 0x66, 0x3e, 0x3e, 0x4c, 0x03, 0x00, 0x02, 0x00, 0xfb, 0xdc, 0xf5, 0xd7, 0x03, 0x00, 0x02, 0x00, 0x09, 0xa3, 0x26, 0xe2, 0x03, 0x00, 0x02, 0x00, 0xf4, 0x1d, 0x1c, 0xdc, 0x03, 0x00, 0x02, 0x00, 0x42, 0xee, 0x13, 0x1d, 0x03, 0x00, 0x02, 0x00, 0xb3, 0xf7, 0xe6, 0x47, 0x03, 0x00, 0x02, 0x00, 0x32, 0x00 }));
        this->handler_.push_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0x1809, 3, { 0x00, 0x3f, 0x32, 0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }));
    }

    void RoboMaster::set_torque(const bool enable) {
        Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, 0, { 0x40, 0x3f, 0x19, 0x00 });
        msg.set_value_uint8(3, enable);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_brake() {
        const Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->counter_drive_++, { 0x40, 0x3F, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_wheel_rpm(const int16_t fr, const int16_t fl, const int16_t rl, const int16_t rr) {
        const auto w1 = clip<int16_t>(fr, -1000, 1000);
        const auto w2 = clip<int16_t>(fl, -1000, 1000);
        const auto w3 = clip<int16_t>(rl, -1000, 1000);
        const auto w4 = clip<int16_t>(rr, -1000, 1000);

        Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->counter_drive_++, { 0x40, 0x3F, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_int16(3, w1);
        msg.set_value_int16(5, static_cast<int16_t>(-w2));
        msg.set_value_int16(7, static_cast<int16_t>(-w3));
        msg.set_value_int16(9, w4);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_velocity(const float x, const float y, const float z) {
        const auto cx = clip<float>(x,   -3.5f,   3.5f);
        const auto cy = clip<float>(y,   -3.5f,   3.5f);
        const auto cz = clip<float>(z, -600.0f, 600.0f);

        Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->counter_drive_++, { 0x00, 0x3f, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_float(3, cx);
        msg.set_value_float(7, cy);
        msg.set_value_float(11, cz);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_gimbal(const int16_t y, const int16_t z) {
        const auto cy = clip<int16_t>(y, -1024, 1024);
        const auto cz = clip<int16_t>(z, -1024, 1024);

        Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0x0409, this->counter_gimbal_++, { 0x00, 0x04, 0x69, 0x08, 0x05, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_int16(5, cy);
        msg.set_value_int16(7, cz);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_blaster(const BlasterMode mode) {
        const std::vector<uint8_t> ifr_bytes = { 0x00, 0x3f, 0x55, 0x73, 0x00, 0xff, 0x00, 0x01, 0x28, 0x00, 0x00 };
        const std::vector<uint8_t> gel_bytes = { 0x00, 0x3f, 0x51, 0x01 };

        Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0x1709, this->counter_blaster_++);
        msg.set_payload(mode == INFRARED ? ifr_bytes : gel_bytes);
        this->handler_.push_message(msg);
    }

    bool RoboMaster::init(const std::string &can_interface) {
        if (!this->handler_.init(can_interface)) { return false;}
        this->boot_sequence(); return true;
    }

    void RoboMaster::set_led(const LEDMode mode, const uint16_t mask, const uint8_t red, const uint8_t green, const uint8_t blue, const std::optional<uint16_t> up_time, const std::optional<uint16_t> down_time) {
        Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0x1809, this->counter_led_++, { 0x00, 0x3f, 0x32, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_uint8(3, !up_time.has_value() || !down_time.has_value() ? STATIC : mode);
        msg.set_value_uint8(6, red);
        msg.set_value_uint8(7, green);
        msg.set_value_uint8(8, blue);
        if (up_time.has_value()) { msg.set_value_uint16(10, up_time.value()); }
        if (down_time.has_value()) { msg.set_value_uint16(12, down_time.value()); }
        msg.set_value_uint16(14, mask);
        this->handler_.push_message(msg);
    }

    RoboMasterState RoboMaster::get_state() const {
        return this->state_.load(std::memory_order::relaxed);
    }

    RoboMasterState RoboMaster::decode_state(const Message &msg) {
        auto data = RoboMasterState();
        data.velocity = decode_data_velocity(27, msg);
        data.battery = decode_data_battery(51, msg);
        data.esc = decode_data_esc(61, msg);
        data.imu = decode_data_imu(97, msg);
        data.attitude = decode_data_attitude(121, msg);
        data.position = decode_data_position(133, msg);
        return data;
    }

    bool RoboMaster::is_running() const {
        return this->handler_.is_running();
    }
} // namespace robomaster