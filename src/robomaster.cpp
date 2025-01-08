// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#include <utility>

#include "robomaster/robomaster.h"
#include "robomaster/definitions.h"
#include "robomaster/utils.h"

namespace robomaster {
    RoboMaster::RoboMaster():counter_drive_(0), counter_led_(0), counter_gimbal_(0), counter_blaster_(0) { this->handler_.bind_callback([this]<typename T0>(T0 && PH1) { decode_state(std::forward<T0>(PH1)); }); }

    RoboMaster::~RoboMaster() = default;

    void RoboMaster::set_callback(std::function<void(const DataRoboMasterState&)> func) {
        this->callback_data_robomaster_state_ = std::move(func);
    }

    void RoboMaster::boot_sequence() {
        this->handler_.push_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0x0309, 0, { 0x40, 0x48, 0x04, 0x00, 0x09, 0x00 }));
        this->handler_.push_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0x0309, 1, { 0x40, 0x48, 0x01, 0x09, 0x00, 0x00, 0x00, 0x03 }));
        this->handler_.push_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0x0309, 2, { 0x40, 0x48, 0x03, 0x09, 0x01, 0x03, 0x00, 0x07, 0xa7, 0x02, 0x29, 0x88, 0x03, 0x00, 0x02, 0x00, 0x66, 0x3e, 0x3e, 0x4c, 0x03, 0x00, 0x02, 0x00, 0xfb, 0xdc, 0xf5, 0xd7, 0x03, 0x00, 0x02, 0x00, 0x09, 0xa3, 0x26, 0xe2, 0x03, 0x00, 0x02, 0x00, 0xf4, 0x1d, 0x1c, 0xdc, 0x03, 0x00, 0x02, 0x00, 0x42, 0xee, 0x13, 0x1d, 0x03, 0x00, 0x02, 0x00, 0xb3, 0xf7, 0xe6, 0x47, 0x03, 0x00, 0x02, 0x00, 0x32, 0x00 }));
    }

    void RoboMaster::set_work_mode(const bool mode) {
        Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0xc309, 0, { 0x40, 0x3f, 0x19, 0x00 });
        msg.set_value_uint8(3, mode);
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
        msg.set_value_int16(5, w2);
        msg.set_value_int16(7, w3);
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

    void RoboMaster::set_blaster(const BlasterType blaster) {
        Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0x1709, this->counter_blaster_++);
        switch (blaster) {
            case INFRARED: msg.set_payload({ 0x00, 0x3f, 0x55, 0x73, 0x00, 0xff, 0x00, 0x01, 0x28, 0x00, 0x00 }); break;
            case GELBEADS: msg.set_payload({ 0x00, 0x3f, 0x51, 0x01 }); break;
        }
        this->handler_.push_message(msg);
    }

    bool RoboMaster::init(const std::string &can_interface) {
        if (this->handler_.init(can_interface)) {
            this->boot_sequence(); return true;
        } return false;
    }

    void RoboMaster::set_led_off(const uint16_t mask) {
        Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0x1809, this->counter_led_++, { 0x00, 0x3f, 0x32, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_uint16(3, 0x70);
        msg.set_value_uint16(14, mask);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_led_on(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b) {
        Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0x1809, this->counter_led_++, { 0x00, 0x3f, 0x32, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_uint16(3, 0x71);
        msg.set_value_uint8(6, r);
        msg.set_value_uint8(7, g);
        msg.set_value_uint8(8, b);
        msg.set_value_uint16(14, mask);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_led_breath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const uint16_t t_rise, const uint16_t t_down) {
        Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0x1809, this->counter_led_++, { 0x00, 0x3f, 0x32, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_uint16(3, 0x72);
        msg.set_value_uint8(6, r);
        msg.set_value_uint8(7, g);
        msg.set_value_uint8(8, b);
        msg.set_value_uint16(10, t_rise);
        msg.set_value_uint16(12, t_down);
        msg.set_value_uint16(14, mask);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_led_breath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float t_rise, const float t_down) {
        const auto rise = static_cast<uint16_t>(clip<float>(t_rise * 1000.0f, 0.0f, 60000.0f));
        const auto down = static_cast<uint16_t>(clip<float>(t_down * 1000.0f, 0.0f, 60000.0f));
        this->set_led_breath(mask, r, g, b, rise, down);
    }

    void RoboMaster::set_led_breath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float rate) {
        const auto freq = static_cast<uint16_t>(clip<float>(rate * 1000.0f, 0.0f, 60000.0f));
        this->set_led_breath(mask, r, g, b, freq, freq);
    }

    void RoboMaster::set_led_flash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const uint16_t t_on, const uint16_t t_off) {
        Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0x1809, this->counter_led_++, { 0x00, 0x3f, 0x32, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
        msg.set_value_uint16(3, 0x73);
        msg.set_value_uint8(6, r);
        msg.set_value_uint8(7, g);
        msg.set_value_uint8(8, b);
        msg.set_value_uint16(10, t_on);
        msg.set_value_uint16(12, t_off);
        msg.set_value_uint16(14, mask);
        this->handler_.push_message(msg);
    }

    void RoboMaster::set_led_flash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float t_on, const float t_off) {
        const auto on  = static_cast<uint16_t>(clip<float>(t_on * 1000.0f, 0.0f, 60000.0f));
        const auto off = static_cast<uint16_t>(clip<float>(t_off * 1000.0f, 0.0f, 60000.0f));
        this->set_led_flash(mask, r, g, b, on, off);
    }

    void RoboMaster::set_led_flash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float rate) {
        const auto freq = static_cast<uint16_t>(clip<float>(rate * 1000.0f, 0.0f, 60000.0f));
        this->set_led_flash(mask, r, g, b, freq, freq);
    }

    void RoboMaster::decode_state(const Message &msg) const {
        if (this->callback_data_robomaster_state_) {
            DataRoboMasterState data;
            data.velocity   = decode_data_velocity(27, msg);
            data.battery    = decode_data_battery(51, msg);
            data.esc        = decode_data_esc(61, msg);
            data.imu        = decode_data_imu(97, msg);
            data.attitude   = decode_data_attitude(121, msg);
            data.position   = decode_data_position(133, msg);
            this->callback_data_robomaster_state_(data);
        }
    }

    bool RoboMaster::is_running() const {
        return this->handler_.is_running();
    }
} // namespace robomaster