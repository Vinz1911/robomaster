/*
 * Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
 *
 * This project contains contributions from multiple authors.
 * The original code is licensed under the MIT License by Fraunhofer IML.
 * All modifications and additional code are licensed under the MIT License by Vinzenz Weist.
 */

#include <map>

#include "robomaster/handler.h"
#include "robomaster/utils.h"
#include "robomaster/definitions.h"

namespace robomaster {
    static constexpr size_t STD_MAX_ERROR_COUNT = 5;
    static constexpr auto STD_HEARTBEAT_TIME = std::chrono::milliseconds(10);
    static constexpr auto MEMORY_ORDER = std::memory_order::relaxed;

    Handler::Handler(): flag_initialised_(false), flag_stop_(false) { }

    void Handler::join_all() {
        if (this->thread_receiver_.joinable()) { this->thread_receiver_.join(); }
        if (this->thread_sender_.joinable()) { this->thread_sender_.join(); }
    }

    Handler::~Handler() {
        if (!this->flag_initialised_) { return; }
        this->flag_stop_.store(true, MEMORY_ORDER);
        this->cv_sender_.notify_all();
        this->join_all();
    }

    bool Handler::init(const std::string& can_interface) {
        if (this->flag_initialised_) { std::printf("[Robomaster]: already running\n"); return false; }
        if (!this->can_socket_.init(can_interface)) { std::printf("[Robomaster]: initialization failure\n"); return false; }

        this->can_socket_.set_timeout(0.1);
        this->flag_initialised_ = true;
        this->thread_receiver_ = std::thread(&Handler::receiver_thread, this);
        this->thread_sender_ = std::thread(&Handler::sender_thread, this);
        return true;
    }

    bool Handler::is_running() const {
        return this->flag_initialised_ && !this->flag_stop_.load(MEMORY_ORDER);
    }

    void Handler::push_message(const Message& msg) {
        this->queue_sender_.push(msg);
        this->cv_sender_.notify_one();
    }

    void Handler::set_callback(std::function<void(const Message&)> func) {
        this->callback_data_robomaster_state_ = std::move(func);
    }

    bool Handler::send_message(const Message& msg) const {
        const auto id = msg.get_device_id(); const auto data = msg.to_vector(); uint8_t frame_data[8] = {};
        for (size_t i = 0; i < data.size(); i += 8) {
            const size_t frame_length = std::min(static_cast<size_t>(8), data.size() - i);
            std::copy_n(data.begin() + static_cast<long>(i), frame_length, frame_data);
            if (!this->can_socket_.send_frame(id, frame_data, frame_length)) { return false; }
        } return true;
    }

    void Handler::receive_message(const Message& msg) const {
        const auto& payload = msg.get_payload();
        const uint16_t msg_type = msg.get_type();

        if ((msg.get_device_id() != DEVICE_ID_MOTION_CONTROLLER && msg.get_device_id() != DEVICE_ID_GIMBAL) || (msg_type != 0x0903 && msg_type != 0x0904)) { return; }
        if (msg.get_device_id() == DEVICE_ID_MOTION_CONTROLLER && (payload.size() < 4 || payload[0] != 0x20 || payload[1] != 0x48 || payload[2] != 0x08 || payload[3] != 0x00)) { return; }
        if (msg.get_device_id() == DEVICE_ID_GIMBAL && (payload.size() < 3 || payload[0] != 0x00 || payload[1] != 0x3f || payload[2] != 0x76)) { return; }
        if (this->callback_data_robomaster_state_) { this->callback_data_robomaster_state_(msg); }
    }

    void Handler::sender_thread() {
        uint16_t heartbeat_counter = 0; size_t error_counter = 0; auto heartbeat_time_point = std::chrono::high_resolution_clock::now();
        while (error_counter <= STD_MAX_ERROR_COUNT && !this->flag_stop_.load(MEMORY_ORDER)) {
            if (heartbeat_time_point < std::chrono::high_resolution_clock::now()) {
                const auto is_send = this->send_message(Message(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, heartbeat_counter++, { 0x00, 0x3f, 0x60, 0x00, 0x04, 0x20, 0x00, 0x01, 0x00, 0x40, 0x00, 0x02, 0x10, 0x00, 0x03, 0x00, 0x00 }));
                if (is_send) { heartbeat_time_point += STD_HEARTBEAT_TIME; error_counter = 0; } else { error_counter++; }
            } else if (!this->queue_sender_.empty()) {
                if (Message msg = queue_sender_.pop(); msg.is_valid()) { if (this->send_message(msg)) { error_counter = 0; } else { error_counter++; } }
            } else { std::unique_lock lock(this->cv_sender_mutex_); this->cv_sender_.wait_until(lock, heartbeat_time_point); }
        } if (error_counter != 0) { this->flag_stop_.store(true, MEMORY_ORDER); std::printf("[Robomaster]: sender frame failure\n"); }
    }

    void Handler::receiver_thread() {
        struct CANMessage { std::vector<uint8_t> buffer; size_t length = 0; };
        std::map<uint32_t, CANMessage> can_message { { DEVICE_ID_MOTION_CONTROLLER, CANMessage() }, { DEVICE_ID_GIMBAL, CANMessage() } };
        uint32_t frame_id; uint8_t frame_buffer[8] = {}; size_t frame_length; size_t error_counter = 0;

        while (error_counter <= STD_MAX_ERROR_COUNT && !this->flag_stop_.load(MEMORY_ORDER)) {
            if (!can_socket_.read_frame(frame_id, frame_buffer, frame_length)) { error_counter++; continue; }
            auto slice = can_message.find(frame_id); if (slice == can_message.end()) { continue; }
            auto&[buffer, length] = slice->second; buffer.insert(std::end(buffer), frame_buffer, frame_buffer + frame_length);

            if (length == 0) {
                auto iterator = buffer.cbegin();
                while (iterator != buffer.cend()) {
                    iterator = std::find(iterator, std::cend(buffer), 0x55); buffer.erase(std::cbegin(buffer), iterator);
                    if(buffer.size() < 4) { break; } if(buffer[3] == calculate_crc8(buffer.data(), 3)) { length = buffer[1]; break; } ++iterator;
                }
            } else if (length <= buffer.size()) {
                if (const uint16_t crc16 = little_endian_to_uint16(buffer[length - 2], buffer[length - 1]); crc16 == calculate_crc16(buffer.data(), length - 2)) {
                    auto const msg = Message(frame_id, std::vector(std::cbegin(buffer), std::cbegin(buffer) + static_cast<long>(length))); if (msg.is_valid()) { this->receive_message(msg); }
                } buffer.erase(std::cbegin(buffer), std::cbegin(buffer) + static_cast<long>(length)); length = 0;
            }
        } if (error_counter != 0) { this->flag_stop_.store(true, MEMORY_ORDER); std::printf("[Robomaster]: receiver frame failure\n"); }
    }
} // namespace robomaster