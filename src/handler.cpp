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

#include <map>

#include "robomaster/handler.h"
#include "robomaster/utils.h"
#include "robomaster/payload.h"

namespace robomaster {
    static constexpr size_t STD_MAX_ERROR_COUNT = 5;
    static constexpr auto STD_HEARTBEAT_TIME = std::chrono::milliseconds(10);
    static constexpr auto STD_MEMORY_ORDER = std::memory_order::relaxed;

    Handler::Handler(): is_initialised_{false}, is_stopped_{false} { }

    Handler::~Handler() {
        if (!this->is_initialised_) { return; }
        this->is_stopped_.store(true, STD_MEMORY_ORDER);
        this->condition_sender_.notify_all();
        this->join_all();
    }

    bool Handler::init(const std::string& interface) {
        if (this->is_initialised_) { std::printf("[Robomaster]: already running\n"); return false; }
        if (!this->can_bus_.init(interface)) { std::printf("[Robomaster]: initialization failure\n"); return false; }

        this->can_bus_.set_timeout(0.1);
        this->is_initialised_ = true;
        this->thread_receiver_ = std::thread{&Handler::receiver_thread, this};
        this->thread_sender_ = std::thread{&Handler::sender_thread, this};
        return true;
    }

    void Handler::join_all() {
        if (this->thread_receiver_.joinable()) { this->thread_receiver_.join(); }
        if (this->thread_sender_.joinable()) { this->thread_sender_.join(); }
    }

    bool Handler::is_running() const {
        return this->is_initialised_ && !this->is_stopped_.load(STD_MEMORY_ORDER);
    }

    void Handler::push_message(const Message& message) {
        this->queue_sender_.push(message);
        this->condition_sender_.notify_one();
    }

    void Handler::set_callback(std::function<void(const Message&)> completion) {
        this->state_callback_ = std::move(completion);
    }

    bool Handler::send_message(const Message& message) const {
        const auto id = message.get_device_id(); const auto data = message.vector(); uint8_t frame_data[8] = {};

        for (size_t i = 0; i < data.size(); i += 8) {
            const size_t frame_length = std::min(static_cast<size_t>(8), data.size() - i);
            std::copy_n(data.begin() + static_cast<long>(i), frame_length, frame_data);
            if (!this->can_bus_.send_frame(id, frame_data, frame_length)) { return false; }
        } return true;
    }

    void Handler::receive_message(const Message& message) const {
        const auto& payload = message.get_payload(); const auto msg_type = message.get_type(); const auto device_id = message.get_device_id();
        static const std::unordered_map<uint16_t, std::pair<uint16_t, std::vector<uint8_t>>> device_ids = {
            { Payload::DEVICE_ID_MOTION_CONTROLLER, { 0x0903, { 0x20, 0x48, 0x08, 0x00 } } }, { Payload::DEVICE_ID_GIMBAL, { 0x0904, { 0x00, 0x3f, 0x76 } } },
            { Payload::DEVICE_ID_HIT_DETECTOR_1, { 0x0938, { 0x00, 0x3f, 0x02, 0x10 } } }, { Payload::DEVICE_ID_HIT_DETECTOR_2, { 0x0958, { 0x00, 0x3f, 0x02, 0x20 } } },
            { Payload::DEVICE_ID_HIT_DETECTOR_3, { 0x0978, { 0x00, 0x3f, 0x02, 0x30 } } }, { Payload::DEVICE_ID_HIT_DETECTOR_4, { 0x0998, { 0x00, 0x3f, 0x02, 0x40 } } }
        };

        const auto ids = device_ids.find(device_id); if (ids == device_ids.end() || msg_type != ids->second.first) { return; }
        const auto& sequence = ids->second.second; if (payload.size() < sequence.size() || !std::equal(sequence.begin(), sequence.end(), payload.begin())) { return; }
        if (this->state_callback_) { this->state_callback_(message); }
    }

    void Handler::sender_thread() {
        uint16_t heartbeat_counter = 0x0; size_t error_counter = 0x0; auto heartbeat_time_point = std::chrono::high_resolution_clock::now();

        while (error_counter <= STD_MAX_ERROR_COUNT && !this->is_stopped_.load(STD_MEMORY_ORDER)) {
            if (heartbeat_time_point < std::chrono::high_resolution_clock::now()) {
                const auto msg = Message{Payload::DEVICE_ID_INTELLI_CONTROLLER, Payload::DEVICE_TYPE_CHASSIS, heartbeat_counter++, Payload::HEART_BEAT};
                if (this->send_message(msg)) { heartbeat_time_point += STD_HEARTBEAT_TIME; error_counter = 0x0; } else { error_counter++; }
            } else if (!this->queue_sender_.empty()) {
                if (Message msg = queue_sender_.pop(); msg.is_valid()) { if (this->send_message(msg)) { error_counter = 0x0; } else { error_counter++; } }
            } else { std::unique_lock lock{this->condition_sender_mutex_}; this->condition_sender_.wait_until(lock, heartbeat_time_point); }
        }
        if (error_counter != 0x0) { this->is_stopped_.store(true, STD_MEMORY_ORDER); std::printf("[Robomaster]: sender frame failure\n"); }
    }

    void Handler::receiver_thread() {
        struct CANMessage { std::vector<uint8_t> buffer; size_t length = 0x0; };
        uint32_t frame_id; uint8_t frame_buffer[8] = {}; size_t frame_length; size_t error_counter = 0x0;
        std::map<uint32_t, CANMessage> can_message {
            { Payload::DEVICE_ID_MOTION_CONTROLLER, CANMessage{} }, { Payload::DEVICE_ID_GIMBAL, CANMessage{} },
            { Payload::DEVICE_ID_HIT_DETECTOR_1, CANMessage{} }, { Payload::DEVICE_ID_HIT_DETECTOR_2, CANMessage{} },
            { Payload::DEVICE_ID_HIT_DETECTOR_3, CANMessage{} }, { Payload::DEVICE_ID_HIT_DETECTOR_4, CANMessage{} }
        };

        while (error_counter <= STD_MAX_ERROR_COUNT && !this->is_stopped_.load(STD_MEMORY_ORDER)) {
            if (!can_bus_.read_frame(frame_id, frame_buffer, frame_length)) { error_counter++; continue; }
            auto slice = can_message.find(frame_id); if (slice == can_message.end()) { continue; }
            auto&[buffer, length] = slice->second; buffer.insert(std::end(buffer), frame_buffer, frame_buffer + frame_length);

            if (length == 0) {
                auto iterator = buffer.cbegin();
                while (iterator != buffer.cend()) {
                    iterator = std::find(iterator, std::cend(buffer), 0x55); buffer.erase(std::cbegin(buffer), iterator);
                    if(buffer.size() < 4) { break; } if(buffer[3] == get_crc8(buffer.data(), 3)) { length = buffer[1]; break; } ++iterator;
                }
            } else if (length <= buffer.size()) {
                if (get_crc16(buffer.data(), length - 2) == get_little_endian(buffer[length - 2], buffer[length - 1])) {
                    auto const msg = Message{frame_id, std::vector(std::cbegin(buffer), std::cbegin(buffer) + static_cast<long>(length))};
                    if (msg.is_valid()) { this->receive_message(msg); }
                } buffer.erase(std::cbegin(buffer), std::cbegin(buffer) + static_cast<long>(length)); length = 0x0;
            }
        }
        if (error_counter != 0x0) { this->is_stopped_.store(true, STD_MEMORY_ORDER); std::printf("[Robomaster]: receiver frame failure\n"); }
    }
} // namespace robomaster