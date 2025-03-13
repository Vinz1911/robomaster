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

#include <cassert>
#include <string>

#include "robomaster/message.h"
#include "robomaster/utils.h"

namespace robomaster {
    Message::Message(const uint32_t device_id, const std::vector<uint8_t>& message_data): is_valid_{false}, device_id_{device_id}, sequence_{}, type_{} {
        if (message_data.size() <= 10) { return; }
        this->type_ = get_little_endian(message_data[4], message_data[5]);
        this->sequence_ = get_little_endian(message_data[6], message_data[7]);
        this->payload_.clear();
        this->payload_.insert(std::begin(this->payload_), std::cbegin(message_data) + 8, std::cend(message_data) - 2);
        this->is_valid_ = true;
    }

    Message::Message(
        const uint32_t device_id,
        const uint16_t device_type,
        const uint16_t sequence,
        std::vector<uint8_t> payload): is_valid_{true}, device_id_{device_id}, sequence_{sequence}, type_{device_type}, payload_{std::move(payload)} {
    }

    bool Message::is_valid() const {
        return this->is_valid_;
    }

    uint32_t Message::get_device_id() const {
        return this->device_id_;
    }

    uint16_t Message::get_sequence() const {
        return this->sequence_;
    }

    uint16_t Message::get_type() const {
        return this->type_;
    }

    std::vector<uint8_t> Message::get_payload() const {
        return this->payload_;
    }

    size_t Message::get_length() const {
        return this->payload_.size() + 10;
    }

    uint8_t Message::get_uint8(const size_t index) const {
        assert(index < this->payload_.size());
        return this->payload_[index];
    }

    uint16_t Message::get_uint16(const size_t index) const {
        assert(index + 1 < this->payload_.size());
        uint16_t value = this->payload_[index + 1];
        value = value << 8 | this->payload_[index];
        return value;
    }

    uint32_t Message::get_uint32(const size_t index) const {
        assert(index + 3 < this->payload_.size());
        uint32_t value = this->payload_[index + 3];
        value = value << 8 | this->payload_[index + 2];
        value = value << 8 | this->payload_[index + 1];
        value = value << 8 | this->payload_[index];
        return value;
    }

    int8_t Message::get_int8(const size_t index) const {
        assert(index < this->payload_.size());
        return static_cast<int8_t>(this->payload_[index]);
    }

    int16_t Message::get_int16(const size_t index) const {
        assert(index + 1 < this->payload_.size());
        int16_t value = this->payload_[index + 1];
        value = static_cast<int16_t>(value << 8 | this->payload_[index]);
        return value;
    }

    int32_t Message::get_int32(const size_t index) const {
        assert(index + 3 < this->payload_.size());
        int32_t value = this->payload_[index + 3];
        value = value << 8 | this->payload_[index + 2];
        value = value << 8 | this->payload_[index + 1];
        value = value << 8 | this->payload_[index];
        return value;
    }

    float Message::get_float(const size_t index) const {
        assert(index + 3 < this->payload_.size());
        union { uint32_t u; float f; } float_uint32_t_union{};
        float_uint32_t_union.u = this->get_uint32(index);
        return float_uint32_t_union.f;
    }

    void Message::set_type(const uint16_t type) {
        this->type_ = type;
    }

    void Message::set_payload(const std::vector<uint8_t>& payload) {
        this->payload_ = payload;
    }

    void Message::set_uint8(const size_t index, const uint8_t value) {
        assert(index < this->payload_.size());
        this->payload_[index] = value;
    }

    void Message::set_uint16(const size_t index, const uint16_t value) {
        assert(index + 1 < this->payload_.size());
        this->payload_[index] = static_cast<uint8_t>(value);
        this->payload_[index + 1] = static_cast<uint8_t>(value >> 8);
    }

    void Message::set_uint32(const size_t index, const uint32_t value) {
        assert(index + 3 < this->payload_.size());
        this->payload_[index] = static_cast<uint8_t>(value);
        this->payload_[index + 1] = static_cast<uint8_t>(value >> 8);
        this->payload_[index + 2] = static_cast<uint8_t>(value >> 16);
        this->payload_[index + 3] = static_cast<uint8_t>(value >> 24);
    }

    void Message::set_int8(const size_t index, const int8_t value) {
        assert(index < this->payload_.size());
        this->payload_[index] = value;
    }

    void Message::set_int16(const size_t index, const int16_t value) {
        assert(index + 1 < this->payload_.size());
        this->payload_[index] = static_cast<uint8_t>(value);
        this->payload_[index + 1] = static_cast<uint8_t>(value >> 8);
    }

    void Message::set_int32(const size_t index, const int32_t value) {
        assert(index + 3 < this->payload_.size());
        this->payload_[index] = static_cast<uint8_t>(value);
        this->payload_[index + 1] = static_cast<uint8_t>(value >> 8);
        this->payload_[index + 2] = static_cast<uint8_t>(value >> 16);
        this->payload_[index + 3] = static_cast<uint8_t>(value >> 24);
    }

    void Message::set_float(const size_t index, const float value) {
        assert(index + 3 < this->payload_.size());
        union { uint32_t u; float f; } float_uint32_t_union{};
        float_uint32_t_union.f = value;
        this->set_uint32(index, float_uint32_t_union.u);
    }

    std::vector<uint8_t> Message::vector() const {
        std::vector<uint8_t> vector;
        if (!this->is_valid_) { return vector; }

        vector.resize(10 + this->payload_.size());
        vector[0] = 0x55;
        vector[1] = static_cast<uint8_t>(vector.size());
        vector[2] = 0x04;
        vector[3] = get_crc8(vector.data(), 3);
        vector[4] = static_cast<uint8_t>(this->type_);
        vector[5] = static_cast<uint8_t>(this->type_ >> 8);
        vector[6] = static_cast<uint8_t>(this->sequence_);
        vector[7] = static_cast<uint8_t>(this->sequence_ >> 8);

        for (size_t i = 0; i < this->payload_.size(); i++) { vector[8 + i] = this->payload_[i]; }
        const uint16_t crc16 = get_crc16(vector.data(), vector.size() - 2);

        vector[vector.size() - 2] = static_cast<uint8_t>(crc16);
        vector[vector.size() - 1] = static_cast<uint8_t>(crc16 >> 8);
        return vector;
    }
} // namespace robomaster