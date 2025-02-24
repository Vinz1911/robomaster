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

#include <cstring>
#include <cmath>
#include <unistd.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "robomaster/can.h"

namespace robomaster {
    CANBus::CANBus(): socket_(), ifr_{}, addr_{} {
        memset(&this->ifr_, 0x0, sizeof(this->ifr_));
        memset(&this->addr_, 0x0, sizeof(this->addr_));
    }

    CANBus::~CANBus() {
        close(socket_);
    }

    void CANBus::set_timeout(const double seconds) const {
        const auto max_time = std::max(seconds, 0.0);
        const auto seconds_t = static_cast<long>(std::floor(max_time)), microseconds_t = static_cast<long>((max_time - std::floor(max_time)) * 1e6);

        timeval time = {}; time.tv_sec = seconds_t; time.tv_usec = microseconds_t;
        setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, &time, sizeof(time));
    }

    bool CANBus::init(const std::string& interface) {
        this->socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (this->socket_ < 0) { std::printf("[Robomaster]: failed to open can interface\n"); return false; }
        std::memcpy(this->ifr_.ifr_name, interface.c_str(), interface.size());

        if (ioctl(this->socket_, SIOCGIFFLAGS, &this->ifr_) < 0) { std::printf("[Robomaster]: failed to request can interface %s\n", interface.c_str()); close(this->socket_); return false; }
        if (!(this->ifr_.ifr_flags & IFF_UP)) { std::printf("[Robomaster]: interface %s is down\n", interface.c_str()); close(this->socket_); return false; }

        ioctl(this->socket_, SIOGIFINDEX, &this->ifr_); this->addr_.can_ifindex = this->ifr_.ifr_ifindex; this->addr_.can_family = PF_CAN;
        if (bind(this->socket_, reinterpret_cast<sockaddr *>(&this->addr_), sizeof(this->addr_)) < 0) { std::printf("[Robomaster]: failed to bind can address\n"); close(this->socket_); return false; } return true;
    }

    bool CANBus::send_frame(const uint32_t id, const uint8_t data[8], const size_t length) const {
        if (length > 8) { std::printf("[Robomaster]: failed to send can frame\n"); return false; }
        can_frame frame = {}; memset(&frame, 0, sizeof(frame));
        frame.can_id = static_cast<int>(id); frame.can_dlc = length; std::memcpy(frame.data, data, length);
        if (write(this->socket_, &frame, sizeof(frame)) < 0) { std::printf("[Robomaster]: failed to send can frame\n"); return false; } return true;
    }

    bool CANBus::read_frame(uint32_t& id, uint8_t data[8], size_t& length) const {
        can_frame frame = {}; memset(&frame, 0, sizeof(frame));
        if(read(this->socket_, &frame, sizeof(frame)) < 0) { std::printf("[Robomaster]: failed to read can frame\n"); return false; }
        id = frame.can_id & CAN_EFF_FLAG ? frame.can_id & CAN_EFF_MASK: frame.can_id & CAN_SFF_MASK;
        length = frame.can_dlc; std::memcpy(data, frame.data, length); return true;
    }
} // namespace robomaster