/*
 * Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
 *
 * This project contains contributions from multiple authors.
 * The original code is licensed under the MIT License by Fraunhofer IML.
 * All modifications and additional code are licensed under the MIT License by Vinzenz Weist.
 */

#include <cstring>
#include <cmath>
#include <unistd.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "robomaster/canbus.h"

namespace robomaster {
    CANBus::CANBus(): socket_(), ifr_{}, addr_{} {
        memset(&this->ifr_, 0x0, sizeof(this->ifr_));
        memset(&this->addr_, 0x0, sizeof(this->addr_));
    }

    CANBus::~CANBus() {
        close(socket_);
    }

    void CANBus::set_timeout(const size_t seconds, const size_t microseconds) {
        timeval t{}; t.tv_sec = static_cast<long>(seconds); t.tv_usec = static_cast<long>(microseconds);
        setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, &t, sizeof(t));
    }

    void CANBus::set_timeout(const double seconds) {
        if (seconds < 0.0) { this->set_timeout(0, 0); return; }
        const auto seconds_t = static_cast<size_t>(std::floor(seconds));
        const auto microseconds_t = static_cast<size_t>((seconds - std::floor(seconds)) * 1e6);
        this->set_timeout(seconds_t, microseconds_t);
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
        frame.can_id = static_cast<int>(id); frame.can_dlc = length; std::memcpy(static_cast<uint8_t *>(frame.data), data, length);
        if (write(this->socket_, &frame, sizeof(frame)) < 0) { std::printf("[Robomaster]: failed to send can frame\n"); return false; } return true;
    }

    bool CANBus::read_frame(uint32_t& id, uint8_t data[8], size_t& length) const {
        can_frame frame = {}; memset(&frame, 0, sizeof(frame));
        if(read(this->socket_, &frame, sizeof(frame)) < 0) { std::printf("[Robomaster]: failed to read can frame\n"); return false; }
        id = frame.can_id & CAN_EFF_FLAG ? frame.can_id & CAN_EFF_MASK: frame.can_id & CAN_SFF_MASK;
        length = frame.can_dlc; std::memcpy(data, frame.data, length); return true;
    }
} // namespace robomaster