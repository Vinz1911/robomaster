/*
 * Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
 *
 * This project contains contributions from multiple authors.
 * The original code is licensed under the MIT License by Fraunhofer IML.
 * All modifications and additional code are licensed under the MIT License by Vinzenz Weist.
 */

#include "robomaster/queue.h"

namespace robomaster {
    static constexpr size_t STD_MAX_QUEUE_SIZE = 10;

    Queue::Queue() = default;

    void Queue::push(const Message& msg) {
        std::lock_guard lock(this->mutex_);
        if (STD_MAX_QUEUE_SIZE <= this->queue_.size()) { this->queue_.pop(); }
        this->queue_.push(msg);
    }

    void Queue::push(Message&& msg) {
        std::lock_guard lock(this->mutex_);
        if (STD_MAX_QUEUE_SIZE <= this->queue_.size()) { this->queue_.pop(); }
        this->queue_.emplace(std::move(msg));
    }

    Message Queue::pop() {
        std::lock_guard lock(this->mutex_);
        if (this->queue_.empty()) { const auto msg = Message(0, {}); return msg; }
        const Message msg = queue_.front();
        this->queue_.pop(); return msg;
    }

    size_t Queue::size() {
        std::lock_guard lock(this->mutex_);
        return this->queue_.size();
    }

    bool Queue::empty() {
        std::lock_guard lock(this->mutex_);
        return this->queue_.empty();
    }

    size_t Queue::max_queue_size() {
        return STD_MAX_QUEUE_SIZE;
    }

    void Queue::clear() {
        std::lock_guard lock(this->mutex_);
        while(!this->queue_.empty()) { this->queue_.pop(); }
    }
} // namespace robomaster