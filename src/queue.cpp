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

#include "robomaster/queue.h"

namespace robomaster {
    static constexpr size_t STD_MAX_QUEUE_SIZE = 10;

    Queue::Queue() = default;

    size_t Queue::max_size() {
        return STD_MAX_QUEUE_SIZE;
    }

    void Queue::push(const Message& message) {
        std::lock_guard lock{this->mutex_};
        if (STD_MAX_QUEUE_SIZE <= this->queue_.size()) { this->queue_.pop(); }
        this->queue_.push(message);
    }

    Message Queue::pop() {
        std::lock_guard lock{this->mutex_};
        if (this->queue_.empty()) { const auto msg = Message(0x0, {}); return msg; }
        const Message msg = queue_.front();
        this->queue_.pop(); return msg;
    }

    size_t Queue::size() {
        std::lock_guard lock{this->mutex_};
        return this->queue_.size();
    }

    bool Queue::empty() {
        std::lock_guard lock{this->mutex_};
        return this->queue_.empty();
    }

    void Queue::clear() {
        std::lock_guard lock{this->mutex_};
        while(!this->queue_.empty()) { this->queue_.pop(); }
    }
} // namespace robomaster