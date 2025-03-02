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

#pragma once
#include <mutex>
#include <queue>

#include "message.h"

namespace robomaster {
    /**
     * @brief This class is queue for RoboMaster messages which is protected by a mutex.
     */
    class Queue {
        /**
         * @brief The message queue.
         */
        std::queue<Message> queue_;

        /**
         * @brief The mutex to protect the critical section.
         */
        std::mutex mutex_;

    public:
        /**
         * @brief Constructor of the Queue class.
         */
        Queue(/* args */);

        /**
         * @brief Destructor of the Queue class.
         */
        ~Queue() = default;

        /**
         * @brief Push a Message into the queue. If the maximal queue size is reached the front message will be pop.
         *
         * @param message A RoboMaster message.
         */
        void push(const Message& message);

        /**
         * @brief Pop and return the message of the queue. If the queue is empty an empty message is returned.
         *
         * @return RoboMaster message.
         */
        Message pop();

        /**
         * @brief The current size of the queue.
         *
         * @return size_t as size.
         */
        size_t size();

        /**
         * @brief Get the maximal allow queue size.
         *
         * @return size_t as maximal queue size.
         */
        static size_t max_size() ;

        /**
         * @brief True when the queue is empty.
         *
         * @return true, when empty, false, when not empty.
         */
        bool empty();

        /**
         * @brief Clear all RoboMaster messages from the queue.
         */
        void clear();
    };
} // namespace robomaster