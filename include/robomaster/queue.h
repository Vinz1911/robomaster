// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#ifndef ROBOMASTER_QUEUE_MSG_H_
#define ROBOMASTER_QUEUE_MSG_H_

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
         *
         */
        std::queue<Message> queue_;

        /**
         * @brief The mutex to protect the critical section.
         */
        std::mutex mutex_;

    public:
        /**
         * @brief Construct a new Queue Msg object.
         */
        Queue();

        /**
         * @brief Push a Message into the queue. If the maximal queue size is reached the front message will be pop.
         *
         * @param msg A RoboMaster message.
         */
        void push(const Message &msg);

        /**
         * @brief Push a Message into the queue. If the maximal queue size is reached the front message will be pop.
         *
         * @param msg A RoboMaster message.
         */
        void push(Message && msg);

        /**
         * @brief Pop and return the message of the queue. If the queue is empty a empty message is returned.
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
        size_t max_queue_size();

        /**
         * @brief True when the queue is empty.
         *
         * @return true, when empty.
         * @return false, when not empty.
         */
        bool empty();

        /**
         * @brief Clear all RoboMaster messages from the queue.
         */
        void clear();
    };
} // namespace robomaster

#endif // ROBOMASTER_QUEUE_MSG_H_