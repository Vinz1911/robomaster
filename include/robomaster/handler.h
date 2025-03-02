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
#include <thread>
#include <condition_variable>
#include <functional>
#include <atomic>

#include "can.h"
#include "message.h"
#include "queue.h"

namespace robomaster {
    /**
     * @brief This class handles the incoming and outgoing RoboMaster message over the can bus.
     *
     */
    class Handler {
        /**
         * @brief CanSocket class for the can bus io.
         */
        CANBus can_bus_;

        /**
         * @brief Thread for reading on the can socket and put valid messages in the receiver queue.
         */
        std::thread thread_receiver_;

        /**
         * @brief Thread for sending messages on the can bus. Also triggers the 10 ms heartbeat to keep the RoboMaster alive.
         */
        std::thread thread_sender_;

        /**
         * @brief Sender queue for sending messages.
         */
        Queue queue_sender_;

        /**
         * @brief Conditional variable for the sender thread, when new messages put into the sender queue.
         */
        std::condition_variable condition_sender_;

        /**
         * @brief Mutex of the sender conditional variable.
         */
        std::mutex condition_sender_mutex_;

        /**
         * @brief callback function for the data of the robomaster motion controller.
         */
        std::function<void(const Message&)> state_callback_;

        /**
         * @brief Status of the initialisation of the handler class. True when the can socket was successfully initialised.
         */
        bool is_initialised_;

        /**
         * @brief Status if the threads are running to prevent multiply starts.
         */
        std::atomic<bool> is_stopped_;

        /**
         * @brief Run function of the sender thread.
         */
        void sender_thread();

        /**
         * @brief Run function of the receiver thread.
         */
        void receiver_thread();

        /**
         * @brief Joining all started threads.
         */
        void join_all();

        /**
         * @brief Send the message to the can socket.
         *
         * @param message The RoboMaster message.
         * @return true, by success.
         * @return false, by failing to send the message.
         */
        [[nodiscard]] bool send_message(const Message& message) const;

        /**
         * @brief Process the received messages from the message queue and triggers callback functions.
         *
         * @param message RoboMaster message.
         */
        void receive_message(const Message& message) const;

    public:
        /**
         * @brief Construct a new Handler object.
         *
         */
        Handler();

        /**
         * @brief Destroy the Handler object and stop the threads.
         */
        ~Handler();

        /**
         * @brief Init the can socket and start the threads.
         *
         * @param interface The can interface name.
         * @return true, when successful initialised.
         * @return false, by failing the initialisation.
         */
        bool init(const std::string& interface="can0");

        /**
         * @brief State if the handler is running or not.
         *
         * @return true If the Handler is running and ready to receive and send messages.
         * @return false If the handler was stopped due to error.
         */
        [[nodiscard]] bool is_running() const;

        /**
         * @brief Push a message to the sender queue to send it over the can bus.
         *
         * @param message A RoboMaster message.
         */
        void push_message(const Message& message);

        /**
         * @brief Bind the given callback for triggering when the message for the RoboMasterState is received.
         *
         * @param completion The callback to trigger.
         */
        void set_callback(std::function<void(const Message&)> completion);
    };
} // namespace robomaster