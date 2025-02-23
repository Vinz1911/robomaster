/*
 * Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
 *
 * This project contains contributions from multiple authors.
 * The original code is licensed under the MIT License by Fraunhofer IML.
 * All modifications and additional code are licensed under the MIT License by Vinzenz Weist.
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
        CANBus can_socket_;

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
        std::condition_variable cv_sender_;

        /**
         * @brief Mutex of the sender conditional variable.
         */
        std::mutex cv_sender_mutex_;

        /**
         * @brief callback function for the data of the robomaster motion controller.
         */
        std::function<void(const Message&)> state_callback_;

        /**
         * @brief Flag of the initialisation of the handler class. True when the can socket was successfully initialised.
         */
        bool flag_initialised_;

        /**
         * @brief Flag then the threads are running to prevent multiply starts.
         */
        std::atomic<bool> flag_stop_;

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
         * @param msg The RoboMaster message.
         * @return true, by success.
         * @return false, by failing to send the message.
         */
        [[nodiscard]] bool send_message(const Message& msg) const;

        /**
         * @brief Process the received messages from the message queue and triggers callback functions.
         *
         * @param msg RoboMaster message.
         */
        void receive_message(const Message& msg) const;

    public:
        /**
         * @brief Construct a new Handler object.
         *
         */
        Handler();

        /**
         * @brief Destroy the Handler object and stopped the threads.
         */
        ~Handler();

        /**
         * @brief Init the can socket and start the threads.
         *
         * @param can_interface The can interface name.
         * @return true, when successful initialised.
         * @return false, by failing the initialisation.
         */
        bool init(const std::string& can_interface="can0");

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
         * @param msg A RoboMaster message.
         */
        void push_message(const Message& msg);

        /**
         * @brief Bind the given callback for triggering when the message for the RoboMasterState is received.
         *
         * @param func The callback to trigger.
         */
        void set_callback(std::function<void(const Message&)> func);
    };
} // namespace robomaster