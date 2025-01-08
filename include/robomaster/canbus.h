// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#ifndef ROBOMASTER_CAN_SOCKET_H_
#define ROBOMASTER_CAN_SOCKET_H_

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>

namespace robomaster {
    /**
     * @brief This class manage the io of the can bus.
     */
    class CANBus {
        /**
         * @brief The Socket for the CanBus.
         */
        int socket_;

        /**
         * @brief Struct to request the Can Bus interface.
         */
        ifreq ifr_;

        /**
         * @brief Struct for the Can Bus address.
         */
        sockaddr_can addr_;

    public:
        /**
         * @brief Construct the CanSocket object.
         */
        CANBus(/* args */);

        /**
         * @brief Destroy the Can Socket object and close socket.
         */
        ~CANBus();

        /**
         * @brief Set the timeout for reading the can socket.
         *
         * @param seconds Timeout in seconds.
         * @param microseconds Timeouts in microseconds.
         */
        void set_timeout(size_t seconds, size_t microseconds);

        /**
         * @brief Set the timeout for the reading the can socket.
         *
         * @param seconds Float in seconds.
         */
        void set_timeout(double seconds);

        /**
         * @brief Open the can socket by the given can interface name.
         *
         * @param can_interface The name of the can interface.
         * @return true, when the socket is open successfully.
         * @return false, when this socket failed to open.
         */
        bool init(const std::string &can_interface);

        /**
         * @brief Send a can frame over the socket.
         *
         * @param id The device id.
         * @param data The data of the can frame.
         * @param length The length of the data.
         * @return true, by success.
         * @return false, when failed.
         */
        bool send_frame(uint32_t id, const uint8_t data[8], size_t length) const;

        /**
         * @brief Read the next incoming can frame from the can socket. This function is blocking until the timeout is reached.
         *
         * @param id The device id.
         * @param data The data of the can frame.
         * @param length The length of the data. The length is zero, when the timeout is reached.
         * @return true, by success.
         * @return false  when failed.
         */
        bool read_frame(uint32_t &id, uint8_t data[8], size_t &length) const;
    };
} // namespace robomaster

#endif // ROBOMASTER_CAN_SOCKET_H_