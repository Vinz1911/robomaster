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
#include <net/if.h>
#include <linux/can.h>
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
         * @brief Set the timeout for the reading the can socket.
         *
         * @param seconds Double in seconds.
         */
        void set_timeout(double seconds) const;

        /**
         * @brief Open the can socket by the given can interface name.
         *
         * @param interface The name of the can interface.
         * @return true, when the socket is open successfully.
         * @return false, when this socket failed to open.
         */
        bool init(const std::string& interface);

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
        bool read_frame(uint32_t& id, uint8_t data[8], size_t& length) const;
    };
} // namespace robomaster