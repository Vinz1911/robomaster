// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#ifndef ROBOMASTER_ROBOMASTER_H_
#define ROBOMASTER_ROBOMASTER_H_

#include <optional>
#include <atomic>

#include "handler.h"
#include "data.h"
#include "definitions.h"

namespace robomaster {
    /**
     * @brief This class manage the control of the RoboMaster via can socket.
     *
     */
    class RoboMaster {
        /**
         * @brief Handler class for the RoboMaster message io and thread management.
         */
        Handler handler_;

        /**
         * @brief Counter for the message sequence of the drive messages.
         */
        uint16_t counter_drive_;

        /**
         * @brief Counter for the message sequence of the LED messages.
         */
        uint16_t counter_led_;

        /**
        * @brief Counter for the message sequence of the gimbal messages.
        */
        uint16_t counter_gimbal_;

        /**
        * @brief Counter for the message sequence of the blaster messages.
        */
        uint16_t counter_blaster_;

        /**
         * @brief Store for the data state
         */
        std::atomic<RoboMasterState> state_;

        /**
         * @brief The boot sequence to configure the RoboMasterState messages.
         */
        void boot_sequence();

        /**
         * @brief Decode the RoboMasterState message and trigger the callback function.
         *
         * @param msg The RoboMasterState message.
         * @return the current data state
         */
        static RoboMasterState decode_state(const Message &msg);

    public:
        /**
         * @brief Constructor of the RoboMaster class.
         */
        RoboMaster(/* args */);
        /**
         * @brief Destructor of the RoboMaster class.
         */
        ~RoboMaster();

        /**
         * @brief get the current state from DataRoboMasterState
         * @return the collected state data
         */
        [[nodiscard]] RoboMasterState get_state() const;

        /**
         * @brief Enable or Disable the torque of the RoboMaster chassis.
         */
        void set_torque(bool enable);

        /**
         * @brief Drive the RoboMaster with the given velocities.
         *
         * @param x Linear x velocity in m/s.
         * @param y Linear y velocity in m/s.
         * @param z Angular velocity in radiant/s.
         */
        void set_velocity(float x, float y, float z);

        /**
         * @brief Control each individual wheel of the RoboMaster in rpm.
         *
         * @param front_right wheel in rpm.
         * @param front_left wheel in rpm.
         * @param rear_left wheel in rpm.
         * @param rear_right wheel in rpm.
         */
        void set_wheel_rpm(int16_t front_right, int16_t front_left, int16_t rear_left, int16_t rear_right);

        /**
         * @brief Control the gimbal of the RoboMaster
         *
         * @param y Angular y velocity in radiant/s
         * @param z  Angular z velocity in radiant/s
         */
        void set_gimbal(int16_t y, int16_t z);

        /**
         * @brief Fire the blaster of the RoboMaster
         */
        void set_blaster(BlasterMode mode);

        /**
         * @brief Stop the RoboMaster with zero velocities.
         */
        void set_brake();

        /**
         * @brief @brief Set the LED with a breath effect with given mask and timer.
         *
         * @param mode the `LEDMode` (STATIC, BREATHE, FLASH)
         * @param mask Mask for selecting the LED. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
         * @param red colour between 0-255.
         * @param green colour between 0-255.
         * @param blue colour between 0-255.
         * @param up_time The rising time of the LED in seconds.
         * @param down_time The falling time of the LED in seconds.
         */
        void set_led(LEDMode mode, uint16_t mask, uint8_t red, uint8_t green, uint8_t blue, std::optional<uint16_t> up_time = std::nullopt, std::optional<uint16_t> down_time = std::nullopt);

        /**
         * @brief Init the RoboMaster can socket to communicate with the motion controller.
         *
         * @param can_interface Can interface name.
         * @return true, by success.
         * @return false, when initialization failed.
         */
        bool init(const std::string &can_interface="can0");

        /**
         * @brief True when the robomaster is successful initialized and ready to receive and send messages.
         *
         * @return true if the robomaster is successful initialized and is running. false when a can error is appeared.
         */
        [[nodiscard]] bool is_running() const;
    };
} // namespace robomaster

#endif // ROBOMASTER_ROBOMASTER_H_