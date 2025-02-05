// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#ifndef ROBOMASTER_ROBOMASTER_H_
#define ROBOMASTER_ROBOMASTER_H_

#include <optional>

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
         * @brief Counter for the message sequences.
         */
        uint16_t message_counter_;

        /**
         * @brief Store for the data state
         */
        RoboMasterState state_;

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
        static RoboMasterState decode_state(const Message& msg);

    public:
        /**
         * @brief Constructor of the RoboMaster class.
         */
        RoboMaster(/* args */);

        /**
         * @brief Destructor of the RoboMaster class.
         */
        ~RoboMaster() = default;

        /**
         * @brief Init the RoboMaster can socket to communicate with the motion controller.
         *
         * @param interface can interface name.
         * @param state enable or disable reading from can [get_state()]
         * @return true, by success.
         * @return false, if initialization failed.
         */
        bool init(const std::string& interface="can0", bool state = true);

        /**
         * @brief True when the robomaster is successful initialized and ready to receive and send messages.
         *
         * @return true if the robomaster is successful initialized and is running. false when a can error is appeared.
         */
        [[nodiscard]] bool is_running() const;

        /**
         * @brief get the current state from DataRoboMasterState
         *
         * @return the collected state data
         */
        [[nodiscard]] RoboMasterState get_state() const;

        /**
         * @brief Set the work mode of the RoboMaster Chassis.
         */
        void set_chassis_mode(ChassisMode mode);

        /**
         * @brief Set the work mode of the RoboMaster Gimbal.
         */
        void set_gimbal_mode(GimbalMode mode);

        /**
         * @brief Drive the RoboMaster with the given velocities.
         *
         * @param pitch Linear x velocity in m/s.
         * @param yaw Linear y velocity in m/s.
         * @param roll Angular z velocity in radiant/s.
         */
        void set_chassis_velocity(float pitch, float yaw, float roll);

        /**
         * @brief Control each individual wheel of the RoboMaster in rpm.
         *
         * @param front_right wheel in rpm.
         * @param front_left wheel in rpm.
         * @param rear_left wheel in rpm.
         * @param rear_right wheel in rpm.
         */
        void set_chassis_rpm(int16_t front_right, int16_t front_left, int16_t rear_left, int16_t rear_right);

        /**
         * @brief Set the gimbal fixed degree of the RoboMaster
         *
         * @param pitch Linear x (continues) position in degrees
         * @param yaw  Linear y (continues) position in degrees
         */
        void set_gimbal_degree(int16_t pitch, int16_t yaw);

        /**
         * @brief Set the gimbal (continues velocity) of the RoboMaster
         *
         * @param pitch Linear x velocity in radiant/s
         * @param yaw  Linear y velocity in radiant/s
         */
        void set_gimbal_velocity(int16_t pitch, int16_t yaw);

        /**
         * @brief Recenter the gimbal of the RoboMaster
         *
         * @param pitch Linear x velocity in radiant/s
         * @param yaw  Linear y velocity in radiant/s
         */
        void set_gimbal_recenter(int16_t pitch, int16_t yaw);

        /**
         * @brief Fire the blaster of the RoboMaster
         *
         * @param mode set the `BlasterMode`
         * @param count set the count (1-8)
         */
        void set_blaster(BlasterMode mode, uint8_t count = 1);

        /**
         * @brief @brief Set the LED with a breath effect with given mask and timer.
         *
         * @param mode the `LEDMode` (STATIC, BREATHE, FLASH)
         * @param mask the `LEDMask` for selecting the LED. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
         * @param red colour between 0-255.
         * @param green colour between 0-255.
         * @param blue colour between 0-255.
         * @param up_time The rising time of the LED in seconds.
         * @param down_time The falling time of the LED in seconds.
         */
        void set_led(LEDMode mode, LEDMask mask, uint8_t red, uint8_t green, uint8_t blue, std::optional<uint16_t> up_time = std::nullopt, std::optional<uint16_t> down_time = std::nullopt);
    };
} // namespace robomaster

#endif // ROBOMASTER_ROBOMASTER_H_