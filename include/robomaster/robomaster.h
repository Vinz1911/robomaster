// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#ifndef ROBOMASTER_ROBOMASTER_H_
#define ROBOMASTER_ROBOMASTER_H_

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
         * @return true, on success.
         * @return false, if initialization failed.
         */
        bool init(const std::string& interface="can0");

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
         * @param mode the chassis work mode.
         */
        void set_chassis_mode(ChassisMode mode);

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
         * @brief Drive the RoboMaster with the given velocities.
         *
         * @param linear_x Linear x velocity in m/s.
         * @param linear_y Linear y velocity in m/s.
         * @param angular_z Angular z velocity in radiant/s.
         */
        void set_chassis_velocity(float linear_x, float linear_y, float angular_z);

        /**
         * @brief Move the RoboMaster to the given position.
         *
         * @param linear_x Linear x position.
         * @param linear_y Linear y position.
         * @param angular_z Angular z position.
         */
        void set_chassis_position(int16_t linear_x, int16_t linear_y, int16_t angular_z);

        /**
         * @brief Set the work mode of the RoboMaster Gimbal.
         * @param mode the gimbal's work mode.
         */
        void set_gimbal_mode(GimbalMode mode);

        /**
         * @brief Set the state of the RoboMaster Gimbal.
         * @param state the gimbal's state (suspend or resume).
         */
        void set_gimbal_state(GimbalState state);

        /**
         * @brief Set the gimbal movement of the RoboMaster.
         *
         * @param pitch Angular x movement in radiant/s.
         * @param yaw Angular y movement in radiant/s.
         */
        void set_gimbal_degree(int16_t pitch, int16_t yaw);

        /**
         * @brief Set the gimbal velocity of the RoboMaster.
         *
         * @param pitch Angular x velocity in radiant/s.
         * @param yaw Angular y velocity in radiant/s.
         */
        void set_gimbal_velocity(int16_t pitch, int16_t yaw);

        /**
         * @brief Set the gimbal position of the RoboMaster.
         *
         * @param pitch Angular x position in degrees.
         * @param yaw Angular y position in degrees.
         * @param pitch_acceleration the acceleration in m/s^2.
         * @param yaw_acceleration the acceleration in m/s^2.
         */
        void set_gimbal_position(int16_t pitch, int16_t yaw, uint16_t pitch_acceleration = 250, uint16_t yaw_acceleration = 250);

        /**
         * @brief Recenter the gimbal of the RoboMaster.
         *
         * @param pitch Angular x velocity in radiant/s.
         * @param yaw Angular y velocity in radiant/s.
         */
        void set_gimbal_recenter(int16_t pitch, int16_t yaw);

        /**
         * @brief Fire the blaster of the RoboMaster.
         *
         * @param mode set the `BlasterMode`.
         * @param count set the count (1-8).
         */
        void set_blaster(BlasterMode mode, uint8_t count = 1);

        /**
         * @brief @brief Set the LED with a breath effect with given mask and timer.
         *
         * @param mode the `LEDMode` (STATIC, BREATHE, FLASH).
         * @param mask the `LEDMask` for selecting the LED. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
         * @param red colour between 0-255.
         * @param green colour between 0-255.
         * @param blue colour between 0-255.
         * @param up_time The rising time of the LED in seconds.
         * @param down_time The falling time of the LED in seconds.
         */
        void set_led(LEDMode mode, LEDMask mask, uint8_t red, uint8_t green, uint8_t blue, uint16_t up_time = 1000, uint16_t down_time = 1000);
    };
} // namespace robomaster

#endif // ROBOMASTER_ROBOMASTER_H_