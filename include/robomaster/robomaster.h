// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#ifndef ROBOMASTER_ROBOMASTER_H_
#define ROBOMASTER_ROBOMASTER_H_

#include "handler.h"
#include "data.h"

namespace robomaster {
    /**
     * @brief The blaster type
     */
    enum BlasterType {
        INFRARED,
        GELBEADS
    };

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
         * @brief Callback function to trigger when new RoboMasterState data arte received.
         */
        std::function<void(const DataRoboMasterState &)> callback_data_robomaster_state_;

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
         * @brief The boot sequence to configure the RoboMasterState messages.
         */
        void boot_sequence();

        /**
         * @brief Decode the RoboMasterState message and trigger the callback function.
         *
         * @param msg The RoboMasterState message.
         */
        void decode_state(const Message &msg);

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
         * @brief Enable or Disable the work mode of the RoboMaster chassis.
         */
        void set_work_mode(bool mode);

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
         * @param fr Front right wheel in rpm.
         * @param fl Front left wheel in rpm.
         * @param rl Rear left wheel in rpm.
         * @param rr Rear right wheel in rpm.
         */
        void set_wheel_rpm(int16_t fr, int16_t fl, int16_t rl, int16_t rr);

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
        void set_blaster(BlasterType blaster);

        /**
         * @brief Stop the RoboMaster with zero velocities.
         */
        void set_brake();

        /**
         * @brief Set the LED off by the given mask.
         *
         * @param mask Mask for selecting the LED. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
         */
        void set_led_off(uint16_t mask);

        /**
         * @brief Set the LED on by the given mask.
         *
         * @param mask Mask for selecting the LED. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
         */
        void set_led_on(uint16_t mask, uint8_t r, uint8_t g, uint8_t b);

        /**
         * @brief @brief Set the LED with a breath effect with given mask and timer.
         *
         * @param mask Mask for selecting the LED. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
         * @param r Red value colour between 0-255.
         * @param g Green value colour between 0-255.
         * @param b Blue value colour between 0-255.
         * @param t_rise The rising time of the LED in seconds.
         * @param t_down The falling time of the LED in seconds.
         */
        void set_led_breath(uint16_t mask, uint8_t r, uint8_t g, uint8_t b, uint16_t t_rise, uint16_t t_down);

        /**
         * @brief Set the LED with a breath effect with given mask and timer.
         *
         * @param mask Mask for selecting the LED. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
         * @param r Red value colour between 0-255.
         * @param g Green value colour between 0-255.
         * @param b Blue value colour between 0-255.
         * @param t_rise The rising time of the LED in seconds in milliseconds.
         * @param t_down The falling time of the LED in seconds in milliseconds.
         */
        void set_led_breath(uint16_t mask, uint8_t r, uint8_t g, uint8_t b, float t_rise, float t_down);

        /**
         * @brief Set the LED with a breath effect with given mask and rate.
         *
         * @param mask Mask for selecting the LED. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
         * @param r Red value colour between 0-255.
         * @param g Green value colour between 0-255.
         * @param b Blue value colour between 0-255.
         * @param rate The rate of the breath effect.
         */
        void set_led_breath(uint16_t mask, uint8_t r, uint8_t g, uint8_t b, float rate);

        /**
         * @brief Set the LED with a flash effect with given mask and timer.
         *
         * @param mask Mask for selecting the LED. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
         * @param r Red value colour between 0-255.
         * @param g Green value colour between 0-255.
         * @param b Blue value colour between 0-255.
         * @param t_on The on time of the LED in seconds.
         * @param t_off The off time of the LED in seconds.
         */
        void set_led_flash(uint16_t mask, uint8_t r, uint8_t g, uint8_t b, uint16_t t_on, uint16_t t_off);

        /**
         * @brief Set the LED with a flash effect with given mask and timer.
         *
         * @param mask Mask for selecting the LED. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
         * @param r Red value colour between 0-255.
         * @param g Green value colour between 0-255.
         * @param b Blue value colour between 0-255.
         * @param t_on The on time of the LED in milliseconds.
         * @param t_off The off time of the LED in milliseconds.
         */
        void set_led_flash(uint16_t mask, uint8_t r, uint8_t g, uint8_t b, float t_on, float t_off);

        /**
         * @brief Set the LED with a flash effect with given mask and rate.
         *
         * @param mask Mask for selecting the LED. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
         * @param r Red value colour between 0-255.
         * @param g Green value colour between 0-255.
         * @param b Blue value colour between 0-255.
         * @param rate The rate of the flash effect.
         */
        void set_led_flash(uint16_t mask, uint8_t r, uint8_t g, uint8_t b, float rate);

        /**
         * @brief Bind a function to the callback which get triggered when a new RoboMasterState message is received.
         *
         * @param func Function to bind as callback.
         */
        void set_callback(std::function<void(const DataRoboMasterState&)> func);

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
        bool is_running();
    };
} // namespace robomaster

#endif // ROBOMASTER_ROBOMASTER_H_