// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#ifndef ROBOMASTER_DATA_H_
#define ROBOMASTER_DATA_H_

#include <array>

#include "message.h"

namespace robomaster {
    /**
     * @brief Struct for the data of the ESC from the RoboMaster. The data array is ordered in front right, front left, rear left and rear right.
     */
    struct StateESC {
        /**
         * @brief True when the ESC data are successfully parsed.
         */
        bool has_data = false;

        /**
         * @brief Speed in RPM -> value range -8192~8191;
         */
        std::array<int16_t, 4> speed = { 0, 0, 0, 0 };

        /**
         * @brief Angle position -> value range 0~32767 maps to -> 0~360
         */
        std::array<int16_t, 4> angle = { 0, 0, 0, 0 };

        /**
         * @brief Timestamp.
         */
        std::array<uint32_t, 4> time_stamp = { 0, 0, 0, 0 };

        /**
         * @brief State of the ESC.
         */
        std::array<uint8_t, 4> state = { 0, 0, 0, 0 };
    };

    /**
     * @brief Struct for the imu data from the RoboMaster.
     */
    struct StateIMU {
        /**
         * @brief True when the Imu data are successfully parsed.
         */
        bool has_data = false;

        /**
         * @brief Acceleration on x-axis in 9,81 /m^2 s.
         */
        float acc_x = 0.0f;

        /**
         * @brief Acceleration on y-axis in 9,81 /m^2 s.
         */
        float acc_y = 0.0f;

        /**
         * @brief Acceleration on z axis in 9,81 /m^2 s.
         */
        float acc_z = 0.0f;

        /**
         * @brief Angular velocity on x-axis in radiant.
         */
        float gyro_x = 0.0f;

        /**
         * @brief Angular velocity on y-axis in radiant.
         */
        float gyro_y = 0.0f;

        /**
         * @brief Angular velocity on z axis in radiant.
         */
        float gyro_z = 0.0f;
    };

    /**
     * @brief Struct for the attitude data from the RoboMaster.
     */
    struct StateAttitude {
        /**
         * @brief True when the attitude data are successfully parsed.
         */
        bool has_data = false;

        /**
         * @brief Roll in degree.
         */
        float roll = 0.0f;

        /**
         * @brief Pitch in degree.
         */
        float pitch = 0.0f;

        /**
         * @brief Yaw in degree.
         */
        float yaw = 0.0f;
    };

    /**
     * @brief Struct for the battery data from the RoboMaster.
     */
    struct StateBattery {
        /**
         * @brief True when the battery data are successfully parsed.
         */
        bool has_data = false;

        /**
         * @brief ADC value of the battery in milli volt.
         */
        uint16_t adc_value = 0;

        /**
         * @brief Temperature in 10*e-1.
         */
        uint16_t temperature = 0;

        /**
         * @brief Current in milli ampere.
         */
        int32_t current = 0;

        /**
         * @brief Percent of the battery.
         */
        uint8_t percent = 0;

        /**
         * @brief Unknown.
         */
        uint8_t recv = 0;
    };

    /**
     * @brief Struct for the velocity data from the RoboMaster.
     */
    struct StateVelocity {
        /**
         * @brief True when the velocity data are successfully parsed.
         */
        bool has_data = false;

        /**
         * @brief Velocity m/s on the x-axis in the global coordinate system where the RoboMaster is turned on.
         */
        float vgx = 0.0f;

        /**
         * @brief Velocity m/s on the y-axis in the global coordinate system where the RoboMaster is turned on.
         */
        float vgy = 0.0f;

        /**
         * @brief Velocity m/s on the z axis in the global coordinate system where the RoboMaster is turned on.
         */
        float vgz = 0.0f;

        /**
         * @brief Velocity m/s on the x-axis in local coordinate system.
         */
        float vbx = 0.0f;

        /**
         * @brief Velocity m/s on the y-axis in local coordinate system.
         */
        float vby = 0.0f;

        /**
         * @brief Velocity m/s on the z axis in local coordinate system.
         */
        float vbz = 0.0f;
    };

    /**
     * @brief Struct for the position data from the RoboMaster.
     */
    struct StatePosition {
        /**
         * @brief True when the position data are successfully parsed.
         */
        bool has_data = false;

        /**
         * @brief X position on the x-axis in the global coordinate system where the RoboMaster is turned on.
         */
        float x = 0.0f;

        /**
         * @brief Y position on the x-axis in the global coordinate system where the RoboMaster is turned on.
         */
        float y = 0.0f;

        /**
         * @brief Rotation angle in the global coordinate system where the RoboMaster is turned on.
         */
        float z = 0.0f;
    };

    /**
     * @brief Collection of all data struct from the RoboMaster.
     */
    struct RoboMasterState {
        /**
         * @brief Battery data.
         */
        StateBattery battery;

        /**
         * @brief Esc data.
         */
        StateESC esc;

        /**
         * @brief Imu data.
         */
        StateIMU imu;

        /**
         * @brief Velocity data.
         */
        StateVelocity velocity;

        /**
         * @brief Position data.
         */
        StatePosition position;

        /**
         * @brief Attitude data.
         */
        StateAttitude attitude;
    };

    /**
     * @brief Decode the message payload at the given index for esc data.
     *
     * @param index Index for the payload.
     * @param msg Message from the motion controller.
     * @return struct DataEsc. has_data is true, by successful decoding.
     */
    StateESC decode_data_esc(size_t index, const Message &msg);

    /**
     * @brief Decode the message payload at the given index for imu data.
     *
     * @param index Index for the payload.
     * @param msg Message from the motion controller.
     * @return struct DataImu. has_data is true, by successful decoding.
     */
    StateIMU decode_data_imu(size_t index, const Message &msg);

    /**
     * @brief Decode the message payload at the given index for imu data.
     *
     * @param index Index for the payload.
     * @param msg Message from the motion controller.
     * @return struct DataAttitude. has_data is true, by successful decoding.
     */
    StateAttitude decode_data_attitude(size_t index, const Message &msg);

    /**
     * @brief Decode the message payload at the given index for battery data.
     *
     * @param index Index for the payload.
     * @param msg Message from the motion controller.
     * @return struct DataBattery. has_data is true, by successful decoding.
     */
    StateBattery decode_data_battery(size_t index, const Message &msg);

    /**
     * @brief Decode the message payload at the given index for velocity data.
     *
     * @param index Index for the payload.
     * @param msg Message from the motion controller.
     * @return struct DataVelocity. has_data is true, by successful decoding.
     */
    StateVelocity decode_data_velocity(size_t index, const Message &msg);

    /**
     * @brief Decode the message payload at the given index for position data.
     *
     * @param index Index for the payload.
     * @param msg Message from the motion controller.
     * @return struct DataPosition. has_data is true, by successful decoding.
     */
    StatePosition decode_data_position(size_t index, const Message &msg);
} // namespace robomaster

#endif // ROBOMASTER_DATA_H_