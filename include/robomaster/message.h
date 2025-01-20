// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#ifndef ROBOMASTER_MESSAGE_H_
#define ROBOMASTER_MESSAGE_H_

#include <vector>
#include <cstdint>
#include <cstddef>

namespace robomaster {
    /**
     * @brief This class defined a RoboMaster message. The information values in the messages are saved in little endian.
     */
    class Message {
        /**
         * @brief Flag for a valid message. This includes when the message is long enough with header and the right crc.
         */
        bool is_valid_;

        /**
         * @brief The can device id for this message.
         */
        uint32_t device_id_;

        /**
         * @brief The sequence or counter for the message.
         */
        uint16_t sequence_;

        /**
         * @brief The type of the message. Every message from the intelli controller or motion controller has a fixed value which seems to be a type or command id.
         */
        uint16_t type_;

        /**
         * @brief The payload of the message which contains the information.
         */
        std::vector<uint8_t> payload_;

    public:
        /**
         * @brief Construct a new Message object from the given raw data.
         *
         * @param device_id The can device id.
         * @param msg_data The raw data for example can bus to parse into a RoboMaster message.
         */
        Message(uint32_t device_id, const std::vector<uint8_t>& msg_data);

        /**
         * @brief Construct a new Message object.
         *
         * @param device_id The can device id.
         * @param type The type of the message.
         * @param sequence The current sequence.
         * @param payload The payload for the information.
         */
        Message(uint32_t device_id, uint16_t type, uint16_t sequence, std::vector<uint8_t> payload=std::vector<uint8_t>());

        /**
         * @brief Destructor of the Message class.
         */

        ~Message() = default;

        /**
         * @brief Increment the sequence by one.
         */
        void increment_sequence();

        /**
         * @brief Returns true for a valid message, when message length and crc is correct.
         *
         * @return true
         * @return false
         */
        [[nodiscard]] bool is_valid() const;

        /**
         * @brief Get the can device id.
         *
         * @return uint32_t as device id.
         */
        [[nodiscard]] uint32_t get_device_id() const;

        /**
         * @brief Get the sequence.
         *
         * @return uint16_t as sequence.
         */
        [[nodiscard]] uint16_t get_sequence() const;

        /**
         * @brief Get the message type.
         *
         * @return uint16_t as type.
         */
        [[nodiscard]] uint16_t get_type() const;

        /**
         * @brief Get the payload from the message.
         *
         * @return std::vector<uint8_t> as payload.
         */
        [[nodiscard]] std::vector<uint8_t> get_payload() const;

        /**
         * @brief Get the complete length from the message including header, crc and payload.
         *
         * @return size_t as length.
         */
        [[nodiscard]] size_t get_length() const;

        /**
         * @brief Set the payload.
         *
         * @param payload The payload.
         */
        void set_payload(const std::vector<uint8_t>& payload);

        /**
         * @brief Set the type.
         *
         * @param type the type data.
         */
        void set_type(uint16_t type);

        /**
         * @brief Set the uint8 value into the payload at given index.
         *
         * @param index The index for the payload position.
         * @param value The value to be set.
         */
        void set_value_uint8(size_t index, uint8_t value);

        /**
         * @brief Set the int8 value into the payload at given index.
         *
         * @param index The index for the payload position.
         * @param value The value to be set.
         */
        void set_value_int8(size_t index, int8_t value);

        /**
         * @brief Set the uint16 value into the payload at given index.
         *
         * @param index The index for the payload position.
         * @param value The value to be set.
         */
        void set_value_uint16(size_t index, uint16_t value);

        /**
         * @brief Set the int16 value into the payload at given index.
         *
         * @param index The index for the payload position.
         * @param value The value to be set.
         */
        void set_value_int16(size_t index, int16_t value);

        /**
         * @brief Set the uint32 value into the payload at given index.
         *
         * @param index The index for the payload position.
         * @param value The value to be set.
         */
        void set_value_uint32(size_t index, uint32_t value);

        /**
         * @brief Set the int32 value into the payload at given index.
         *
         * @param index The index for the payload position.
         * @param value The value to be set.
         */
        void set_value_int32(size_t index, int32_t value);

        /**
         * @brief Set the float value into the payload at given index.
         *
         * @param index The index for the payload position.
         * @param value The value to be set.
         */
        void set_value_float(size_t index, float value);

        /**
         * @brief Get the uint8 value form the palyoad at given index.
         *
         * @param index The index for the payload position.
         * @return uint8_t as value.
         */
        [[nodiscard]] uint8_t get_value_uint8(size_t index) const;

        /**
         * @brief Get the int8 value form the palyoad at given index.
         *
         * @param index The index for the payload position.
         * @return int8_t as value.
         */
        [[nodiscard]] int8_t get_value_int8(size_t index) const;

        /**
         * @brief Get the uint16 value form the palyoad at given index.
         *
         * @param index The index for the payload position.
         * @return uint16_t as value.
         */
        [[nodiscard]] uint16_t get_value_uint16(size_t index) const;

        /**
         * @brief Get the int16 value form the palyoad at given index.
         *
         * @param index The index for the payload position.
         * @return uint16_t as value.
         */
        [[nodiscard]] int16_t get_value_int16(size_t index) const;

        /**
         * @brief Get the uint32 value form the palyoad at given index.
         *
         * @param index The index for the payload position.
         * @return uint32_t as value.
         */
        [[nodiscard]] uint32_t get_value_uint32(size_t index) const;

        /**
         * @brief Get the int32 value form the palyoad at given index.
         *
         * @param index The index for the payload position.
         * @return int32_t as value.
         */
        [[nodiscard]] int32_t get_value_int32(size_t index) const;

        /**
         * @brief Get the float value form the payload at given index.
         *
         * @param index The index for the payload position.
         * @return float as value.
         */
        [[nodiscard]] float get_value_float(size_t index) const;

        /**
         * @brief Create a vector as raw data from the message including header, crc and payload.
         *
         * @return std::vector<uint8_t> as raw data message.
         */
        [[nodiscard]] std::vector<uint8_t> to_vector() const;
    };
} // namespace robomaster

#endif // ROBOMASTER_MESSAGE_H_