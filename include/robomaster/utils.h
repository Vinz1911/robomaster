// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#ifndef ROBOMASTER_UTILS_H_
#define ROBOMASTER_UTILS_H_

#include <algorithm>
#include <sstream>
#include <vector>

namespace robomaster {
    /**
     * @brief Clip the value in the given range from min to max.
     *
     * @tparam T Template for the specific data type.
     * @param value Value to clip.
     * @param min Min value.
     * @param max Max value.
     * @return T Clipped value.
     */
    template <typename T>
    T clip(T value, T min, T max) {
        return std::min(std::max(min, value), max);
    }

    /**
     * @brief Calculated the crc8 for the given data.
     *
     * @param data Data for the crc8 calculation.
     * @param length Length of the data.
     * @return uint8_t Crc8 value.
     */
    uint8_t calculate_crc8(const uint8_t *data, size_t length);

    /**
     * @brief Calculated the crc16 for the given data.
     *
     * @param data Data for the crc8 calculation.
     * @param length Length of the data.
     * @return uint8_t Crc16 value.
     */
    uint16_t calculate_crc16(const uint8_t *data, size_t length);


    /**
     * @brief Give the given uint8 data array in hex as string back.
     *
     * @param data Input data.
     * @param length Length of the data.
     * @return std::string Data as string visualization.
     */
    std::string string_to_hex(const uint8_t * data, size_t length);

    /**
     * @brief
     *
     * @param data
     * @return std::string
     */
    std::string string_to_hex(const std::vector<uint8_t> &data);

    /**
     * @brief Put the two bytes from little endian in the right host platform order.
     *
     * @param lsb Least significant bit.
     * @param msb Most significant bit.
     * @return uint16_t The uint16_t value.
     */
    uint16_t little_endian_to_uint16(uint8_t lsb, uint8_t msb);
} // namespace robomaster

#endif // ROBOMASTER_UTILS_H_