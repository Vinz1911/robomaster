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

namespace robomaster {
    /**
     * @brief Calculated the CRC8 for the given data.
     *
     * @param data Data for the CRC8 calculation.
     * @param length Length of the data.
     * @return uint8_t CRC8 value.
     */
    uint8_t get_crc8(const uint8_t* data, size_t length);

    /**
     * @brief Calculated the CRC16 for the given data.
     *
     * @param data Data for the CRC16 calculation.
     * @param length Length of the data.
     * @return uint8_t CRC16 value.
     */
    uint16_t get_crc16(const uint8_t* data, size_t length);

    /**
     * @brief Put the two bytes from little endian in the right host platform order.
     *
     * @param ls_byte Least significant bit.
     * @param ms_byte Most significant bit.
     * @return uint16_t The uint16_t value.
     */
    uint16_t get_little_endian(uint8_t ls_byte, uint8_t ms_byte);
} // namespace robomaster