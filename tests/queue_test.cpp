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

#include "robomaster/queue.h"
#include "gtest/gtest.h"

namespace robomaster {
    TEST(QueueTest, PushAndPop) {
        Queue queue;

        queue.push(Message(0x202, 1337, 0, std::vector{static_cast<uint8_t>(10)}));
        queue.push(Message(0x202, 1337, 1, std::vector{static_cast<uint8_t>(11)}));
        queue.push(Message(0x202, 1337, 2, std::vector{static_cast<uint8_t>(12)}));

        Message m = queue.pop();
        ASSERT_EQ(m.get_sequence(), 0);
        ASSERT_EQ(m.get_payload()[0], 10);
        ASSERT_TRUE(m.is_valid());

        m = queue.pop();
        ASSERT_EQ(m.get_sequence(), 1);
        ASSERT_EQ(m.get_payload()[0], 11);
        ASSERT_TRUE(m.is_valid());

        m = queue.pop();
        ASSERT_EQ(m.get_sequence(), 2);
        ASSERT_EQ(m.get_payload()[0], 12);
        ASSERT_TRUE(m.is_valid());

        m = queue.pop();
        ASSERT_EQ(m.get_sequence(), 0);
        ASSERT_EQ(m.get_payload().size(), 0);
        ASSERT_FALSE(m.is_valid());
    }

    TEST(QueueTest, Clear) {
        Queue queue;

        queue.push(Message(0x202, 1337, 0, std::vector{static_cast<uint8_t>(10)}));
        queue.push(Message(0x202, 1337, 1, std::vector{static_cast<uint8_t>(11)}));
        queue.push(Message(0x202, 1337, 2, std::vector{static_cast<uint8_t>(12)}));

        ASSERT_EQ(queue.size(), 3);
        ASSERT_FALSE(queue.empty());

        queue.clear();

        ASSERT_EQ(queue.size(), 0);
        ASSERT_TRUE(queue.empty());
    }
} // namespace robomaster
