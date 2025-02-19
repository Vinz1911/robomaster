/*
 * Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
 *
 * This project contains contributions from multiple authors.
 * The original code is licensed under the MIT License by Fraunhofer IML.
 * All modifications and additional code are licensed under the MIT License by Vinzenz Weist.
 */

#include "robomaster/queue.h"
#include "robomaster/definitions.h"
#include "gtest/gtest.h"

namespace robomaster {
    TEST(QueueTest, PushAndPop) {
        Queue queue;

        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, 0, std::vector{static_cast<uint8_t>(10)}));
        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, 1, std::vector{static_cast<uint8_t>(11)}));
        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, 2, std::vector{static_cast<uint8_t>(12)}));

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

    TEST(QueueTest, Overflow) {
        Queue queue;

        for (size_t i = 0; i < queue.max_queue_size() + 1; i++) { queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, i, std::vector{static_cast<uint8_t>(i)})); }

        Message m = queue.pop();

        ASSERT_EQ(m.get_sequence(), 1);
        ASSERT_EQ(m.get_payload()[0], 1);
        ASSERT_TRUE(m.is_valid());

        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, queue.max_queue_size() + 1, std::vector{static_cast<uint8_t>(queue.max_queue_size() + 1)}));

        while (!queue.empty()) { m = queue.pop(); }

        ASSERT_EQ(m.get_sequence(), queue.max_queue_size() + 1);
        ASSERT_EQ(m.get_payload()[0], queue.max_queue_size() + 1);
        ASSERT_TRUE(m.is_valid());   
    }

    TEST(QueueTest, Clear) {
        Queue queue;

        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, 0, std::vector{static_cast<uint8_t>(10)}));
        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, 1, std::vector{static_cast<uint8_t>(11)}));
        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, 2, std::vector{static_cast<uint8_t>(12)}));

        ASSERT_EQ(queue.size(), 3);
        ASSERT_FALSE(queue.empty());

        queue.clear();

        ASSERT_EQ(queue.size(), 0);
        ASSERT_TRUE(queue.empty());
    }
} // namespace robomaster
