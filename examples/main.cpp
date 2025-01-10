// Copyright (c) 2023 Fraunhofer IML, 2024 Vinzenz Weist
//
// This project contains contributions from multiple authors.
// The original code is licensed under the MIT License by Fraunhofer IML.
// All modifications and additional code are licensed under the MIT License by Vinzenz Weist.

#include <iostream>
#include <robomaster/robomaster.h>
#include <robomaster/definitions.h>

/**
 * Example for the usage of the robomaster_can_controller library.
 */
int main() {
    // Using namespace for simplicity
    using namespace robomaster;

    // Create the robomaster interface class.
    RoboMaster robomaster;

    // Try to init
    if (!robomaster.init("can0")) { std::printf("[Example]: robomaster initialization failed"); return 1; }

    // Enable the robomaster to execute drive commands.
    robomaster.set_torque(true);

    // CAUTION: Sleep for a short period to not overfill the can bus communication.
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // Breath animation
    robomaster.set_led(BREATHE, LED_MASK_BOTTOM_FRONT, 128, 0, 255, 500, 500);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // Let the robomaster drive forward with increasing wheel speed and increase set led brightness.
    for (size_t i = 0; i < 100; i++) {
        robomaster.set_led(STATIC, LED_MASK_ALL, i * 2, i * 2, i * 2);
        robomaster.set_wheel_rpm(static_cast<int16_t>(i * 2), static_cast<int16_t>(i * 2), static_cast<int16_t>(i * 2), static_cast<int16_t>(i * 2));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Slow the robomaster and decrease the LED light.
    for (size_t i = 100; i --> 0;) {
        robomaster.set_led(STATIC, LED_MASK_ALL, i * 2, i * 2, i * 2);
        robomaster.set_wheel_rpm(static_cast<int16_t>(i * 2), static_cast<int16_t>(i * 2), static_cast<int16_t>(i * 2), static_cast<int16_t>(i * 2));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Stop the wheel of the robomaster.
    robomaster.set_brake();

    // Use the LED Flash of all LED.
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Turn of the LED.
    robomaster.set_led(STATIC, LED_MASK_ALL, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Disable the robomaster after finish the example.
    robomaster.set_torque(false); return 0;
}
