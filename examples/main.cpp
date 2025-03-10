/*
 * MIT License
 *
 * Copyright (c) 2024 Vinzenz Weist
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

#include <iostream>
#include <robomaster/robomaster.h>
#include <robomaster/definitions.h>

void state_data(const robomaster::RoboMaster& robomaster) {
    while (robomaster.is_running()) {
        if (const auto state = robomaster.get_state(); state.is_active) {
            std::printf("Battery: %u\n", state.battery.percent);
            std::printf("Pitch: %i, Yaw: %i\n", state.gimbal.pitch, state.gimbal.yaw);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main() {
    // Using namespace for simplicity
    using namespace robomaster;

    // Create the robomaster interface class.
    auto robomaster = RoboMaster();

    // Try to init
    if (!robomaster.init()) { std::printf("[Example]: robomaster initialization failed\n"); return 1; }
    std::thread state_thread(state_data, std::ref(robomaster)); state_thread.detach();

    // Enable the robomaster to execute drive commands.
    robomaster.set_chassis_mode(CHASSIS_MODE_ENABLE);

    // CAUTION: Sleep for a short period to not overfill the can bus communication.
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // Breath animation and recenter gimbal.
    robomaster.set_led_mode(LED_MODE_BREATHE, LED_MASK_ALL, 128, 0, 255, 500, 500);
    robomaster.set_gimbal_recenter(150, 150);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // Set gimbal to fixed degree, will hold his position.
    robomaster.set_gimbal_velocity(0, 0);

    // Let the robomaster drive forward with increasing wheel speed and increase set led brightness.
    for (size_t i = 0; i < 50; i++) {
        robomaster.set_led_mode(LED_MODE_STATIC, LED_MASK_ALL, i * 2, i * 2, i * 2);
        robomaster.set_chassis_rpm(static_cast<int16_t>(i * 2), static_cast<int16_t>(i * -2), static_cast<int16_t>(i * -2), static_cast<int16_t>(i * 2));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Slow the robomaster and decrease the LED light.
    for (size_t i = 50; i --> 0;) {
        robomaster.set_led_mode(LED_MODE_STATIC, LED_MASK_ALL, i * 2, i * 2, i * 2);
        robomaster.set_chassis_rpm(static_cast<int16_t>(i * 2), static_cast<int16_t>(i * -2), static_cast<int16_t>(i * -2), static_cast<int16_t>(i * 2));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Stop the wheel of the robomaster.
    robomaster.set_chassis_rpm(0, 0, 0, 0);

    // Move gimbal to position
    robomaster.set_gimbal_position(250, 1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Turn the LED back to static.
    robomaster.set_led_mode(LED_MODE_STATIC, LED_MASK_ALL, 128, 0, 255);

    // Fire Blaster
    robomaster.set_blaster_mode(BLASTER_MODE_GEL, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Recenter the gimbal
    robomaster.set_gimbal_recenter(150, 150);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // Disable the robomaster after finish the example.
    robomaster.set_chassis_mode(CHASSIS_MODE_DISABLE); return 0;
}
