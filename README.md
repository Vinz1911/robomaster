# Robomaster

**RoboMaster** is a C++ library designed to control the DJI RoboMaster via a direct CAN Bus interface, allowing users to use the RoboMaster chassis as a mobile robotics platform without the Intelli Controller.
By communicating directly over CAN Bus, the need for Wi-Fi, USB, or other intermediary connections is eliminated, reducing latency and complexity.
The library provides a simple C++ API and requires a computer with a CAN Bus interface, such as an NVIDIA Jetson board or a Raspberry Pi with a CAN Bus module.
Additionally, the 12V power supply from the CAN Bus can be used as a power source for the RoboMaster.

**Caution:** it is recommended to send commands only every 10–20 milliseconds to not overflow the CAN Bus, otherwise the RoboMaster can behave unintentionally.

## Original Library
This library was original provided by `Fraunhofer IML130` and can be found here: [Robomaster Can Controller](https://github.com/iml130/robomaster_can_controller).
This repo contains a modified version for my personal needs. I've added missing features like Gimbal and Blaster control.

## C++ Version:
[![C++23](https://img.shields.io/badge/C++-23-blue.svg?logo=c%2B%2B&style=flat)](https://isocpp.org)

## Build and Install
Build the library and use the given example to test the library on the RoboMaster.

```sh
cd robomaster
mkdir build
cd build
cmake ..
make && sudo make install
```

Run the example in the build directory.
Give the RoboMaster enough space to drive or put the Robomaster on a box that the wheels don't touch the ground.

```sh
./robomaster_demo
```

## Class RoboMaster
The class RoboMaster provides simple access to control the chassis, the gimbal, the blaster and the LEDs.

| Method                                                                                                                         | Description                                                                                                                            |
|--------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------|
| `bool init(std::string& interface)`                                                                                            | Initialize the RoboMaster by opening the CAN bus by the given can_interface and set CAN receiving. Return true on success.             |
| `bool is_running()`                                                                                                            | Return true when the RoboMaster is successfully initialized and running. Switch to false when an error occurs.                         |
| `RoboMasterState get_state()`                                                                                                  | Return the current `RoboMasterState` this is frequently updated.                                                                       |
| `void set_chassis_mode(ChassisMode mode)`                                                                                      | Set the RoboMaster's `ChassisMode` (`Enable` or `Disable`.                                                                             |
| `void set_chassis_rpm(int16_t front_right, int16_t front_left, int16_t rear_left, int16_t rear_right)`                         | Set the wheel speed in rpm in the wheel order front right, front left, rear left and rear right [-1000, 1000].                         |
| `void set_chassis_velocity(float pitch, float yaw, float roll)`                                                                | Set the pitch, yaw and angular roll velocity. The velocity and acceleration limits are handled by the config of the motion controller. |
| `void set_chassis_position(int16_t linear_x, int16_t linear_y, int16_t angular_z)`                                             | Set the chassis position for linear x, linear y [-500, 500] and angular z [-18000, 18000].                                             |
| `void set_gimbal_mode(GimbalMode mode)`                                                                                        | Set the RoboMaster's `GimbalMode` (`Free` or `Follow`).                                                                                |
| `void set_gimbal_hibernate(GimbalState state)`                                                                                 | Set the `GimbalHibernate` (`Suspend` or `Resume`).                                                                                     |
| `void set_gimbal_motion(int16_t pitch, int16_t yaw)`                                                                           | Set the pitch and yaw degree's for the gimbal [-1000, 1000].                                                                           |
| `void set_gimbal_velocity(int16_t pitch, int16_t yaw)`                                                                         | Set the yaw and roll velocity for the gimbal [-1000, 1000].                                                                            |
| `void set_gimbal_position(int16_t pitch, int16_t yaw, uint16_t pitch_acceleration, uint16_t yaw_acceleration)`                 | Set the gimbal position for pitch [-500, 500] and yaw [-2500, 2500].                                                                   | 
| `void set_gimbal_recenter(int16_t pitch, int16_t yaw)`                                                                         | Set the gimbal back to center position [10, 500].                                                                                      |
| `void set_blaster_mode(BlasterMode mode, uint8_t count)`                                                                       | Set the `BlasterMode` and fire the blaster [1, 8] times based on the given count.                                                      |
| `void set_led_mode(LEDMode mode, LEDMask mask, uint8_t red, uint8_t green, uint8_t blue, uint16_t up_time, uint16_t down_time` | Set the `LEDMode` and `LEDMask`, the color for Red, Green and Blue and the up and down time for some modes.                            |

## Struct RoboMasterState
Information of the state of the RoboMaster Chassis.

| struct (C++)                | Description                                                                     |
|-----------------------------|---------------------------------------------------------------------------------|
| `StateBattery battery`      | Contains the state of the RoboMaster battery.                                   |
| `StateESC esc`              | Contains the states of the wheels.                                              |
| `StateIMU imu`              | Contains the sensor data of the IMU in the motion controller.                   |
| `StateVelocity velocity`    | Contains the estimated velocities of the RoboMaster from the motion controller. |
| `StatePosition position`    | Contains the odometry data of the motion controller.                            |
| `StateAttitude attitude`    | Contains the attitude of the RoboMaster estimated from the motion controller.   |
| `StateGimbal gimbal`        | Contains the state of the RoboMaster gimbal.                                    |
| `StateDetector detector[4]` | Contains the states of the RoboMaster hit detector's.                           |

## Struct StateBattery
Information of the RoboMaster battery.

| Name          | datatype   | Description                              |
|---------------|------------|------------------------------------------|
| `adc`         | `uint16_t` | ADC value of the battery in milli volts. |
| `temperature` | `int16_t`  | Temperature in 10\*e-1                   |
| `current`     | `int32_t`  | Current in milli amperes.                |
| `percent`     | `uint8_t`  | Percent of the battery [0, 100].         |
| `recv`        | `uint8_t`  | N/A                                      |

## Struct StateESC
Information of the state of the wheels. The wheel order in the arrays is front left, front right, rear left and rear right

| Name    | datatype      | Description                                             |
|---------|---------------|---------------------------------------------------------|
| `speed` | `int16_t[4]`  | Speed in RPM -> value range -8192~8191.                 |
| `angle` | `int16_t[4]`  | Angle position -> value range 0-32767 maps to -> 0-360. |
| `speed` | `uint32_t[4]` | Timestamp -> units N/A                                  |
| `speed` | `uint8_t[4]`  | State of the ESC -> units N/A                           |

## Struct StateIMU
Information of the measured sensor data from the IMU in the Motion Controller.

| Name     | datatype | Description                            |
|----------|----------|----------------------------------------|
| `acc_x`  | `float`  | Acceleration on x axis in 9.81 /m^2 s. |
| `acc_y`  | `float`  | Acceleration on y axis in 9.81 /m^2 s. |
| `acc_z`  | `float`  | Acceleration on x axis in 9.81 /m^2 s. |
| `gyro_x` | `float`  | Angular velocity on x axis in radiant. |
| `gyro_y` | `float`  | Angular velocity on y axis in radiant. |
| `gyro_z` | `float`  | Angular velocity on z axis in radiant. |

## Struct StateVelocity
Information of the chassis velocities which are measured from the Motion Controller.

| Name  | datatype | Description                                                                                                  |
|-------|----------|--------------------------------------------------------------------------------------------------------------|
| `vgx` | `float`  | Velocity m/s on the x axis in the global coordinate system where the RoboMaster is turned on.                |
| `vgy` | `float`  | Velocity m/s on the y axis in the global coordinate system where the RoboMaster is turned on.                |
| `vgz` | `float`  | Velocity m/s on the z axis in the global coordinate system where the RoboMaster is turned on. Is always 0.0. |
| `vbx` | `float`  | Velocity m/s on the x axis in local coordinate system.                                                       |
| `vby` | `float`  | Velocity m/s on the y axis in local coordinate system.                                                       |
| `vbz` | `float`  | Velocity m/s on the z axis in local coordinate system.. Is always 0.0.                                       |

## Struct StatePosition
Information of the measured position of the RoboMaster since the RoboMaster is powered on.

| Name    | datatype | Description                                                                                                |
|---------|----------|------------------------------------------------------------------------------------------------------------|
| `pos_x` | `float`  | X position on the x axis in the global coordinate system where the RoboMaster is turned on.                |
| `pos_y` | `float`  | Y position on the x axis in the global coordinate system where the RoboMaster is turned on.                |
| `pos_z` | `float`  | Z position on the x axis in the global coordinate system where the RoboMaster is turned on. Is always 0.0. |

## Struct StateAttitude
Information of the Attitude which is measured by the Motion Controller.

| Name    | datatype | Description      |
|---------|----------|------------------|
| `roll`  | `float`  | Roll in degree   |
| `pitch` | `float`  | Pitch in degree. |
| `yaw`   | `float`  | Yaw in degree.   |

## Struct StateGimbal
Information of the Attitude which is measured by the Gimbal.

| Name    | datatype  | Description      |
|---------|-----------|------------------|
| `pitch` | `int16_t` | Pitch in degree. |
| `yaw`   | `int16_t` | Yaw in degree.   |

## Struct StateDetector
Information of the last Hit which is measured by the Hit detector.

| Name        | datatype     | Description                  |
|-------------|--------------|------------------------------|
| `hit_time`  | `time_point` | Timestamp of the latest hit. |
| `intensity` | `uint16_t`   | Intensity of the latest hit. |