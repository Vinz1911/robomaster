# Robomaster

**Robomaster** is a library to control the RoboMaster via CAN Bus interface and is for everybody who want to use the RoboMaster's chassis as mobile robot plattform. 
The library provides a simple C++ interface which is also accessible via python3 through pybind11 bindings.
To use the library you need a computer with a CAN bus interface like the NVIDIA jetson boards or a raspberry pi with a CAN Bus module as extension.
One advantage to communicated directly over the CAN Bus is that the intelli controller is no longer needed.
This eliminates the detours via Wi-Fi, usb, etc. to the intelli controller back to the motion controller.
Also, the 12 volt power supply from CAN bus can be used as a power source.
In the current state the library is limited to controlling the RoboMaster chassis and the LEDs.

**Caution by using this library**:
The user is free to decide when to send a command and the library can handle multiple commands by using a small queue.
Nevertheless, the CAN Bus is limited by its bandwidth.
To prevent the CAN Bus from overfilling with data it is recommended to send only every ~10-20 milliseconds a command.
otherwise commands will be ignored and the RoboMaster could drive unintentional.

## Original Authors
This library was original provided by `Fraunhofer IML130` and can be found here: [Robomaster Can Controller](https://github.com/iml130/robomaster_can_controller).
It was modified by me for my personal needs.

## Usage C++

Build the library and use the given example to test the library on the RoboMaster.

```sh
cd robomaster
mkdir build
cd build
cmake ..
make && make install
```

Run the example in the build directory. 
Give the RoboMaster enough space to drive or put the Robomaster on a box that the wheels don't touch the ground.

```sh
./robomaster_demo
```

Add the **robomaster_can_controller** to your project as submodule to used it with **C++**.

## Class RoboMaster C++

The class RoboMaster provides simple access to control the chassis and the LEDs.

| Method | Description                                                                                                                                                                       |
| --|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `bool init(const std::string &can_interface="can0")` | Initialize the RoboMaster by opening the CAN bus by the given can_interface. Return true by success.                                                                              |
| `bool is_running() const` | Return true when the RoboMaster is successfully initialized and running. Switch to false when an error occurs.                                                                    |
| `void set_callback(std::function< void(const DataRoboMasterState&)> func)` | Register a callback function that returns the states of the RoboMaster at a rate of 50 Hertz.                                                                                     |
| `void set_work_mode(bool mode)` | Enable/Disable the RoboMaster and the motors are supplied with power.                                                                                                             |
| `void set_brake()` | Stop immediately the wheels.                                                                                                                                                      | 
| `void set_wheel_rpm(const int16_t fr, const int16_t fl, const int16_t rl, const int16_t rr)` | Set the wheel speed in rpm in the wheel order front right, front left, rear left and rear right [-1000, 1000].                                                                    |
| `void set_velocity(const float x, const float y, const float z)` | Set the linear x, linear y and angular as z velocity. The velocity and acceleration limits are handled by the config of the motion controller.                                    |
| `void set_led_off(const uint16_t mask)` | Turn off the LEDs with the seleceted mask. Mask flags can be concatenated and represented as enums: LED_MASK_ALL, LED_MASK_FRONT, LED_MASK_RIGHT, LED_MASK_LEFT and LED_MASK_BACK |
| `void set_led_on(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b)` | Turn on the LEDs by the selected mask and the given rgb values [0, 255].                                                                                                          |
| `void set_led_breath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const uint16_t t_rise, const uint16_t t_down)` | Make a breathing effect on the LED by the selected mask and given rgb values. The LEDs rise and down phase are set in milliseconds  [0, 60000].                                   | 
| `void set_led_breath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float t_rise, const float t_down)` | Make a breathing effect on the LED by the selected mask and given rgb values. The LEDs rise and down phase are set in seconds [0.0, 60.0].                                        |
| `void set_led_breath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float rate)` | Make a breathing effect on the LED by the selected mask and given rgb values. The rate is set in seconds [0.0, 60.0].                                                             |
| `void set_led_flash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const uint16_t t_on, const uint16_t t_off)` | Let the LED make a flash effect by the selected mask and given rgb values. The LEDs on and off phase are set in milliseconds [0, 60000].                                          |
| `void set_led_flash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float t_on, const float t_off)` | Let the LED make a flash effect by the selected mask and given rgb values. The LEDs on and off phase are set in seconds [0.0, 60.0].                                              |
| `void set_led_flash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float rate)` | Let the LED make a flash effect by the selected mask and given rgb values. The rate is set in seconds [0.0, 60.0].                                                                |

## Struct DataRoboMasterState

The struct DataRoboMasterState represents the state of the RoboMaster which passed by the registered callback function.

| struct (C++) | class (Python) | Description |
| ------------ | -------------- | ----------- |
| `struct DataBattery battery` | `Battery`| Contains the state of the RoboMastern battery. |
| `struct DataEsc esc` | `Esc`| Contains the states of the wheels. |
| `struct DataImu imu` | `Imu`| Contains the sensor data of the imu in the motion controller. |
| `struct DataImu velocity` | `Velocity`| Contains the estimated velocities of the RoboMaster from the motion controller. |
| `struct DataPosition position` | `Position`| Contains the odometry data of the motion controller. |
| `struct DataAttitude attitude` | `Attitude`| Contains the attitude of the RoboMaster estimated from the motion controller. |

## Struct DataBattery

Information of the RoboMaster battery.

| Name | datatype | Description |
| ---- | -------- | ----------- |
| `adc_value` | `uint16_t` | ADC value of the battery in milli volts. |
| `temperature` | `int16_t` | Temperature in 10*e-1 |
| `current` | `int32_t` | Current in milli amperes. |
| `percent` | `uint8_t` | Percent of the battery [0, 100]. |
| `recv` | `uint8_t` | N/A |

## Struct DataEsc

This struct presents the state of the wheels. The wheel order in the arrays is front left, front right, rear left and rear right

| Name | datatype | Description |
| ---- | -------- | ---------- |
| `speed` | `int16_t[4]` | Speed in RPM -> value range -8192~8191. |
| `angle` | `int16_t[4]` | Angle position -> value range 0-32767 maps to -> 0-360. |
| `speed` | `uint32_t[4]` | Timestamp -> units N/A |
| `speed` | `uint8_t[4]` | State of the ESC -> units N/A |

## Struct DataImu

The measured sensor data from the Imu in the motion controller.

| Name | datatype | Description |
| ---- | -------- | ----------- |
| `acc_x` | `float` | Acceleration on x axis in 9.81 /m^2 s. |
| `acc_y` | `float` | Acceleration on y axis in 9.81 /m^2 s. | 
| `acc_z` | `float` | Acceleration on x axis in 9.81 /m^2 s. |
| `gyro_x` | `float` | Angular velocity on x axis in radiant. |
| `gyro_y` | `float` | Angular velocity on y axis in radiant. |
| `gyro_z` | `float` | Angular velocity on z axis in radiant. |

## Struct DataVelocity

Information of the chassis velocites which are measured from the motion controller.

| Name | datatype | Description |
| ---- | -------- | ----------- |
| `vgx` | `float` | Velocity m/s on the x axis in the global coordinate system where the RoboMaster is turned on. |
| `vgy` | `float` | Velocity m/s on the y axis in the global coordinate system where the RoboMaster is turned on. |
| `vgz` | `float` | Velocity m/s on the z axis in the global coordinate system where the RoboMaster is turned on. Is always 0.0. |
| `vbx` | `float` | Velocity m/s on the x axis in local coordinate system. |
| `vby` | `float` | Velocity m/s on the y axis in local coordinate system. |
| `vbz` | `float` | Velocity m/s on the z axis in local coordinate system.. Is always 0.0. |

## Struct DataPosition

This struct presents the measured position of the RoboMaster since the RoboMaster is powered on.

| Name | datatype | Description |
| ---- | -------- | ----------- |
| `x` | `float` | X position on the x axis in the global coordinate system where the RoboMaster is turned on. |
| `y` | `float` | Y position on the x axis in the global coordinate system where the RoboMaster is turned on. |
| `z` | `float` | Z position on the x axis in the global coordinate system where the RoboMaster is turned on. Is always 0.0. |

## Struct DataAttitude

Information of the Attitude which is measured by the motion controller.

| Name | datatype | Description |
| ---- | -------- | ----------- |
| `roll` | `float` | Roll in degree |
| `pitch` | `float` | Pitch in degree. |
| `yaw` | `float` | Yaw in degree. |
