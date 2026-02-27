# REV Hub ROS2 Driver
this is an ROS Driver that uses [librhsp](https://github.com/REVrobotics/node-rhsplib/tree/main/packages/rhsplib/librhsp)

the library was obtained by studying the Node-JS implementation of the Rev Hub Serial Protocol (RHSP). 
turns out, there is a C library that does the actual communication. (librhsp)

all the code inside the librhsp folder belongs to REV and have their respective authors and licences.

## Usage

All package functionalities can be started via the main launch file:

```bash
ros2 launch work_expansion_hub_driver expansion.launch.py
```

## Odometry

The driver includes a Mecanum Odometry node `encoder_odometry.py` that processes encoder data to estimate the robot's position (x,y) and orientation (θ).

Odometry can be enabled or disabled directly through the main launch file:

| Argument | Default Value |
| :--- | :--- |
| `use_enc_odom` | `true` |

## Config File

To ensure consistency between the C++ driver and the Python odometry node, both share the `src/config.hpp` file. This file centralizes the physical constants of the robot.

| Parameter | Description |
| :--- | :--- |
| `WHEEL_RADIUS` | Wheel radius (in meters). |
| `TICK_PER_ROT` | Number of encoder pulses per full wheel rotation. |
| `WHEEL_SEPARATION_WIDTH` | Distance between left and right wheels. |
| `WHEEL_SEPARATION_LENGTH` | Distance between front and rear wheels. |
| `Kp, Ki, Kd, Kf` | Gains for the motor PIDF control. |
	
### Motor Inversion:

Depending on the physical assembly and motor orientation on the chassis, encoders may count in reverse (e.g., decreasing when the robot moves forward).

***Warning:*** If the odometry displays inverted behavior (e.g., the robot rotates but the log indicates forward movement), you must adjust the multiplier signs in the `encoder_callback` within the `encoder_odometry.py` file to normalize the rotation direction.

## PIDF Calibration

The following tools are recommended for PIDF calibration:

Use `rqt_plot` to visualize motor behavior in real-time. This allows you to compare the target speed with the current speed and observe oscillations or lag. Remember to check the graph scale to ensure data is visible.

```bash
ros2 run rqt_plot rqt_plot /m0/speed/data /m0/speed_alvo/data
```

Since the expansion_hub_node uses the ROS 2 Lifecycle system, you must deactivate the node before applying new gains to ensure the changes are processed correctly.

*Workflow*:
1. Deactivate the control node.
2. Change the desired parameter (kP,kI,kD,or kF).
3. Reactivate the node to test the new value.

```bash
ros2 lifecycle set /expansion_hub_node deactivate
ros2 param set /expansion_hub_node kf 20.0
ros2 lifecycle set /expansion_hub_node activate
```
To test the system's response to commands, use the keyboard teleop package to send different velocities to the robot:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
