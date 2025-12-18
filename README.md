# Motionmind Hardware Interface
ROS2 control hardware interface for Solution Cubed Motionmind Rev2/3 motor controllers.

## Usage
Add the plugin to the `ros2_control` tag in your URDF file:
```xml
<ros2_control name="motionmind_hardware" type="system">
    <hardware>
        <plugin>motionmind_hardware/MotionmindHardware</plugin>

        <param name="port">/dev/ttyS0</param>
        <param name="baudrate">19200</param>
        <param name="timeout">100</param>
    </hardware>
    <joint name="front_left_steering_joint">
        <param name="address">2</param>
        <param name="offset_limit">350</param>
        <param name="home">503</param>

        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>
    <joint name="front_right_steering_joint">
        <param name="address">6</param>
        <param name="offset_limit">350</param>
        <param name="home">550</param>
        
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>
</ros2_control>
```

Each node should be connected to a single motor controller which are connected by TTL signals. The following parameters are required:

| Parameter name | Explanation |
|---|---|
| `port` | Port at which the master motor controller is connected by RS232 (e.g. `/dev/ttyS0`) |
| `baudrate` | Baudrate for serial communication |
| `timeout` | Serial communication timeout |

For each joint, an `address` parameter should be defined containing the physical device id of the motor controller.

## Supported command interfaces
Command interface `position` and `velocity` are supported.

The position command interface requires the motor controller to be in mode 5 (Analog PID control). Additional parameters required for each joint are:

| Parameter name | Explanation |
|---|---|
| `home` | The value at which the position angle is zero degree |
| `offset_limit` | The maximum offset from the `home` position |

The velocity command interface requires the motor controllers to be in mode 4 (Serial PID control). Additional parameters required for each joint are:

| Parameter name | Explanation |
|---|---|
| `cpr` | Counts per revolution for the encoder |
| `gear_ratio` | Gear ratio of the gearbox |
| `encoder_type` | Either be `1x` for 1 channel encoders or `4x` for quadrature encoders |

## Supported state interfaces
The following state interfaces are supported:

* `position` (Position, only in position mode)
* `velocity` (Velocity, only in velocity mode)
* `neglimit` (Set when the input NEG_LIM (J4 P16) is at 0V)
* `poslimit` (Set when the input POS_LIM (J4 P17) is at 0V)
* `brake` (Set when the input _BRAKE (J4 P12) is at 0V)
* `index` (Set when the input IND/RC (J6 P2) is at 0V)
* `badrc` (Set when the R/C pulse is not received within 50ms, is shorter than RCMIN – 0.5ms, or is longer than RCMAX + 0.5ms)
* `vnlimit` (Set when the negative virtual limit position is reached)
* `vplimit` (Set when the positive virtual limit position is reached)
* `currentlimit` (Set when the AMPS register has exceeded the AMPS LIMIT setting and the PWM signal is being regulated down)
* `pwmlimit` (Set when the PWMOUT register has exceeded the PWM LIMIT setting and the PWM signal is being regulated down)
* `inposition` (Set when the in closed loop mode and the DESIRED POSITION equals the ACTUAL POSITION register. If the dead band is enabled this bit is set when the DESIRED POSITION equals the ACTUAL POSITION and remains set as long as the ACTUAL POSITION is DESIRED POSITION +/- DEADBAND)
* `tempfault` (Set when the thermistor reads higher than ˚90C, cleared when the temperature drops below ˚80C)

## Check single device script
To check whether the connection to a motor controller is correct and check for faults, run:

```commandline
ros2 run motionmind_hardware check_single_device <address of motor controller>
```

## Calibration script
For calculating the `home` parameter when the motor controllers are in mode 4 (Analog PID control), a Python script is provided.

> :warning: **For using the calibration script, the limit switches need to work**: Be very careful here! The motors will turn till the negative and positive limit switches are reached. When they are not present, or do not work, the motors will indefinitely turn and may break your cables! 

To use this script, run:

```commandline
ros2 run motionmind_hardware calibrate_steering <address of motor controller>
```

## Adapt PID settings
To adapt the PID filter inside the motor controller, run:

```commandline
ros2 run motionmind_hardware set_pid_values <address of motor controller>
```