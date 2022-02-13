# dynamixel-controller
This is a HardwareInterface for [`ros2_control`](https://github.com/ros-controls/ros2_control) for dynamixel actuators from [ROBOTIS](http://en.robotis.com/). Its mainly developed for and with the [XL](http://en.robotis.com/shop_en/list.php?ca_id=202030) series of actuators,
so other dynamixels may or may not work. Open up a issue when you encounter problems. Pull requests
are welcome too.

## Build
Make sure the [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) for ROS2 is installed.
Clone it in your workspace and checkout the branch ros2:
```
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b ros2
cd DynamixelSDK/
```
finally build with colcon
`colcon build`

## Configure Dynamixel motor parameters via URDF

Add the `serial_port`, `baud_rate`, and `id` parameters to the ros2_control part of your `.urdf` / `.xarco` files. 

```xml
<ros2_control name="my_robot" type="system">
      <hardware>
        <plugin>dynamixel_ros2_control/DynamixelHardwareInterface</plugin>
        <param name="serial_port">/dev/ttyUSB0</param>
        <param name="baud_rate">4000000</param>
      </hardware>
      <joint name="joint1">
        <param name="id">10</param>
        <!--param name="reboot_on_error">true</param-->
        <command_interface name="position"/>
        <!--command_interface name="velocity"/-->
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      ...
```
When `reboot_on_error` is "true" the motor automaticaly reboots on mechanical overload error (it still loses torque for a brief moment, but that should be enogh to let the motor recover the fault).

## Run a test
You need the urdf of the robot with the ros2_control xml added.
After starting the control nodes and activating the Joint Trajectory Controller you can use ros2cli commands to test the actuators:
```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{header: {stamp: {sec: '"$(date '+%s + 6' | bc)"'}}, joint_names: ["joint1"], points: [{positions: [0.0, ], velocities: [0, ], time_from_start: {sec: 0}}]}' --once
```
The example launch file expects the urdf in its robot description package and a robot bringup package with the `controller.yaml` (example for that is also in this package).

## USB Latency tuning

To increase the frequency the actuators can be read and be written to via usb you can decrease the usb latency. On Linux you can do that with the following command (if the dynamixel serial device is on `ttyUSB0`):
```
sudo chmod o+w /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

This change however gets lost on reboot. You can create a systemd-service to automatically do this on reboot:
```
[Unit]
Description=lower usb latency timer

[Service]
Type=oneshot
User=root
ExecStart=/usr/bin/bash -c '/usr/bin/echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'

[Install]
WantedBy=multi-user.target
```
(Filepath `/etc/systemd/system/usb-latency.service`)
and enable with:
```
sudo systemctl enable usb-latency.service --now
```

## Credits
This is partly based on the work of Youtalk ([GitHub link](https://github.com/youtalk/dynamixel_control)), who made a dynamixel ros2_control driver based on the official dynamixel_workbench packages.
I swaped out the dynamixel_workbench parts with my own implementation, that only uses the dynamixelSDK.
