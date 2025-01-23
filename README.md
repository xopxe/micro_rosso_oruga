# micro_rosso_oruga

This is the firmware for the Oruga robot. It allows you to interact with the hardware through ROS topics. For example, you can move the robot by sending it `cmd_vel` topics or read the onboard IMU on the `imu_raw` topic.

This firmware is based on the [micro_rosso_platformio](https://github.com/xopxe/micro_rosso_platformio) library.

This software runs on the onboard ESP32 microcontroller. The associated ROS system intended to run on a computer / SBC is available here: [oruga_ws](TODO). There, you can find the robot's URDF model and other high-level control tools.

## The robot

Oruga is a differential tracked robot for outdoor use. It is based on the [Lynxmotion A4WD3](https://www.lynxmotion.com/a4wd3-rugged-rovers/) platform and has a custom-built ESP32-based control system.

## Electronic System

//TODO Schematics

## micro-ros interface

//FIXME

The default node name for this firmware is `oruga_rclc`.

The robot subscribes to the following topics:

* `geometry_msgs.msg.TwistStamped /cmd_vel`

* `sensor_msgs.msg.Joy /joy`

The robot publishes the following topics:

* `sensor_msgs.msg.Imu /imu/raw`

* `sensor_msgs.msg.Temperature /imu/temperature`

* `sensor_msgs.msg.JointState /joint_states`

* `nav_msgs.msg.Odometry /odom` (and the associated `/tf` transform)

Some other interfaces associated with micro_rosso are available, such as the `/rosout` topic and time synchronization service.

## Installation tutorial

//TODO

## Connecting to the robot

//TODO udev

Connect your PC to the USB cable, and on a console run:

```sh
docker run -it --rm --device=/dev/ttyUSB0 --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0 -b 115200
```

Do `ros2 topic list` to see the robot's topics.

## Authors and acknowledgment

<jvisca@fing.edu.uy> - [Grupo MINA](https://www.fing.edu.uy/inco/grupos/mina/), Facultad de Ingeniería - Udelar, 2024

## License

Apache 2.0
