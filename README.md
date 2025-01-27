# micro_rosso_oruga

This is the firmware for the Oruga robot. It allows you to interact with the hardware through ROS topics. For example, you can move the robot by sending it `/cmd_vel` topics or read the generated odometry from the `/odom` topic.

This firmware is based on the [micro_rosso_platformio](https://github.com/xopxe/micro_rosso_platformio) library.

This software runs on the onboard ESP32 microcontroller. The associated ROS system intended to run on a computer / SBC is available here: [oruga_ws](https://github.com/xopxe/oruga_ws). You can find the robot's URDF model and other high-level control tools there.

## The robot

Oruga is a differential tracked robot for outdoor use. It is based on the [Lynxmotion A4WD3](https://www.lynxmotion.com/a4wd3-rugged-rovers/) platform and has a custom-built ESP32-based control system.

## Electric schematics

The wiring schematics are in the [docs](docs/) directory. As always, the first movements must be done with the robot "in the air". If the motor's polarities or encoder directions are reversed, the robot can misbehave violently. It is recommended to run with rviz2 enabled to verify that all directions match: physical motors vs reported odometry and joint rotation in rviz2 (see [oruga_ws](https://github.com/xopxe/oruga_ws) for instructions).

## micro-ros interface

//FIXME

The default node name for this firmware is `oruga_rclc`.

The robot subscribes to the following topics:

* `geometry_msgs.msg.TwistStamped /cmd_vel`.

The robot publishes the following topics to report on movement:

* `sensor_msgs.msg.JointState /joint_states` for each wheel. The joints are named `fl_wheel_joint` (forward-left), `rr_wheel_joint` (rear-right), and so on. Notice that there are 4 joints reported, but if you have tracks installed, the same-side pairs should move in sync.

* `nav_msgs.msg.Odometry /odom` (and the associated `/tf` transform)

The robot publishes the following topics from and onboard MPU6050 IMU:

* `sensor_msgs.msg.Imu /imu/raw`.

* `sensor_msgs.msg.Temperature /imu/temperature`.

The robot publishes the following topics from and onboard BME680 environmental sensor (installed inside the chassis):

* `sensor_msgs.msg.Temperature /internal/temperature`

* `sensor_msgs.msg.RelativeHumidity /internal/humidity`

* `sensor_msgs.msg.FluidPressure /internal/pressure`

* `std_msgs.msg.Float32 /internal/gas_resistance`

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

## TODO

Some pending work:

* Add a kill switch: pull  Sabertooth's S2 low. Also sense (through a divider or a 5 - 3.3 converter) on the ESP32, notify throug topic.

* Emergency stop on IMU limits

* Perhaps isolate 3.3 - 5 everything, S1 serial, and encoders.

* Sense battery voltage (with ADC?), report status

* Alternative ROS architecture, with ros2_control and only joint driver on micro-ros.

## FIXME

Problems with the firmware to be fixed:

* Too high topic rates break something in the serial link. 50Hz works fine, 100Hz corrupts serial. Sometimes the USB has to be reconnected.

## Authors and acknowledgment

<jvisca@fing.edu.uy> - [Grupo MINA](https://www.fing.edu.uy/inco/grupos/mina/), Facultad de Ingeniería - Udelar, 2024

## License

Apache 2.0
